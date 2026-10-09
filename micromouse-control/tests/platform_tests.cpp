#include "micromouse_app.h"
#include "stm32_platform.hpp"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "usart.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <limits>

#define CHECK(x) do { if(!(x)) { std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
GPIO_TypeDef gpio_a,gpio_b,gpio_c;
TIM_TypeDef timers[6];
TIM_HandleTypeDef htim2{&timers[0]},htim3{&timers[1]},htim4{&timers[2]},
    htim5{&timers[3]},htim6{&timers[4]},htim8{&timers[5]};
SPI_HandleTypeDef hspi2; ADC_HandleTypeDef hadc1; UART_HandleTypeDef huart3;
static uint32_t irq_mask=0,adc_channel=0;
static uint64_t clock_us=0,next_tick=0;
static bool scheduler=false,spi_failure=false,adc_failure=false,rx_failure=false;
static uint8_t registers[128]{},*rx_destination=nullptr;
static std::string last_tx;
static unsigned adc_reads[5]{},forbidden_accel_reads=0;
uint32_t __get_PRIMASK() { return irq_mask; }
void __disable_irq() { irq_mask=1; }
void __set_PRIMASK(uint32_t m) { irq_mask=m; }
void __DMB() {}
static void advance(uint32_t us) {
    clock_us+=us; htim2.Instance->CNT=uint32_t(clock_us);
    while(scheduler && clock_us>=next_tick) {
        next_tick+=1000;
        if(!irq_mask) MM_ControlTickISR();
    }
}
void HAL_Delay(uint32_t ms) { advance(ms*1000); }
void HAL_GPIO_WritePin(GPIO_TypeDef* p,uint16_t pin,GPIO_PinState state) {
    if(state==GPIO_PIN_SET) p->state|=pin; else p->state&=uint16_t(~pin);
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* p,uint16_t pin) { return p->state&pin?GPIO_PIN_SET:GPIO_PIN_RESET; }
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*) { scheduler=true; next_tick=clock_us+1000; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef* c) {
    CHECK(c->Rank==1 && c->SamplingTime==ADC_SAMPLETIME_144CYCLES); adc_channel=c->Channel; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t) {
    advance(adc_failure?1000:8); return adc_failure?HAL_TIMEOUT:HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*) {
    unsigned i=0; while(i<5 && board::receivers[i].adc_channel!=adc_channel) ++i;
    CHECK(i<5); ++adc_reads[i];
    unsigned active=0;
    for(auto p:board::emitters) if(p.port->state&p.number) ++active;
    CHECK(active<=1); // Không bao giờ bật hai nhóm cùng lúc.
    const auto p=board::emitters[board::receivers[i].group];
    const bool on=(p.port->state&p.number)!=0;
    CHECK(!active || on); // Nếu có LED sáng, ADC phải thuộc chính nhóm đang sáng.
    return on?300+i*20:100;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t* tx,uint8_t* rx,uint16_t n,uint32_t) {
    advance(spi_failure?1000:uint32_t(n)*13);
    const unsigned reg=tx[0]&0x7f;
    if(reg>=0x3b && reg<=0x42) ++forbidden_accel_reads;
    if(reg==0x43) CHECK(n==7);
    if(spi_failure) return HAL_TIMEOUT;
    rx[0]=0; for(unsigned i=1;i<n;++i) rx[i]=registers[reg+i-1];
    if(reg==0x3a && !(registers[0x38]&1)) rx[1]=0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t* tx,uint16_t n,uint32_t) {
    CHECK(n==2); advance(26);
    if(spi_failure) return HAL_TIMEOUT;
    registers[tx[0]]=tx[1]; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*,uint8_t* out,uint16_t n) {
    CHECK(n==1); rx_destination=out; return rx_failure?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t* data,uint16_t n) {
    last_tx.assign(reinterpret_cast<char*>(data),n); MM_UartTxISR(); return HAL_OK;
}
static void run(unsigned milliseconds) {
    const uint64_t until=clock_us+milliseconds*1000u;
    while(clock_us<until) { advance(10); MM_Poll(); }
}
static void send(char c) { CHECK(rx_destination); *rx_destination=uint8_t(c); MM_UartRxISR(); MM_Poll(); }
static bool outputsZero() {
    return htim3.Instance->CCR[0]==0 && htim3.Instance->CCR[1]==0 &&
           htim4.Instance->CCR[0]==0 && htim4.Instance->CCR[1]==0;
}
int main() {
    gpio_c.state=GPIO_PIN_8|GPIO_PIN_9; // Hai nút đang nhả.
    htim3.Instance->ARR=htim4.Instance->ARR=4199;
    htim5.Instance->ARR=0xffffffffu; htim8.Instance->ARR=65535;
    registers[0x75]=0x12; registers[0x3a]=1;
    CHECK(MM_Init()); CHECK(outputsZero());
    CHECK(registers[0x6c]==0x38 && registers[0x1b]==0x18);
    CHECK(registers[0x38]==1 && registers[0x37]==0x20);
    CHECK(!MM_ArmMotion() && !MM_TestStraight(0.18f,0));
    run(200);
    CHECK(firmware::diagnostics.ir_frames>40);
    for(unsigned i=0;i<5;++i) {
        CHECK(adc_reads[i]>40);
        CHECK(firmware::diagnostics.ir_difference[i]==int32_t(200+i*20));
    }
    htim8.Instance->CNT=65530; run(2); htim8.Instance->CNT=5; run(2);
    const auto before=firmware::readSensors(); CHECK(before.ticks[0]==5); // -6 rồi +11.
    CHECK(MM_ArmBench() && MM_TestMotorPulse(0,1)); run(10);
    CHECK(htim3.Instance->CCR[0]>0 && htim3.Instance->CCR[1]==0);
    // Ngắt STOP ngắt PWM ngay, kể cả khi main chưa chạy.
    *rx_destination='s'; MM_UartRxISR(); CHECK(outputsZero()); MM_Poll();
    CHECK(!MM_TestMotorPulse(0,1)); // Arm bị tiêu thụ.
    CHECK(MM_ArmBench() && MM_TestMotorPulse(1,0)); run(10);
    CHECK(htim4.Instance->CCR[1]>0);
    advance(6000); CHECK(outputsZero()); // Timer ISR watchdog khi main bị treo.
    MM_Poll(); CHECK(!MM_ArmBench());
    send('r'); CHECK(MM_ArmBench()); // Regression: fault latch không được nuốt lệnh reset.
    CHECK(MM_TestMotorPulse(1,1)); run(310); CHECK(outputsZero());

    CHECK(MM_CalibrateGyro()); run(2200);
    CHECK(firmware::diagnostics.calibrated && !firmware::isCalibrating());
    CHECK(firmware::diagnostics.calibration_count==board::gyro_calibration_samples);
    CHECK(!MM_ArmMotion()); // Chưa nhập geometry/gain dù gyro đã tốt.
    CHECK(MM_CalibrateGyro()); run(10); ++htim8.Instance->CNT; run(10);
    CHECK(!firmware::diagnostics.calibrated && firmware::diagnostics.calibration_failed);
    CHECK(MM_CalibrateGyro()); registers[0x3a]=0; run(5100);
    CHECK(!firmware::diagnostics.calibrated && firmware::diagnostics.calibration_failed);
    registers[0x3a]=1; CHECK(MM_CalibrateGyro()); run(2200);
    CHECK(firmware::diagnostics.calibrated);
    board::control.geometry_verified=board::control.polarity_verified=true;
    for(auto& w:board::control.wheel) { w.metres_per_count=0.00001f; w.pi_gain={1,7,0,0.2f,0.02f}; }
    CHECK(MM_ResetController()); run(20);
    CHECK(MM_ArmMotion() && MM_TestTurn90(1)); run(60);
    CHECK(htim3.Instance->CCR[1]>0 && htim4.Instance->CCR[0]>0); // Quay trái đúng mixer.
    MM_Stop(); CHECK(outputsZero());
    // Chặn output không hữu hạn trước clamp, kể cả +/-infinity.
    firmware::setDriveEnabled(true);
    firmware::setMotorOutputs(std::numeric_limits<float>::infinity(),0);
    CHECK(outputsZero() && !firmware::driveEnabled());
    // 32-bit microsecond clock wrap không gây dt âm/timeout giả.
    clock_us=0xfffffff0u; htim2.Instance->CNT=uint32_t(clock_us); next_tick=clock_us+1000;
    CHECK(MM_ResetController()); run(30); CHECK(MM_ArmMotion());
    CHECK(MM_TestStraight(0.18f,0)); run(20); CHECK(!outputsZero());
    spi_failure=true; run(5); CHECK(outputsZero() && !firmware::gyroHardwareReady()); spi_failure=false;
    MM_Stop(); CHECK(MM_ResetController());
    adc_failure=true; run(20); CHECK(!firmware::readSensors().ir_frame_ok);
    for(auto p:board::emitters) CHECK(!(p.port->state&p.number));
    CHECK(forbidden_accel_reads==0);
    std::puts("PASS: full C++ adapter build, pin groups/IR frames, gyro-only registers, config interlock, pulse, ISR stop, watchdog/reset, calibration, yaw mixer, micros wrap, SPI/ADC faults.");
}
