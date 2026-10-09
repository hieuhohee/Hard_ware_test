#include "hardware_tests.hpp"
#include "test_config.hpp"
#include <cstdlib>
#include <cstdio>

// Các test cố tình đưa lỗi bus, tràn counter và thiếu cấu hình vào module.
// Đây là mô phỏng logic, không mô phỏng analog hoặc thời gian ADC thực tế.
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
DwtType fake_dwt;
CoreDebugType fake_debug;
uint32_t SystemCoreClock=168000000;
static uint32_t tick, adc_channel;
static bool adc_failure, spi_failure;
static uint8_t registers[128]{};
static GPIO_TypeDef led_ports[5], cs_port;
uint32_t HAL_GetTick() { return tick; }
void HAL_Delay(uint32_t n) { tick+=n; fake_dwt.CYCCNT+=168000*n; }
void HAL_GPIO_WritePin(GPIO_TypeDef* p,uint16_t pin,GPIO_PinState s) {
    if (s==GPIO_PIN_SET) p->state|=pin; else p->state&=uint16_t(~pin);
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef* c) { adc_channel=c->Channel; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t) { return adc_failure?HAL_TIMEOUT:HAL_OK; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*) { return led_ports[adc_channel].state?300:100; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t) { return HAL_TIMEOUT; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t* tx,uint8_t* rx,uint16_t n,uint32_t) {
    // Chặn hồi quy sang đọc accel/temp; gói gyro phải đủ ba trục, 6 byte.
    const uint8_t reg=tx[0]&0x7F;
    CHECK(reg<0x3B || reg>0x42);
    if (reg==0x43) CHECK(n==7); // Sáu byte dữ liệu + một byte lệnh.
    if (spi_failure) return HAL_TIMEOUT;
    rx[0]=0; for(unsigned i=1;i<n;++i) rx[i]=registers[(tx[0]&0x7F)+i-1];
    if (reg==0x3A && !(registers[0x38]&1)) rx[1]=0; // Data ready chưa được enable.
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t* tx,uint16_t,uint32_t) {
    CHECK(tx[0]!=0x1C && tx[0]!=0x1D); // Không cấu hình accel.
    if (spi_failure) return HAL_TIMEOUT;
    registers[tx[0]]=tx[1]; return HAL_OK;
}
static void step(uint32_t ms=1) { HAL_Delay(ms); bench::poll(); }
static void send(char c) { bench::command=uint8_t(c); bench::poll(); }
int main() {
    using namespace bench;
    init(); // Cấu hình mặc định trống: không truy cập ngoại vi.
    CHECK(live.motor_ready_mask==0 && live.ir_ready_mask==0 && !live.imu_ready);
    send('1'); CHECK(live.motor_active==0);
    TIM_TypeDef pwm_hw, enc_l, enc_r;
    pwm_hw.ARR=4199; enc_l.CNT=65530; enc_r.ARR=0xFFFFFFFFu;
    TIM_HandleTypeDef pwm{&pwm_hw}, el{&enc_l}, er{&enc_r};
    ADC_HandleTypeDef adc; SPI_HandleTypeDef spi;
    motors[0]={{&pwm,TIM_CHANNEL_1},{&pwm,TIM_CHANNEL_2}};
    motors[1]={{&pwm,TIM_CHANNEL_3},{&pwm,TIM_CHANNEL_4}};
    encoders[0]={&el,+1}; encoders[1]={&er,-1};
    ir_adc=&adc; imu_spi=&spi; imu_cs={&cs_port,1};
    for(unsigned i=0;i<5;++i) ir[i]={{&led_ports[i],1},i,+1};
    registers[0x75]=0x12; registers[0x3A]=1;
    registers[0x43]=0xFF; registers[0x44]=0xFF; // Gyro X=-1.
    registers[0x45]=0x7F; registers[0x46]=0xFF; // Gyro Y=32767.
    registers[0x47]=0x80; registers[0x48]=0x00; // Gyro Z=-32768.
    init();
    CHECK(live.motor_ready_mask==3 && live.encoder_ready_mask==3 && live.ir_ready_mask==31 && live.imu_ready);
    CHECK(registers[0x6C]==0x38); // Accel standby, gyro XYZ bật.
    CHECK(registers[0x37]==0x20 && registers[0x38]==0x01); // Polling cần bật nguồn sự kiện.
    CHECK(live.motor_active==0);
    send('1'); CHECK(live.motor_active==1 && pwm_hw.CCR[0]==504 && pwm_hw.CCR[1]==0);
    step(100); send('3'); CHECK(live.motor_active==1); // Không gia hạn/đổi motor.
    step(200); CHECK(live.motor_active==0 && pwm_hw.CCR[0]==0);
    send('2'); CHECK(pwm_hw.CCR[1]==504); send('s'); CHECK(pwm_hw.CCR[1]==0);
    send('3'); CHECK(pwm_hw.CCR[2]==504); send(' '); CHECK(pwm_hw.CCR[2]==0);
    // Deadline qua tràn HAL tick.
    tick=0xFFFFFF80u; send('4'); step(300); CHECK(!live.motor_active && pwm_hw.CCR[3]==0);
    enc_l.CNT=5; enc_r.CNT=0xFFFFFFFFu; step();
    CHECK(live.encoder_total[0]==11 && live.encoder_total[1]==1);
    enc_l.CNT=65530; enc_r.CNT=0; step();
    CHECK(live.encoder_total[0]==0 && live.encoder_total[1]==0);
    send('i'); for(unsigned i=0;i<50;++i) step();
    CHECK(live.ir_frames>0);
    for(unsigned i=0;i<5;++i) CHECK(live.ir_off[i]==100 && live.ir_on[i]==300 && live.ir_difference[i]==200);
    send('s'); for(auto& p:led_ports) CHECK(p.state==0);
    adc_failure=true; send('i'); step(); step();
    CHECK(live.io_errors>0); for(auto& p:led_ports) CHECK(p.state==0);
    adc_failure=false;
    send('g'); step();
    CHECK(live.gyro_raw[0]==-1 && live.gyro_raw[1]==32767 && live.gyro_raw[2]==-32768);
    send('c'); for(unsigned i=0;i<1010;++i) step();
    CHECK(live.gyro_calibrated && !live.gyro_calibrating);
    CHECK(live.gyro_dps[0]<0.001f && live.gyro_dps[0]>-0.001f);
    registers[0x3A]=0; send('c'); for(unsigned i=0;i<5001;++i) step();
    CHECK(!live.gyro_calibrating && !live.gyro_calibrated);
    spi_failure=true; send('g'); step(); CHECK(!live.imu_ready && cs_port.state==1);
    spi_failure=false; registers[0x75]=0x68; init(); CHECK(!live.imu_ready && live.imu_whoami==0x68);
    ir[1].emitter=ir[0].emitter; init(); CHECK((live.ir_ready_mask&3)==0);
    motors[1].in1=motors[0].in1; init(); CHECK(live.motor_ready_mask==0);
    std::puts("PASS: empty config, motor pulse/stop/tick wrap, encoder wrap/sign, IR subtraction/error, IMU decode/calibration/timeout/wrong ID, resource conflicts");
}
