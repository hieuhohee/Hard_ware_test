#include "hardware_tests.hpp"
#include "test_config.hpp"
#include <cstdlib>
#include <cstdio>
extern "C" uint8_t bench_test_c_api_start_gyro(void);

// Các test cố tình đưa lỗi bus, tràn counter và thiếu cấu hình vào module.
// Đây là mô phỏng logic, không mô phỏng analog hoặc thời gian ADC thực tế.
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
DwtType fake_dwt;
CoreDebugType fake_debug;
uint32_t SystemCoreClock=168000000;
static uint32_t tick, adc_channel;
static bool adc_failure, spi_failure;
static bool unstable_identity, bad_config_readback;
static bool require_slow_mode3, fail_spi_init;
static unsigned identity_reads, gyro_config_writes;
static unsigned uart_receive_calls;
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
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t) { ++uart_receive_calls; return HAL_TIMEOUT; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef*) { return fail_spi_init ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* spi,uint8_t* tx,uint8_t* rx,uint16_t n,uint32_t) {
    // Chặn hồi quy sang đọc accel/temp; gói gyro phải đủ ba trục, 6 byte.
    const uint8_t reg=tx[0]&0x7F;
    CHECK(reg<0x3B || reg>0x42);
    if (reg==0x43) CHECK(n==7); // Sáu byte dữ liệu + một byte lệnh.
    if (spi_failure) return HAL_TIMEOUT;
    rx[0]=0; for(unsigned i=1;i<n;++i) rx[i]=registers[(tx[0]&0x7F)+i-1];
    if (reg==0x75) {
        ++identity_reads;
        // Mô phỏng chip yêu cầu I2C_IF_DIS trước khi đọc qua SPI.
        // Driver cũ đọc ID trước nên không thể vượt qua test này.
        if (!(registers[0x70]&0x40)) rx[1]=0;
        else if (unstable_identity && (identity_reads%2)==0) rx[1]=0x2E;
        else if(require_slow_mode3 && (spi->Init.CLKPolarity!=SPI_POLARITY_HIGH ||
                spi->Init.CLKPhase!=SPI_PHASE_2EDGE ||
                spi->Init.BaudRatePrescaler!=SPI_BAUDRATEPRESCALER_256)) rx[1]=0x2E;
    }
    if (reg==0x1B && bad_config_readback) rx[1]=0;
    if (reg==0x3A && !(registers[0x38]&1)) rx[1]=0; // Data ready chưa được enable.
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t* tx,uint16_t,uint32_t) {
    CHECK(tx[0]!=0x1C && tx[0]!=0x1D); // Không cấu hình accel.
    if (spi_failure) return HAL_TIMEOUT;
    if (tx[0]==0x1B) ++gyro_config_writes;
    registers[tx[0]]=tx[1];
    if (tx[0]==0x6B && tx[1]==0x80) registers[0x70]=0; // Reset interface.
    return HAL_OK;
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
    CHECK(live.imu_init_step==uint32_t(ImuInitStep::Ready));
    CHECK(live.imu_id_reads==8 && live.imu_id_matches==8);
    for (unsigned i=0;i<8;++i) CHECK(live.imu_id_history[i]==0x12);
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
    CHECK(live.imu_init_step==uint32_t(ImuInitStep::IdentityRejected));
    CHECK(live.imu_id_reads==8 && live.imu_id_matches==0 && live.imu_hal_status==HAL_OK);
    // ID 46 như ảnh thực tế: bus truyền xong nhưng driver phải giữ not-ready.
    const unsigned before_config=gyro_config_writes;
    registers[0x75]=0x2E; init();
    CHECK(!live.imu_ready && live.imu_whoami==46 && live.io_errors==0);
    CHECK(live.imu_id_matches==0 && gyro_config_writes==before_config);
    // ID chập chờn phải bị từ chối, không lấy đa số để báo ready giả.
    registers[0x75]=0x12; unstable_identity=true; identity_reads=0; init();
    CHECK(!live.imu_ready && live.imu_id_reads==8 && live.imu_id_matches==4);
    CHECK(live.imu_id_history[0]==0x12 && live.imu_id_history[1]==0x2E);
    CHECK(gyro_config_writes==before_config);
    unstable_identity=false;
    // Phân biệt lỗi HAL với sai ID, rồi phân biệt lỗi readback thanh ghi.
    spi_failure=true; init();
    CHECK(live.imu_init_step==uint32_t(ImuInitStep::BusError));
    CHECK(live.imu_last_reg==0x70 && live.imu_hal_status==HAL_TIMEOUT && live.imu_id_reads==0);
    CHECK(cs_port.state==1);
    spi_failure=false; bad_config_readback=true; init();
    CHECK(!live.imu_ready && live.imu_init_step==uint32_t(ImuInitStep::VerifyError));
    CHECK(live.imu_last_reg==0x1B && live.imu_expected_value==0x18 && live.imu_actual_value==0);
    bad_config_readback=false;

    // Dò thực sự bốn cấu hình: chỉ dòng mode3/256 trả ID đúng; sau đó
    // init/readback phải đạt trước khi công bố ready. Không chạy motor.
    require_slow_mode3=true; init(); CHECK(!live.imu_ready);
    CHECK(HardwareTests_ProbeGyroSpi());
    CHECK(live.spi_probe_done==1 && live.spi_probe_selected==4 && live.imu_ready);
    CHECK(!live.motor_active && live.imu_id_reads==8 && live.imu_id_matches==8);
    for(unsigned i=0;i<4;++i) CHECK(live.spi_trials[i].matches==(i==3?8u:0u));
    CHECK(live.spi_tx[0]==0xB8 && live.spi_trace_size==2); // Readback cuối INT_ENABLE.
    require_slow_mode3=false;
    // Sai ID ở cả bốn dòng: không báo ready, khôi phục mode/divider cũ.
    spi.Init.CLKPolarity=SPI_POLARITY_LOW; spi.Init.CLKPhase=SPI_PHASE_1EDGE;
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_64;
    registers[0x75]=0x2E; init(); CHECK(!HardwareTests_ProbeGyroSpi());
    CHECK(!live.imu_ready && live.spi_probe_selected==0);
    CHECK(spi.Init.CLKPolarity==SPI_POLARITY_LOW && spi.Init.BaudRatePrescaler==SPI_BAUDRATEPRESCALER_64);
    registers[0x75]=0x12; unstable_identity=true; identity_reads=0;
    CHECK(!HardwareTests_ProbeGyroSpi()); CHECK(!live.imu_ready);
    unstable_identity=false; bad_config_readback=true;
    CHECK(!HardwareTests_ProbeGyroSpi()); CHECK(!live.imu_ready);
    CHECK(live.imu_init_step==uint32_t(ImuInitStep::VerifyError));
    bad_config_readback=false; fail_spi_init=true;
    CHECK(!HardwareTests_ProbeGyroSpi()); CHECK(!live.imu_ready);
    CHECK(live.imu_init_step==uint32_t(ImuInitStep::BusError));
    fail_spi_init=false;

    // API gọi trực tiếp: có UART để log nhưng Update tuyệt đối không nhận lệnh.
    registers[0x75]=0x12; registers[0x3A]=1;
    UART_HandleTypeDef uart; console=&uart;
    CHECK(bench_test_c_api_start_gyro()); // Hàm trong file .c gọi API C++ thực.
    CHECK(!live.motor_active);
    CHECK(!HardwareTests_TestMotorLeft(0));
    CHECK(HardwareTests_TestMotorLeft(+1));
    CHECK(pwm_hw.CCR[0]==504 && pwm_hw.CCR[1]==0);
    CHECK(!HardwareTests_TestMotorRight(+1)); // Không đổi/gia hạn xung đang chạy.
    const unsigned old_rx_calls=uart_receive_calls;
    command='s'; HAL_Delay(100); HardwareTests_Update();
    CHECK(live.motor_active==1); // Biến command cũng không có tác dụng ở chế độ này.
    HAL_Delay(200); HardwareTests_Update();
    CHECK(!live.motor_active && pwm_hw.CCR[0]==0);
    command='3'; HAL_Delay(500); HardwareTests_Update();
    CHECK(!live.motor_active && uart_receive_calls==old_rx_calls);
    CHECK(HardwareTests_TestMotorRight(-1)); CHECK(pwm_hw.CCR[3]==504);
    HardwareTests_Stop(); CHECK(!live.motor_active && pwm_hw.CCR[3]==0);
    CHECK(HardwareTests_TestIR());
    for(unsigned i=0;i<50;++i) { HAL_Delay(1); HardwareTests_Update(); }
    CHECK(live.ir_frames>0 && live.ir_difference[0]==200);
    CHECK(HardwareTests_TestEncoders());
    const int64_t before=live.encoder_total[0];
    enc_l.CNT+=7; HAL_Delay(1); HardwareTests_Update();
    CHECK(live.encoder_total[0]==before+7);
    CHECK(HardwareTests_TestGyro()); HAL_Delay(1); HardwareTests_Update();
    CHECK(live.gyro_raw[2]==-32768);
    CHECK(HardwareTests_CalibrateGyro());
    for(unsigned i=0;i<1010;++i) { HAL_Delay(1); HardwareTests_Update(); }
    CHECK(live.gyro_calibrated && !live.gyro_calibrating);
    CHECK(HardwareTests_TestAllSensors());
    HardwareTests_Stop();
    for(auto& p:led_ports) CHECK(p.state==0);
    const uint32_t samples=live.imu_samples, frames=live.ir_frames;
    HAL_Delay(10); HardwareTests_Update();
    CHECK(live.imu_samples==samples && live.ir_frames==frames);
    live.imu_ready=0; CHECK(!HardwareTests_TestGyro());
    console=nullptr;
    ir[1].emitter=ir[0].emitter; init(); CHECK((live.ir_ready_mask&3)==0);
    motors[1].in1=motors[0].in1; init(); CHECK(live.motor_ready_mask==0);
    std::puts("PASS: empty config, motor pulse/stop/tick wrap, encoder wrap/sign, IR subtraction/error, IMU decode/calibration/timeout/wrong ID, direct API without command reception, resource conflicts");
}
