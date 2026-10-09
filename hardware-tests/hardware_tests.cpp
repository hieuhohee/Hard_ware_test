#include "hardware_tests.hpp"
#include "test_config.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace bench {
static_assert(kMotorDutyPercent > 0 && kMotorDutyPercent <= 20, "Bench motor duty must be 1..20 percent");
static_assert(kMotorPulseMs > 0 && kMotorPulseMs <= 500, "Bench pulse must be at most 500 ms");
volatile Live live{};
volatile uint8_t command = 0;
static uint32_t motor_started, last_report, last_imu, last_encoder;
static uint32_t old_count[2];
static bool monitor_ir, monitor_encoder, monitor_imu;
static uint32_t cycles_per_us;
static unsigned ir_index, ir_phase;
static uint32_t ir_started;
static uint32_t calibration_started, calibration_samples;
static float calibration_sum[3];

static bool valid(Pin p) { return p.port && p.pin; }
static bool same_pin(Pin a, Pin b) { return a.port == b.port && a.pin == b.pin; }
static bool same_pwm(Pwm a, Pwm b) {
    return a.timer && b.timer && a.timer->Instance == b.timer->Instance &&
           a.channel == b.channel;
}
static void pin(Pin p, bool high) {
    if (valid(p)) HAL_GPIO_WritePin(p.port, p.pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void all_ir_off() {
    for (unsigned i = 0; i < kIrCount; ++i) pin(ir[i].emitter, false);
}
// Dùng DWT CYCCNT của Cortex-M4 để chờ thời gian IR mà không chiếm CPU bằng
// vòng while. Hiệu unsigned xử lý được một lần tràn bộ đếm (~25 s ở 168 MHz).
static bool elapsed_us(uint32_t start, uint32_t us) {
    return uint32_t(DWT->CYCCNT - start) >= us * cycles_per_us;
}
static void log(const char* fmt, ...) {
    if (!console) return;
    char text[220];
    va_list args; va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    if (n <= 0) return;
    const uint16_t size = uint16_t(n < int(sizeof(text)) ? n : sizeof(text)-1);
    // Chỉ in khi motor đã dừng. Có timeout, không dùng HAL_MAX_DELAY.
    if (HAL_UART_Transmit(console, reinterpret_cast<uint8_t*>(text), size, 30) != HAL_OK)
        ++live.io_errors;
}
static void pwm_zero(Pwm p) {
    if (p.timer) __HAL_TIM_SET_COMPARE(p.timer, p.channel, 0);
}
static void stop_motors() {
    for (unsigned i = 0; i < 2; ++i) {
        pwm_zero(motors[i].in1); pwm_zero(motors[i].in2);
    }
    live.motor_active = 0;
}
static void stop_everything() {
    stop_motors(); all_ir_off();
    monitor_ir = monitor_encoder = monitor_imu = false;
    ir_phase = 0;
    live.gyro_calibrating = 0;
}
static bool pwm_valid(Pwm p) {
    return p.timer && (p.channel == TIM_CHANNEL_1 || p.channel == TIM_CHANNEL_2 ||
                      p.channel == TIM_CHANNEL_3 || p.channel == TIM_CHANNEL_4);
}
static bool motor_valid(unsigned i) {
    const Motor& m = motors[i];
    if (!pwm_valid(m.in1) || !pwm_valid(m.in2) || same_pwm(m.in1, m.in2)) return false;
    const Pwm own[] = {m.in1, m.in2};
    const Pwm other[] = {motors[1-i].in1, motors[1-i].in2};
    for (Pwm a : own) {
        for (Pwm b : other) if (same_pwm(a,b)) return false;
        for (const Encoder& e : encoders)
            if (e.timer && e.timer->Instance == a.timer->Instance) return false;
    }
    return true;
}
static void sample_encoders() {
    for (unsigned i = 0; i < 2; ++i) {
        if (!(live.encoder_ready_mask & (1u << i))) continue;
        auto* timer = encoders[i].timer;
        const uint64_t modulus = uint64_t(__HAL_TIM_GET_AUTORELOAD(timer)) + 1u;
        const uint32_t current = __HAL_TIM_GET_COUNTER(timer);
        int64_t delta = int64_t(current) - int64_t(old_count[i]);
        if (delta > int64_t(modulus/2)) delta -= int64_t(modulus);
        if (delta < -int64_t(modulus/2)) delta += int64_t(modulus);
        old_count[i] = current;
        live.encoder_counter[i] = current;
        live.encoder_total[i] += delta * encoders[i].sign;
        // Điều kiện: dưới nửa vòng bộ đếm giữa hai lần đọc; đây không phải
        // nửa vòng cơ khí. Poll nhanh ~1 ms để dễ thỏa điều kiện này.
    }
}
static bool adc_read(unsigned i, uint16_t& result) {
    ADC_ChannelConfTypeDef c{};
    c.Channel = ir[i].adc_channel; c.Rank = 1;
    c.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    if (HAL_ADC_ConfigChannel(ir_adc, &c) != HAL_OK || HAL_ADC_Start(ir_adc) != HAL_OK) {
        ++live.io_errors; HAL_ADC_Stop(ir_adc); return false;
    }
    const bool ok = HAL_ADC_PollForConversion(ir_adc, kIoTimeoutMs) == HAL_OK;
    if (ok) result = uint16_t(HAL_ADC_GetValue(ir_adc));
    else ++live.io_errors;
    HAL_ADC_Stop(ir_adc);
    return ok;
}
static void ir_update() {
    if (!monitor_ir || !live.ir_ready_mask) return;
    if (ir_phase == 0) {
        // Tìm kênh đã cấu hình. Mỗi pha được tiếp tục ở lần poll tiếp theo.
        while (!(live.ir_ready_mask & (1u << ir_index))) ir_index = (ir_index+1)%kIrCount;
        all_ir_off(); ir_started = DWT->CYCCNT; ir_phase = 1;
    } else if (ir_phase == 1 && elapsed_us(ir_started, kIrSettleUs)) {
        uint16_t value;
        if (!adc_read(ir_index, value)) { monitor_ir = false; all_ir_off(); ir_phase=0; return; }
        live.ir_off[ir_index] = value;
        pin(ir[ir_index].emitter, true);
        ir_started = DWT->CYCCNT; ir_phase = 2;
    } else if (ir_phase == 2 && elapsed_us(ir_started, kIrSettleUs)) {
        uint16_t value;
        const bool ok = adc_read(ir_index, value);
        pin(ir[ir_index].emitter, false); // Tắt cả khi ADC timeout.
        if (!ok) { monitor_ir=false; ir_phase=0; return; }
        live.ir_on[ir_index] = value;
        live.ir_difference[ir_index] = ir[ir_index].signal_sign *
            (int32_t(value)-int32_t(live.ir_off[ir_index]));
        unsigned next = (ir_index+1)%kIrCount;
        while (!(live.ir_ready_mask & (1u<<next))) next = (next+1)%kIrCount;
        if (next <= ir_index) ++live.ir_frames;
        ir_index = next;
        ir_phase = 0;
    }
}
static bool spi_read(uint8_t reg, uint8_t* out, uint16_t n) {
    uint8_t tx[7]{}, rx[7]{}; // Tối đa một byte lệnh + sáu byte gyro XYZ.
    if (!imu_spi || !valid(imu_cs) || n > 6) return false;
    tx[0] = reg | 0x80;
    pin(imu_cs, false);
    const bool ok = HAL_SPI_TransmitReceive(imu_spi, tx, rx, n+1, kIoTimeoutMs) == HAL_OK;
    pin(imu_cs, true);
    if (ok) std::memcpy(out, rx+1, n); else ++live.io_errors;
    return ok;
}
static bool spi_write(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {uint8_t(reg & 0x7F), value};
    pin(imu_cs, false);
    const bool ok = HAL_SPI_Transmit(imu_spi, tx, 2, kIoTimeoutMs) == HAL_OK;
    pin(imu_cs, true);
    if (!ok) ++live.io_errors;
    return ok;
}
static bool imu_init() {
    if (!imu_spi || !valid(imu_cs)) return false;
    pin(imu_cs, true); HAL_Delay(100);
    uint8_t id = 0;
    if (!spi_read(0x75, &id, 1)) return false;
    live.imu_whoami = id;
    if (id != 0x12) return false; // Chỉ ICM20602; KHÔNG tự coi mọi MPU là cùng chip.
    if (!spi_write(0x6B, 0x80)) return false; // DEVICE_RESET.
    HAL_Delay(100);
    if (!spi_write(0x70, 0x40) || // I2C_IF_DIS của ICM20602 nằm ở 0x70.
        !spi_write(0x6B, 0x01) || // Thoát sleep, chọn clock tự động.
        !spi_write(0x6C, 0x38) || // PWR_MGMT_2: STBY_XA/YA/ZA=1, tắt accel;
                                 // STBY_XG/YG/ZG=0, giữ ba trục gyro hoạt động.
        !spi_write(0x1A, 0x02) || // Gyro DLPF 92 Hz, nguồn mẫu 1 kHz.
        !spi_write(0x19, 0x00) || // Divider=0: 1 kHz.
        !spi_write(0x1B, 0x18) || // Gyro ±2000 dps => 16.4 LSB/(deg/s).
        !spi_write(0x37, 0x20) || // INT dạng latch, chỉ clear khi đọc INT_STATUS.
        !spi_write(0x38, 0x01)) return false; // Bật nguồn DATA_RDY trong sensor;
                                            // MCU vẫn polling SPI, không cần EXTI.
    HAL_Delay(100); // Chỉ init khi motor đã dừng; không delay trong bài motor.
    // Đọc lại thanh ghi để không báo ready khi ghi cấu hình thất bại âm thầm.
    const uint8_t regs[] = {0x70,0x6B,0x6C,0x1A,0x19,0x1B,0x37,0x38};
    const uint8_t vals[] = {0x40,0x01,0x38,0x02,0x00,0x18,0x20,0x01};
    for (unsigned i=0; i<sizeof(regs); ++i) {
        uint8_t v=0; if (!spi_read(regs[i], &v, 1) || v != vals[i]) return false;
    }
    return true;
}
static int16_t signed_be(const uint8_t* p) {
    const uint16_t u = uint16_t((uint16_t(p[0])<<8) | p[1]);
    return int16_t(u < 0x8000u ? int32_t(u) : int32_t(u)-65536);
}
static void imu_update() {
    if (!live.imu_ready) return;
    uint8_t ready=0, data[6];
    if (!spi_read(0x3A, &ready, 1)) { live.imu_ready=0; live.gyro_calibrating=0; return; }
    if (!(ready & 1)) return; // Không đếm mẫu cũ lặp lại trong hiệu chuẩn.
    // GYRO_XOUT_H=0x43: đọc liền XH, XL, YH, YL, ZH, ZL.
    // Không truy cập vùng dữ liệu accel hoặc nhiệt độ.
    if (!spi_read(0x43, data, 6)) { live.imu_ready=0; live.gyro_calibrating=0; return; }
    ++live.imu_samples;
    for (unsigned i=0; i<3; ++i) {
        live.gyro_raw[i] = signed_be(data+2*i);
        const float raw_dps = live.gyro_raw[i]/16.4f;
        live.gyro_dps[i] = raw_dps - live.gyro_bias_dps[i];
        if (live.gyro_calibrating) calibration_sum[i] += raw_dps;
    }
    if (live.gyro_calibrating && ++calibration_samples >= 1000) {
        for (unsigned i=0; i<3; ++i) live.gyro_bias_dps[i] = calibration_sum[i]/calibration_samples;
        live.gyro_calibrating=0; live.gyro_calibrated=1;
        log("Gyro bias saved in RAM. Keep still during calibration.\r\n");
    }
}
static void help() {
    log("h help | s/SPACE stop | 1 L+ | 2 L- | 3 R+ | 4 R-\r\n");
    log("i IR | e encoders | g gyro | a all sensors | c gyro zero | r re-init\r\n");
    log("READY motor=%lu encoder=%lu IR=%lu IMU=%lu WHO=0x%02lX\r\n",
        (unsigned long)live.motor_ready_mask,(unsigned long)live.encoder_ready_mask,
        (unsigned long)live.ir_ready_mask,(unsigned long)live.imu_ready,(unsigned long)live.imu_whoami);
}
void init() {
    // Init chỉ gọi sau SystemClock_Config và tất cả MX_*_Init cần thiết.
    // Không sửa cấu hình trong lúc đang test: dừng rồi reset/init lại.
    stop_everything();
    // HAL đánh dấu channel BUSY sau Start. Stop trước khi Start lại để lệnh r
    // dùng được nhiều lần, kể cả khi lần trước chỉ một channel khởi động được.
    for (const Motor& m : motors) {
        if (pwm_valid(m.in1)) HAL_TIM_PWM_Stop(m.in1.timer,m.in1.channel);
        if (pwm_valid(m.in2)) HAL_TIM_PWM_Stop(m.in2.timer,m.in2.channel);
    }
    for (const Encoder& e : encoders)
        if (e.timer) HAL_TIM_Encoder_Stop(e.timer,TIM_CHANNEL_ALL);
    // Ghi từng byte qua con trỏ volatile; không gọi toán tử gán aggregate lên
    // đối tượng volatile. STM32F4 dùng IEEE-754: bit 0 biểu diễn float 0.0.
    auto* bytes = reinterpret_cast<volatile unsigned char*>(&live);
    for (unsigned i=0; i<sizeof(live); ++i) bytes[i]=0;
    cycles_per_us = SystemCoreClock / 1000000u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (unsigned i=0; i<2; ++i) {
        if (motor_valid(i)) {
            const bool a=HAL_TIM_PWM_Start(motors[i].in1.timer,motors[i].in1.channel)==HAL_OK;
            const bool b=HAL_TIM_PWM_Start(motors[i].in2.timer,motors[i].in2.channel)==HAL_OK;
            if (a && b) live.motor_ready_mask |= 1u<<i;
        }
        auto* t=encoders[i].timer;
        bool conflict=false;
        for (const Motor& m : motors)
            if (t && ((m.in1.timer && t->Instance==m.in1.timer->Instance) ||
                      (m.in2.timer && t->Instance==m.in2.timer->Instance))) conflict=true;
        if (t && encoders[1-i].timer && t->Instance==encoders[1-i].timer->Instance) conflict=true;
        if (t && !conflict && (encoders[i].sign==1 || encoders[i].sign==-1) &&
            (__HAL_TIM_GET_AUTORELOAD(t)==65535u || __HAL_TIM_GET_AUTORELOAD(t)==0xFFFFFFFFu) &&
            HAL_TIM_Encoder_Start(t,TIM_CHANNEL_ALL)==HAL_OK) {
            live.encoder_ready_mask |= 1u<<i;
            old_count[i]=__HAL_TIM_GET_COUNTER(t);
        }
    }
    for (unsigned i=0; i<kIrCount; ++i) {
        bool unique=true;
        for (unsigned j=0;j<kIrCount;++j)
            if (i!=j && valid(ir[j].emitter) &&
                (same_pin(ir[i].emitter,ir[j].emitter) || ir[i].adc_channel==ir[j].adc_channel)) unique=false;
        if (ir_adc && valid(ir[i].emitter) && ir[i].adc_channel<=ADC_CHANNEL_15 && unique &&
            (ir[i].signal_sign==1 || ir[i].signal_sign==-1) && cycles_per_us)
            live.ir_ready_mask |= 1u<<i;
    }
    live.imu_ready=imu_init();
    ir_index=ir_phase=0; command=0;
    last_report=last_imu=last_encoder=HAL_GetTick();
    help();
}
static void dispatch(uint8_t c) {
    // Không nối thêm xung motor khi đang chạy: phải đợi dừng rồi ra lệnh mới.
    if (c=='s' || c==' ') { stop_everything(); log("STOP\r\n"); return; }
    if (live.motor_active) return;
    if (c=='\r' || c=='\n') return;
    // Một lệnh help cũng có thể in nhiều dòng. Kết thúc xung IR trước khi
    // xử lý lệnh để UART chậm không giữ LED sáng ngoài ý muốn.
    all_ir_off(); ir_phase=0;
    if (c=='h' || c=='?') { help(); return; }
    if (c=='r') { init(); return; }
    if (c>='1' && c<='4') {
        const unsigned index=(c-'1')/2;
        stop_everything();
        if (!(live.motor_ready_mask & (1u<<index))) { log("Motor not configured/ready\r\n"); return; }
        const bool forward=((c-'1')%2)==0;
        Pwm active=forward?motors[index].in1:motors[index].in2;
        const uint32_t counts=uint32_t((uint64_t(__HAL_TIM_GET_AUTORELOAD(active.timer))+1)*kMotorDutyPercent/100);
        log("Motor %u %c: %u%%, %lu ms. s/SPACE stops.\r\n",index,forward?'+':'-',
            kMotorDutyPercent,(unsigned long)kMotorPulseMs);
        motor_started=HAL_GetTick(); live.motor_active=index+1;
        __HAL_TIM_SET_COMPARE(active.timer,active.channel,counts);
        return;
    }
    stop_everything();
    monitor_ir=(c=='i'||c=='a'); monitor_encoder=(c=='e'||c=='a');
    monitor_imu=(c=='g'||c=='a'||c=='c');
    if (c=='c' && live.imu_ready) {
        for (float& x : calibration_sum) x=0;
        calibration_samples=0; calibration_started=HAL_GetTick();
        live.gyro_calibrated=0; live.gyro_calibrating=1;
        log("Keep robot STILL: collect 1000 fresh gyro samples; timeout 5 s.\r\n");
    }
    help();
}
void poll() {
    const uint32_t now=HAL_GetTick();
    // Ưu tiên dừng motor trước mọi I/O chặn. Trong xung motor chỉ đọc UART
    // nonblocking và encoder; không đọc SPI/ADC hoặc in log.
    if (live.motor_active && uint32_t(now-motor_started)>=kMotorPulseMs) {
        stop_motors(); log("Motor pulse finished.\r\n");
    }
    uint8_t c=command; command=0;
    if (!c && console) HAL_UART_Receive(console,&c,1,0);
    if (c) dispatch(c);
    if (uint32_t(now-last_encoder)>=1) { last_encoder=now; sample_encoders(); }
    if (live.motor_active) return;
    ir_update();
    // Không chèn SPI hoặc log khi LED đang sáng: tránh kéo dài xung phát.
    if (ir_phase==2) return;
    if (monitor_imu && uint32_t(now-last_imu)>=1) { last_imu=now; imu_update(); }
    if (live.gyro_calibrating && uint32_t(now-calibration_started)>=5000) {
        live.gyro_calibrating=0; log("Gyro calibration TIMEOUT; check data ready/ODR.\r\n");
    }
    if (uint32_t(now-last_report)<kReportMs) return;
    last_report=now;
    if (monitor_encoder) log("ENC raw L=%lu R=%lu total L=%lld R=%lld\r\n",
        (unsigned long)live.encoder_counter[0],(unsigned long)live.encoder_counter[1],
        (long long)live.encoder_total[0],(long long)live.encoder_total[1]);
    if (monitor_ir) for (unsigned i=0;i<kIrCount;++i)
        if (live.ir_ready_mask&(1u<<i)) log("IR%u off=%u on=%u diff=%ld\r\n",i,
            unsigned(live.ir_off[i]),unsigned(live.ir_on[i]),(long)live.ir_difference[i]);
    // In raw counts để không cần bật printf float trong newlib-nano.
    // Tốc độ góc deg/s và bias đã có sẵn trong biến live cho debugger.
    if (monitor_imu) log("GYRO ready=%lu samples=%lu raw=%d,%d,%d errors=%lu\r\n",
        (unsigned long)live.imu_ready,(unsigned long)live.imu_samples,
        int(live.gyro_raw[0]),int(live.gyro_raw[1]),int(live.gyro_raw[2]),(unsigned long)live.io_errors);
}
}
extern "C" void HardwareTests_Init() { bench::init(); }
extern "C" void HardwareTests_Poll() { bench::poll(); }
