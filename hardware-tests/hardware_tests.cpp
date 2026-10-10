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
    if (!imu_spi || !valid(imu_cs) || !out || n==0 || n > 6) return false;
    tx[0] = reg | 0x80;
    pin(imu_cs, false);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(imu_spi, tx, rx, n+1, kIoTimeoutMs);
    live.imu_hal_status = uint32_t(status);
    live.imu_last_reg = reg;
    const bool ok = status == HAL_OK;
    pin(imu_cs, true);
    live.spi_trace_size=n+1;
    for (unsigned i=0; i<n+1u; ++i) { live.spi_tx[i]=tx[i]; live.spi_rx[i]=rx[i]; }
    if (ok) {
        std::memcpy(out, rx+1, n);
        live.imu_actual_value = rx[1]; // Byte đầu tiên của dữ liệu, không phải byte dummy.
    } else ++live.io_errors;
    return ok;
}
static bool spi_write(uint8_t reg, uint8_t value) {
    if (!imu_spi || !valid(imu_cs)) return false;
    uint8_t tx[2] = {uint8_t(reg & 0x7F), value};
    pin(imu_cs, false);
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(imu_spi, tx, 2, kIoTimeoutMs);
    live.imu_hal_status = uint32_t(status);
    live.imu_last_reg = reg;
    live.imu_expected_value = value;
    const bool ok = status == HAL_OK;
    pin(imu_cs, true);
    if (!ok) ++live.io_errors;
    return ok;
}
static void imu_step(ImuInitStep step) { live.imu_init_step=uint32_t(step); }
static bool imu_bus_failed() { imu_step(ImuInitStep::BusError); return false; }
static bool imu_init() {
    if (!imu_spi || !valid(imu_cs)) return false;
    pin(imu_cs, true); HAL_Delay(100);
    // Datasheet DS-000176 §6.1: tắt I2C ngay sau thời gian startup để giữ
    // giao diện SPI. Chỉ viết I2C_IF ở đây; các thanh ghi gyro chỉ được cấu
    // hình sau khi nhận đúng ID ICM20602. Không tự nhận mọi MPU là cùng chip.
    imu_step(ImuInitStep::Interface);
    if (!spi_write(0x70, 0x40)) return imu_bus_failed();
    imu_step(ImuInitStep::Identity);
    // Có thể init lại sau phép dò SPI; không cộng dồn ID của lần thất bại trước.
    live.imu_id_reads=live.imu_id_matches=0;
    live.imu_expected_value=0x12;
    // HAL_OK chỉ xác nhận MCU truyền xong. Đọc 8 giao dịch riêng để biết
    // ID có ổn định không; không lấy đa số để che giấu các mẫu sai/nhiễu.
    for (unsigned i=0; i<8; ++i) {
        uint8_t id=0;
        if (!spi_read(0x75, &id, 1)) return imu_bus_failed();
        live.imu_id_history[i]=id;
        ++live.imu_id_reads;
        if (id==0x12) ++live.imu_id_matches;
        live.imu_whoami=id;
        if (i!=7) HAL_Delay(2); // Init một lần, motor đã dừng.
    }
    if (live.imu_id_matches!=8) {
        imu_step(ImuInitStep::IdentityRejected);
        return false;
    }
    imu_step(ImuInitStep::Reset);
    if (!spi_write(0x6B, 0x80)) return imu_bus_failed(); // DEVICE_RESET.
    HAL_Delay(100);
    // Reset xóa cấu hình interface, vì vậy phải tắt I2C lại trước bước tiếp.
    if (!spi_write(0x70, 0x40)) return imu_bus_failed();
    imu_step(ImuInitStep::Configure);
    if (!spi_write(0x6B, 0x01) || // Thoát sleep, chọn clock tự động.
        !spi_write(0x6C, 0x38) || // PWR_MGMT_2: STBY_XA/YA/ZA=1, tắt accel;
                                 // STBY_XG/YG/ZG=0, giữ ba trục gyro hoạt động.
        !spi_write(0x1A, 0x02) || // Gyro DLPF 92 Hz, nguồn mẫu 1 kHz.
        !spi_write(0x19, 0x00) || // Divider=0: 1 kHz.
        !spi_write(0x1B, 0x18) || // Gyro ±2000 dps => 16.4 LSB/(deg/s).
        !spi_write(0x37, 0x20) || // INT dạng latch, chỉ clear khi đọc INT_STATUS.
        !spi_write(0x38, 0x01)) return imu_bus_failed(); // Bật nguồn DATA_RDY trong sensor;
                                            // MCU vẫn polling SPI, không cần EXTI.
    HAL_Delay(100); // Chỉ init khi motor đã dừng; không delay trong bài motor.
    imu_step(ImuInitStep::Verify);
    // Đọc lại thanh ghi để không báo ready khi ghi cấu hình thất bại âm thầm.
    const uint8_t regs[] = {0x70,0x6B,0x6C,0x1A,0x19,0x1B,0x37,0x38};
    const uint8_t vals[] = {0x40,0x01,0x38,0x02,0x00,0x18,0x20,0x01};
    for (unsigned i=0; i<sizeof(regs); ++i) {
        live.imu_expected_value=vals[i];
        uint8_t v=0;
        if (!spi_read(regs[i], &v, 1)) return imu_bus_failed();
        if (v != vals[i]) { imu_step(ImuInitStep::VerifyError); return false; }
    }
    imu_step(ImuInitStep::Ready);
    return true;
}
// Chụp thanh ghi MCU để đối chiếu pinmux/clock thực với file .ioc.
// Guard chỉ dành cho MCU: host test không giả vờ kiểm chứng GPIO vật lý.
static void snapshot_spi() {
#if defined(STM32F405xx)
    live.spi_cr1=imu_spi->Instance->CR1;
    live.spi_cr2=imu_spi->Instance->CR2;
    live.spi_sr=imu_spi->Instance->SR;
    live.gpiob_moder=GPIOB->MODER; live.gpiob_afrh=GPIOB->AFR[1];
    live.gpiob_idr=GPIOB->IDR; live.gpiob_odr=GPIOB->ODR;
#endif
}
// Phép đo độc lập HAL_SPI cho đúng pin SPI2 hiện tại. GPIO được khôi phục
// nguyên trạng sau phép đo. Không dùng cho handle/pin khác hoặc điều khiển.
static void probe_gpio_spi() {
#if defined(STM32F405xx)
    if(imu_spi->Instance!=SPI2 || imu_cs.port!=GPIOB || imu_cs.pin!=GPIO_PIN_12) return;
    const uint32_t old_mode=GPIOB->MODER, old_cr1=SPI2->CR1;
    pin(imu_cs,true);
    __HAL_SPI_DISABLE(imu_spi);
    GPIOB->BSRR=uint32_t(GPIO_PIN_13|GPIO_PIN_15)<<16; // Idle mode0: SCK LOW.
    GPIOB->MODER=(old_mode & ~((3u<<26)|(3u<<28)|(3u<<30))) | (1u<<26)|(1u<<30);
    bool timing_ok=true;
    auto wait_half=[&]() {
        // 2 us mỗi nửa chu kỳ, khoảng 250 kHz. Giới hạn vòng lặp để không
        // treo nếu DWT bị tắt; kết quả khi lỗi timing không được coi hợp lệ.
        const uint32_t start=DWT->CYCCNT;
        unsigned guard=10000;
        while(uint32_t(DWT->CYCCNT-start)<SystemCoreClock/500000u && --guard) {}
        if(!guard) timing_ok=false;
    };
    auto byte=[&](uint8_t tx) {
        uint8_t rx=0;
        for(unsigned bit=0;bit<8;++bit) {
            GPIOB->BSRR=(tx&0x80) ? GPIO_PIN_15 : uint32_t(GPIO_PIN_15)<<16;
            tx=uint8_t(tx<<1); wait_half();
            GPIOB->BSRR=GPIO_PIN_13; // Cạnh lên: sample MISO, MSB trước.
            wait_half(); rx=uint8_t((rx<<1)|((GPIOB->IDR&GPIO_PIN_14)?1:0));
            GPIOB->BSRR=uint32_t(GPIO_PIN_13)<<16;
        }
        return rx;
    };
    auto read=[&](uint8_t reg) {
        pin(imu_cs,false); wait_half(); byte(uint8_t(reg|0x80));
        const uint8_t value=byte(0); wait_half(); pin(imu_cs,true); wait_half();
        return value;
    };
    // Cùng lệnh tắt I2C như driver; không reset và không ghi cấu hình gyro.
    pin(imu_cs,false); wait_half(); byte(0x70); byte(0x40);
    wait_half(); pin(imu_cs,true); wait_half();
    live.gpio_spi_matches=0;
    for(unsigned i=0;i<8;++i) {
        live.gpio_spi_id[i]=read(0x75);
        if(live.gpio_spi_id[i]==0x12) ++live.gpio_spi_matches;
    }
    const uint8_t regs[]={0x00,0x0F,0x19,0x1A,0x1B,0x37,0x38,0x6B,0x6C,0x70,0x75,0x7F};
    for(unsigned i=0;i<12;++i) {
        live.probe_registers[i]=regs[i]; live.probe_values[i]=read(regs[i]);
    }
    pin(imu_cs,true); GPIOB->MODER=old_mode; SPI2->CR1=old_cr1;
    live.gpio_spi_done=timing_ok?1:2;
#endif
}
static bool probe_gyro_spi() {
    stop_everything();
    live.imu_ready=0; live.spi_probe_done=0; live.spi_probe_selected=0;
    live.gpio_spi_done=live.gpio_spi_matches=live.power_probe_done=0;
    if (!imu_spi || !valid(imu_cs)) return false;
    const auto original=imu_spi->Init;
    const uint32_t dividers[]={SPI_BAUDRATEPRESCALER_64,SPI_BAUDRATEPRESCALER_256};
    unsigned selected=4;
    for (unsigned row=0; row<4; ++row) {
        volatile SpiTrial& t=live.spi_trials[row];
        t.mode=row<2 ? 0 : 3; t.divider=(row%2)==0 ? 64 : 256;
        t.matches=0; t.hal_status=HAL_OK;
        for(unsigned i=0;i<8;++i) { t.id[i]=0; t.dummy[i]=0; }
        // CS giữ HIGH khi thay mode. HAL Init giữ cấu hình master/8-bit/MSB
        // của CubeMX, chỉ đổi CPOL/CPHA và divider; không đổi chân hoặc MSP.
        pin(imu_cs,true);
        imu_spi->Init=original;
        imu_spi->Init.CLKPolarity=row<2 ? SPI_POLARITY_LOW : SPI_POLARITY_HIGH;
        imu_spi->Init.CLKPhase=row<2 ? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE;
        imu_spi->Init.BaudRatePrescaler=dividers[row%2];
        HAL_StatusTypeDef status=HAL_SPI_Init(imu_spi);
        t.hal_status=uint32_t(status);
        if(status!=HAL_OK) continue;
        HAL_Delay(2);
        // Chỉ viết thanh ghi interface trước khi nhận ID đúng; không reset
        // hoặc cấu hình gyro của thiết bị chưa xác định.
        if(!spi_write(0x70,0x40)) { t.hal_status=live.imu_hal_status; continue; }
        for(unsigned i=0;i<8;++i) {
            uint8_t id=0;
            if(!spi_read(0x75,&id,1)) { t.hal_status=live.imu_hal_status; break; }
            t.id[i]=id; t.dummy[i]=live.spi_rx[0];
            if(id==0x12) ++t.matches;
            HAL_Delay(2);
        }
        // Vẫn đo đủ bốn dòng để có bằng chứng, chọn dòng hợp lệ đầu tiên.
        if(t.hal_status==HAL_OK && t.matches==8 && selected==4) selected=row;
    }
    // Nếu không có profile HAL hợp lệ, đo độc lập để tách lớp phần mềm.
    // Không tự chạy gyro bằng bit-bang; đây chỉ là phép chẩn đoán.
    if(selected==4) {
        probe_gpio_spi();
#if defined(STM32F405xx)
        // Một số trạng thái nguồn có thể tồn tại qua reset MCU vì ICM vẫn
        // có nguồn. Đối chiếu ID sau wake và reset sensor, tuyệt đối không
        // thay giá trị ID được chấp nhận hoặc bật đọc gyro của chip sai ID.
        bool power_ok=spi_write(0x6B,0x01);
        HAL_Delay(100);
        for(unsigned i=0;i<8;++i) {
            uint8_t id=0; power_ok=spi_read(0x75,&id,1) && power_ok;
            live.power_probe_id[i]=id; HAL_Delay(2);
        }
        power_ok=spi_write(0x6B,0x80) && power_ok;
        HAL_Delay(100);
        power_ok=spi_write(0x70,0x40) && power_ok;
        unsigned reset_matches=0;
        for(unsigned i=8;i<16;++i) {
            uint8_t id=0; power_ok=spi_read(0x75,&id,1) && power_ok;
            live.power_probe_id[i]=id; if(id==0x12) ++reset_matches;
            HAL_Delay(2);
        }
        live.power_probe_done=power_ok?1:2;
        // Chọn mode0/64 để init lại nếu reset khôi phục ID; imu_init vẫn
        // kiểm tra tám ID và toàn bộ readback trước khi công bố ready.
        if(power_ok && reset_matches==8) selected=0;
#endif
    }
    imu_spi->Init=original;
    if(selected<4) {
        imu_spi->Init.CLKPolarity=selected<2 ? SPI_POLARITY_LOW : SPI_POLARITY_HIGH;
        imu_spi->Init.CLKPhase=selected<2 ? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE;
        imu_spi->Init.BaudRatePrescaler=dividers[selected%2];
    }
    const HAL_StatusTypeDef status=HAL_SPI_Init(imu_spi);
    live.imu_hal_status=uint32_t(status);
    bool ok=status==HAL_OK && selected<4 && imu_init();
    if(ok) { live.spi_probe_selected=selected+1; live.imu_ready=1; }
    else {
        // Cả lỗi ID và lỗi đọc lại đều không được coi là cấu hình usable.
        imu_spi->Init=original;
        const HAL_StatusTypeDef restore=HAL_SPI_Init(imu_spi);
        if(restore!=HAL_OK) { live.imu_hal_status=uint32_t(restore); imu_bus_failed(); }
        else if(status!=HAL_OK) imu_bus_failed();
        live.imu_expected_value=0x12; // Tiêu chí ID, không để giá trị ghi interface 0x40.
    }
    snapshot_spi();
    live.spi_probe_done=1;
    return ok;
}
static int16_t signed_be(const uint8_t* p) {
    const uint16_t u = uint16_t((uint16_t(p[0])<<8) | p[1]);
    return int16_t(u < 0x8000u ? int32_t(u) : int32_t(u)-65536);
}
static void imu_update() {
    if (!live.imu_ready) return;
    uint8_t ready=0, data[6];
    if (!spi_read(0x3A, &ready, 1)) { live.imu_ready=0; live.gyro_calibrating=0; imu_bus_failed(); return; }
    if (!(ready & 1)) return; // Không đếm mẫu cũ lặp lại trong hiệu chuẩn.
    // GYRO_XOUT_H=0x43: đọc liền XH, XL, YH, YL, ZH, ZL.
    // Không truy cập vùng dữ liệu accel hoặc nhiệt độ.
    if (!spi_read(0x43, data, 6)) { live.imu_ready=0; live.gyro_calibrating=0; imu_bus_failed(); return; }
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
static void init_impl(bool show_menu) {
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
    if (show_menu) help();
    else log("READY motor=%lu encoder=%lu IR=%lu IMU=%lu WHO=0x%02lX\r\n",
        (unsigned long)live.motor_ready_mask,(unsigned long)live.encoder_ready_mask,
        (unsigned long)live.ir_ready_mask,(unsigned long)live.imu_ready,(unsigned long)live.imu_whoami);
}
void init() { init_impl(true); } // Giữ tương thích project cũ dùng menu.

// Cùng một đường điều khiển cho API trực tiếp và menu cũ: không sao chép
// logic PWM/timeout thành hai bản có thể lệch nhau khi sửa về sau.
static uint8_t start_motor(unsigned index, int direction) {
    if (index>=2 || (direction!=1 && direction!=-1) || live.motor_active) return 0;
    stop_everything();
    if (!(live.motor_ready_mask & (1u<<index))) {
        log("Motor not configured/ready\r\n"); return 0;
    }
    Pwm active=direction>0?motors[index].in1:motors[index].in2;
    const uint32_t counts=uint32_t((uint64_t(__HAL_TIM_GET_AUTORELOAD(active.timer))+1)*kMotorDutyPercent/100);
    // Log trước khi bật PWM để UART chậm không kéo dài xung motor.
    log("Motor %u %c: %u%%, %lu ms.\r\n",index,direction>0?'+':'-',
        kMotorDutyPercent,(unsigned long)kMotorPulseMs);
    motor_started=HAL_GetTick(); live.motor_active=index+1;
    __HAL_TIM_SET_COMPARE(active.timer,active.channel,counts);
    return 1;
}
static uint8_t start_sensors(bool read_ir, bool read_encoder, bool read_gyro, bool calibrate) {
    stop_everything();
    monitor_ir=read_ir && live.ir_ready_mask;
    monitor_encoder=read_encoder && live.encoder_ready_mask;
    monitor_imu=read_gyro && live.imu_ready;
    if (calibrate && monitor_imu) {
        for (float& x : calibration_sum) x=0;
        calibration_samples=0; calibration_started=HAL_GetTick();
        live.gyro_calibrated=0; live.gyro_calibrating=1;
        log("Keep robot STILL: collect 1000 fresh gyro samples; timeout 5 s.\r\n");
    }
    return monitor_ir || monitor_encoder || monitor_imu;
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
        start_motor(index,((c-'1')%2)==0?+1:-1);
        return;
    }
    start_sensors(c=='i'||c=='a',c=='e'||c=='a',c=='g'||c=='a'||c=='c',c=='c');
    help();
}
static void poll_impl(bool accept_commands) {
    const uint32_t now=HAL_GetTick();
    // Ưu tiên dừng motor trước mọi I/O chặn. Trong xung motor chỉ đọc UART
    // nonblocking và encoder; không đọc SPI/ADC hoặc in log.
    if (live.motor_active && uint32_t(now-motor_started)>=kMotorPulseMs) {
        stop_motors(); log("Motor pulse finished.\r\n");
    }
    if (accept_commands) {
        uint8_t c=command; command=0;
        if (!c && console) HAL_UART_Receive(console,&c,1,0);
        if (c) dispatch(c);
    }
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
void poll() { poll_impl(true); }
}
// API chính cho cách test bằng sửa một dòng trong main và build lại.
extern "C" void HardwareTests_Begin() { bench::init_impl(false); }
extern "C" void HardwareTests_Update() { bench::poll_impl(false); }
extern "C" uint8_t HardwareTests_TestIR() { return bench::start_sensors(true,false,false,false); }
extern "C" uint8_t HardwareTests_TestEncoders() { return bench::start_sensors(false,true,false,false); }
extern "C" uint8_t HardwareTests_TestGyro() { return bench::start_sensors(false,false,true,false); }
extern "C" uint8_t HardwareTests_ProbeGyroSpi() { return bench::probe_gyro_spi(); }
extern "C" uint8_t HardwareTests_TestAllSensors() { return bench::start_sensors(true,true,true,false); }
extern "C" uint8_t HardwareTests_CalibrateGyro() { return bench::start_sensors(false,false,true,true); }
extern "C" uint8_t HardwareTests_TestMotorLeft(int direction) { return bench::start_motor(0,direction); }
extern "C" uint8_t HardwareTests_TestMotorRight(int direction) { return bench::start_motor(1,direction); }
extern "C" void HardwareTests_Stop() { bench::stop_everything(); }
extern "C" void HardwareTests_Init() { bench::init(); }
extern "C" void HardwareTests_Poll() { bench::poll(); }
