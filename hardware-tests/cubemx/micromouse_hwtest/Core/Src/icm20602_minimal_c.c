#include "icm20602_minimal_c.h"
#include "spi.h"
#include <math.h>

volatile IcmC_Diagnostics icm_c;
volatile IcmC_ConfigDiagnostics icm_c_config;
volatile IcmC_GyroDiagnostics icm_c_gyro;
volatile IcmC_AngleDiagnostics icm_c_angles;
volatile IcmC_QualityDiagnostics icm_c_quality;
volatile IcmC_WhoAmIDiagnostics icm_c_whoami;
static uint32_t last_poll;
static uint32_t angle_calibration_start, angle_previous_ms, angle_previous_valid;
static uint32_t angle_previous_clock, gyro_previous_clock, no_data_latched;
static float angle_mean[3], angle_m2[3], angle_previous_rate[3];
static void angles_frame(uint32_t now, uint32_t fresh, int saturated, uint32_t sample_clock);
static void angles_gap(void);

/* F405 Cortex-M4 có DWT CYCCNT. Dùng chênh lệch unsigned32 để xử lý wrap
   (~25.6s ở168MHz), không đổi timer/chân/clock CubeMX. dt tối đa50ms nên
   bình thường ngắn hơn một vòng counter. Khi debugger halt, counter có
   thể dừng: không khôi phục được chuyển động diễn ra trong lúc pause. */
static uint32_t gyro_clock_now(void) {
#if defined(__arm__) || defined(__thumb__)
    return DWT->CYCCNT;
#else
    return HAL_GetTick() * 1000u; /* Host fake clock: ticks microsecond. */
#endif
}
static void gyro_cs_guard(void) {
#if defined(__arm__) || defined(__thumb__)
    uint32_t start = gyro_clock_now();
    uint32_t cycles = icm_c_quality.clock_hz / 500000u; /* 2us setup/hold. */
    while ((uint32_t)(gyro_clock_now() - start) < cycles) { __NOP(); }
#endif
}
static void quality_event(uint32_t now, uint32_t reason) {
    unsigned row = icm_c_quality.history_next;
    icm_c_quality.events[row][0] = (int32_t)now;
    icm_c_quality.events[row][1] = (int32_t)reason;
    icm_c_quality.events[row][2] = (int32_t)icm_c_gyro.last_status;
    for (unsigned i = 0; i < 3; ++i) icm_c_quality.events[row][i + 3] = icm_c_gyro.raw[i];
    icm_c_quality.events[row][6] = (int32_t)icm_c_quality.last_dt_us;
    icm_c_quality.events[row][7] = (int32_t)icm_c_gyro.samples;
    ++icm_c_quality.events_written;
    icm_c_quality.history_next = (row + 1) % 64;
}

/* Chính xác hai byte: địa chỉ | bit READ, rồi dummy để tạo 8 clock dữ liệu.
   ID là RX[1]; RX[0] chưa phải dữ liệu của register. Đây là triển khai C
   độc lập, không gọi hardware_tests.cpp hoặc test_config.cpp. */
static int read_register(uint8_t reg, uint8_t *value, uint8_t *dummy) {
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0};
    uint8_t rx[2] = {0, 0};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1); /* Setup CS dài cho bài bench; không dùng trong vòng PID. */
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 20);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    icm_c.hal_status = (uint32_t)status;
    for (unsigned i = 0; i < 2; ++i) {
        icm_c.last_tx[i] = tx[i]; icm_c.last_rx[i] = rx[i];
    }
    if (status != HAL_OK) { ++icm_c.io_errors; return 0; }
    *value = rx[1]; *dummy = rx[0];
    return 1;
}

static int disable_i2c(void) {
    uint8_t tx[2] = {0x70, 0x40};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, tx, 2, 20);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    icm_c.hal_status = (uint32_t)status;
    if (status != HAL_OK) { ++icm_c.io_errors; return 0; }
    return 1;
}

static void identity_samples(int after) {
    for (unsigned i = 0; i < 16; ++i) {
        uint8_t value = 0, dummy = 0;
        int ok = read_register(0x75, &value, &dummy);
        ++icm_c.reads;
        if (after) {
            icm_c.id_after[i] = value; icm_c.dummy_after[i] = dummy;
            if (ok && value == 0x12) ++icm_c.matches_after;
        } else {
            icm_c.id_before[i] = value; icm_c.dummy_before[i] = dummy;
            if (ok && value == 0x12) ++icm_c.matches_before;
        }
        HAL_Delay(10);
    }
}

void IcmC_Begin(void) {
    volatile unsigned char *bytes = (volatile unsigned char *)&icm_c;
    for (unsigned i = 0; i < sizeof(icm_c); ++i) bytes[i] = 0;
    /* CubeMX đã cấu hình master8-bit/MSB/mode0, PB13/14/15 AF5 và CS PB12.
       Ghi lại thanh ghi MCU thực; không tự sửa chân hoặc clock để che lỗi. */
    icm_c.spi_cr1 = hspi2.Instance->CR1;
    icm_c.gpio_moder = GPIOB->MODER; icm_c.gpio_afrh = GPIOB->AFR[1];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(250);
    uint8_t value = 0, dummy = 0;
    if (read_register(0x70, &value, &dummy)) icm_c.interface_before = value;
    identity_samples(0);
    if (disable_i2c() && read_register(0x70, &value, &dummy))
        icm_c.interface_after = value;
    identity_samples(1);
    icm_c.id_ok = icm_c.matches_after == 16 && icm_c.interface_after == 0x40;
    icm_c.done = 1; /* Xong bài đo, không phải mặc định test đạt. */
    last_poll = HAL_GetTick();
}

void IcmC_Poll(void) {
    uint32_t now = HAL_GetTick();
    if (!icm_c.done || (uint32_t)(now - last_poll) < 100) return;
    last_poll = now;
    uint8_t value = 0, dummy = 0;
    int ok = read_register(0x75, &value, &dummy);
    icm_c.current_valid = (uint32_t)ok;
    icm_c.current_id = value;
    ++icm_c.reads;
    if (!ok || value != 0x12) icm_c.id_ok = 0;
}

/* Hai yếu tố khác baudrate: OSPEEDR quyết định độ dốc cạnh SCK/MOSI,
   PUPDR ở MISO giúp thử ảnh hưởng mức nghỉ/floating. Pull nội không phải
   cách sửa đường dây đứt; ID12 vẫn phải ổn định mới chọn cấu hình. */
void IcmC_ConfigAudit(void) {
    volatile unsigned char *bytes = (volatile unsigned char *)&icm_c_config;
    for (unsigned i = 0; i < sizeof(icm_c_config); ++i) bytes[i] = 0;
    const SPI_InitTypeDef original = hspi2.Init;
    const uint32_t original_speed = GPIOB->OSPEEDR, original_pull = GPIOB->PUPDR;
    const uint32_t speed_mask = (3u << 26) | (3u << 28) | (3u << 30);
    const uint32_t divider[] = {SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256};
    unsigned selected = 36;
    icm_c_config.pclk1_hz = HAL_RCC_GetPCLK1Freq();
    icm_c_config.pclk2_hz = HAL_RCC_GetPCLK2Freq();
    icm_c_config.hclk_hz = HAL_RCC_GetHCLKFreq();
    icm_c_config.rcc_cr = RCC->CR;
    icm_c_config.rcc_pllcfgr = RCC->PLLCFGR; icm_c_config.rcc_cfgr = RCC->CFGR;
    for (unsigned row = 0; row < 36; ++row) {
        unsigned mode = row / 18, div = (row / 6) % 3;
        unsigned speed = (row / 3) % 2, pull = row % 3;
        volatile IcmC_ConfigTrial *t = &icm_c_config.trial[row];
        t->mode = mode ? 3 : 0; t->divider = 64u << div;
        t->speed = speed ? 3 : 0; t->pull = pull;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        hspi2.Init = original;
        hspi2.Init.CLKPolarity = mode ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = mode ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
        hspi2.Init.BaudRatePrescaler = divider[div];
        if (HAL_SPI_Init(&hspi2) != HAL_OK) { ++t->errors; continue; }
        GPIOB->OSPEEDR = (original_speed & ~speed_mask) | (speed ? speed_mask : 0);
        GPIOB->PUPDR = (original_pull & ~(3u << 28)) | (pull << 28);
        HAL_Delay(2);
        uint8_t value = 0, dummy = 0;
        if (!disable_i2c() || !read_register(0x70, &value, &dummy)) ++t->errors;
        t->interface_value = value;
        for (unsigned i = 0; i < 8; ++i) {
            value = 0;
            if (!read_register(0x75, &value, &dummy)) ++t->errors;
            else if (value == 0x12) ++t->matches;
            t->id[i] = value; HAL_Delay(1);
        }
        if (t->matches == 8 && !t->errors && t->interface_value == 0x40 && selected == 36)
            selected = row;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    hspi2.Init = original;
    GPIOB->OSPEEDR = original_speed; GPIOB->PUPDR = original_pull;
    if (selected < 36) {
        unsigned mode = selected / 18, div = (selected / 6) % 3;
        unsigned speed = (selected / 3) % 2, pull = selected % 3;
        hspi2.Init.CLKPolarity = mode ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = mode ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
        hspi2.Init.BaudRatePrescaler = divider[div];
        GPIOB->OSPEEDR = (original_speed & ~speed_mask) | (speed ? speed_mask : 0);
        GPIOB->PUPDR = (original_pull & ~(3u << 28)) | (pull << 28);
        icm_c_config.selected = selected + 1;
    }
    icm_c_config.restore_ok = HAL_SPI_Init(&hspi2) == HAL_OK;
    icm_c_config.gpio_speed = GPIOB->OSPEEDR;
    icm_c_config.gpio_pull = GPIOB->PUPDR; icm_c_config.spi_cr1 = SPI2->CR1;
    icm_c_config.done = 1;
    /* Nếu có candidate mới, yêu cầu16/16 ID nữa trước khi công bố id_ok. */
    if (selected < 36 && icm_c_config.restore_ok) {
        uint8_t value = 0, dummy = 0;
        icm_c.matches_after = 0; identity_samples(1);
        icm_c.interface_after = read_register(0x70, &value, &dummy) ? value : 0;
        icm_c.id_ok = icm_c.matches_after == 16 && icm_c.interface_after == 0x40;
    } else icm_c.id_ok = 0;
    last_poll = HAL_GetTick();
}

/* Phần dưới chỉ dành cho bài bench mode5. Bypass chỉ bỏ điều kiện WHO_AM_I;
   driver C++ bình thường vẫn kiểm tra ID. Không đưa kết quả vào PID/motor. */
static int gyro_write(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    gyro_cs_guard();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, tx, 2, 20);
    gyro_cs_guard();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    icm_c_gyro.hal_status = (uint32_t)status;
    if (status != HAL_OK) { ++icm_c_gyro.io_errors; return 0; }
    return 1;
}

/* Địa chỉ đầu + N dummy, giữ CS thấp cả transaction. Đọc XYZ cùng một burst
   để tránh ghép X/Y/Z từ những thời điểm khác nhau; RX0 không phải dữ liệu. */
static int gyro_read(uint8_t reg, uint8_t *out, unsigned count) {
    uint8_t tx[7] = {0}, rx[7] = {0};
    if (!count || count > 6) return 0;
    tx[0] = (uint8_t)(reg | 0x80u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    gyro_cs_guard();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx,
                                                     (uint16_t)(count + 1), 20);
    gyro_cs_guard();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    icm_c_gyro.hal_status = (uint32_t)status;
    if (reg == 0x75 && count == 1) {
        for (unsigned i = 0; i < 2; ++i) {
            icm_c_whoami.tx[i] = tx[i]; icm_c_whoami.rx[i] = rx[i];
        }
    }
    if (status != HAL_OK) { ++icm_c_gyro.io_errors; return 0; }
    for (unsigned i = 0; i < count; ++i) out[i] = rx[i + 1];
    return 1;
}

uint32_t IcmC_ReadWhoAmI(void) {
    uint8_t value = 0;
    uint32_t ok = (uint32_t)gyro_read(0x75, &value, 1);
    icm_c_whoami.reg_addr = 0x75;
    icm_c_whoami.spi_read_cmd = 0xF5;
    icm_c_whoami.expected = 0x12;
    icm_c_whoami.read_ok = ok;
    icm_c_whoami.hal_status = icm_c_gyro.hal_status;
    icm_c_whoami.last_read_ms = HAL_GetTick();
    ++icm_c_whoami.reads;
    unsigned row = icm_c_whoami.history_next;
    icm_c_whoami.history[row] = value;
    icm_c_whoami.history_ok[row] = ok;
    icm_c_whoami.history_next = (row + 1) % 16;
    if (ok) {
        /* Chỉ so hai ID đọc THÀNH CÔNG; lần đọc lỗi không giả làm ID00. */
        if (icm_c_whoami.reads > icm_c_whoami.read_errors + 1 &&
            value != icm_c_whoami.value) ++icm_c_whoami.changed;
        icm_c_whoami.value = value;
        icm_c_gyro.actual_id = value;
        if (value == 0x12) ++icm_c_whoami.matches;
    } else ++icm_c_whoami.read_errors;
    icm_c_gyro.id_read_ok = ok;
    icm_c_gyro.id_unverified = !ok || value != 0x12;
    return ok; /* Không đổi ID sai thành12 và không chặn bài gyro bypass. */
}

uint32_t IcmC_GyroBeginUnverified(void) {
    volatile unsigned char *bytes = (volatile unsigned char *)&icm_c_gyro;
    for (unsigned i = 0; i < sizeof(icm_c_gyro); ++i) bytes[i] = 0;
    /* Re-init gyro làm hiệu chuẩn/góc cũ không còn hợp lệ. */
    bytes = (volatile unsigned char *)&icm_c_angles;
    for (unsigned i = 0; i < sizeof(icm_c_angles); ++i) bytes[i] = 0;
    bytes = (volatile unsigned char *)&icm_c_quality;
    for (unsigned i = 0; i < sizeof(icm_c_quality); ++i) bytes[i] = 0;
    bytes = (volatile unsigned char *)&icm_c_whoami;
    for (unsigned i = 0; i < sizeof(icm_c_whoami); ++i) bytes[i] = 0;
#if defined(__arm__) || defined(__thumb__)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    icm_c_quality.clock_hz = SystemCoreClock;
#else
    icm_c_quality.clock_hz = 1000000u;
#endif
    gyro_previous_clock = gyro_clock_now(); no_data_latched = 0;
    angle_previous_valid = 0;
    icm_c_gyro.bypass_requested = 1;
    icm_c_gyro.id_unverified = 1; /* Chỉ bỏ cờ này khi đọc thật được đúng ID12. */
    icm_c_gyro.init_step = 1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(250);
    /* Tắt I2C theo trình tự SPI, sau reset phải ghi lại vì reset xóa cấu hình. */
    gyro_write(0x70, 0x40);
    icm_c_gyro.init_step = 2;
    gyro_write(0x6B, 0x80);
    HAL_Delay(100);
    gyro_write(0x70, 0x40);
    IcmC_ReadWhoAmI(); /* WHO thực và TX/RX được lưu riêng cho Live Watch. */

    /* Map giả định ICM20602: đánh thức, chỉ gyro; DLPF2, ODR1kHz, ±2000dps.
       Bit OUTPUT_LIMIT được set theo datasheet. INT latch, clear khi đọc
       INT_STATUS, không cần dây INT vì bài bench dùng polling SPI. */
    static const uint8_t reg[9] = {0x70,0x6B,0x6C,0x1A,0x19,0x1B,0x69,0x37,0x38};
    static const uint8_t val[9] = {0x40,0x01,0x38,0x02,0x00,0x18,0x02,0x20,0x01};
    /* Không so các bit reserved. CONFIG bit7 có thể đọc1; chỉ so bit6:0.
       OUTPUT_LIMIT chỉ so bit1, không kết luận về các bit khác của reg69. */
    static const uint8_t mask[9] = {0x40,0x7F,0x3F,0x7F,0xFF,0xFB,0x02,0xFE,0xFF};
    icm_c_gyro.init_step = 3;
    for (unsigned i = 0; i < 9; ++i) {
        icm_c_gyro.reg[i] = reg[i]; icm_c_gyro.expected[i] = val[i];
        gyro_write(reg[i], val[i]);
    }
    HAL_Delay(100); /* Đợi gyro ổn định trước khi đọc, không hiệu chuẩn bias ở đây. */
    icm_c_gyro.init_step = 4;
    for (unsigned i = 0; i < 9; ++i) {
        uint8_t actual = 0;
        int ok = gyro_read(reg[i], &actual, 1);
        icm_c_gyro.actual[i] = actual; icm_c_gyro.read_ok[i] = (uint32_t)ok;
        if (!ok || (actual & mask[i]) != (val[i] & mask[i]))
            ++icm_c_gyro.config_mismatches;
    }
    icm_c_gyro.configured = !icm_c_gyro.config_mismatches && !icm_c_gyro.io_errors;
    icm_c_gyro.init_step = 5;
    icm_c_gyro.last_poll_ms = HAL_GetTick();
    icm_c_gyro.done = 1;
    /* Dù readback thất bại, Poll vẫn cho đọc raw chẩn đoán; configured=0
       được giữ nguyên. Không đổi trạng thái failed thành passed chỉ để đọc. */
    return icm_c_gyro.configured;
}

void IcmC_GyroPollUnverified(void) {
    uint32_t now = HAL_GetTick();
    if (!icm_c_gyro.done) return;
    uint32_t gap = (uint32_t)(now - icm_c_gyro.last_poll_ms);
    if (gap < 1) return; /* Thu nhanh hơn bài ID; không HAL_Delay trong while. */
    if (gap > icm_c_gyro.max_poll_gap_ms) icm_c_gyro.max_poll_gap_ms = gap;
    icm_c_gyro.last_poll_ms = now;
    ++icm_c_gyro.polls;
    /* Một giao dịch2byte mỗi1s; không gọi bài ID cũ có HAL_Delay dài.
       Timestamp góc vẫn lấy tại lúc burst gyro xong, gồm thời gian giao dịch này. */
    if ((uint32_t)(now - icm_c_whoami.last_read_ms) >= 1000)
        IcmC_ReadWhoAmI();
    uint8_t status = 0;
    if (!gyro_read(0x3A, &status, 1)) {
        ++icm_c_quality.read_failures; quality_event(now, 16); angles_gap(); return;
    }
    icm_c_gyro.last_status = status;
    uint32_t reason = 0;
    /* Với INT_ENABLE01, gyro-only thông thường status00/01; ghi thêm các
       bit khác để phân tích, không mặc định mọi bit khác đều do đường dây. */
    if (status & 0xFEu) { ++icm_c_quality.status_other_bits; reason |= 1; }
    uint32_t fresh = (status & 1u) != 0;
    if (fresh) { ++icm_c_gyro.data_ready; no_data_latched = 0; }
    else if ((uint32_t)(now - icm_c_gyro.last_sample_ms) >= 50 && !no_data_latched) {
        no_data_latched = 1; ++icm_c_quality.no_data_windows; reason |= 8;
    }
    /* Đọc cưỡng bức giúp thấy raw ngay cả khi INT chưa hoạt động. Không coi
       lần đọc ấy là mẫu mới; khi không fresh chỉ đọc tối đa10 lần/giây. */
    if (!fresh && (uint32_t)(now - icm_c_gyro.last_sample_ms) < 100) {
        if (reason) quality_event(now, reason);
        return;
    }
    uint8_t data[6];
    if (!gyro_read(0x43, data, 6)) {
        ++icm_c_quality.read_failures; quality_event(now, reason | 16); angles_gap(); return;
    }
    uint32_t sample_clock = gyro_clock_now();
    uint32_t clock_delta = (uint32_t)(sample_clock - gyro_previous_clock);
    /* Chỉ có ý nghĩa sau một frame trước đó; frame đầu dt được công bố0. */
    icm_c_quality.last_dt_us = (!icm_c_gyro.samples && !icm_c_gyro.forced_reads) ? 0 :
        (uint32_t)((uint64_t)clock_delta * 1000000u / icm_c_quality.clock_hz);
    gyro_previous_clock = sample_clock;
    icm_c_quality.last_sample_clock = sample_clock;
    int changed = 0, all_zero = 1, saturated = 0, near_rail = 0, large_step = 0;
    unsigned row = icm_c_gyro.history_next;
    icm_c_gyro.history[row][0] = (int32_t)now;
    icm_c_gyro.history[row][1] = (int32_t)fresh;
    for (unsigned axis = 0; axis < 3; ++axis) {
        uint32_t bits = ((uint32_t)data[axis * 2] << 8) | data[axis * 2 + 1];
        /* Chuyển two's complement rõ ràng, tránh cast unsigned->signed
           ngoài miền biểu diễn; kết quả mở rộng dấu thành int32 cho Watch. */
        int32_t raw = bits < 0x8000u ? (int32_t)bits : (int32_t)bits - 65536;
        if (raw >= 32000 || raw <= -32000) near_rail = 1;
        int32_t step = raw - icm_c_gyro.raw[axis];
        if ((icm_c_gyro.samples || icm_c_gyro.forced_reads) && (step >= 8192 || step <= -8192))
            large_step = 1; /* Flag diagnostic, không cắt/clamp raw. */
        if (raw != icm_c_gyro.raw[axis]) changed = 1;
        if (raw != 0) all_zero = 0;
        if (raw == 32767 || raw == -32768) saturated = 1;
        if (!icm_c_gyro.samples && !icm_c_gyro.forced_reads) {
            icm_c_gyro.minimum[axis] = raw; icm_c_gyro.maximum[axis] = raw;
        }
        if (raw < icm_c_gyro.minimum[axis]) icm_c_gyro.minimum[axis] = raw;
        if (raw > icm_c_gyro.maximum[axis]) icm_c_gyro.maximum[axis] = raw;
        icm_c_gyro.raw[axis] = raw;
        icm_c_gyro.dps_assumed[axis] = (float)raw / 16.4f;
        icm_c_gyro.history[row][axis + 2] = raw;
    }
    if (changed) ++icm_c_gyro.changed_samples;
    if (all_zero) ++icm_c_gyro.zero_frames;
    if (saturated) ++icm_c_gyro.saturated_frames;
    if (fresh) ++icm_c_gyro.samples; else ++icm_c_gyro.forced_reads;
    icm_c_gyro.last_sample_ms = now;
    icm_c_gyro.history_next = (row + 1) % 32;
    if (near_rail) { ++icm_c_quality.near_rail_frames; reason |= 2; }
    if (large_step) { ++icm_c_quality.large_step_frames; reason |= 4; }
    if (reason) quality_event(now, reason);
    angles_frame(now, fresh, saturated, sample_clock);
}

/* Ngưỡng bench để tránh hiệu chuẩn khi đang cầm/xoay module. Đây không phải
   bộ phát hiện đứng yên tuyệt đối: quay đều chậm vẫn có thể giống bias.
   Chỉnh sau khi đo bias/noise thực tế; không tăng ngưỡng chỉ để che lỗi. */
#define ANGLE_CAL_SETTLE_MS 500u
#define ANGLE_MAX_GAP_MS 50u
#define ANGLE_MAX_BIAS_DPS 10.0f
#define ANGLE_MAX_NOISE_DPS 1.5f
#define ANGLE_RAD_PER_DEG 0.017453292519943295f
#define ANGLE_DEG_PER_RAD 57.29577951308232f

uint32_t IcmC_AnglesResetReference(void) {
    if (!icm_c_angles.ready) return 0;
    icm_c_angles.quaternion[0] = 1.0f;
    for (unsigned i = 1; i < 4; ++i) icm_c_angles.quaternion[i] = 0;
    icm_c_angles.roll_deg = icm_c_angles.pitch_deg = icm_c_angles.yaw_deg = 0;
    icm_c_angles.last_dt_s = 0;
    icm_c_angles.updates = 0;
    icm_c_angles.tracking_continuous = 1;
    angle_previous_valid = 0; /* Mẫu kế tiếp chỉ đặt mốc thời gian, không cộng góc. */
    return 1;
}

uint32_t IcmC_AnglesStartCalibration(uint32_t sample_count) {
    volatile unsigned char *bytes = (volatile unsigned char *)&icm_c_angles;
    for (unsigned i = 0; i < sizeof(icm_c_angles); ++i) bytes[i] = 0;
    for (unsigned i = 0; i < 3; ++i) angle_mean[i] = angle_m2[i] = 0;
    angle_previous_valid = 0;
    icm_c_angles.quaternion[0] = 1.0f;
    if (!icm_c_gyro.configured) { icm_c_angles.calibration_state = 4; return 0; }
    icm_c_angles.calibration_target = sample_count < 32 ? 32 : sample_count;
    icm_c_angles.calibration_state = 1;
    angle_calibration_start = HAL_GetTick();
    return 1;
}

/* Lỗi bus/mẫu cưỡng bức/gap: không tích phân dữ liệu cũ qua đoạn bị mất.
   Góc đã tính vẫn được giữ để quan sát nhưng continuity=0 cho tới zero lại.
   HAL tick thường không chạy khi debugger halt: không thể phát hiện mọi
   khoảng pause; thao tác xoay lúc pause luôn bị bỏ lỡ, cần đặt mốc lại. */
static void angles_gap(void) {
    angle_previous_valid = 0;
    if (icm_c_angles.ready) {
        ++icm_c_angles.skipped_frames;
        icm_c_angles.tracking_continuous = 0;
    }
    if (icm_c_angles.calibration_state == 2) icm_c_angles.calibration_state = 4;
}

static void angles_frame(uint32_t now, uint32_t fresh, int saturated, uint32_t sample_clock) {
    if (!fresh || saturated || !icm_c_gyro.configured) { angles_gap(); return; }
    uint32_t state = icm_c_angles.calibration_state;
    if (state == 1) {
        if ((uint32_t)(now - angle_calibration_start) < ANGLE_CAL_SETTLE_MS) return;
        icm_c_angles.calibration_state = state = 2;
    }
    if (state == 2) {
        /* Welford: trung bình và phương sai không cần lưu400 frame. Chỉ dùng
           mẫu fresh, không đưa lần đọc cưỡng bức vào bias. */
        uint32_t n = ++icm_c_angles.calibration_samples;
        for (unsigned i = 0; i < 3; ++i) {
            float rate = icm_c_gyro.dps_assumed[i];
            float delta = rate - angle_mean[i];
            angle_mean[i] += delta / (float)n;
            angle_m2[i] += delta * (rate - angle_mean[i]);
        }
        if (n < icm_c_angles.calibration_target) return;
        int valid = 1;
        for (unsigned i = 0; i < 3; ++i) {
            float variance = angle_m2[i] / (float)(n - 1);
            float noise = sqrtf(variance > 0 ? variance : 0);
            icm_c_angles.bias_dps[i] = angle_mean[i];
            icm_c_angles.noise_std_dps[i] = noise;
            if (!isfinite(angle_mean[i]) || !isfinite(noise) ||
                fabsf(angle_mean[i]) > ANGLE_MAX_BIAS_DPS || noise > ANGLE_MAX_NOISE_DPS)
                valid = 0;
        }
        icm_c_angles.calibration_state = valid ? 3 : 4;
        icm_c_angles.ready = (uint32_t)valid;
        if (valid) IcmC_AnglesResetReference();
        return;
    }
    if (state != 3 || !icm_c_angles.ready) return;
    float rate[3];
    for (unsigned i = 0; i < 3; ++i) {
        rate[i] = icm_c_gyro.dps_assumed[i] - icm_c_angles.bias_dps[i];
        icm_c_angles.corrected_dps[i] = rate[i];
    }
    if (!angle_previous_valid) {
        angle_previous_valid = 1; angle_previous_ms = now; angle_previous_clock = sample_clock;
        for (unsigned i = 0; i < 3; ++i) angle_previous_rate[i] = rate[i];
        return;
    }
    uint32_t dt_ms = (uint32_t)(now - angle_previous_ms);
    uint32_t dt_clock = (uint32_t)(sample_clock - angle_previous_clock);
    angle_previous_ms = now; angle_previous_clock = sample_clock;
    if (!dt_clock || dt_ms > ANGLE_MAX_GAP_MS ||
        dt_clock > icm_c_quality.clock_hz / 20u) {
        ++icm_c_angles.gap_count; angles_gap(); return;
    }
    float dt = (float)dt_clock / (float)icm_c_quality.clock_hz;
    icm_c_angles.last_dt_s = dt;
    float rotation[3], length2 = 0;
    for (unsigned i = 0; i < 3; ++i) {
        /* Trung bình hai rate lân cận (trapezoid), dùng dt ĐO ĐƯỢC, không
           không mặc định1ms theo ODR; dt lấy tại lúc đọc burst xong. */
        rotation[i] = 0.5f * (angle_previous_rate[i] + rate[i]) * ANGLE_RAD_PER_DEG * dt;
        angle_previous_rate[i] = rate[i]; length2 += rotation[i] * rotation[i];
    }
    float length = sqrtf(length2);
    float dw = cosf(0.5f * length);
    float scale = length < 1e-6f ? 0.5f : sinf(0.5f * length) / length;
    float dx = rotation[0] * scale, dy = rotation[1] * scale, dz = rotation[2] * scale;
    float w = icm_c_angles.quaternion[0], x = icm_c_angles.quaternion[1];
    float y = icm_c_angles.quaternion[2], z = icm_c_angles.quaternion[3];
    /* Gyro là body rates: q_new = q_old * dq, không phải dq * q_old. */
    float q[4] = {w*dw-x*dx-y*dy-z*dz, w*dx+x*dw+y*dz-z*dy,
                  w*dy-x*dz+y*dw+z*dx, w*dz+x*dy-y*dx+z*dw};
    float norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!isfinite(norm) || norm < 1e-6f) {
        icm_c_angles.ready = 0; icm_c_angles.calibration_state = 4; angles_gap(); return;
    }
    for (unsigned i = 0; i < 4; ++i) { q[i] /= norm; icm_c_angles.quaternion[i] = q[i]; }
    w=q[0]; x=q[1]; y=q[2]; z=q[3];
    float sin_pitch = 2.0f * (w*y-z*x);
    if (sin_pitch > 1) sin_pitch = 1;
    if (sin_pitch < -1) sin_pitch = -1; /* Tránh asin NaN do sai số làm tròn. */
    icm_c_angles.roll_deg = atan2f(2*(w*x+y*z), 1-2*(x*x+y*y)) * ANGLE_DEG_PER_RAD;
    icm_c_angles.pitch_deg = asinf(sin_pitch) * ANGLE_DEG_PER_RAD;
    icm_c_angles.yaw_deg = atan2f(2*(w*z+x*y), 1-2*(y*y+z*z)) * ANGLE_DEG_PER_RAD;
    ++icm_c_angles.updates;
}
