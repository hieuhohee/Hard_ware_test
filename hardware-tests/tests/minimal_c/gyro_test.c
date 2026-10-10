/* Test source C thật bằng bus HAL giả. Xác minh bypass chỉ bỏ kiểm tra ID,
   readback vẫn được công bố trung thực; signed raw và mẫu cưỡng bức tách biệt. */
#include "icm20602_minimal_c.h"
#include "spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

GPIO_TypeDef fake_gpio;
SPI_TypeDef fake_spi;
RCC_TypeDef fake_rcc;
SPI_HandleTypeDef hspi2 = {&fake_spi, {0, 0, 5}};
static uint8_t registers[128];
static uint32_t tick, wrong_readback, timeout_next, id_writes, accel_reads;
static GPIO_PinState cs = GPIO_PIN_SET;
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState state) {
    assert(p == GPIOB && pin == GPIO_PIN_12); cs = state;
}
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 84000000; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 168000000; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *s) { (void)s; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *s, uint8_t *tx, uint16_t n, uint32_t t) {
    (void)s; (void)t; assert(cs == GPIO_PIN_RESET && n == 2 && tx[0] < 128);
    if (tx[0] == 0x75) ++id_writes;
    if (tx[0] == 0x6B && tx[1] == 0x80) {
        uint8_t id = registers[0x75]; memset(registers, 0, sizeof(registers));
        registers[0x75] = id; registers[0x6B] = 0x41;
    } else registers[tx[0]] = tx[1];
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *s, uint8_t *tx, uint8_t *rx,
                                        uint16_t n, uint32_t t) {
    (void)s; (void)t; assert(cs == GPIO_PIN_RESET && (tx[0] & 0x80) && n >= 2);
    if (timeout_next) { timeout_next = 0; return HAL_TIMEOUT; }
    uint32_t reg = tx[0] & 0x7Fu;
    rx[0] = 0xFF; /* Dummy khác0 để phát hiện lỗi nhầm offset payload. */
    for (unsigned i = 1; i < n; ++i) rx[i] = registers[reg + i - 1];
    if (reg == 0x1A) rx[1] |= 0x80; /* Reserved bit đọc1 không được báo mismatch. */
    if (wrong_readback && reg == 0x1B) rx[1] = 0;
    if (reg >= 0x3B && reg <= 0x40) ++accel_reads;
    if (reg == 0x3A) registers[0x3A] = 0; /* Clear Data Ready khi đọc status. */
    return HAL_OK;
}
static void reset_fixture(uint8_t id) {
    memset(registers, 0, sizeof(registers)); registers[0x75] = id;
    tick = wrong_readback = timeout_next = id_writes = accel_reads = 0;
    cs = GPIO_PIN_SET;
}
static void supply_sample(int16_t x, int16_t y, int16_t z, int fresh) {
    int16_t values[3] = {x,y,z};
    for (unsigned i = 0; i < 3; ++i) {
        uint16_t word = (uint16_t)values[i];
        registers[0x43 + i * 2] = (uint8_t)(word >> 8);
        registers[0x44 + i * 2] = (uint8_t)word;
    }
    registers[0x3A] = (uint8_t)fresh; tick += 10;
    IcmC_GyroPollUnverified();
}
static void prepare_angles(void) {
    reset_fixture(0x2E);
    assert(IcmC_GyroBeginUnverified() == 1);
    assert(IcmC_AnglesStartCalibration(64) == 1);
    supply_sample(16, -32, 8, 1);
    assert(icm_c_angles.calibration_samples == 0 && !icm_c_angles.ready);
    tick += 500;
    for (unsigned i = 0; i < 64; ++i) supply_sample(16, -32, 8, 1);
    assert(icm_c_angles.ready == 1 && icm_c_angles.calibration_state == 3);
    assert(fabsf(icm_c_angles.bias_dps[0] - 16.0f/16.4f) < 1e-5f);
    assert(icm_c_angles.noise_std_dps[0] == 0);
}
static void at_tick(uint32_t ms, int16_t x, int16_t y, int16_t z) {
    tick = ms - 10; supply_sample(x, y, z, 1);
}
/* Gyro = bias + tốc độ biết trước, timestamp10ms chính xác (không theo số
   lần gọi). ±90°/s trong1s phải cho±90°, không phụ thuộc bench delay HAL. */
static void test_angles(void) {
    prepare_angles();
    uint32_t start = tick + 10;
    at_tick(start, 16, -32, 1484); /* Z 90dps; mẫu đầu đặt mốc. */
    for (unsigned i = 1; i <= 100; ++i) at_tick(start + 10*i, 16, -32, 1484);
    assert(fabsf(icm_c_angles.yaw_deg - 90) < 0.02f);
    assert(fabsf(icm_c_angles.roll_deg) < 0.02f && fabsf(icm_c_angles.pitch_deg) < 0.02f);
    assert(icm_c_angles.updates == 100 && icm_c_angles.tracking_continuous == 1);
    float norm2 = 0;
    for (unsigned i = 0; i < 4; ++i) norm2 += icm_c_angles.quaternion[i]*icm_c_angles.quaternion[i];
    assert(fabsf(norm2 - 1) < 1e-5f);
    assert(IcmC_AnglesResetReference() == 1 && icm_c_angles.yaw_deg == 0);
    assert(icm_c_angles.calibration_samples == 64 && icm_c_angles.ready);

    start = tick + 10;
    at_tick(start, 16, 1444, 8);
    for (unsigned i = 1; i <= 50; ++i) at_tick(start + 10*i, 16, 1444, 8);
    assert(fabsf(icm_c_angles.pitch_deg - 45) < 0.02f);
    assert(fabsf(icm_c_angles.roll_deg) < 0.02f && fabsf(icm_c_angles.yaw_deg) < 0.02f);
    assert(IcmC_AnglesResetReference() == 1);

    /* Hai phép quay body nối tiếp không tương đương cộng Euler X rồi Y:
       Rx90 * Ry45 có Euler ZYX gần roll90/pitch0/yaw45. */
    start = tick + 10;
    at_tick(start, 1492, -32, 8);
    for (unsigned i = 1; i <= 100; ++i) at_tick(start + 10*i, 1492, -32, 8);
    for (unsigned i = 1; i <= 50; ++i) at_tick(start + 1000 + 10*i, 16, 1444, 8);
    assert(fabsf(icm_c_angles.roll_deg - 90) < 1);
    assert(fabsf(icm_c_angles.pitch_deg) < 1 && fabsf(icm_c_angles.yaw_deg - 45) < 1);

    uint32_t updates = icm_c_angles.updates;
    tick += 100; supply_sample(16, 1444, 8, 1);
    assert(icm_c_angles.updates == updates && icm_c_angles.gap_count == 1);
    assert(!icm_c_angles.tracking_continuous);
    assert(IcmC_AnglesResetReference() == 1);
    supply_sample(16, -32, 8, 1);
    updates = icm_c_angles.updates;
    tick += 100; supply_sample(1640, 1640, 1640, 0);
    assert(icm_c_angles.updates == updates && !icm_c_angles.tracking_continuous);
    assert(IcmC_AnglesResetReference() == 1);
    supply_sample(32767, 0, 0, 1);
    assert(icm_c_angles.updates == 0 && !icm_c_angles.tracking_continuous);

    /* HAL tick wrap-around không được tạo dt âm hoặc góc nhảy. */
    assert(IcmC_AnglesResetReference() == 1);
    start = UINT32_MAX - 50;
    at_tick(start, 16, -32, -1468);
    for (unsigned i = 1; i <= 10; ++i) at_tick(start + 10*i, 16, -32, -1468);
    assert(fabsf(icm_c_angles.yaw_deg + 9) < 0.02f && icm_c_angles.gap_count == 1);

    /* Không tự chấp nhận cửa sổ bias có module chuyển động. */
    assert(IcmC_AnglesStartCalibration(32) == 1);
    tick += 500;
    for (unsigned i = 0; i < 32; ++i) supply_sample(i%2 ? 1640 : -1640, 0, 0, 1);
    assert(icm_c_angles.calibration_state == 4 && !icm_c_angles.ready);
    assert(IcmC_AnglesResetReference() == 0);
}
static void test_quality_log(void) {
    reset_fixture(0x2E);
    assert(IcmC_GyroBeginUnverified() == 1);
    assert(icm_c_quality.clock_hz == 1000000);
    supply_sample(0, 0, 0, 1);
    supply_sample(32639, -1, 9, 1);
    assert(icm_c_quality.near_rail_frames == 1 && icm_c_quality.large_step_frames == 1);
    assert(icm_c_quality.events_written == 1 && icm_c_quality.events[0][1] == 6);
    assert(icm_c_quality.events[0][3] == 32639 && icm_c_gyro.raw[0] == 32639);
    assert(icm_c_quality.last_dt_us == 10000); /* Không dùng chu kỳ cố định1ms. */
    registers[0x3A] = 0x81; tick += 10; IcmC_GyroPollUnverified();
    assert(icm_c_quality.status_other_bits == 1 && icm_c_quality.events_written == 2);
    registers[0x3A] = 0; tick += 60; IcmC_GyroPollUnverified();
    assert(icm_c_quality.no_data_windows == 1 && icm_c_gyro.forced_reads == 0);
    uint32_t events = icm_c_quality.events_written;
    tick += 5; IcmC_GyroPollUnverified();
    assert(icm_c_quality.events_written == events); /* Không spam cùng outage. */
    tick += 50; IcmC_GyroPollUnverified();
    assert(icm_c_gyro.forced_reads == 1 && icm_c_quality.no_data_windows == 1);
    timeout_next = 1; tick += 1; IcmC_GyroPollUnverified();
    assert(icm_c_quality.read_failures == 1 && cs == GPIO_PIN_SET);
    for (unsigned i = 0; i < 70; ++i) supply_sample(32639, -1, 9, 1);
    assert(icm_c_quality.history_next == icm_c_quality.events_written % 64);
    unsigned last = (icm_c_quality.history_next + 63) % 64;
    assert(icm_c_quality.events[last][3] == 32639); /* Journal wrap vẫn đúng. */
}
int main(void) {
    reset_fixture(0x2E);
    assert(IcmC_GyroBeginUnverified() == 1);
    assert(icm_c_gyro.actual_id == 0x2E && icm_c_gyro.id_unverified == 1);
    assert(icm_c_gyro.bypass_requested == 1 && icm_c_gyro.config_mismatches == 0);
    assert(id_writes == 0 && registers[0x6C] == 0x38); /* Accel standby. */
    supply_sample(-32768, 32767, -1, 1);
    assert(icm_c_gyro.samples == 1 && icm_c_gyro.forced_reads == 0);
    assert(icm_c_gyro.raw[0] == -32768 && icm_c_gyro.raw[1] == 32767 && icm_c_gyro.raw[2] == -1);
    assert(icm_c_gyro.saturated_frames == 1 && icm_c_gyro.history[0][1] == 1);
    supply_sample(164, -164, 0, 1);
    assert(icm_c_gyro.samples == 2 && icm_c_gyro.dps_assumed[0] == 10.0f);
    assert(icm_c_gyro.minimum[0] == -32768 && icm_c_gyro.maximum[0] == 164);
    supply_sample(0, 0, 0, 0); /* Chưa100ms: không được coi là mẫu mới. */
    assert(icm_c_gyro.samples == 2 && icm_c_gyro.forced_reads == 0);
    tick += 100; IcmC_GyroPollUnverified();
    assert(icm_c_gyro.samples == 2 && icm_c_gyro.forced_reads == 1);
    assert(icm_c_gyro.history[2][1] == 0 && icm_c_gyro.zero_frames == 1);
    timeout_next = 1; supply_sample(1, 2, 3, 1);
    assert(icm_c_gyro.io_errors == 1 && icm_c_gyro.hal_status == HAL_TIMEOUT);
    assert(icm_c_gyro.samples == 2 && cs == GPIO_PIN_SET && accel_reads == 0);

    reset_fixture(0x2E); wrong_readback = 1;
    assert(IcmC_GyroBeginUnverified() == 0);
    assert(icm_c_gyro.configured == 0 && icm_c_gyro.config_mismatches == 1);
    supply_sample(1, 2, 3, 1);
    assert(icm_c_gyro.samples == 1 && icm_c_gyro.configured == 0); /* Raw chỉ chẩn đoán. */
    assert(IcmC_AnglesStartCalibration(32) == 0 && !icm_c_angles.ready);

    reset_fixture(0x12);
    assert(IcmC_GyroBeginUnverified() == 1 && icm_c_gyro.id_unverified == 0);
    assert(icm_c_gyro.actual_id == 0x12 && id_writes == 0);
    test_angles();
    test_quality_log();
    puts("PASS: gyro diagnostics, bias, quaternion/Euler, timestamp, gaps, quality log");
    return 0;
}
