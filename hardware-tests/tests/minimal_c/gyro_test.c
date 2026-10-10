/* Test source C thật bằng bus HAL giả. Xác minh bypass chỉ bỏ kiểm tra ID,
   readback vẫn được công bố trung thực; signed raw và mẫu cưỡng bức tách biệt. */
#include "icm20602_minimal_c.h"
#include "spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

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

    reset_fixture(0x12);
    assert(IcmC_GyroBeginUnverified() == 1 && icm_c_gyro.id_unverified == 0);
    assert(icm_c_gyro.actual_id == 0x12 && id_writes == 0);
    puts("PASS: bypass identity, readback, signed burst, fresh/forced, SPI failure");
    return 0;
}
