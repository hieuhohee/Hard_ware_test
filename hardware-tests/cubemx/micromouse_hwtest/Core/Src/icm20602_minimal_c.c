#include "icm20602_minimal_c.h"
#include "spi.h"

volatile IcmC_Diagnostics icm_c;
static uint32_t last_poll;

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
