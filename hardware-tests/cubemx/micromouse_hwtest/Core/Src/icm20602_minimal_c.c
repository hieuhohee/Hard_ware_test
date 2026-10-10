#include "icm20602_minimal_c.h"
#include "spi.h"

volatile IcmC_Diagnostics icm_c;
volatile IcmC_ConfigDiagnostics icm_c_config;
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
