#ifndef ICM20602_MINIMAL_C_H
#define ICM20602_MINIMAL_C_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Bài kiểm tra ID độc lập bằng C. Mọi biến Watch là số 32-bit, không phải
   char/string. id_ok chỉ xác nhận ID; không có nghĩa gyro đã cấu hình xong. */
typedef struct {
    uint32_t done, id_ok, hal_status, io_errors;
    uint32_t reads, matches_before, matches_after, current_valid, current_id;
    uint32_t interface_before, interface_after, spi_cr1, gpio_moder, gpio_afrh;
    uint32_t id_before[16], id_after[16], dummy_before[16], dummy_after[16];
    uint32_t last_tx[2], last_rx[2];
} IcmC_Diagnostics;

extern volatile IcmC_Diagnostics icm_c;
typedef struct {
    uint32_t mode, divider, speed, pull, matches, errors, interface_value;
    uint32_t id[8];
} IcmC_ConfigTrial;
typedef struct {
    uint32_t done, selected, restore_ok, pclk1_hz, pclk2_hz, hclk_hz;
    uint32_t rcc_cr, rcc_pllcfgr, rcc_cfgr, gpio_speed, gpio_pull, spi_cr1;
    IcmC_ConfigTrial trial[36];
} IcmC_ConfigDiagnostics;
extern volatile IcmC_ConfigDiagnostics icm_c_config;
/* Dò một lần 36 profile: mode0/3, divider64/128/256, GPIO slow/fast,
   MISO không kéo/kéo lên/kéo xuống. Không đổi AF/chân và không sửa .ioc.
   Chỉ giữ profile có8/8 ID12, HAL_OK và readback I2C_IF40; nếu không có
   thì khôi phục SPI/GPIO cũ. Đây là phép đo bench, không gọi trong while. */
void IcmC_ConfigAudit(void);
/* Gọi một lần sau MX_GPIO_Init/MX_SPI2_Init. Chỉ dùng SPI2, CS PB12 đã chốt.
   Đọc ID trước/sau tắt I2C, không reset hay cấu hình gyro của chip sai ID. */
void IcmC_Begin(void);
/* Gọi trong while; mỗi 100 ms đọc lại ID để xem giá trị có ổn định không.
   Không UART, không menu, không motor, không đọc accel hoặc gyro. */
void IcmC_Poll(void);

#ifdef __cplusplus
}
#endif
#endif
