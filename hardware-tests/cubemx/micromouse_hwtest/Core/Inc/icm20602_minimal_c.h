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
