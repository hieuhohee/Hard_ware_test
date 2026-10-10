#pragma once
#include <stdint.h>

// API dùng được từ main.c lẫn main.cpp. Mã triển khai vẫn viết bằng C++.
// Các hàm chỉ gọi trong luồng main, sau toàn bộ MX_*_Init(), không gọi từ ISR.
#ifdef __cplusplus
extern "C" {
#endif

// Gọi MỘT LẦN sau khởi tạo CubeMX. Chỉ khởi tạo module được bật trong
// test_config.cpp; motor dừng, chưa chọn bài test, không mở menu lệnh.
void HardwareTests_Begin(void);

// Gọi LIÊN TỤC trong while(1), không chèn HAL_Delay hoặc công việc chặn lâu.
// Tiếp tục đọc cảm biến, xuất log nếu có UART và dừng xung motor đúng hạn.
// Không đọc lệnh UART hay bench::command. UART chỉ dùng để xuất kết quả.
void HardwareTests_Update(void);

// Chọn một bài test, gọi MỘT LẦN sau Begin. Trả 1 khi có module sẵn sàng,
// trả 0 khi chưa cấu hình/khởi tạo lỗi. Chuyển bài sẽ dừng bài trước.
// Chọn trái/phải encoder hoặc mắt IR bằng cấu hình module; API không đổi dây.
uint8_t HardwareTests_TestIR(void);
uint8_t HardwareTests_TestEncoders(void);
uint8_t HardwareTests_TestGyro(void);
uint8_t HardwareTests_TestAllSensors(void);

// Giữ gyro yên: lấy 1000 mẫu mới, timeout 5 s. Sau hiệu chuẩn vẫn đọc gyro.
// Không gọi lại hàm này trong while: sẽ khởi động lại phép hiệu chuẩn.
uint8_t HardwareTests_CalibrateGyro(void);

// direction chỉ nhận +1 hoặc -1 (chiều điện, chưa chắc là chiều tiến cơ khí).
// Gọi MỘT LẦN: phát một xung 12%/300 ms theo test_config.hpp rồi tự dừng
// khi Update chạy. Trả 0 nếu chưa ready, sai direction hoặc đang có xung.
// Không gọi trong while: sau khi xung kết thúc sẽ khởi động xung mới liên tục.
// Khi reset/nạp lại, main chạy lại nên bài motor đã chọn cũng sẽ chạy lại.
uint8_t HardwareTests_TestMotorLeft(int direction);
uint8_t HardwareTests_TestMotorRight(int direction);

// Dừng PWM, tắt LED phát và hủy bài cảm biến/hiệu chuẩn. Không chờ timeout.
void HardwareTests_Stop(void);

#ifdef __cplusplus
}
#endif
