# Kết quả kiểm tra

Cập nhật 10/10/2026:

- Thêm API gọi hàm trực tiếp Begin/Update/Test.../Stop; giữ driver và bảng chân hiện tại.
- Host tests gồm gọi API từ file C link với C++, motor không bị gia hạn, tự dừng và không lặp lại nếu chỉ Update; không nhận UART/command trong chế độ trực tiếp; đọc IR/encoder/gyro, hiệu chuẩn và Stop qua API mới.
- Build MSVC và CTest sau thay đổi API: PASS, 1/1 test suite.
- CubeMX 6.18.1 đọc IOC mới, `generate code` với F4 V1.28.3 báo thành công. Đã đối chiếu GPIO, clock, timer, ADC, SPI và UART sinh ra với mẫu module.
- Thử tạo toàn bộ project CubeIDE bị treo ở dịch vụ tạo metadata; chưa xác nhận import/build ARM, chưa nạp phần cứng. Mã PID không thay đổi.

Cập nhật 09/10/2026 cho test module rời:

- Bổ sung DATA_RDY enable/latch trong driver ICM20602 và kiểm tra đọc lại thanh ghi.
- Build host và CTest `bench_logic` sau thay đổi: PASS (1/1).
- Mẫu `examples/test_config.modules.cpp` đã compile với HAL giả ở cả hai cấu hình tất cả cờ tắt và tất cả cờ bật, C++11, `/W4 /WX`.
- Chưa build ARM/nạp module thật. Source dự án PID `micromouse-control/` không bị sửa trong lần cập nhật này.

- Build C++ bằng MSVC trên máy tính với HAL giả: thành công.
- CTest `bench_logic`: PASS.
- Đã kiểm tra: cấu hình rỗng không chạy motor; xung motor và lệnh dừng;
  không gia hạn xung bằng lệnh mới; deadline qua tràn tick; encoder qua tràn
  16/32 bit và đảo dấu; IR sáng-trừ-tối; lỗi ADC tắt LED; giải mã gyro XYZ có dấu;
  accel ở standby, không cấu hình/đọc dữ liệu accel, gói gyro đúng 6 byte;
  bias từ mẫu mới; timeout hiệu chuẩn; lỗi SPI nhả CS; sai WHO_AM_I;
  trùng chân IR và trùng kênh PWM.
- Chưa build bằng ARM GCC với project CubeMX thật vì chưa chốt ngoại vi/chân.
- Chưa nạp STM32 hoặc đo board thật. Không suy ra độ chính xác ADC, encoder,
  timing IR, dòng motor hoặc chất lượng PCB từ host test.

Sau khi có `.ioc`, cần kiểm tra lại clock tree, pin alternate function, DMA/IRQ
của các module khác, truth table AT8870 và chế độ ABZ của MT6701.
