# Test ID ICM bằng C thuần — 10/10/2026

Cập nhật mới: [CONFIG_AUDIT.md](CONFIG_AUDIT.md). Main hiện mode4 để rà
36 cấu hình và chỉ khởi tạo GPIO/SPI; dữ liệu mode3 bên dưới là phiên trước.

Theo sự cho phép của người dùng, đã thêm bài test C độc lập để đối chiếu
với bài C++. Không chuyển cả project PID/test sang C và không sửa chân.
Lỗi hiển thị cstring đã hết theo xác nhận của người dùng; vấn đề còn lại
là ID cảm biến không đúng, không phải ký tự trong Watch.

## Các file và cách gọi

- `cubemx/micromouse_hwtest/Core/Src/icm20602_minimal_c.c`: triển khai C,
  gọi trực tiếp HAL_SPI và GPIO; không gọi hardware_tests.cpp/test_config.cpp.
- `Core/Inc/icm20602_minimal_c.h`: API C và cấu trúc chẩn đoán toàn số32-bit.
- main.c hiện `GYRO_IDENTITY_ONLY=3`, gọi `IcmC_Begin()` một lần sau MX_*_Init,
  rồi `IcmC_Poll()` trong while. Không UART/menu hoặc tự chạy motor.
- CMakeLists.txt thêm source.c; cấu hình C/C++ chung giữ nguyên để các bài cũ
  vẫn build được. ELF thực tế ở mode3 không có symbol bench::/HardwareTests_*:
  linker đã loại các driver C++ không được dùng. Linker driver CXX không
  làm các hàm test viết bằng C trở thành C++.

IcmC_Begin chờ250ms, đọc16 ID trước ghi interface, ghi I2C_IF0x70=0x40,
đọc lại interface và đọc16 ID tiếp. Không reset/config gyro của chip ID sai.
IcmC_Poll đọc lại WHO_AM_I mỗi100ms, ghi current_id/current_valid và tăng reads.
Buffer HAL là uint8_t, dữ liệu Watch là uint32_t. CS PB12; SPI2 PB13/14/15.

Xem **`icm_c`** trong Watch. Trong mode3, các biến bench::* không còn được
link vào firmware này; Watch cũ có thể báo không tìm thấy symbol. Khi đổi
mode cần kết thúc phiên debug và mở lại để ELF/kiểu dữ liệu khớp firmware.

| Trường | Cách đọc |
|---|---|
| done | 1: bài khởi tạo đo đã xong, không mặc định ID đúng |
| id_ok | Chỉ1 khi16 ID sau disable đều12 và readback interface40 |
| matches_before/after | Số ID12 trên16 giao dịch tương ứng |
| id_before/after | Mảng ID đo được, không đổi giá trị sai thành12 |
| current_id/current_valid | ID mới nhất và giao dịch đọc có HAL_OK hay không |
| reads | Tổng số lần đọc WHO, tiếp tục tăng khi main chạy tự do |
| io_errors/hal_status | Lỗi hoàn tất giao dịch HAL; HAL_OK không đảm bảo ID đúng |
| interface_before/after | Dữ liệu đọc reg0x70 trước/sau lệnh ghi40 |
| last_tx/last_rx | Hai byte TX/RX của giao dịch gần nhất; ID là RX[1] |

## Kết quả đã đo trực tiếp

- ARM GCC compile các source main.c và icm20602_minimal_c.c: PASS.
- FLASH17376B, RAM2440B. Các hàm C ở ELF: IcmC_Begin/IcmC_Poll, icm_c.
- Nạp và verify bằng ST-Link SN37FF71064E5734368AEC1743: PASS,
  điện áp tham chiếu3.26V, SWD thực tế125kHz, F405 ID0x413.
- Snapshot ở icm_c0x20000238: done1, id_ok0, hal_status0, io_errors0,
  reads156 tại thời điểm đo, matches_before0, matches_after0,
  current_valid1, current_id46 (0x2E).
- Cả hai mảng ID không có12. Có2E,FF,00 trước ghi vàF8/FF/2E sau ghi.
  interface_before00, interface_afterF8 trong lần đo này: readback **chưa đạt40**.
  Không báo bus luôn ổn định chỉ vì HAL_OK; không giấu các byte sai này.
- SPI CR1 lúc Begin32C (cấu hình đúng, SPE chưa bật tới giao dịch đầu);
  GPIO MODER A900AA8F, AFRH55500000. Gói mới nhất TX F5/00, RX00/2E.

## Kết luận

Bài C độc lập cũng không nhận ID12. Chưa có bằng chứng lỗi nhận dạng do
lớp C++ hoặc namespace. Trước đây SPI roundtrip mẫu bit đã đạt và I2C cũng
trả2E; hiện C có thêm các mẫu không ổn định/readback sai. Các phép đo này
chưa chứng minh toàn bộ phần cứng hoàn hảo hoặc xác định được loại chip.

Firmware không thể ghi WHO_AM_I thành12 vì đây là register chỉ đọc trong
datasheet ICM20602. Cần xác minh IC/breakout thực tế hoặc đối chiếu module
ICM20602 đã biết đúng để chốt nguyên nhân. Không chấp nhận2E làm12, không
khởi động gyro của chip chưa nhận dạng hợp lệ.

Mốc trước thêm bài C: `81f9c22`. Đặt selector bằng0 để trở lại bài gyro C++,
1 để chỉ đọc ID C++,2 để đo SPI/I2C C++,3 để dùng bài C hiện tại. Giữ code
C này như bài kiểm tra độc lập; firmware PID trong micromouse-control không đổi.

Nguồn ID/giao thức: [TDK ICM20602 DS-000176 rev1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf).
