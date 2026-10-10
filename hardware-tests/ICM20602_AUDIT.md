# Rà soát ICM20602 và firmware đang chạy — 10/10/2026

Mốc sao lưu trước khi sửa: **6650a89**. Firmware được kiểm tra ở `cubemx/micromouse_hwtest`, không phải executable host trong thư mục `tests`.

## Kết luận từ dữ liệu hiện có

Ảnh debugger cho thấy CPU đã tới `HardwareTests_Update()`, `imu_whoami=46` (0x2E), `imu_ready=0`, `test_started=0`, `io_errors=0`. Vì vậy không còn là trường hợp đang dừng trước khởi tạo. Theo driver cũ, giao dịch HAL đọc WHO_AM_I hoàn tất nhưng ID khác 0x12; code dừng trước cấu hình gyro. HAL_OK không xác nhận cảm biến đã trả lời đúng.

Không có cơ sở nhận diện một loại chip khác chỉ từ 0x2E. Chưa có ảnh/module schematic hoặc phép đo trên SCK/MISO/CS, nên chưa thể kết luận lỗi phần cứng, sai loại module hay lỗi tín hiệu. Điểm thiếu trong trình tự phần mềm được sửa dưới đây cũng chưa được chứng minh là nguyên nhân duy nhất của 0x2E.

## Các phần đã kiểm tra

| Phần | Kết quả |
|---|---|
| Nguồn theo người dùng xác nhận | F405 qua USB-C; ICM lấy 3V3 từ bo. Chưa đo điện áp ngay tại module hoặc sụt áp/nhiễu |
| Main | Gọi Begin sau toàn bộ MX_*_Init; TestGyro một lần; Update trong while |
| Test configuration | GYRO=1; UART/IR/encoder/motor=0; `imu_spi=&hspi2`, CS GPIOB pin 12 |
| Clock | HSI → PLL M8/N168/P2/Q7; HCLK168 MHz, PCLK1 42 MHz |
| SPI2 | Full duplex master, 8 bit, MSB, CPOL low/CPHA first edge, NSS software, divider64 → 656.25 kHz danh định |
| GPIO | PB13 SCK / PB14 MISO / PB15 MOSI, AF5; PB12 output HIGH trước giao dịch |
| SPI transaction | WHO_AM_I: địa chỉ 0x75, gửi 0xF5 rồi một byte dummy; bỏ byte RX đầu, lấy RX byte thứ hai; CS LOW suốt cả giao dịch |
| Gyro | Chỉ đọc 6 byte ở 0x43; accel standby 0x38; không đọc accel |
| Build | GCC/G++ Arm, Cortex-M4, FPU fpv4-sp-d16, hard ABI; Debug -O0/-g3; link Flash; hai cpp đều thuộc target |
| Launch | ST-Link debugger, SWD1 MHz, software system reset, đúng ELF build/Debug |

Pin và chế độ SPI khớp giữa IOC, code sinh và cấu hình C++. Không thay bảng chân hoặc tự đổi mode/clock để che dữ liệu sai.

## Đã sửa

1. **Trình tự SPI startup:** datasheet DS-000176 §6.1 yêu cầu I2C_IF_DIS ngay sau thời gian startup. Driver cũ chỉ ghi 0x70=0x40 sau khi đọc ID và reset. Driver mới ghi nó sau chờ 100 ms, trước đọc ID; sau DEVICE_RESET chờ 100 ms rồi ghi lại. Đây là driver riêng cho phần cứng ICM20602, không phải trình dò mọi loại IMU.
2. **ID một lần → tám giao dịch:** ghi lịch sử tám byte WHO_AM_I, số mẫu HAL_OK và số mẫu đúng 0x12. Chỉ init gyro khi cả tám ID đều đúng; không lấy đa số hay thay expected ID thành 0x2E.
3. **Chẩn đoán:** ghi bước init, HAL status, register cuối, byte expected/actual. Sai ID vẫn có io_errors=0 là có chủ đích: đó không phải lỗi timeout của HAL.
4. **Main:** bỏ hai marker USER CODE Includes bị lồng trùng; giữ nguyên cách gọi test. Marker trùng không giải thích được 0x2E, nhưng làm việc sinh lại code khó tin cậy.
5. **CMake:** khai báo C/C++/ASM rõ ràng ở root; chuyển include ứng dụng và chuẩn C++11 khỏi CMake con do CubeMX sinh. Hai source ứng dụng nằm ở root để giữ khi generate lại.
6. **VS Code:** bỏ configureArgs ghi CMAKE_COMMAND gây cảnh báo; bật preset cho project đang mở.

Hai bản driver/header trong `hardware-tests` và `Core` đã được đồng bộ. Khi cập nhật source mẫu về sau, cần copy đúng cặp cpp/hpp cùng phiên bản; Watch dùng layout mới chỉ khi đã nạp ELF mới.

## Nạp bản mới và đọc lỗi

1. Dừng phiên debug cũ (Shift+F5), lưu file, build/nạp ELF mới qua cấu hình ST-Link.
2. Nên tắt rồi bật nguồn USB-C của bo trước phiên thử để đưa cả MCU và cảm biến về cùng trạng thái khởi động. Không đồng thời cấp thêm nguồn 3V3 từ ST-Link chưa xác minh.
3. Cho Continue qua main và dừng ở `HardwareTests_Update()` (dòng đã dịch chuyển sau chỉnh marker). Đặt lại breakpoint theo tên lệnh, không giữ số dòng cũ một cách máy móc.
4. Thêm các biểu thức Watch dưới đây. Mở rộng mảng history để xem đủ tám byte.

```cpp
test_started
bench::live.imu_init_step
bench::live.imu_whoami
bench::live.imu_id_reads
bench::live.imu_id_matches
bench::live.imu_id_history
bench::live.imu_hal_status
bench::live.imu_last_reg
bench::live.imu_expected_value
bench::live.imu_actual_value
bench::live.io_errors
```

| imu_init_step | Ý nghĩa |
|---:|---|
| 0 | Module chưa được cấu hình |
| 1 | Đang tắt I2C để chọn SPI |
| 2 | Đang kiểm tra ID |
| 3 | Tám mẫu ID không đồng loạt đúng 0x12 |
| 4 | Đang reset cảm biến |
| 5 | Đang ghi cấu hình gyro |
| 6 | Đang đọc lại cấu hình |
| 7 | Init hoàn tất |
| 8 | Giao dịch SPI có lỗi HAL |
| 9 | HAL đọc thành công nhưng giá trị cấu hình đọc lại không đúng |

Ở breakpoint Update sau init thường thấy 0/3/7/8/9, không phải các bước trung gian đang chạy. HAL status: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT. `imu_last_reg` là địa chỉ register (thường Watch hiển thị thập phân). `imu_actual_value` chỉ cập nhật khi đọc HAL_OK và là byte dữ liệu đầu tiên; không dùng giá trị cũ này để suy ra dữ liệu của giao dịch bị timeout.

| Kết quả | Bước tiếp |
|---|---|
| step7, reads8, matches8, history toàn18 | ID đúng; xem imu_samples/gyro_dps khi chạy rồi thử xoay |
| step3, history toàn46 | ID sai ổn định; kiểm tra mã chip/module, mapping SPI, CS và nguồn; chưa suy ra là loại IMU khác |
| step3, history thay đổi | Có dữ liệu không ổn định; ưu tiên đo SPI/kiểm tra dây và nguồn, không kết luận chỉ từ HAL status |
| step8 | Xem HAL status và register nơi lỗi; kiểm tra handle, timing, bus và nguồn |
| step9 | Gửi register, expected/actual để xác định lệnh ghi hoặc readback nào sai |

## Kiểm tra dây thực tế nếu ID còn sai

Tắt nguồn trước khi đổi dây. ICM CS/NCS → PB12; SPC/SCK/SCL → PB13; SDO/AD0 → PB14; SDI/SDA → PB15; GND chung. Với ICM20602 dùng SPI, AD0/SDO là đường dữ liệu ra, không nối cố định GND như khi chọn địa chỉ I2C. Breakout có regulator hoặc level shifter cần đối chiếu sơ đồ riêng; tên VIN/VCC chưa xác nhận được cách cấp nguồn chỉ bằng chữ trên chân.

Đo VDD và VDDIO/nguồn module tại cảm biến. Nếu có logic analyzer: giao dịch WHO_AM_I phải có CS LOW, MOSI `F5 00`, và byte RX thứ hai `12`; mode0 lấy mẫu ở cạnh lên, SCK danh định 656.25 kHz. MISO trả 2E trên dây khác với MISO trả 12 nhưng MCU giải mã thành 2E: hai kết quả dẫn đến hướng xử lý khác nhau. Dùng dây ngắn và bảo đảm đầu cắm tiếp xúc tốt.

Nếu chỉ có debugger, gửi lịch sử tám ID cùng ảnh nhãn chân/mã chip; phần mềm không thể xác minh dây vật lý hay loại breakout từ một giá trị ID sai.

## Kiểm chứng đã thực hiện

- Configure và build Debug bằng Arm GCC/G++14.3.1: PASS, ELF tạo thành công, không cảnh báo compiler trong lần build này. FLASH29868 B, RAM2912 B theo linker report.
- Host CTest bench_logic: PASS (1/1). Có test buộc tắt I2C trước đọc ID; reset xóa interface; ID đúng, sai0x2E, ID chập chờn, lỗi HAL và lỗi đọc lại cấu hình. Các test motor/encoder/IR/gyro cũ vẫn chạy.
- Chưa nạp hoặc đọc ICM bằng bản mới trong lượt rà soát này: firmware đã build cần được nạp lại. Host mock xác minh logic, không mô phỏng điện áp, ringing hoặc timing dây thật.

Nguồn đối chiếu: [TDK ICM20602 DS-000176 rev1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf), §6.1 và WHO_AM_I §9.43.
