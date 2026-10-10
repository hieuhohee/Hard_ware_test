# Báo cáo tự kiểm thử ICM trên F405 — 10/10/2026

**Cập nhật lỗi Watch:** xem [DEBUG_WATCH_FIX.md](DEBUG_WATCH_FIX.md).
Đã đổi các bản sao byte chẩn đoán sang uint32_t và kiểm chứng GDB/MI
hiển thị số trên bo. Các địa chỉ/layout RAM trong snapshot bên dưới thuộc
phiên trước sửa Watch, không dùng cho ELF mới.

**Build, nạp/verify và giao tiếp ghi/đọc đã đạt trong các phép thử bên dưới.
Nhận dạng ICM20602 chưa đạt: cùng module trả WHO_AM_I=0x2E qua cả SPI và I2C,
trong khi datasheet quy định0x12. Chưa có mẫu gyro được driver công bố hợp lệ.**

Không đổi chân PCB, không sửa firmware PID và không gán ID giả. Các phép dò
chỉ thuộc firmware hardware-tests, motor không chạy. Kết quả khác0x12 vẫn bị
driver gyro bình thường từ chối.

## Các bước đã tự thực hiện

1. Kiểm tra Git, dùng `bd6cc35` làm mốc trước đợt sửa. Lưu tiếp bản roundtrip
   tại `19d2466` trước khi bổ sung I2C; lưu bản I2C tại `4a83b77` trước cập
   nhật báo cáo/cấu hình debug. Không gom build/cache hoặc thay đổi khác vào Git.
2. Xác định ST-LINK_gdbserver đang giữ cổng61234, dừng riêng server để giải
   phóng probe. Không đóng VS Code hay thay các file đang mở của người dùng.
3. Kết nối bo qua ST-Link SN37FF71064E573436C2DE1443, F405 ID0x413,
   điện áp tham chiếu3.29V. Đọc được kết quả sau lần cấp lại nguồn: một
   ID0x00 rồi31 ID0x2E, HAL_OK. CONFIG0x80/PWR_MGMT_1=0x41/I2C_IF=0x00
   khớp các giá trị reset liên quan trong datasheet, WHO_AM_I vẫn0x2E.
4. Thêm `HardwareTests_ProbeGyroBus()`: lưu divider0x19, ghi và đọc lại
   55/AA/FF/00, khôi phục divider và xác minh. Thử WHO bằng dummy00,
   dummyFF, burst6 byte dữ liệu và CS setup/hold1ms. Không cấu hình/bật gyro.
5. Chạy host test để kiểm tra roundtrip đúng/sai, khôi phục, timeout và
   từ chối ID sai. Build C++/ARM thành công; nạp và verify trên bo.
6. Một lần nạp ở SWD khoảng950kHz không lấy được core ID. Chuyển sang
   yêu cầu140kHz, thực tế125kHz, kết nối Normal + software reset: nạp và
   verify thành công. Cập nhật launch.json về140 để tiếp tục debug. Đây là
   tốc độ SWD của ST-Link, **không phải tốc độ SPI/I2C của ICM**.
7. Thêm `HardwareTests_ProbeGyroI2c()`: với CS HIGH, dùng PB13 làm SCL,
   PB15 làm SDA open-drain, điện trở kéo lên nội và PB14 input kéo xuống
   yếu; dò cả0x68/0x69. Không đổi dây hoặc .ioc, không ghi cấu hình ICM.
   Đọc8 ID bằng repeated START, master NACK ở byte cuối rồi STOP.
8. Đọc trực tiếp RAM và GPIO/SPI sau test, xác nhận pin trở về AF5/SPI2.
   Lưu snapshot RAM và giải mã có cấu trúc, không suy đoán từ ảnh Watch.

## Kết quả đo trên bo

| Phép đo | Kết quả |
|---|---|
| SPI ghi55/AA/FF/00 vào0x19 rồi đọc lại | Đúng4/4 |
| Khôi phục divider | Ban đầu00, khôi phục00, readback đúng |
| WHO: TX `F5 00` | RX `FF 2E`; byte đầu dummy, byte dữ liệu2E |
| WHO: TX `F5 FF` | RX `00 2E` |
| WHO burst: TX `F5 00 00 00 00 00 00` | RX `00 2E 00 FF 70 00 02` |
| HAL của các gói WHO | HAL_OK |
| I2C địa chỉ0x68 | Không ACK trong lần dò này |
| I2C địa chỉ0x69 | ACK;8/8 ID2E,0 ID12 |
| GPIOB MODER sau test | A900AA8F, đã khôi phục |
| GPIOB OTYPER/PUPDR sau test | 00000000/00000000, đã khôi phục |
| GPIOB AFRH sau test | 55500000: PB13/PB14/PB15 AF5 |
| SPI2 CR1/CR2 sau test | 0000036C/00000000, đã khôi phục |
| Host CTest | 1/1 PASS; không mô phỏng phần điện I2C GPIO |
| ARM GCC/G++ build cuối | PASS; FLASH33248B, RAM3400B |
| Nạp Flash/verify bản cuối | PASS |

Mã SPI roundtrip `bench::bus_probe.result=5` nghĩa ghi/đọc đạt nhưng ID
không đúng, không phải gyro ready. I2C `status=1` nghĩa đo xong; `matches=0`
nghĩa không ID nào đúng. Địa chỉ I2C0x69 khác với nội dung WHO_AM_I0x2E.
Byte dummy SPI có thể khác nhau và không được dùng làm ID.

## Kết luận và giới hạn

Kết quả đã giảm khả năng lỗi chỉ nằm ở HAL_SPI, vị trí byte dummy, clock
mode, hoặc sai bit truyền thông: roundtrip nhiều mẫu bit đạt và giao diện
I2C độc lập cũng trả cùng ID. Điều này không chứng nhận toàn bộ phần cứng
hoạt động hoàn hảo, nhưng **chưa có bằng chứng về lỗi phần mềm làm0x12 bị
đổi thành0x2E**. Module phản hồi một ID không khớp thông số ICM20602.

Chưa thể kết luận module là chip nào, chip giả hoặc chip hỏng. Xác minh
mã in trên IC, tài liệu của đúng breakout/lô hàng hoặc đối chiếu một module
ICM20602 đã biết đúng là bước bên ngoài phần mềm còn cần để chốt nguyên nhân.
WHO_AM_I là thanh ghi chỉ đọc; không thể sửa firmware để ghi một ID mới
cho silicon. Không thay điều kiện chấp nhận thành2E để công bố gyro đạt.

Chưa kiểm chứng Data Ready, tốc độ quay, bias và hướng trục trên cảm biến
thực vì chưa có nhận dạng hợp lệ. Driver vẫn dùng ±2000dps/16.4LSB/dps,
chỉ đọc gyroXYZ, accel standby; các logic này đã qua host test, không thay
thế phép đo thực tế với chip được nhận dạng đúng.

## Firmware hiện tại và cách quay về bài gyro

Trong main.c, `GYRO_IDENTITY_ONLY=2` chạy một lần:

```cpp
HardwareTests_ReadOnlyGyroProbe();
HardwareTests_ProbeGyroBus();
HardwareTests_ProbeGyroI2c();
```

Trong while vẫn gọi `HardwareTests_Update()`. Không gọi lại các phép dò
trong while. Mode2 không lấy mẫu gyro liên tục; `test_started=0` và
`imu_ready=0` là chủ ý. Xem `bench::readonly_probe`, `bench::bus_probe`,
`bench::i2c_probe` để đọc kết quả. I2C probe chỉ áp dụng đúng F405/SPI2/CS PB12;
nếu I2C_IF_DIS đang1 thì phép dò bỏ qua, không tự sửa interface sensor.

Đặt selector bằng1 để chỉ đọc ID thuần túy; bằng0 để chạy Begin/TestGyro
và fallback SPI như trước. Không cần lệnh UART hoặc menu. Bài gyro đầy đủ
chỉ đạt khi ID12 ổn định, readback cấu hình đúng, `imu_ready=1` và có mẫu mới.

## Bằng chứng và sao lưu

- `tmp/imu-probe/before-probe.bin`: sao lưu toàn bộ Flash1MiB trước các phép
  nạp chẩn đoán đầu tiên, giữ nguyên, không đưa vào Git.
- `tmp/imu-probe/session-results.bin`: snapshot RAM240B tại200004A0 của
  phiên cuối, gồm readonly136B, roundtrip76B, I2C28B.
- `tmp/imu-probe/results-20261010.json`: dữ liệu đã giải mã và SHA256 ELF.
  Các địa chỉ RAM này gắn với ELF phiên này; build khác phải tra symbol lại.
- Mốc trước đợt sửa hiện tại: `bd6cc35`; trước I2C: `19d2466`; bản I2C:
  `4a83b77`. Flash snapshot và các file tạm không được stage tự động.

Nguồn thông số và giao thức:
[TDK ICM20602 DS-000176 rev1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf),
SPI/I2C §6, register reset defaults, SMPLRT_DIV và WHO_AM_I. Số liệu trong
bảng là phép đo trên bo người dùng, không phải số liệu từ datasheet.
