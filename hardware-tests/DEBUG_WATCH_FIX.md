# Sửa lỗi Watch `Failed to decode cstring` — 10/10/2026

Đã xác minh lỗi nằm trong khâu hiển thị GDB/MI của extension STM32,
không phải không đọc được main.c. Chương trình đã dừng tại breakpoint
`HardwareTests_Update()` trong while; mỗi lần Continue lại chạy tới
breakpoint đó là hành vi bình thường khi breakpoint vẫn bật.

## Nguyên nhân đã xác minh

Extension đang cài `stm32cube-ide-debug-stlink-gdbserver-1.5.0` gọi
`utf8.decode(cstring)` trong parser MI; catch tại file
`lib/adapter/STLinkDebugTargetAdapter.js:44261` in lỗi người dùng gặp.
Giá trị GDB `253 'ý'` chứa ký tự byte không hợp lệ khi parser giải mã
UTF-8. Cấu trúc chẩn đoán trước đây dùng uint8_t (unsigned char), khiến
GDB in các byte kèm ký tự và có thể diễn giải cả mảng như chuỗi.

Không sửa file extension của người dùng. Không đổi hoặc che dữ liệu ID.

## Thay đổi trong project

- Đổi bản sao số liệu chẩn đoán trong `SpiTrial`, `Live`, `IdentityReadOnly`,
  `SpiRoundTrip`, `I2cIdentity` từ uint8_t sang uint32_t để Watch hiển thị số.
- **Buffer truyền/nhận SPI và dữ liệu protocol trong .cpp vẫn uint8_t**,
  SPI vẫn8-bit. Đây là thay kiểu lưu số liệu hiển thị, không đổi giao thức.
- `test_started` dùng uint32_t và được gán0 trước chọn bài test, giữ symbol
  trong ELF kể cả mode chẩn đoán; không đọc nhầm biến đã bị linker bỏ.
- Đồng bộ header canonical và Core/Inc của firmware thật, comment rõ lý do.
- Thêm host assertions về kiểu Watch và test các byte253/255 để chứng minh
  chúng còn nguyên giá trị, không bị lọc thành0 hoặc12.

## Kiểm chứng

- Host build mới và CTest: PASS1/1. Lần build trong sandbox không truy cập
  Windows SDK; đã build lại với quyền phù hợp rồi mới chạy test bản mới.
- ARM GCC build: PASS; FLASH33272B, RAM4360B. RAM tăng từ3400B do các bản
  sao chẩn đoán dùng word32-bit, vẫn chỉ khoảng3.33% RAM128KiB.
- Nạp/verify trên bo: PASS. Probe phiên này SN37FF71064E5734368AEC1743,
  FW V2J37S7, điện áp tham chiếu3.26V, F405 ID0x413. Đây khác serial của
  các phiên đo trước; không giả định hai probe là một thiết bị.
- GDB/MI trực tiếp trên bo, tại breakpoint HardwareTests_Update:

```text
test_started = 0
readonly_probe.id = {46 <repeats 32 times>}
bus_probe.rx = {{255,46,0,0,0,0,0}, {0,46,0,0,0,0,0}, {0,46,0,255,112,0,2}}
i2c_probe = {status=1, address=105, reads=8, matches=0, ack={0,1}, id={46,46,46,46,46,46,46,46}}
(uint32_t)253 = 253
```

Các giá trị là số, không có hậu tố ký tự `'ý'` hoặc chuỗi byte. Log MI lưu
ở `tmp/imu-probe/watch-gdb-mi.log`. Phiên kiểm chứng đã detach/resume target
và đóng GDB server riêng cổng61235, để VS Code có thể khởi động phiên mới.
Không kiểm chứng bằng thao tác giao diện VS Code; xác minh bằng GDB/MI,
chính đường dữ liệu mà extension dùng.

## Cách xem trong VS Code

1. Nhấn Shift+F5 để kết thúc phiên cũ, rồi F5 để tạo phiên mới bằng ELF đã build.
   Việc này cần thiết vì phiên cũ giữ symbol/kiểu dữ liệu của firmware trước.
2. Cho chạy tới breakpoint Update. Xem `bench::readonly_probe`,
   `bench::bus_probe`, `bench::i2c_probe`; các phần tử hiện là số như46/255.
3. Muốn chạy tự do thì bỏ breakpoint trong while và nhấn F5. Muốn xem
   snapshot đã đo xong thì có thể giữ breakpoint; việc dừng CPU sẽ khiến
   dữ liệu cần cập nhật liên tục không tăng cho tới khi Continue.

Main vẫn ở `GYRO_IDENTITY_ONLY=2`: bài chẩn đoán ID/SPI/I2C, không phải
gyro stream. `test_started=0`, `imu_ready=0` là chủ ý. Sửa lỗi hiển thị không
làm ID0x2E thành0x12 và chưa biến kết quả sensor thành gyro ready.

## Layout RAM và sao lưu

Mốc trước sửa: `e1de21b`. Các địa chỉ/layout RAM đã đổi sau khi nâng kiểu:
`readonly_probe=0x20000630`, `bus_probe=0x20000820`,
`i2c_probe=0x2000091C`, `test_started=0x200000E8` trong ELF phiên này.
Không dùng lại bộ giải mã snapshot240B của phiên cũ cho firmware mới;
file snapshot/JSON cũ vẫn được giữ nguyên và chỉ áp dụng cho ELF cũ.
