# Test gyro tạm thời dù WHO_AM_I sai — 10/10/2026

**Đã nạp và đọc được dữ liệu trên bo F405 thật.** Theo yêu cầu người dùng,
bài bench mode5 bỏ chặn khởi tạo theo ID, nhưng vẫn lưu ID thực `0x2E`
và `id_unverified=1`. Đây là kiểm tra khả năng cấu hình/đọc gyro theo map
ICM20602; chưa xác nhận đúng loại chip hoặc độ chính xác của thang đo.

Mốc sao lưu trước đợt thay đổi: `b5e0903`. Giữ nguyên CS PB12, SPI2
PB13/PB14/PB15, mode0, prescaler64, clock và .ioc. Chỉ khởi tạo GPIO/SPI;
không chạy PWM, PID hay đọc accel. Driver C++ thường vẫn kiểm tra ID như cũ.

## Các thay đổi

- `cubemx/micromouse_hwtest/Core/Src/main.c`: chọn `GYRO_IDENTITY_ONLY=5`.
- `Core/Src/icm20602_minimal_c.c`: thêm hai hàm C độc lập bên dưới.
- `Core/Inc/icm20602_minimal_c.h`: thêm biến chẩn đoán `icm_c_gyro`.
- `tests/minimal_c`: HAL giả để kiểm thử chính source C dùng trên bo.

Một lần sau `MX_GPIO_Init()` và `MX_SPI2_Init()`:

```c
test_started = IcmC_GyroBeginUnverified();
```

Trong `while (1)`:

```c
IcmC_GyroPollUnverified();
```

Main đã có đúng hai lời gọi này. Không gọi đồng thời `HardwareTests_Begin`,
`HardwareTests_TestGyro`, `HardwareTests_Update` hay bài probe ID cũ trong
mode5. Không cần UART, menu hoặc gửi lệnh. CMake đã có source C này.

Begin chờ nguồn ổn định, tắt I2C, reset IMU, tắt I2C lần nữa, đọc ID thực,
rồi cấu hình gyro. Hàm đọc lại 9 register và so các bit có nghĩa. Khi ID sai
vẫn thử cấu hình, nhưng khi readback sai vẫn công bố `configured=0` và tăng
`config_mismatches`. Poll có thể tiếp tục đọc raw để chẩn đoán ngay cả khi
readback chưa đạt. `test_started` chỉ là kết quả readback lúc Begin,
không phải xác nhận ID hay bằng chứng dữ liệu đã đúng.

## Cấu hình gyro giả định và readback đo được

| Register | Ghi / đọc lại | Mục đích |
|---|---|---|
| I2C_IF 0x70 | 0x40 / 0x40 | Tắt I2C khi dùng SPI |
| PWR_MGMT_1 0x6B | 0x01 / 0x01 | Thoát sleep, chọn clock tự động |
| PWR_MGMT_2 0x6C | 0x38 / 0x38 | Accel XYZ standby, gyro XYZ hoạt động |
| CONFIG 0x1A | 0x02 / 0x02 | DLPF_CFG=2 |
| SMPLRT_DIV 0x19 | 0x00 / 0x00 | ODR yêu cầu 1kHz khi DLPF hoạt động |
| GYRO_CONFIG 0x1B | 0x18 / 0x18 | ±2000°/s, giả định 16.4 LSB/(°/s) |
| ACCEL_INTEL_CTRL 0x69 | 0x02 / 0x02 | OUTPUT_LIMIT theo datasheet |
| INT_PIN_CFG 0x37 | 0x20 / 0x20 | Latch; đọc INT_STATUS để clear |
| INT_ENABLE 0x38 | 0x01 / 0x01 | Bật Data Ready |

Không cần nối chân INT cho bài này vì đọc `INT_STATUS` qua SPI. Khi bit0 bật,
đọc burst 6 byte `0x43..0x48`, ghép big-endian signed XYZ. Byte RX đầu tiên
chỉ là dummy và bị bỏ qua. Khi không có Data Ready, mỗi100ms vẫn có thể đọc
cưỡng bức; lần ấy chỉ tăng `forced_reads`, **không** tăng `samples`.

Các thanh ghi/thang đo tham chiếu từ [datasheet TDK ICM20602 DS-000176 rev1.1](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf).
ID chưa khớp nên khả năng ghi/đọc cùng map không chứng minh linh kiện đúng mã.

## Mở Watch trong VS Code

1. Kết thúc phiên debug cũ, rồi F5 để dùng đúng ELF vừa build. Agent đã nạp
   firmware này và cho bo chạy; chỉ cần F5 khi muốn theo dõi bằng debugger.
2. Bỏ breakpoint trong vòng `while`/Poll/Update. Nếu dừng ở `HAL_Init`, nhấn
   F5 tiếp để khởi tạo xong. Breakpoint mỗi vòng sẽ làm bộ đếm đứng yên.
3. Xóa biểu thức `bench::live.*` của bài C++ cũ khỏi Watch nếu chúng báo
   không tìm thấy symbol. Mode5 chỉ link các hàm C cần dùng.
4. Thêm các biểu thức:

   ```text
   test_started
   icm_c_gyro.configured
   icm_c_gyro.actual_id
   icm_c_gyro.id_unverified
   icm_c_gyro.samples
   icm_c_gyro.raw
   icm_c_gyro.dps_assumed
   icm_c_gyro.io_errors
   icm_c_gyro.config_mismatches
   ```

5. Dùng STM32Cube Live Watch khi chương trình đang chạy. Watch thường chỉ
   cập nhật khi dừng: cho chạy vài giây, bấm Pause, đọc giá trị, rồi F5 tiếp.
   Không so bộ đếm giữa hai lần đọc khi MCU vẫn đang pause.
6. Để module yên: raw thường có bias và nhiễu, không bắt buộc bằng0. Xoay
   quanh một trục rồi xoay ngược: kiểm tra thành phần tương ứng tăng/giảm
   và đổi dấu quanh mức bias. Để yên lại phải trở về gần mức trước xoay.
   Hướng trục tùy cách gắn module; không mặc định mọi động tác là trục Z.

| Biến | Ý nghĩa |
|---|---|
| done / init_step | Begin đã xong / bước5: xong readback |
| actual_id / id_read_ok | ID đọc thực và giao dịch đọc có thành công không |
| id_unverified / bypass_requested | ID chưa xác nhận / bài này được phép bỏ chặn ID |
| configured | Chín readback đạt, không có lỗi HAL trong Begin |
| reg, expected, actual, read_ok | Xem từng thanh ghi nếu config_mismatches khác0 |
| samples / data_ready | Lần đọc burst thành công có fresh / số lần thấy bit Data Ready |
| forced_reads | Lần đọc chẩn đoán không có fresh, không tính mẫu mới |
| raw[0..2] | X/Y/Z signed counts, chưa bù bias |
| dps_assumed[0..2] | raw/16.4, chỉ là thang giả định khi ID chưa xác nhận |
| minimum / maximum | Cực trị raw từ lúc Begin, bao gồm cả giai đoạn ban đầu |
| history | Vòng32 bản ghi: tick(ms), fresh, rawX, rawY, rawZ |
| io_errors / hal_status | Lỗi hoàn tất giao dịch HAL / trạng thái gần nhất |
| zero_frames / saturated_frames | Frame cả3trục bằng0 / có trục đúng32767 hoặc-32768 |

`configured` là kết quả lúc khởi tạo. Khi đang chạy vẫn phải theo dõi
`io_errors`, `samples` và phản ứng raw. `saturated_frames=0` chỉ loại hai mã
biên chính xác, không chứng minh dữ liệu không có outlier hoặc bão hòa sớm.

## Kiểm chứng đã hoàn tất

- ARM GCC build: PASS, FLASH10144B, RAM2592B; nạp và verify: PASS.
- ST-Link SN37FF71064E5734368AEC1743, V2J37S7, điện áp3.26V,
  SWD thực125kHz. F405 ID0x413; reset bằng software, không cần NRST.
- Snapshot đầu: `configured=1`, `actual_id=46` (0x2E), `id_unverified=1`,
  `config_mismatches=0`, `io_errors=0`, `samples=2860`, raw `[9,-22,-6]`.
- Sau người dùng báo đã xoay: snapshot dừng core ngắn để đọc, rồi run lại:
  `samples=12063`, `changed_samples=11745`, `data_ready=12063`,
  `forced_reads=0`, `io_errors=0`, `config_mismatches=0`, raw `[8,-22,-5]`.
- Cực trị sau thao tác: X `-3780..4426`, Y `-7779..32639`,
  Z `-21085..32639`; trước thao tác cực trị X `0..12`, Y `-26..0`, Z `-8..4348`.
  Số liệu cho thấy dữ liệu thay đổi sau thao tác và trở lại gần nền khi để
  yên. Chưa có phép quay góc/tốc độ chuẩn đồng bộ, nên chưa kết luận hệ số
  quy đổi đúng. Một số cực trị gần thang đo cần kiểm tra bằng quay chậm có
  kiểm soát trước khi dùng cho điều khiển. Không diễn giải mọi cực trị như
  tốc độ quay vật lý đã xác nhận.
- Hai snapshot tăng9203 mẫu trong73749ms theo tick MCU: khoảng124.79Hz.
  Poll có ngưỡng5ms nhưng delay bench ở CS khiến thực tế khoảng8ms/mẫu.
  Đây là tốc độ đọc MCU; không phải phép đo độc lập ODR cảm biến. Không FIFO,
  nên firmware bỏ qua những mẫu trung gian của ODR yêu cầu1kHz.
- Host tests: `bench_logic` và `gyro_unverified_logic` đều PASS. Bài C kiểm
  tra ID2E vẫn đọc, không ghi WHO_AM_I, readback sai vẫn báo failed, ghép
  burst signed hai biên, tách fresh/cưỡng bức và lỗi SPI không tăng samples.

Snapshot gốc + JSON ở `tmp/imu-probe/gyro-unverified-{1,2}.{bin,json}`,
địa chỉ biến của **ELF lần này** là0x20000088, sizeof920. Đây là dữ liệu
đo cục bộ, không đưa build/cache vào Git. Không dùng địa chỉ này sau khi
thay đổi firmware mà chưa kiểm tra symbol lại.

## Sau khi test

Đặt selector về0 và build/nạp lại để quay về driver C++ thường; driver ấy
sẽ tiếp tục từ chối ID2E. Không tự đưa bypass mode5 sang firmware PID.
Để dùng gyro cho quay90°, bước tiếp theo là xác minh thang đo bằng quay
chuẩn, hiệu chuẩn bias khi đứng yên và kiểm tra dấu trục. Bài này chưa
tích phân góc và không tự hiệu chuẩn khi module có thể đang được cầm/xoay.
