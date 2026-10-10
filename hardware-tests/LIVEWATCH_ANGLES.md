# Xem góc roll, pitch, yaw trên Live Watch

Kết quả test thực mới: [ANGLES_HARDWARE_CHECK.md](ANGLES_HARDWARE_CHECK.md).
Đã đọc được góc và hiệu chuẩn, nhưng lần quay về mốc có mất đoạn mẫu,
`tracking_continuous=0`; chưa xác nhận góc đủ chính xác cho PID quay90°.

Bản source mới: [GYRO_TIMING_UPDATE.md](GYRO_TIMING_UPDATE.md), poll1ms,
dt bằng DWT, hiệu chuẩn2000mẫu và nhật ký raw bất thường. Đã build/host test;
chưa xác minh bản này trên bo vì ST-Link lỗi kết nối USB. Số liệu400mẫu/
125Hz cuối tài liệu thuộc phiên bản trước. Khi F5 nạp ELF mới, chờ đúng
`calibration_target` và ready thay vì mặc định400mẫu.

Bài test gyro C mode5 đã có tính góc. Giữ nguyên SPI2 PB13/PB14/PB15,
CS PB12 và cấu hình chân cũ. Chỉ gyro XYZ được bật, accel vẫn standby.
Main tự gọi hiệu chuẩn2000 mẫu sau khởi tạo; trong while vẫn chỉ gọi
`IcmC_GyroPollUnverified()`, hàm này cập nhật cả dữ liệu gyro và góc.

## Các bước thực hiện trong VS Code

1. Đặt module trên mặt phẳng, giữ yên. Không bắt buộc module phải nằm
   ngang: tư thế ban đầu được chọn làm góc0, không đo góc so với trọng lực.
2. Kết thúc debug cũ bằng Shift+F5, Build rồi F5 để nạp ELF mới. Khi dừng
   tại `HAL_Init()`, nhấn F5 tiếp. Không đặt breakpoint trong while/Poll
   khi hiệu chuẩn hoặc đo góc.
3. Mở Run and Debug bằng Ctrl+Shift+D. Mở **STM32Cube Live Watch** và dùng
   nút thêm biểu thức/dấu+ trong panel. Watch thông thường chỉ cập nhật khi
   pause; không dùng nó để quan sát liên tục lúc đang xoay.
4. Thêm từng biểu thức dưới đây. Giữ yên tới khi đủ2000mẫu và `ready=1`,
   `calibration_state=3`. Thời gian tính từ lúc firmware chạy qua Begin,
   không tính thời gian đang dừng ở breakpoint.

   ```text
   icm_c_angles.ready
   icm_c_angles.calibration_state
   icm_c_angles.calibration_samples
   icm_c_angles.calibration_target
   icm_c_angles.roll_deg
   icm_c_angles.pitch_deg
   icm_c_angles.yaw_deg
   icm_c_angles.bias_dps
   icm_c_angles.corrected_dps
   icm_c_angles.tracking_continuous
   icm_c_gyro.io_errors
   ```

5. Xoay chậm từng trục rồi xoay ngược. Xem các góc trong bảng bên dưới.
   Khi dừng xoay, góc giữ gần giá trị mới; tốc độ `corrected_dps` về gần0.
   Góc không tự về0 khi bạn dừng. Nó về gần0 khi quay trở lại tư thế đầu.
6. Với mouse chạy trên mặt phẳng, gắn trục Z của IMU gần vuông góc mặt sàn:
   theo dõi `yaw_deg` khi quay trái/phải. Chiều tăng phụ thuộc hướng gắn và
   quy ước trục; kiểm tra thực nghiệm trước khi đưa vào PID quay90°.

| Đại lượng | Ý nghĩa | Đơn vị / phạm vi hiển thị |
|---|---|---|
| roll_deg | Góc Euler roll, liên quan trục X | °, khoảng-180..180 |
| pitch_deg | Góc Euler pitch, liên quan trục Y | °, -90..90 |
| yaw_deg | Góc Euler yaw, liên quan trục Z | °, khoảng-180..180 |
| corrected_dps[0..2] | Gyro X/Y/Z sau trừ bias | °/s, không phải góc |
| quaternion[0..3] | Tư thế tương đối q=[w,x,y,z] | Không đơn vị |

Ba tên bạn hỏi được hiểu là **roll, pitch, yaw**. Phạm vi Euler có wrap:
yaw từ179° sang-179° là tiếp tục quay qua180°, không phải quay ngược358°.
Không dùng phép trừ góc thông thường qua biên±180° trong PID. Khi pitch
gần±90°, roll và yaw có thể nhảy do biểu diễn Euler; quaternion vẫn liên
tục. Các góc phụ thuộc nhau khi quay kết hợp nhiều trục, không phải ba
tổng độc lập của gyro X/Y/Z.

## Nguyên lý tính góc và giới hạn

Chuỗi tính: raw XYZ → chia16.4 → trừ bias đứng yên → tích phân quaternion
với dt thực → chuẩn hóa q → chuyển sang Euler theo
`R = Rz(yaw) * Ry(pitch) * Rx(roll)`.

Với phép quay đơn trục, có thể hình dung `góc += tốc_độ_đã_bù * dt`.
Ví dụ90°/s trong1giây cho90°. Với chuyển động3D tổng quát, gyro đo tốc độ
quay theo hệ trục module, không trực tiếp đo đạo hàm ba góc Euler; vì vậy
source dùng quaternion và phép nhân `q_new = q_old * dq`.

`dt` của bản mới lấy từ DWT giữa hai lần đọc burst, có độ phân giải cao hơn
HAL tick1ms. Poll yêu cầu1ms nhưng chưa đo tốc độ thật của bản mới trên bo.
Không mặc định1ms theo ODR yêu cầu1kHz. Bản trước thu khoảng125Hz. Chưa
dùng FIFO nên có thể bỏ mẫu khi MCU bận; quay nhanh vẫn cần kiểm chứng.
Không tích phân mẫu đọc cưỡng bức, frame bão hòa chính xác32767/-32768,
hoặc qua gap tick lớn hơn50ms. Khi thiếu đoạn dữ liệu, góc cũ còn được giữ
nhưng `tracking_continuous=0` tới khi đặt mốc lại. Counter `gap_count` và
`skipped_frames` giúp nhận biết phần dữ liệu đã bỏ.

Gyro-only chỉ cho **tư thế tương đối từ mốc** và có drift do bias/nhiễu.
Không có accel để hiệu chỉnh roll/pitch theo trọng lực; không có tham chiếu
hướng tuyệt đối cho yaw. Tư thế lúc zero được gọi0 dù module đang nghiêng.
Hiệu chuẩn bias làm giảm drift, không triệt tiêu nó. Xem giải thích từ
[Analog Devices về drift gyro và phản hồi accel](https://www.analog.com/en/resources/analog-dialogue/raqs/raq-issue-139.html).

Chip hiện trả ID0x2E. Phép quy đổi16.4LSB/(°/s) vẫn là **giả định** theo
map ICM20602 ±2000°/s. Do đó góc trên Live Watch cũng là kết quả ước lượng
theo giả định này; cần kiểm chứng bằng xoay một góc biết trước, chẳng hạn90°.
Bỏ kiểm tra ID không xác nhận được sai số scale hoặc loại chip thực.

## Nếu ready vẫn bằng0

| calibration_state | Cách xử lý |
|---|---|
| 0 | Chưa gọi hiệu chuẩn; kiểm tra đang nạp đúng mode5 và ELF mới |
| 1 | Đợi500ms cho mẫu ổn định; để chương trình chạy |
| 2 | Đang thu mẫu fresh tới calibration_target (main mới2000); giữ yên |
| 3 | Hiệu chuẩn đạt; ready phải1 và updates tăng khi có mẫu |
| 4 | Hiệu chuẩn không đạt; xem readback, io_errors và noise_std_dps |

Nếu state2 và số mẫu đứng yên, kiểm tra MCU có đang pause, `samples` có
tăng hay không. Nếu state4, **không sửa ready thành1 trong debugger**.
Để module yên rồi reset firmware để thử lại, hoặc gọi lại hàm hiệu chuẩn
từ chương trình. Hai ngưỡng bench ở source là `ANGLE_MAX_BIAS_DPS=10`
và `ANGLE_MAX_NOISE_DPS=1.5`. Noise là độ lệch chuẩn trong cửa sổ hiệu chuẩn;
nếu cao, kiểm tra rung/chuyển động/outlier trước khi nới ngưỡng.

Gyro-only không thể phân biệt hoàn toàn bias với quay đều rất chậm; một
cửa sổ có nhiễu thấp chưa tự chứng minh module đứng yên. Người dùng cần
đặt module yên trong suốt hiệu chuẩn. Không tự cập nhật bias trong lúc xoay.

## Hai hàm có thể gọi trong chương trình

Hiệu chuẩn lại khi module đứng yên (gọi **một lần**, không gọi mỗi vòng):

```c
IcmC_AnglesStartCalibration(2000);
```

Đặt tư thế hiện tại làm mốc0, giữ bias cũ (gọi một lần khi ready1):

```c
IcmC_AnglesResetReference();
```

Không chèn ResetReference vào while vô điều kiện vì góc sẽ liên tục bị
xóa. Sau zero, mẫu fresh đầu tiên đặt mốc thời gian, những mẫu sau mới
tích phân. ResetReference không hiệu chuẩn bias lại.

Nếu pause/debug lâu, firmware không thu được chuyển động trong khoảng
pause; HAL tick cũng thường không đếm thời gian thực khi MCU bị halt.
Không dựa riêng vào gap_count để phát hiện mọi lần pause. Cho chạy lại
rồi đặt mốc/khởi động lại để thực hiện một bài đo liên tục mới.

## Kiểm thử bản trước (400mẫu, khoảng125Hz)

Mốc trước đợt thêm góc: `bdd6426`. Host test dùng đúng source C firmware,
kiểm tra bias đứng yên, cửa sổ có chuyển động bị từ chối, yaw90° trong1s,
quay body X rồi Y, quaternion chuẩn hóa, reset giữ bias, frame không fresh,
frame bão hòa, gap lớn và HAL tick wrap-around. Hai test CTest đều PASS.
ARM build FLASH18604B/RAM3136B; nạp và verify ST-Link đều PASS.

Đọc trực tiếp RAM F405 sau nạp: cửa sổ đầu có noise_std trụcX khoảng1.590°/s,
cao hơn ngưỡng1.5 nên state4, ready0; không ép đạt hoặc tăng ngưỡng. Đã reset
để chạy lại. Cửa sổ tiếp theo đủ400mẫu, state3, ready1, bias XYZ khoảng
`[0.5253,-1.3819,-0.2846]`°/s, noise_std khoảng`[1.3291,0.0863,0.0880]`°/s.
Snapshot có1661 lần cập nhật góc, dt0.008s, roll0.774°, pitch0.064°, yaw-0.024°,
io_errors0, config_mismatches0; ID vẫn46. Đây là phép kiểm tra cập nhật góc
và hiệu chuẩn, chưa phải đo độ chính xác bằng thiết bị quay chuẩn.
Snapshot/JSON gốc được giữ tại `tmp/imu-probe/gyro-angles-{1,2}.{bin,json}`.

Các lần đo cũ trong GYRO_UNVERIFIED_TEST.md mô tả firmware trước khi thêm
góc. Địa chỉ biến trên RAM thay đổi theo ELF, Live Watch dùng tên symbol,
không nhập địa chỉ snapshot cũ để xem góc.
