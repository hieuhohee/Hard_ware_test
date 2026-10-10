# Sửa thời gian lấy mẫu và ghi chất lượng gyro — 10/10/2026

Mốc trước đợt sửa: `1d9b04f`. Giữ pin/clock/SPI mode/prescaler cũ, không
đổi .ioc hay firmware PID. Người dùng báo dữ liệu góc không chuẩn; sau đó
báo đang thấy dữ liệu khá tốt. Điều này chưa xác nhận góc đúng trên thước.

## Điểm đã sửa trong source

- Đường gyro_read/gyro_write trước có HAL_Delay1ms ở cả đầu và cuối CS;
  một lần poll gồm đọc status và burst nên đã đo khoảng8ms/mẫu (~125Hz).
  Đây là giới hạn của bài bench, chưa chứng minh là nguyên nhân duy nhất
  khiến lần quay về lệch lớn.
- Thay delay ấy bằng guard2us qua DWT, đổi ngưỡng poll5ms thành1ms.
  Delay250/100ms lúc startup/reset vẫn giữ; bài ID/config audit cũ không đổi.
- Dùng CYCCNT tại lúc đọc burst xong để tính dt góc; chênh lệch unsigned32
  xử lý wrap. Clock DWT là SystemCoreClock (168MHz ở cấu hình hiện tại).
  Host test dùng fakeclock1MHz. Khi pause, DWT/HAL tick có thể dừng;
  không thể phục hồi chuyển động lúc debugger halt.
- Main chọn hiệu chuẩn2000mẫu fresh thay vì400 do poll mới nhanh hơn.
  Đợi ready1 và samples==target, không chỉ chờ một số giây cố định.
- Thêm `icm_c_quality` để ghi trạng thái, raw gần biên, raw nhảy lớn,
  cửa sổ50ms không có Data Ready và lỗi HAL. Journal64 bản ghi gần nhất
  có dấu hiệu. Không clamp/làm mượt raw, không ép continuity/ready thành1,
  không sửa hệ số16.4 theo phép xoay bằng mắt, không đổi ID2E thành12.

Guard2us dài hơn yêu cầu setup/hold CS của ICM20602 trong
[datasheet TDK DS-000176](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf).
Tuy nhiên chip chưa nhận dạng đúng và các thay đổi transport mới vẫn cần
đo thực tế. Poll1ms không đảm bảo thu đủ mọi mẫu1kHz; chưa có FIFO.

## Các biến cần xem trong STM32Cube Live Watch

```text
icm_c_angles.ready
icm_c_angles.calibration_samples
icm_c_angles.calibration_target
icm_c_angles.tracking_continuous
icm_c_angles.last_dt_s
icm_c_gyro.raw
icm_c_gyro.samples
icm_c_gyro.io_errors
icm_c_quality.last_dt_us
icm_c_quality.status_other_bits
icm_c_quality.near_rail_frames
icm_c_quality.large_step_frames
icm_c_quality.no_data_windows
icm_c_quality.read_failures
icm_c_quality.events
```

Mỗi event có8số: HALms, reason_mask, INT_STATUS, rawX/Y/Z, dt_us, samples.
Reason bit0=bit status khác DataReady, bit1=|raw|>=32000, bit2=step>=8192,
bit3=không fresh>=50ms, bit4=lỗi HAL đọc. Bit0/1/2 là dấu hiệu cần phân tích,
không tự khẳng định frame sai vì một chuyển động thật cũng có thể lớn.
Nếu không fresh/đọc HAL thất bại/gap lớn, phần góc vẫn đánh dấu mất liên tục
như trước. Journal không phải checksum và không bảo đảm phát hiện mọi lỗi.

## Trạng thái kiểm chứng

- ARM build PASS: FLASH19592B, RAM5232B.
- Host tests PASS2/2: bài gyro dùng source C thật, có thêm kiểm tra journal
  giữ nguyên raw32639, báo step/near-rail/status, cửa sổ thiếu Data Ready
  không spam, lỗi HAL và wrap index journal. Các test góc/bias cũ cũng PASS.
- **Chưa nạp/xác minh bản này trên bo bằng công cụ của agent.** Trong lần
  kết nối này đã gặp DEV_CONNECT_ERR rồi DEV_USB_COMM_ERR, công cụ list
  thấy firmware ST-Link trống. Không suy ra MCU/IMU hỏng từ lỗi USB này.
- Không công bố độ chính xác đã được sửa hoặc tốc độ thật đã đạt1kHz.
  Những số liệu125Hz và phép quay về lệch49.8° thuộc firmware trước.

Khi kết nối lại, F5 sẽ build/nạp source mới. Giữ module yên tới ready1,
quan sát raw lúc đứng yên, journal và sample rate trước khi đo góc bằng
vạch90°. Dừng debug cũ trước khi agent dùng CubeProgrammer để tránh
hai công cụ giữ ST-Link cùng lúc. Chỉ dùng Watch thường khi pause;
để quan sát liên tục phải thêm biến vào STM32Cube Live Watch.
