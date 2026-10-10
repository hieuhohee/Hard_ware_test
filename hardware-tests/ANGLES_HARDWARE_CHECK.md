# Kiểm tra góc trực tiếp trên bo — 10/10/2026

Firmware tại commit `0a92e27`, source không thay đổi trong lần test này.
ELF SHA256: `78f482c000203e416cbe5f6833e1a9490acb8976749e91adad9eae3a66b1b6cc`.
ST-Link37FF71064E5734368AEC1743, V2J37S7, điện áp3.26V, SWD125kHz.
Đã tạm đóng GDB server của VS Code để lấy snapshot bằng CubeProgrammer;
mỗi lần halt/upload xong đều run lại. Khi xem Live Watch, mở phiên mới F5.

**Đọc/hiệu chuẩn hoạt động; lần kiểm tra quay về mốc chưa đạt.** CTest
`bench_logic` và `gyro_unverified_logic` đều PASS, nhưng host test không
chứng minh thang đo hoặc dữ liệu vật lý trên module chưa nhận dạng đúng.

Đầu test: calibration_state3, đủ400mẫu, ready1, configured1, io_errors0,
config_mismatches0. Bias XYZ `[0.57561,-1.37973,-0.28338]`°/s;
noise_std XYZ `[0.32691,0.08887,0.06130]`°/s. ID thực vẫn0x2E.

| Mốc | Samples | Updates | Euler tuyệt đối từ zero firmware: roll/pitch/yaw (°) | Continuity |
|---|---:|---:|---|---:|
| Trước thao tác | 29733 | 29270 | 3.779 / -2.050 / -0.043 | 1 |
| Người dùng báo đã xoay | 35875 | 35412 | -5.851 / 3.216 / -82.092 | 1 |
| Người dùng báo về ban đầu | 49014 | 48541 | 28.703 / -29.330 / 49.372 | 0 |

Để loại góc tích lũy trước test, tính quaternion tương đối
`q_rel = inverse(q_before) * q_after`, chuẩn hóa rồi đổi sang Euler ZYX:

- Lần xoay đi: roll-8.400°, pitch-0.246°, yaw-82.182°;
  độ lớn phép quay3D khoảng82.553°. Thêm6142mẫu trong49401ms theo tick MCU.
- Khi báo đã về: so với mốc đầu còn roll27.705°, pitch-25.124°, yaw49.804°;
  độ lớn phép quay tương đối còn66.217°. Toàn cửa sổ tăng19281mẫu trong155450ms.
- Người dùng xác nhận góc xoay chỉ **ước lượng bằng mắt**. Vì vậy không
  dùng chênh lệch82°/90° để hiệu chỉnh scale hoặc công bố sai số cảm biến.

Snapshot cuối: io_errors0, config_mismatches0, forced_reads4, gap_count3,
skipped_frames7, tracking_continuous0. Counter samples vẫn tăng, nhưng có
những đoạn không có mẫu fresh để tích phân liên tục. ready1 chỉ xác nhận
hiệu chuẩn đã đạt; không đảm bảo góc hiện tại còn chính xác sau mất mẫu.
Firmware không tự bù chuyển động đã bỏ lỡ hoặc xóa cờ continuity để che lỗi.

max_poll_gap_ms8, saturated_frames0, zero_frames1. raw cực trị XYZ từ lúc
Begin: X-4994..3905, Y-3096..32639, Z-3875..32639. Các cực trị này có thể
gồm chuyển động/outlier ở các thời điểm khác nhau; lịch sử32frame chỉ giữ
đoạn cuối, không đủ xác định thời điểm hay nguyên nhân cụ thể. HAL_OK không
xác minh checksum SPI hoặc chứng minh mọi frame vật lý đúng. Chưa kết luận
gián đoạn do dây, chip, ODR, thao tác người dùng hoặc cách đọc.

Trong lần test này không thay pin, clock, dt, bias hoặc hệ số16.4. Bài bench
vẫn thu khoảng125Hz dù ODR yêu cầu1kHz, không FIFO. Cần phép quay được căn
vạch/thước, giữ trục cố định, ghi dữ liệu toàn hành trình và tìm nguyên nhân
gián đoạn trước khi coi kết quả đủ tin cậy cho PID quay90°.

Dữ liệu gốc/JSON tại `tmp/imu-probe/angles-check-{current,before-turn,after-turn,return}`,
và `angles-{turn,return}-comparison.json`; không stage các snapshot/build vào Git.
Mốc trước ghi báo cáo: `0a92e27`.
