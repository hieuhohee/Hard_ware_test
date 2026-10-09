# Hiệu chuẩn và tinh chỉnh trên robot thật

Mục tiêu của các bước dưới là tách lỗi cơ khí, đo lường và điều khiển. Không thay đồng thời gain của ba lớp. Mỗi lần thay một nhóm thông số, lưu log và ghi tình trạng pin, mặt sàn, tải, nhiệt độ motor.

## 1. Hiểu đường đi của tín hiệu

Lớp 1 chạy ở nhịp nền: encoder → tốc độ bánh đã lọc → PI → duty PWM. Lớp 2: gyro đã trừ bias → tích phân góc → PID góc → tốc độ quay yêu cầu. Lớp 3: IR đã đổi ra khoảng cách → sai lệch ngang → một độ lệch góc nhỏ cộng vào hướng hành lang.

Các phương trình tương ứng mã:

```text
delta_s_L = delta_count_L * metres_per_count_L
delta_s_R = delta_count_R * metres_per_count_R
v_measured = lowpass(delta_s / dt)
s = (s_L + s_R)/2
theta += (gyro_raw_rad_s - bias) * gyro_scale * dt

remaining = distance_goal - s
v_target = clamp(distance_kp * remaining,
                 +/- min(straight_speed, sqrt(2*linear_accel*abs(remaining))))
v_reference = slew_limit(v_target, linear_accel)

theta_reference = heading_at_command_start + wall_offset
omega_target = PID_heading(theta_reference - theta, error_rate=-gyro)
omega_reference = slew_limit(omega_target, yaw_accel)

v_L_ref = v_reference - omega_reference*track_m/2
v_R_ref = v_reference + omega_reference*track_m/2
duty_i = PI_i(v_i_ref - v_i_measured) + kv_i*v_i_ref + ks_i*sign(v_i_ref)
```

Khi quay tại chỗ, `v_reference=0`, góc yêu cầu là góc bắt đầu cộng ±90°, giới hạn tốc độ quay giảm theo góc còn lại. Cả hai bánh co cùng tỷ lệ nếu vượt tốc độ cho phép. Khoảng dừng nhỏ được hiệu chỉnh hai chiều, không chỉ chạy đến số count rồi cắt điện.

Không có tầng PI tốc độ góc riêng trong phiên bản này: PID góc tạo omega, mixer đổi omega sang tốc độ bánh, hai PI bánh thực hiện yêu cầu đó. Nhánh D góc dùng gyro làm giảm dao động. Cấu trúc này bám sơ đồ đã cung cấp.

## 2. Kiểm tra phần cứng và đơn vị trước khi tune

Test nguồn logic, UART, ADC và SPI khi motor chưa cấp nguồn. Đo PWM đúng 20kHz; duty=0 khi reset và idle. Kiểm tra các thay đổi đã đề xuất ở schematic thật, đặc biệt hướng mắc LED IR, đầu vào ADC không vượt điện áp cho phép, GND và decoupling của ICM20602. Code không sửa được lỗi netlist/PCB.

Nâng bánh, `b` rồi `1`, quan sát bánh trái; `b` rồi `3`, quan sát bánh phải. Dương phải tương ứng bánh đẩy robot tiến. Nếu sai chiều điện, đổi `board::motors[i].sign`, không dùng encoder sign để che chiều motor sai. Hai input PWM của một driver phải đúng IN1/IN2.

Quay bánh bằng tay theo chiều tiến; count tương ứng phải tăng. Nếu giảm, đổi `board::encoders[i].sign`. Nhìn từ trên, xoay thân robot sang trái; gyro trục yaw phải dương. Đặt `gyro_axis=0/1/2` theo hướng lắp IC và `gyro_sign=+1/-1`. Tên trục Z không tự bảo đảm là trục đứng khi module bị xoay cạnh.

Chỉ đặt `polarity_verified=true` khi cả ba phép thử đã đúng. Nếu giả định chiều sai, phản hồi âm có thể biến thành phản hồi dương và tăng duty đến giới hạn. Bảo vệ sai chiều trong code là lớp bổ sung, không thay bước thử dấu.

## 3. Encoder và kích thước cơ khí

Đo số count khi quay bánh đúng một hoặc nhiều vòng thật chậm. Gọi tổng số count là C, số vòng bánh là N:

```text
counts_per_wheel_turn = abs(C)/N
metres_per_count = PI * effective_wheel_diameter / counts_per_wheel_turn
```

Đường kính danh nghĩa 25mm chỉ là điểm bắt đầu. Đường kính hiệu dụng phụ thuộc lốp/tải. Đo trên 5–10 vòng giúp giảm sai số đánh dấu.

Nếu encoder ở trục motor trước hộp số: `counts_per_wheel_turn=4*PPR*gear_ratio` với điều kiện PPR là chu kỳ A/vòng trục encoder. Nếu encoder ở trục bánh: không nhân tỷ số hộp số. Một số nhà bán hàng đã gọi con số x4 là CPR; **không nhân 4 lần nữa**. MT6701 có cấu hình số xung, không mặc định 1024 hoặc một con số đọc từ repo khác.

Ví dụ hình thức nhập trong `platform/board_config.cpp`:

```cpp
// Thay D_LEFT và COUNTS_LEFT bằng số đo của bạn, không phải tên biến có sẵn.
c.wheel[0].metres_per_count = mm::pi * D_LEFT / COUNTS_LEFT;
c.wheel[1].metres_per_count = mm::pi * D_RIGHT / COUNTS_RIGHT;
c.track_m = TRACK_MEASURED;
c.geometry_verified = true;
```

Sau khi robot lăn thẳng một quãng đã đo bằng thước, sửa:

```text
metres_per_count_new = metres_per_count_old * distance_ruler / distance_reported
```

Hiệu chỉnh từng bánh khi có thể. Nếu thường xuyên trượt, sửa ma sát/áp lực bánh/tăng tốc trước; không hấp thụ trượt thất thường vào hệ số encoder.

`track_m` là khoảng cách hiệu dụng giữa hai bánh. Ước lượng sau một lần quay chậm đo góc thật `angle_true`:

```text
track_effective = (s_R - s_L) / angle_true_rad
```

Lặp trái/phải nhiều lần, tránh trượt. Gyro vẫn là phản hồi góc; track ảnh hưởng mixer và đáp ứng quay, không tự sửa scale gyro sai.

## 4. Zero gyro và kiểm tra drift

Để robot đứng yên, chờ nguồn/nhiệt ổn định, gửi `c`. Firmware lấy 2000 mẫu **mới** theo data-ready, tính mean và standard deviation; không đếm lặp một mẫu cũ. Timeout 5s; bánh dịch count trong lúc zero hoặc variance/mean quá lớn sẽ hủy hiệu chuẩn.

Mặc định mean cho phép dưới 0.15rad/s (~8.6°/s), std dưới 0.015rad/s (~0.86°/s). Đây là ngưỡng bench, cần đối chiếu nhiễu đo được. Các kiểm tra này không phát hiện tuyệt đối được mọi chuyển động đều rất chậm; bạn vẫn phải giữ robot đứng yên.

`diagnostics.calibrated` phải true trước khi arm. Bias chỉ lưu RAM; reset MCU cần zero lại. `r` chỉ xóa góc tương đối/controller, không xóa bias tốt. Sau zero tốt, góc controller tự đặt lại 0.

Đứng yên 10s, quan sát drift. Nếu drift lớn: kiểm tra rung, nguồn, nhiệt, hướng lắp và zero trước khi tăng gain. Đo scale bằng quay thủ công qua góc lớn có mốc chuẩn, chậm và dừng ổn định:

```text
gyro_scale_new = gyro_scale_old * true_angle / reported_angle
```

Đo cả chiều trái/phải. Thay gyro scale phải build/nạp lại và zero lại. Gyro-only không giữ hướng tuyệt đối vô hạn; lớp bám tường có thể sửa sai lệch cục bộ trong hành lang, không thay cảm biến hướng tuyệt đối.

## 5. Tune lớp 1: PI tốc độ từng bánh

Giữ wall off, chưa test quay. Với gain mặc định 0, closed-loop bị khóa. Điền một Kp dương nhỏ có cơ sở đo; Ki=0, Kd=0, kv=ks=0, duty limit thấp. Có thể test một bánh bằng `MM_TestWheelSpeed(0.05f,0,1)` rồi bánh còn lại; cần gyro zero và kiểm tra robot được giữ/đặt phù hợp.

Để chọn điểm bắt đầu theo số đo, cấp một xung duty nhỏ đã biết và đo tốc độ ổn định. Xấp xỉ motor + cơ khí là `G(s)=K/(tau*s+1)`:

- `K` là tốc độ ổn định chia duty, đơn vị (m/s)/duty.
- `tau` là thời gian đạt khoảng 63% biến thiên tốc độ; xung 300ms có thể chưa đủ xác định steady state với một số cơ cấu.
- Một ước lượng PI theo mô hình là `Kp=tau*omega_c/K`, `Ki=omega_c/K`, với `omega_c=2*PI*f_c`. Đây là xấp xỉ không tính ma sát, lượng tử encoder, delay và bão hòa; bắt đầu thấp hơn và kiểm nghiệm.

Không lấy gain của plant giả trong `tests/control_tests.cpp` làm gain của N20 thật. Nếu chưa có mô hình tốt, tăng Kp từng bước nhỏ đến khi đáp ứng rõ mà chưa dao động, rồi tăng Ki từ 0 để giảm sai số tĩnh. Giảm gain nếu duty bật tắt/chạm giới hạn kéo dài. Giữ cùng điều kiện pin và mặt sàn để so sánh.

Đơn vị:

| Hệ số | Đơn vị và ý nghĩa |
|---|---|
| Wheel Kp | duty/(m/s), phản ứng ngay với lỗi tốc độ |
| Wheel Ki | duty/m, tích lỗi tốc độ theo giây |
| integral_limit | duty; giới hạn riêng phần tích phân, không phải tổng output |
| kv | duty/(m/s), cấp trước phần gần tuyến tính |
| ks | duty, gần ngưỡng thắng ma sát, giữ 0 nếu chưa đo |
| speed_filter_tau | giây; tăng sẽ mượt hơn nhưng phản hồi trễ hơn |

`speed_filter_tau=0.015s` có tần số cắt xấp xỉ 10.6Hz. Vòng PI chạy 1kHz không có nghĩa băng thông cơ khí 1kHz. Chọn băng thông đóng vòng thấp hơn giới hạn cảm biến/delay/plant; vòng hướng cần chậm hơn vòng tốc độ, vòng tường chậm hơn vòng hướng. Tần số gọi hàm và băng thông đáp ứng là hai khái niệm khác nhau.

Với encoder ít xung, chênh count mỗi 1ms thường bằng 0 rồi nhảy. Có thể tăng lọc hoặc giảm tốc độ cập nhật phép đo, nhưng phải đánh giá lại delay. Không tăng Ki để bù tín hiệu tốc độ lượng tử quá thô. Nếu cần dải tốc độ rất thấp, cân nhắc đo thời gian giữa cạnh bằng phương pháp khác; driver hiện chỉ dùng delta-count.

Sau PI ổn định, đo feedforward `kv`/`ks` riêng cho mỗi motor nếu cần. Driver có chống tích phân khi tổng lệnh chạm duty limit; cơ chế này không bù được mọi giới hạn tốc độ/slew ở tầng ngoài. Không giảm duty limit đến mức motor không thể thắng ma sát rồi kết luận PI sai.

## 6. Tune lớp 2: giữ góc, chạy thẳng và quay 90°

Giữ wall off. Có thể đặt heading Ki=0, Kd=0 trước, giữ Kp nhỏ. Chạy `e` → `f`, xem sai góc, tốc độ hai bánh và duty. Nếu robot lệch nhưng encoder/gyro đúng, tăng Kp góc dần; nếu lắc hai bên, giảm Kp hoặc thêm Kd nhỏ. Nhánh D dùng gyro đã lọc, không lấy hiệu góc bằng số học nên ít nhạy bước nhảy setpoint hơn.

| Thông số | Khi nên chỉnh |
|---|---|
| heading.kp | Tốc độ sửa lệch hướng; quá lớn dễ dao động |
| heading.kd | Hãm chuyển động quay; quá lớn làm ì hoặc truyền nhiễu |
| heading.ki | Chỉ thêm nhỏ nếu còn lệch tĩnh có nguyên nhân đo rõ; thường bắt đầu 0 |
| yaw_rate_limit | Tốc độ quay tối đa |
| yaw_accel | Độ nhanh tăng/giảm omega, ảnh hưởng trượt và rung |
| straight_speed / linear_accel | Tốc độ/gia tốc tiến, không phải duty |
| distance_kp | Mức tiến sát mục tiêu quãng đường; tăng quá lớn dễ overshoot |
| angle_tolerance / distance_tolerance | Vùng đạt mục tiêu, không dùng để che sai hiệu chuẩn |
| settle_s | Phải nằm trong vùng đạt và gần đứng yên liên tục đủ thời gian |

Quay `e` → `L`, đo bằng góc vuông/mốc trên sàn sau khi dừng; làm tương tự `R`. Nếu cả hai đều sai tỷ lệ: kiểm tra gyro_scale. Nếu chỉ một chiều sai: kiểm tra ma sát, dấu, lệch motor và tải. Nếu vượt góc rồi quay bù nhiều lần: giảm tốc/gia tốc hoặc tune Kp/Kd; đừng đổi 90° thành 87° để bù lỗi động học chưa hiểu.

Thử 4 lần +90° với arm lại mỗi lần; góc controller unwrapped sẽ gần 360°, không về 0. Để sai số tích lũy nhỏ cần cơ khí và gyro tốt, không chỉ PID. Hoàn tất là kiểm tra góc, khoảng cách (với lệnh thẳng), tốc độ hai bánh, tốc độ quay và thời gian ổn định; khi hoàn tất PWM chuyển coast nên phải đo cả quãng trôi sau cắt.

Tăng `command_timeout_s` chỉ khi quãng đường/tốc độ hợp lệ thực sự cần lâu hơn. Mặc định 10s không đủ cho mọi lệnh tới giới hạn 2m ở 0.08m/s; API cho tổng quát nhưng bạn phải đặt timeout tương ứng. Không bỏ watchdog thời gian để che CPU quá tải.

## 7. Hiệu chuẩn IR theo nhóm phát thật

Chạy IR ở trạng thái idle, chưa bật bám tường. Quan sát `ir_off`, `ir_on`, `ir_difference`. Với emitter follower TEFT4300 như schematic đã xét, bật LED thường làm ADC tăng: signal_sign=+1. Nếu mạch khuếch đại khác đảo tín hiệu, đặt -1 sau khi đo.

Kiểm tra ADC không chạm 0/4095 kéo dài, nguồn không sụt khi phát, điện áp trên mắt thu ổn định trước lấy mẫu. Đo oscilloscope với đúng tải 10k và transistor thật; không lấy thời gian đáp ứng ở một tải khác trong datasheet làm thời gian settle của bo bạn.

Lấy mẫu khi có tường ở nhiều khoảng cách: ví dụ vùng 20–150mm nếu cơ khí cho phép. Dùng đúng vật liệu/màu tường dự kiến; đo riêng từng mắt và theo đúng nhóm phát (hai LED bên hông bật cùng nhau). Ghi giá trị trung bình `on-off`, độ lệch và điều kiện ánh sáng. Sắp các điểm **signal tăng, khoảng cách giảm**, tối đa 8 điểm.

Ví dụ CÚ PHÁP, số dưới chỉ minh họa, không dùng thay phép đo:

```cpp
mm::IrCurve ir_curves[5] = {
    {3, {100,500,1000}, {0.12f,0.06f,0.025f}}, // IR1: thay bằng số đo thật.
    {}, {}, {}, {}                          // Các mắt khác chưa hiệu chuẩn.
};
```

Trong project đã có mảng `ir_curves`, hãy thay initializer của mảng đó; không khai báo thêm mảng trùng tên. `count=0` nghĩa chưa hiệu chuẩn. Mã từ chối bật bám tường nếu IR1/IR5 chưa có bảng hợp lệ.

Tín hiệu yếu hơn điểm xa nhất được coi không có tường trong vùng đã đo. Tín hiệu mạnh hơn điểm gần nhất kẹp về khoảng cách gần nhất. Bão hòa ADC làm khoảng cách không hợp lệ; riêng IR3 có cờ saturation cho front guard. Nếu mắt hông bão hòa, lớp tường chuyển về chế độ còn dữ liệu hợp lệ/hướng gyro; không thể bảo đảm tránh va chạm trong trạng thái đó. Cần thiết kế lại dải điện trở/gain nếu vùng bám thường xuyên bão hòa.

Trừ nền giảm ảnh hưởng ánh sáng nền biến đổi chậm, không loại được mọi nhiễu ánh sáng hoặc phản xạ chéo. Hai mắt trong cùng nhóm bật đồng thời nên không tách được cross-talk bằng phần mềm hiện tại; nếu cross-talk lớn, thay hướng che quang hoặc tách phần cứng phát độc lập.

## 8. Tune lớp 3: bám tường

Chỉ bắt đầu khi chạy thẳng bằng gyro đã ổn. Hướng hành lang ban đầu chính là góc robot tại lúc ra lệnh; phải đặt robot gần song song hành lang. Với hai cảm biến hông đơn điểm, sai số khoảng cách còn bị ảnh hưởng bởi góc thân, nên không thể suy ra độc lập toàn bộ vị trí và hướng từ một lần đo.

```text
Có hai tường: error = (distance_left - distance_right)/2
Chỉ tường trái: error = distance_left - wall_target_m
Chỉ tường phải: error = wall_target_m - distance_right
wall_offset = giới hạn PID_wall(error) trong +/- wall_offset_limit
```

Gần tường trái → error âm → offset âm → xoay phải. Đặt robot lệch trái/phải khi chưa chạy, kiểm tra dấu này bằng watch `wall_offset` trước test có motor.

Đo `wall_target_m` từ mắt thu đến tường khi robot ở vị trí mong muốn; không lấy nửa chiều rộng mê cung mà quên bề ngang robot. Bám giữa hai tường dùng khoảng cách đã hiệu chuẩn; nếu hai cảm biến không đối xứng cơ khí, cần sửa offset trong hiệu chuẩn/hình học hoặc chỉnh công thức trung tâm cho robot đó.

Đặt Ki=Kd=0 trước; tăng wall Kp rất từ từ. Giới hạn góc ban đầu 8°, giới hạn tốc độ đổi offset 30°/s; giảm nếu robot lắc tại cửa tường. Có tường khi khoảng cách dưới `wall_enter_m`, mất khi trên `wall_exit_m`, tạo hysteresis chống bật/tắt liên tục. Đổi từ hai tường sang một tường sẽ reset tích phân/nhánh D; khi không còn tường offset trở dần về 0, robot giữ hướng gốc.

Frame ADC lỗi/cũ khi đang bám tường gây fault, khác với một mắt có tín hiệu yếu hợp lệ báo không có tường. Nếu muốn tiếp tục bằng gyro khi ADC hỏng thì đó là thay đổi chính sách cần thiết kế có chủ ý; bản này dừng khi mất dữ liệu cần thiết.

Đầu tiên chạy hành lang thẳng kín hai bên ở tốc độ thấp. Sau đó thử một bên tường, cửa tường và góc. Không dùng ngay tại giao lộ để suy ra maze map; project chưa có state machine nhận diện ô/góc mê cung.

## 9. Front guard và giới hạn chuyển động

Mặc định `front_guard=false` vì IR3 chưa có calibration. Để bật: điền LUT IR3, chọn khoảng cách dừng thực tế, đặt `front_guard=true`. Khi lệnh thẳng đang tiến, IR3 quá gần hoặc bão hòa sẽ dừng và giữ fault. Front guard không xử lý vật cản khi lùi, không quét toàn thể thân robot lúc quay tại chỗ.

Khoảng cách dừng phải lớn hơn tổng khoảng hở cơ khí + độ trễ đo/xử lý + quãng hãm/thả trôi đo thực tế. Ước lượng `v²/(2*a)` chỉ là khởi đầu; coast và mặt sàn có thể làm quãng dừng khác. Khi tăng tốc độ, phải đo lại. Không xem một ngưỡng 45mm mặc định là khoảng dừng an toàn ở mọi tốc độ.

Pin 2S thay đổi điện áp theo mức sạc; duty không đồng nghĩa điện áp motor cố định. Project chưa đo điện áp pin/chưa bù nguồn. Thử ở nhiều mức pin; chọn duty, current limit và motor nominal voltage phù hợp phần cứng thật. Chưa biết dòng kẹt N20 nên không tự chốt điện trở ISEN cho bạn trong firmware.

## 10. Đọc log, lỗi và trình tự đưa lên sàn

UART CSV dùng hệ số: tốc độ `mm/s`, yaw `millidegree`, duty `x1000`, quãng count là số nguyên tích lũy. `cmd=1` nghĩa lệnh gần nhất đã được nhận; `0` là từ chối, không phải báo thành công chuyển động. `cal=1` là gyro đã zero; N là số mẫu của lần zero, không phải sample counter tổng. Header in bằng `h` nếu terminal mở sau boot.

| State | Số | Ý nghĩa |
|---|---:|---|
| Idle | 0 | Không chạy controller, vẫn đo cảm biến |
| Straight | 1 | Chạy thẳng theo quãng đường |
| Turn | 2 | Quay tại chỗ |
| WheelSpeed | 3 | Test PI bánh theo thời gian |
| Complete | 4 | Đạt điều kiện kết thúc, duty=0 |
| Fault | 5 | Dừng do lỗi, cần xử lý nguyên nhân rồi reset |

| Fault | Số | Kiểm tra trước |
|---|---:|---|
| None | 0 | Không lỗi core |
| Configuration | 1 | m/count, PI, dấu/cờ xác nhận, giới hạn |
| Timing | 2 | Tick bị mất, main bị chặn, IRQ, period/work time |
| Sensor | 3 | Gyro/encoder chưa sẵn sàng, số đo không hợp lệ |
| GyroStale | 4 | Data-ready không cập nhật, chưa zero |
| IrStale | 5 | ADC/frame quá cũ hoặc lỗi khi đang dùng IR |
| Obstacle | 6 | IR3 gần hoặc bão hòa, front guard đang bật |
| Timeout | 7 | Chưa hoàn tất trong thời gian lệnh |
| Stall | 8 | Duty và tốc độ yêu cầu lớn nhưng bánh gần đứng yên |
| WrongDirection | 9 | Encoder đo chiều ngược yêu cầu kéo dài |
| Emergency | 10 | UART lỗi/tràn queue hoặc sự kiện khẩn nội bộ |

Stall không phải phép đo dòng: nếu duty bị giới hạn dưới `stall_duty` thì nhánh này không kích hoạt, timeout vẫn còn. Dấu ngược được phát hiện theo thời gian để tránh coi quán tính khi đổi chiều là lỗi ngay lập tức.

Thứ tự thực hành:

1. Không cấp nguồn motor: nạp code, WHO_AM_I=0x12, xem IR và encoder bằng tay.
2. Nâng bánh: xung motor từng bánh, kiểm tra dấu, đo count mỗi vòng.
3. Đo hình học, zero gyro, điền cấu hình, test PI tốc độ thấp với thời gian ngắn.
4. Trên sàn trống: chạy thẳng đoạn ngắn; đo quãng đường/góc thực sau dừng.
5. Quay trái/phải 90°; điều chỉnh gyro scale và động học quay.
6. Đo LUT IR, thử bám tường khi chạy chậm.
7. Tăng dần tốc độ/gia tốc, kiểm tra pin yếu, cửa tường và số tick bị mất.

Nếu sửa gain trong debugger lúc đang chạy, trạng thái tích phân cũ có thể không còn phù hợp. Hãy dừng, sửa thông số, `r`, arm lại. Các biến const pin/sign cần build/nạp lại. Không có tự động ghi calibration vào flash trong phiên bản này.
