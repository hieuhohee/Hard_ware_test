# Kết quả kiểm thử và giới hạn xác nhận

Ngày: 09/10/2026, Asia/Saigon.

Build trên Windows bằng CMake + MSVC 19.51, C++17, `/W4 /WX /utf-8`. Tất cả source ứng dụng mới đã được compile: core thật và adapter STM32 với header/HAL giả. Không có warning được bỏ qua bằng việc tắt `/WX`.

Kết quả CTest sau sửa cuối:

```text
controller_scenarios         Passed
stm32_adapter_scenarios      Passed
100% tests passed, 0 tests failed out of 2
```

Hai mục CTest là hai bộ nhiều kịch bản, không phải chỉ hai assertion.

## Những gì đã kiểm tra

### Lõi điều khiển

- Timer encoder tràn xuôi/ngược 16 và 32 bit.
- PI bị giới hạn đầu ra mà không tích phân tăng vô hạn.
- LUT IR nội suy, cắt vùng xa, kẹp vùng gần, loại điểm không đơn điệu.
- Cấu hình chưa đo không được nhận lệnh di chuyển.
- Plant giả hai motor lệch gain 8%, đáp ứng bậc một và lượng tử hóa encoder: chạy tiến 180mm, lùi 100mm, quay trái/phải 90°, quay tiếp 4 lần +90° qua mốc 360°, test tốc độ bánh theo thời gian.
- Output hữu hạn và không vượt duty limit; hoàn tất thì duty bằng 0.
- Lỗi dt, gyro cũ, số đo NaN, kẹt bánh, sai chiều encoder, timeout đều dừng/giữ fault.
- Dấu sửa bám tường: gần trái sửa phải, gần phải sửa trái; mất tường giảm offset về 0; IR cũ làm dừng khi đang dùng IR.
- Front guard dừng với vật cản gần và ADC phía trước bão hòa.

Điều kiện pass trong mô phỏng gồm sai quãng tiến dưới 6mm và góc quay đơn dưới 2.5°. Đây là dung sai của **plant giả và test**, không phải cam kết độ chính xác của robot thật.

### Adapter và ứng dụng STM32 qua HAL giả

- Compile cả `board_config.cpp`, `stm32_platform.cpp`, `micromouse_app.cpp`.
- Mapping ADC đúng nhóm phát, chỉ một nhóm sáng, không đọc mắt của nhóm khác khi đang phát.
- Đủ frame 5 IR và phép trừ nền đúng trong các kịch bản thời gian giả.
- Ghi/đọc lại cấu hình gyro, accel standby, bật data-ready, không đọc vùng dữ liệu accel.
- Cấu hình chưa hiệu chuẩn chặn motion, nhưng cho phép test xung thô có arm.
- Xung PWM đúng chiều, dừng theo thời gian, arm chỉ dùng một lần.
- UART STOP ngắt PWM ngay trong ISR trước khi main được gọi lại.
- TIM6 watchdog cắt PWM khi main không được gọi; lệnh reset qua UART vẫn dùng được sau fault.
- Zero gyro đủ mẫu; hủy khi bánh dịch chuyển hoặc data-ready không đến/timeout.
- Quay trái tạo PWM trái lùi và phải tiến.
- Timer microsecond qua mốc 32bit không tạo lỗi dt giả.
- Output infinity bị ngắt trước clamp; lỗi SPI/ADC ngăn dùng dữ liệu như còn tốt.

## Chưa được kiểm chứng

- Chưa build/link với STM32Cube HAL thật, startup/linker script và `arm-none-eabi-g++`; chưa tạo hoặc xác nhận file `.ioc` bằng CubeMX.
- Chưa nạp, chạy, đo PWM/ADC/SPI hoặc WCET trên STM32F405RGT6 thật.
- HAL giả không mô phỏng đầy đủ preload timer, ưu tiên ngắt, jitter, nguồn analog, quá nhiệt, sụt áp, nhiễu quang, dòng kẹt hoặc lực bám.
- Chưa có số đo m/count, khoảng cách hai bánh, gain PI, gyro scale, LUT IR của robot này.
- Chưa thể khẳng định tần số IR thực đạt 500Hz hay controller giữ đúng 1kHz trên bo; phải đo diagnostics/oscilloscope.
- Chưa có IWDG, voltage/current sensing, maze solver hoặc cua slalom.

Các cờ/gain chưa biết được giữ khóa trong cấu hình. Xem `CUBEMX.md` để tạo phần firmware nền và `TUNING.md` để đưa lên phần cứng theo từng bước. Kiểm thử PC không thay kiểm thử trên robot.

## Chạy lại

```text
cmake -S micromouse-control -B micromouse-control/build
cmake --build micromouse-control/build --config Release
ctest --test-dir micromouse-control/build -C Release --output-on-failure
```

Bộ test cũ `hardware-tests/` không bị sửa và không được dùng thay cho bộ kiểm thử mới. Gói source không kèm thư mục build hoặc executable Windows.
