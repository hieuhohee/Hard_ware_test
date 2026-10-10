# Test bằng hàm trực tiếp — không nhập lệnh

Nếu gyro không ready hoặc WHO_AM_I khác 0x12, xem [ICM20602_AUDIT.md](ICM20602_AUDIT.md). Bản cập nhật có lịch sử tám ID và mã bước init để phân biệt lỗi ID, lỗi HAL và lỗi readback; cần nạp lại ELF mới để xem các trường này.

Mỗi lần muốn đổi bài, thay **một dòng chọn hàm test**, build và nạp lại. Không cần copy/sửa nội dung driver cảm biến hoặc comment các phần trong `hardware_tests.cpp`.

API nằm trong [hardware_tests_api.h](hardware_tests_api.h), triển khai C++ trong `hardware_tests.cpp`. Header API dùng được cả từ `main.c` và `main.cpp`.

## 1. Chuẩn bị một lần

1. Sinh project từ [micromouse_hwtest.ioc](cubemx/micromouse_hwtest/micromouse_hwtest.ioc), theo [hướng dẫn CubeMX](cubemx/README.md). Giữ nguyên bảng chân đã chốt.
2. Copy `hardware_tests.hpp`, `test_config.hpp`, **`hardware_tests_api.h`** vào `Core/Inc`.
3. Copy `hardware_tests.cpp` vào `Core/Src`.
4. Copy **`examples/test_config.modules.cpp` thành `Core/Src/test_config.cpp`**. Không compile thêm một file cấu hình khác. Bật C++11 hoặc mới hơn, C++ linker, include path `Core/Inc`.
5. Trong bản `test_config.cpp` vừa copy, bật cờ đúng module đang cắm. Ví dụ test gyro: `BENCH_TEST_GYRO=1`, các cờ test khác 0. `BENCH_USE_UART=1` nếu muốn log; bằng 0 nếu chỉ xem debugger. Tất cả cờ mẫu ban đầu đều 0, cần chọn trước khi build.

Hàm test không thay cấu hình GPIO/handle. Nếu cờ module vẫn 0 hoặc init thiết bị thất bại, hàm trả 0. Ngoại vi vẫn cần được CubeMX sinh đầy đủ dù chưa cắm tất cả module.

## 2. Copy khung này vào main

Giữ phần HAL, clock và `MX_*_Init()` do CubeMX sinh. Trong vùng include người dùng:

```c
/* USER CODE BEGIN Includes */
#include "hardware_tests_api.h"
/* USER CODE END Includes */
```

Thêm một biến để xem kết quả chọn bài trong debugger:

```c
/* USER CODE BEGIN PV */
volatile uint8_t test_started = 0; // 1: bài được nhận; 0: kiểm tra cấu hình/ready.
/* USER CODE END PV */
```

Sau **tất cả** các `MX_*_Init()`:

```c
/* USER CODE BEGIN 2 */
HardwareTests_Begin();
test_started = HardwareTests_TestGyro(); // CHỈ đổi dòng này khi đổi bài.
/* USER CODE END 2 */
```

Vòng lặp:

```c
while (1)
{
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  HardwareTests_Update();
}
/* USER CODE END 3 */
```

Không gọi thêm `HardwareTests_Init()`, `HardwareTests_Poll()` hoặc `ModuleBench_SelectMonitor()` trong khung mới. Không thêm `HAL_Delay()` vào while. `Begin` và hàm chọn bài gọi một lần; `Update` gọi liên tục. `Begin` không tự chạy motor.

## 3. Chọn hàm cho từng bài

Chỉ copy **một** dòng ở cột thứ hai vào vùng `USER CODE BEGIN 2` sau `Begin()`:

| Muốn test | Dòng chọn bài | Cờ trong `test_config.cpp` |
|---|---|---|
| Một mắt IR | `test_started = HardwareTests_TestIR();` | `BENCH_TEST_IR=1` |
| Encoder trái | `test_started = HardwareTests_TestEncoders();` | L=1, R=0 |
| Encoder phải | `test_started = HardwareTests_TestEncoders();` | L=0, R=1 |
| Cả hai encoder | `test_started = HardwareTests_TestEncoders();` | L=1, R=1 |
| Gyro XYZ | `test_started = HardwareTests_TestGyro();` | `BENCH_TEST_GYRO=1` |
| Hiệu chuẩn bias rồi đọc gyro | `test_started = HardwareTests_CalibrateGyro();` | `BENCH_TEST_GYRO=1` |
| Motor trái chiều + | `test_started = HardwareTests_TestMotorLeft(+1);` | `BENCH_TEST_MOTOR_L=1` |
| Motor trái chiều − | `test_started = HardwareTests_TestMotorLeft(-1);` | `BENCH_TEST_MOTOR_L=1` |
| Motor phải chiều + / − | `test_started = HardwareTests_TestMotorRight(+1);` hoặc `(-1)` | `BENCH_TEST_MOTOR_R=1` |
| Các cảm biến đã bật | `test_started = HardwareTests_TestAllSensors();` | Bật các cờ cảm biến muốn dùng |

Các cờ encoder đầy đủ là `BENCH_TEST_ENCODER_L` và `BENCH_TEST_ENCODER_R`. Hàm trả 1 nghĩa là có ít nhất một cảm biến yêu cầu sẵn sàng; bài “all” không khẳng định mọi thiết bị đều tốt. Xem các ready mask trong `bench::live` để biết cụ thể.

**Motor là xung một lần:** mặc định 12% trong 300 ms, sau đó dừng. Không đặt hàm `TestMotor...()` trong while vì sẽ tạo xung mới sau mỗi lần dừng. Khi nạp hoặc reset, main bắt đầu lại, nên bài motor đã chọn sẽ chạy lại. Nâng bánh trước khi chạy/reset. Chiều +/− là chiều điện; cần kiểm tra chiều cơ khí thực tế.

`HardwareTests_Stop()` dừng bài đang chạy khi được chương trình gọi. Ở cách dùng mới, gửi ký tự qua UART hoặc ghi `bench::command` **không điều khiển robot**. Dừng xung tự động phụ thuộc `Update()` và HAL tick vẫn chạy; nếu debugger dừng CPU khi PWM đang bật, không coi timeout phần mềm là bảo vệ độc lập.

## 4. Xem kết quả

UART chỉ xuất log: PC10→RX USB–UART, GND chung, mức tín hiệu 3.3 V, terminal 115200/8N1/no flow. Không phải gõ bất kỳ lệnh nào. Nếu chỉ đọc log thì không cần nối TX adapter vào PC11; `.ioc` vẫn giữ USART3 TX/RX theo bảng chân.

Không UART thì xem `test_started` và `bench::live` trong debugger:

| Bài | Giá trị cần xem |
|---|---|
| IR | `ir_ready_mask`, `ir_frames`, `ir_off[0]`, `ir_on[0]`, `ir_difference[0]` |
| Encoder | `encoder_ready_mask`, `encoder_counter[]`, `encoder_total[]` |
| Gyro | `imu_whoami` phải 0x12; `imu_ready`, `imu_samples`, `gyro_raw[]`, `gyro_dps[]` |
| Hiệu chuẩn gyro | `gyro_calibrating` chuyển 1→0; `gyro_calibrated=1`; `gyro_bias_dps[]` |
| Motor | `motor_ready_mask`, `motor_active` chuyển 1/trái hoặc 2/phải → 0 sau xung |
| Lỗi giao tiếp | `io_errors` tăng khi có lỗi HAL; ready=0 cũng có thể do sai ID/cấu hình |

Gyro chỉ dùng ICM20602 và gyro XYZ, accel tắt. Đặt yên khi hiệu chuẩn; lấy 1000 mẫu mới, timeout 5 s, bias lưu RAM nên mất khi reset. Sau hiệu chuẩn hàm `Update` tiếp tục đọc gyro, không cần gọi thêm `TestGyro()` trong while. Việc tự phát hiện robot đang chuyển động chưa được triển khai.

## 5. Thay mắt IR, nguồn và tinh chỉnh

Mẫu dùng một mắt logic `ir[0]`: phát PC1 qua MOSFET, thu PA3/ADC_CHANNEL_3. Để thử một mắt khác, sửa đúng phần tử đó trong bản `test_config.cpp` đang build; bốn phần tử khác giữ unused. Ví dụ bộ phát PC2/thu PC0:

```cpp
{{GPIOC, GPIO_PIN_2}, ADC_CHANNEL_10, +1},
```

Chỉ chọn cặp này nếu dây thực tế nối như vậy. `.ioc` dành PC1/PA5/PC2 cho ba nhóm phát, và PA3/PA4/PB0/PB1/PC0 cho năm mắt thu. Driver hiện tại chưa hỗ trợ năm mắt dùng chung ba nhóm phát; không khai báo trùng emitter trong nhiều phần tử IR.

Nguồn motor cấp VM riêng đúng module, GND chung; không cấp motor từ GPIO/3.3 V. IR cần MOSFET và điện trở hạn dòng. Điện áp thu vào ADC phải trong dải MCU. MT6701 cần chế độ A/B, không dùng module chỉ xuất I²C/SSI với driver này. Xem bảng dây đầy đủ trong [CubeMX README](cubemx/README.md).

Các tham số chỉnh ở `test_config.hpp`: `kIrSettleUs` (ban đầu 80 µs, đo đáp ứng để chỉnh), `kReportMs` (200 ms), `kMotorDutyPercent` (12%), `kMotorPulseMs` (300 ms). Code giới hạn xung tối đa 500 ms và duty tối đa 20%; kiểm tra PWM, nguồn và ma sát trước khi tăng. Đổi dấu encoder bằng `sign` trong `test_config.cpp`, không đổi bảng chân.

Không cần comment các hàm driver. Chỉ đổi cờ module, một dòng chọn bài và thông số cần đo; firmware PID ở thư mục khác không bị ảnh hưởng.
