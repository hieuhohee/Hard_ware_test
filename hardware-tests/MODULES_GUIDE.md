# Test module rời khi chưa có PCB — không sửa dự án PID

Bạn có thể test một mắt IR, một encoder, một motor/AT8870 hoặc một ICM20602 chỉ với core board F405, module và dây nối. Không cần lắp đủ robot, không cần có thông số PI hay bảng hiệu chuẩn khoảng cách để đọc tín hiệu thô.

## 1. Tách project để tránh ảnh hưởng firmware chính

Tạo một project CubeIDE/CubeMX riêng, ví dụ `Mouse_Module_Bench`, chọn STM32F405RGT6. Giữ `micromouse-control/` nguyên vẹn. Copy năm file sau từ `hardware-tests/` vào project bench:

- `hardware_tests.cpp`, `hardware_tests.hpp`, `hardware_tests_api.h`.
- `test_config.cpp`, `test_config.hpp`.

Sau đó copy nội dung `examples/test_config.modules.cpp` **thay vào** `test_config.cpp` trong project bench. Không thêm cả hai file cấu hình vào build cùng lúc. Thư mục examples không phải source cần tự động compile.

Chọn C++11 hoặc mới hơn, link bằng C++ toolchain. Project bench không compile `micromouse_app.cpp`, `controller.cpp`, không gọi `MM_Init/MM_Poll`, không chạy callback TIM6/UART của dự án PID. Mỗi project có file `.ioc` và cấu hình riêng; thay chân thử nghiệm ở project bench không đổi thiết kế chính.

Nếu đã tạo project CubeMX cho robot, có thể tạo bản sao project để làm bench rồi loại source ứng dụng PID khỏi build của bản sao. Không chỉ comment `MM_Poll()` nhưng vẫn gọi `MM_Init()`: init vẫn có thể start timer/PWM, đăng ký UART và init sensor.

## 2. Chọn module bằng cờ trong test_config.cpp

Mẫu đã có các cờ, mặc định toàn bộ bằng 0:

```cpp
#define BENCH_USE_UART       0
#define BENCH_TEST_IR        0
#define BENCH_TEST_ENCODER_L 0
#define BENCH_TEST_ENCODER_R 0
#define BENCH_TEST_MOTOR_L   0
#define BENCH_TEST_MOTOR_R   0
#define BENCH_TEST_GYRO      0
```

Muốn test một IR: đặt `BENCH_TEST_IR=1`, còn lại giữ 0. Muốn test gyro: IR=0, GYRO=1. Muốn xem encoder lúc motor quay: bật đúng MOTOR_L và ENCODER_L. Có thể bật nhiều cờ, nhưng chỉ bật khối đã đấu dây. Cờ USE_UART độc lập, bật khi đã có USB–UART mức logic 3.3V.

Mẫu tự gán handle của khối không dùng thành `nullptr`. Với con trỏ rỗng, bộ test bỏ qua khối đó, không giao tiếp với IC chưa cắm. Motor được chọn chỉ khởi tạo PWM bằng 0; chỉ quay khi bạn gọi hàm TestMotor tương ứng. Nếu đặt hàm đó sau Begin, bài motor chạy lại mỗi lần boot/reset.

## 3. Gọi hàm chọn bài một lần, Update trong while

Xem [DIRECT_TEST_GUIDE.md](DIRECT_TEST_GUIDE.md) để copy đúng các vùng USER CODE vào main.c/main.cpp. Include `hardware_tests_api.h`; sau tất cả MX_*_Init gọi:

```c
HardwareTests_Begin();
HardwareTests_TestIR(); // Hoặc TestEncoders(), TestGyro(), CalibrateGyro()...
```

Trong while chỉ gọi `HardwareTests_Update()` liên tục, không HAL_Delay. Không cần nhận lệnh hay ghi biến command. Muốn đổi bài, thay dòng chọn hàm rồi build/nạp lại. Không đặt hàm chọn bài trong while, đặc biệt bài motor sẽ bị chạy lặp lại sau mỗi lần dừng.

## 4. CubeMX: phần MCU khác với module ngoài

`nullptr` chỉ bỏ qua thiết bị lúc chạy. `hardware_tests.cpp` vẫn được compile với các kiểu/hàm HAL ADC, TIM, SPI, UART, nên phải có header và source HAL của cả bốn nhóm trong build. Nếu chỉ bật SPI rồi xóa hết HAL_TIM/HAL_ADC/HAL_UART, có thể lỗi compile/link dù cờ chỉ test gyro.

Cách ít thay đổi nhất là giữ cấu hình ngoại vi MCU đầy đủ theo bảng dưới và gọi các `MX_*_Init()` đã sinh; chỉ giới hạn module ngoài bằng các cờ. Khởi tạo peripheral của MCU không bắt buộc IC ngoài phải có mặt. Không bật interrupt/DMA hay tự start PWM ngoài bộ test.

Clock baseline: HSI16, PLLM8/PLLN168/PLLP2/PLLQ7; SYSCLK168MHz, APB1=42MHz, timer APB1=84MHz, APB2=84MHz. SysTick timebase, Debug Serial Wire. Chưa biết thạch anh module thì không tự chọn HSE theo phỏng đoán.

| MCU peripheral | Cấu hình bench |
|---|---|
| TIM3 PB4/PB5 | CH1/2 PWM mode1 high, PSC0, ARR4199, pulse0 => 20kHz |
| TIM4 PB6/PB7 | Tương tự TIM3 |
| TIM8 PC6/PC7 | Encoder TI12, PSC0, ARR65535, IC1/2 DirectTI, polarity Rising, filter0 ban đầu |
| TIM5 PA0/PA1 | Encoder TI12, PSC0, ARR4294967295 |
| ADC1 PA3 | IN3, 12bit, right, single conversion, software trigger, scan/continuous/DMA off, clock21MHz, sampling144cycles |
| PC1 | GPIO output push-pull, mặc định LOW; điều khiển gate MOSFET phát IR |
| SPI2 PB13/PB14/PB15 | Master full duplex mode0,8bit,MSB,software NSS,prescaler64 (~656kHz) |
| PB12 | GPIO output push-pull, mặc định HIGH, CS gyro |
| USART3 PC10/PC11 | 115200,8N1,TX/RX,no flow; polling, không cần UART IRQ cho bộ bench này |

Chọn Generate peripheral initialization as a pair of .c/.h files để có `tim.h`, `adc.h`, `spi.h`, `usart.h`. Giữ HAL SysTick hoạt động cho timeout. Bộ bench dùng DWT để đo microsecond nên không cần TIM2/TIM6 như dự án PID. Nếu trong bản sao CubeMX vẫn có TIM2/TIM6 thì không start ngắt của chúng và không giữ callback ứng dụng PID.

## 5. Cách đấu và thử từng module

Chân dưới đây là tên GPIO, không phải số chân connector của module. Nối GND chung; kiểm tra điện áp cấp và logic của từng breakout theo đúng module bạn có. Nhãn VCC trên breakout không luôn đồng nghĩa có thể cấp 5V; một số module có regulator/level shifter, một số không.

### Một encoder MT6701

Chọn `BENCH_TEST_ENCODER_L=1`, gọi `HardwareTests_TestEncoders()` sau Begin. A → PC6, B → PC7; nguồn/GND theo module. Z chưa cần dùng. Đặt nam châm đúng cơ khí, quay từ từ, xem `bench::live.encoder_total[0]` tăng/giảm. Chỉ có encoder thì motor không cần nối.

`encoder_ready_mask=1` nghĩa chỉ encoder trái đã được bật; không phải lỗi vì thiếu encoder phải. Nếu module chỉ lộ SDA/SCL hay SSI, đây chưa phải ngõ A/B: cần xác nhận/chuyển chế độ đúng MT6701 hoặc viết driver giao tiếp đó. Không nối SDA/SCL vào PC6/PC7 rồi mong timer encoder hoạt động.

### Một IR

Chọn `BENCH_TEST_IR=1`, gọi `HardwareTests_TestIR()` sau Begin. Mạch phát: PC1 điều khiển MOSFET/transistor phù hợp và điện trở hạn dòng LED. Mạch thu analog đưa tín hiệu 0–3.3V vào PA3. Không cấp dòng LED công suất trực tiếp từ GPIO.

Quan sát `ir_off[0]`, `ir_on[0]`, `ir_difference[0]`. Đưa vật/tường gần xa và xoay hướng. Với mạch thu cho điện áp tăng khi nhận nhiều ánh sáng, `signal_sign=+1`; nếu đảo, đổi dấu sau khi đo thực tế.

Nếu bạn mua module IR có chân AO/DO: DO thường là ngõ so sánh nhị phân, không thay ADC analog để đo mức phản xạ. AO có thể dùng để đọc analog nếu điện áp phù hợp. Nhưng phép đo bật/tắt ở bộ test còn cần bạn điều khiển được emitter. Nếu LED trên module luôn được cấp nguồn và không có EN/gate, cấu hình PC1 không thể tự bật/tắt nó; cần mạch điều khiển emitter hoặc bài test ADC thô riêng. Không cắt toàn bộ nguồn module qua GPIO để giả lập chân EN.

Bộ bench cũ yêu cầu các kênh phát độc lập, mẫu này chỉ bật một mắt. Để thử năm mắt trên ba nhóm như PCB dự kiến, dùng driver nhóm ở dự án `micromouse-control` khi lắp đủ, không điền trùng emitter vào năm phần tử của bộ bench.

### ICM20602, chỉ gyro

Chọn `BENCH_TEST_GYRO=1`, gọi `HardwareTests_TestGyro()` sau Begin. CS→PB12, SCK→PB13, SDO/MISO→PB14, SDI/MOSI→PB15; nguồn và GND theo breakout. Chân INT không cần nối cho bài polling. Dây SPI nên ngắn, khởi đầu clock thấp như bảng.

Xem `imu_ready=1`, `imu_whoami=0x12`, `imu_samples` tăng. `gyro_raw[0..2]` là số thô, `gyro_dps[0..2]` là độ/giây. Muốn trừ bias, chọn `HardwareTests_CalibrateGyro()` thay cho TestGyro rồi giữ yên để lấy 1000 mẫu; bộ bench không tự biết bạn đang xoay nên phải giữ yên. WHO đúng nhưng samples không tăng thì kiểm tra data-ready và lỗi bus.

Trong lượt cập nhật này bộ bench đã được bổ sung bật DATA_RDY và latch INT bên trong ICM20602, đồng thời đọc lại thanh ghi. MCU vẫn đọc trạng thái qua SPI, không bật EXTI. Đây là cập nhật riêng của bộ bench, không sửa source PID.

Nếu module thật là **MPU6050/MPU6500/MPU9250**, không dùng nguyên driver ICM20602 bằng cách bỏ kiểm tra WHO_AM_I. Phải xác nhận mã chip và giao thức rồi thay driver phù hợp. Từ “MPU” không đủ để chọn driver.

### Một động cơ và một AT8870

Chọn `BENCH_TEST_MOTOR_L=1`. IN1→PB4, IN2→PB5, motor vào OUT1/OUT2; VM dùng nguồn motor đúng định mức, GND chung MCU. Không cấp motor từ chân 3.3V của F405/USB–UART. Nối dòng motor bằng dây/đầu nối phù hợp, không cho dòng motor đi qua đường breadboard tín hiệu yếu.

Nâng bánh/giữ cơ cấu chắc. Gọi `HardwareTests_TestMotorLeft(+1)` hoặc `(-1)` một lần sau Begin: mặc định 12%/300 ms, Update tự dừng. Motor phải dùng `HardwareTests_TestMotorRight(+1)` hoặc `(-1)`. Không cần encoder, gyro hoặc IR để test xung motor thô. Hàm `HardwareTests_Stop()` cho phép chương trình dừng bài. UART không nhận lệnh trong cách dùng mới.

Muốn kiểm tra motor + encoder: bật cả MOTOR_L và ENCODER_L. Trong xung motor, encoder vẫn được cập nhật vào Watch, log được hoãn đến sau xung. Bộ bench tạm ngưng IR/gyro trong xung; kiểm tra đồng thời nhiều sensor khi chạy liên tục thuộc dự án điều khiển mới.

## 6. Không có USB–UART thì dùng ST-Link

Giữ `BENCH_USE_UART=0`; chọn hàm test trong main như trên. Trong CubeIDE Debug/Live Expressions thêm `bench::live`; không cần sửa `bench::command`. Với bài motor, không đặt breakpoint giữa xung: PWM có thể vẫn chạy khi CPU dừng và Update không thể thực hiện timeout. Kết quả Watch là snapshot, không đảm bảo các trường được đọc đồng thời.

Nếu dùng USB–UART để xem log: RX adapter→PC10, GND chung, 115200/8N1, mức logic 3.3 V. Không cần nối TX adapter khi chỉ đọc log; .ioc vẫn giữ PC11 RX. Khi bo đã cấp nguồn riêng, không nối VCC adapter. Không cần gõ lệnh. Mẫu không dùng USB CDC.

## 7. Chuyển về dự án chính

1. Dừng, ngắt nguồn motor trước khi đổi dây; đặt các module về sơ đồ chân chính khi đã có PCB.
2. Chỉ chép lại kết quả đã đo: chiều encoder/motor, số count/vòng, gyro axis/sign, thời gian settle và dữ liệu IR.
3. Những kết quả đó được nhập vào `micromouse-control/platform/board_config.cpp` và thông số liên quan; không chép `main`/callback/menu bench sang project PID.
4. Build/nạp đúng project điều khiển, dùng lại hướng dẫn tune PI → giữ hướng → bám tường. Không xóa các khóa xác nhận chỉ vì sensor đã trả dữ liệu.

Mục tiêu của bench là xác minh tín hiệu thô. Chưa cần tuning PID, chưa cần chuyển IR ra cm và chưa cần lắp đủ robot.
