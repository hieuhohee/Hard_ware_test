# CubeMX cho hardware-tests — STM32F405RGT6

Mở **[micromouse_hwtest/micromouse_hwtest.ioc](micromouse_hwtest/micromouse_hwtest.ioc)**.
Đây là cấu hình dành riêng cho bộ test module rời C++ trong `hardware-tests`, theo bảng chân của `examples/test_config.modules.cpp`. Không dùng vòng lặp, callback hoặc cấu hình DMA của `micromouse-control` trong firmware này.

Bản gốc `Downloads/micromousev2.ioc` không bị sửa. Bản sao nguyên gốc được lưu tại [original/micromousev2.original.ioc](original/micromousev2.original.ioc), commit sao lưu **42a89e3**.

## 1. Bảng chân được giữ theo bộ test hiện tại

Tên PA0/PB4 là GPIO của MCU, không phải số thứ tự chân connector. Kiểm tra nhãn và chiều cắm core board trước khi đấu.

| Tín hiệu | Chân MCU | Cấu hình | Tần số / cách hoạt động |
|---|---|---|---|
| Motor trái IN1 / IN2 | PB4 / PB5 | TIM3 CH1 / CH2, AF2, PWM1 active HIGH | 20 kHz; PSC=0, ARR=4199; pulse ban đầu 0 |
| Motor phải IN1 / IN2 | PB6 / PB7 | TIM4 CH1 / CH2, AF2, PWM1 active HIGH | 20 kHz; PSC=0, ARR=4199; pulse ban đầu 0 |
| Encoder trái A / B | PC6 / PC7 | TIM8 CH1 / CH2, AF3, Encoder TI12 | Đếm x4; PSC=0, ARR=65535 |
| Encoder phải A / B | PA0 / PA1 | TIM5 CH1 / CH2, AF2, Encoder TI12 | Đếm x4; PSC=0, ARR=4294967295 |
| ICM20602 CS | PB12 | GPIO output push-pull | Mặc định HIGH, driver kéo LOW khi giao dịch |
| ICM20602 SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 AF5, master, full duplex, mode 0, 8 bit, MSB first, NSS software | PCLK1/64 = 656.25 kHz |
| ICM20602 INT | PB8 | GPIO input, no-pull | Dự phòng, không EXTI; driver đọc INT_STATUS qua SPI |
| Phát IR hông / chéo / trước | PC1 / PA5 / PC2 | GPIO output push-pull, LOW lúc khởi tạo | Điều khiển MOSFET bằng phần mềm; không phải ADC/PWM |
| Thu IR1 | PA3 | Analog, ADC1_IN3 | Rank 1 ban đầu |
| Thu IR2 / IR3 / IR4 / IR5 | PA4 / PB0 / PB1 / PC0 | GPIO Analog/no-pull; ADC1_IN4 / IN8 / IN9 / IN10 khi driver chọn | Driver đổi kênh rank 1 trước mỗi phép đo |
| UART debug TX / RX | PC10 / PC11 | USART3 AF7, asynchronous, polling | 115200 baud, 8N1, no flow control |
| Start / Mode | PC9 / PC8 | GPIO input/no-pull; theo mạch có pull-up ngoài | Đã giữ chân; bộ test hiện tại chưa xử lý nút |
| SWDIO / SWCLK | PA13 / PA14 | SYS → Serial Wire | ST-Link nạp và debug; PB4 được dùng cho PWM |

Encoder không có “tần số phát” do MCU đặt; tốc độ xung phụ thuộc encoder và tốc độ bánh. Filter đầu vào để 0 làm mốc thử. Chỉ tăng filter sau khi kiểm tra nhiễu, đồng thời bảo đảm bộ lọc vẫn nhận đủ xung ở tốc độ lớn nhất.

**Những chỗ khác file `.ioc` người dùng gửi:** motor trái chuyển từ PC6/PC7 về PB4/PB5; encoder trái từ PA6/PA7 về PC6/PC7; encoder phải giữ PA0/PA1 nhưng đổi TIM2 sang TIM5 và đặt A→PA0, B→PA1; phát IR đổi từ Analog sang Output; bổ sung IR5 PC0 và UART3 PC10/PC11; nút về PC9/PC8. Đây là khớp lại với bảng chân bộ test đã chốt, không phải khẳng định dây PCB trong ảnh đã nối như vậy. Nếu đang đấu theo file gốc thì phải đổi dây tương ứng trước khi test.

## 2. Clock và phương pháp đọc

- HSI 16 MHz → PLL M=8, N=168, P=2, Q=7 → SYSCLK/HCLK=168 MHz.
- APB1 chia 4: PCLK1=42 MHz; clock TIM3/TIM4/TIM5=84 MHz.
- APB2 chia 2: PCLK2=84 MHz; clock TIM8=168 MHz.
- PWM: `84 MHz / ((0+1)*(4199+1)) = 20 kHz`.
- Nhánh PLLQ=48 MHz danh định, nhưng USB chưa bật. HSI được chọn vì chưa xác nhận tần số thạch anh trên module; không suy ra HSE=25 MHz chỉ từ ô nhập trong ảnh CubeMX. Sai số HSI ảnh hưởng UART/PWM/thời gian; khi đã xác nhận thạch anh, có thể đổi nguồn PLL và tính lại để giữ các clock trên.
- ADC1: 12 bit, right aligned, PCLK2/4=21 MHz; software trigger, scan/continuous/discontinuous/DMA đều tắt; 1 conversion; sample time 144 cycles. Clock ADC 21 MHz không phải tốc độ xuất 21 triệu mẫu mỗi giây.
- Bốn mắt thu còn lại được đặt **GPIO_Analog** có chủ đích: driver `HAL_ADC_ConfigChannel()` chọn ADC channel và rank 1 khi chạy. Không bật scan 5 rank hoặc DMA; cách đó không khớp driver bench hiện tại.
- IR: driver đo khi LED tắt và bật, chờ ban đầu 80 µs theo `kIrSettleUs`. Không có tần số IR cố định trong `.ioc`; tốc độ khung phụ thuộc số mắt đang bật và vòng `poll()`.
- Gyro ±2000 °/s, DLPF 92 Hz, nguồn mẫu 1 kHz, tắt accel là cấu hình **trong driver C++**, không phải mục CubeMX. Polling/ghi log có thể bỏ qua mẫu; đây không phải cam kết log 1000 mẫu/s.
- HAL tick dùng SysTick. Bộ test dùng DWT đo thời gian ngắn, không dùng TIM2/TIM6, DMA hay ngắt EXTI/UART/timer riêng. UART in trạng thái khoảng 5 Hz theo `kReportMs`.

## 3. Mở và sinh mã

1. Để giữ thư mục nguồn sạch, copy cả thư mục `micromouse_hwtest` sang một thư mục làm firmware riêng, rồi mở file `.ioc` ở bản copy bằng CubeMX.
2. Xác nhận MCU là **STM32F405RGTx / STM32F405RGT6, LQFP64**, gói **STM32Cube FW_F4 V1.28.3**. Nếu CubeMX đề nghị migrate, lưu một bản mới rồi kiểm tra lại bảng chân và clock trước khi dùng.
3. Trong **Clock Configuration**, kiểm tra HSI là nguồn PLL và các giá trị ở mục 2. Trong **SYS**, Debug phải là **Serial Wire**.
4. Trong **Project Manager**, đặt Project Location tới thư mục firmware mới, tên `micromouse_hwtest`, Toolchain **STM32CubeIDE**. Chọn thư mục mới để tránh ghi vào firmware PID.
5. Trong **Code Generator**, giữ `Generate peripheral initialization as a pair of .c/.h files per peripheral`, `Keep User Code when re-generating` và backup file trước khi sinh lại. File đã đặt không xóa file sinh trước tự động.
6. Bấm **Generate Code**. Kết quả phải có `Core/Inc/{adc,tim,spi,usart,gpio}.h`, các file `.c` tương ứng, và `main.c` gọi đủ `MX_ADC1_Init`, `MX_SPI2_Init`, `MX_TIM3/4/5/8_Init`, `MX_USART3_UART_Init` sau GPIO/clock.
7. Mở/import project trong CubeIDE. Bật hỗ trợ C++ (Convert to C++ nếu project đang chỉ có C), dùng C++11 hoặc mới hơn và C++ linker. Mã do CubeMX sinh vẫn có thể là C; hai file triển khai bench phải giữ đuôi `.cpp`.

Không cần sửa PSC/ARR hay đổi GPIO thủ công để dùng bộ test này. Chỉ cấu hình ngoại vi chưa tạo ra firmware test hoàn chỉnh: tiếp tục mục 4.

## 4. Ghép code C++ và chọn module

Copy đúng các file sau vào project vừa sinh:

| File nguồn trong `hardware-tests` | File đích trong project CubeIDE |
|---|---|
| `hardware_tests.hpp` | `Core/Inc/hardware_tests.hpp` |
| `test_config.hpp` | `Core/Inc/test_config.hpp` |
| `hardware_tests.cpp` | `Core/Src/hardware_tests.cpp` |
| `examples/test_config.modules.cpp` | **`Core/Src/test_config.cpp`** |

Chỉ có **một** `test_config.cpp`; không compile thêm bản gốc rỗng hoặc bản example thứ hai. Thư mục `Core/Inc` phải nằm trong include path của C++.

Trong bản `Core/Src/test_config.cpp` vừa copy, đặt `BENCH_USE_UART=1` nếu có USB–UART. Đặt 1 cho đúng module đang cắm, giữ các cờ còn lại bằng 0. Ví dụ chỉ test gyro qua UART:

```cpp
#define BENCH_USE_UART       1
#define BENCH_TEST_IR        0
#define BENCH_TEST_ENCODER_L 0
#define BENCH_TEST_ENCODER_R 0
#define BENCH_TEST_MOTOR_L   0
#define BENCH_TEST_MOTOR_R   0
#define BENCH_TEST_GYRO      1
```

Mẫu mặc định có tất cả cờ bằng 0. Điều đó không phải lỗi `.ioc`: driver chủ động bỏ qua module chưa chọn. Các handle ADC/TIM/SPI/UART vẫn phải được CubeMX sinh để code compile được; không tắt các peripheral đó trong CubeMX chỉ vì chưa cắm module.

Trong `main.c`, thêm khai báo vào đúng vùng người dùng:

```c
/* USER CODE BEGIN PFP */
void HardwareTests_Init(void);
void HardwareTests_Poll(void);
void ModuleBench_SelectMonitor(void);
/* USER CODE END PFP */
```

Thêm sau toàn bộ `MX_*_Init()`:

```c
/* USER CODE BEGIN 2 */
HardwareTests_Init();
ModuleBench_SelectMonitor();  /* Tự xem cảm biến đã bật, không tự chạy motor. */
/* USER CODE END 2 */
```

Trong vòng lặp do CubeMX sinh:

```c
while (1)
{
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  HardwareTests_Poll();
}
/* USER CODE END 3 */
```

Không include header C++ vào `main.c`; dùng ba khai báo C ở trên. Không gọi thêm vòng điều khiển PID, không `HAL_Delay()` trong vòng lặp, không gọi `HardwareTests_Init()` lặp lại.

## 5. Test từng module và giới hạn IR hiện tại

1. Nối GND chung và nguồn logic đúng từng module. UART mức 3.3 V: **PC10→RX adapter, PC11←TX adapter**. ST-Link: PA13/PA14/GND và NRST nếu có. Xem chi tiết đấu nguồn trong [README của bộ test](../README.md).
2. Build firmware bằng ARM toolchain trong CubeIDE, nạp ST-Link, cho CPU Run. Mở terminal 115200, 8N1, no flow control rồi gửi `h`. Nếu không UART, xem `bench::live` và ghi mã ASCII vào `bench::command` bằng debugger.
3. Bật cờ encoder tương ứng, gửi `e`, xoay tay: count phải đổi. Nếu chiều tiến count âm, đổi `sign` của encoder đó trong cấu hình sang -1; không đổi timer/pin.
4. Bật gyro, gửi `g`: WHO_AM_I phải là `0x12`, số mẫu tăng. Gửi `c` khi đặt yên để lấy bias; driver chỉ dùng gyro.
5. Bật IR, gửi `i`: mẫu hiện tại chỉ dùng **một bộ phát PC1 + thu PA3** ở phần tử logic `ir[0]`. Theo dõi `ir_off`, `ir_on`, `ir_difference` khi đổi khoảng cách.
6. Với motor, nâng bánh khỏi bàn, đấu nguồn VM đúng phần cứng. Bật cờ bên muốn thử, gửi `1`/`2` cho trái, `3`/`4` cho phải: xung mặc định 12% trong 300 ms. `s` hoặc SPACE để dừng. Chỉ riêng bật cờ không tự quay motor.

**Năm mắt IR nhưng ba nhóm phát:** `.ioc` đã dành đủ cả 5 chân thu và 3 chân phát. Tuy nhiên driver bench hiện tại từ chối hai phần tử `ir[]` trùng chân phát; chưa có lịch đo 5 mắt theo 3 nhóm. Vì vậy bản cấu hình này dành cho test **từng mắt**: giữ chỉ `ir[0]` hoạt động và bốn phần tử còn lại unused, sửa cặp emitter/channel trong `test_config.cpp` theo dây thực tế rồi build lại.

Ví dụ nếu muốn thử mắt thu PC0 đang được chiếu bởi LED nhóm trước PC2:

```cpp
// Thay phần tử ir[0] trong nhánh #if BENCH_TEST_IR của mẫu:
{{GPIOC, GPIO_PIN_2}, ADC_CHANNEL_10, +1},
```

PC1/PA5/PC2 lần lượt dành cho hông/chéo/trước; chọn đúng nhóm chiếu cho mắt đang đo. Không suy ra nhóm phát của IR1…IR5 chỉ từ số thứ tự. Không điền cả 5 mắt với các chân phát trùng nhau rồi mong driver hiện tại đo được toàn bộ. Dùng đủ 5 mắt theo 3 nhóm cần thay lịch IR riêng, nằm ngoài thay đổi `.ioc` này.

PC8/PC9 là input no-pull theo sơ đồ có điện trở kéo lên ngoài. Nếu test nút rời không có điện trở ngoài thì đổi input pull-up trong CubeMX; code bench hiện tại không dùng nút để chạy motor. PB8 INT cũng không cần nối cho bài test gyro polling.

## 6. Phạm vi kiểm tra

Đã dùng STM32CubeMX 6.18.1 đọc file và sinh các file khởi tạo C với thư viện F4 V1.28.3; đối chiếu code sinh ra với bảng chân, clock, ADC polling, PWM, encoder, SPI và UART của mẫu module. Code sinh thử được để riêng trong thư mục tạm của workspace, không trộn vào nguồn bench.

Đây là kiểm tra cấu hình và code khởi tạo, chưa phải kiểm chứng bằng ARM build hay đo trên phần cứng. Khi nạp thực tế vẫn cần kiểm tra nguồn, chân connector, mức tín hiệu, chiều motor/encoder và kết quả từng module.
