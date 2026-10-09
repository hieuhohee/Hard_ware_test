# Bộ test phần cứng Micromouse — C++ / STM32F405 HAL

Dành cho **2 AT8870 + 2 motor N20 + 2 MT6701 ở chế độ A/B + gyro ICM20602 qua SPI + 5 IR analog**.
ICM20602 chỉ dùng gyro XYZ; ba trục accelerometer được đưa vào standby, không đọc hoặc xuất dữ liệu accel.
Đây là module chẩn đoán để thêm vào một project STM32CubeIDE/CubeMX, chưa phải file HEX độc lập.
Mọi chân và handle đều để trống trong `test_config.cpp`; không dùng sơ đồ chân của SINDiB hay của project khác.

**Chưa có PCB, muốn thử từng module rời:** xem [MODULES_GUIDE.md](MODULES_GUIDE.md) và mẫu [test_config.modules.cpp](examples/test_config.modules.cpp). Dùng project bench riêng; không sửa vòng lặp của dự án PID. Mẫu cấu hình phải thay cho `test_config.cpp`, không compile cả hai cùng lúc.

## Kết nối phần cứng thực và bắt đầu test module rời

Phần này dùng mẫu `examples/test_config.modules.cpp`, với các cờ chọn module; `test_config.cpp` gốc vẫn để trống. Chỉ cắm và bật cờ cho phần cứng muốn thử. Các lệnh bên dưới thuộc **firmware bench**, khác với menu của dự án PID: `e` ở đây là xem encoder, không phải arm; `1` trực tiếp chạy xung motor, không cần `b`.

### Nguồn và đầu nối máy tính

1. Kiểm tra điện áp cấp, mức logic và pinout của từng module. F405 dùng logic 3.3V; ADC chỉ nhận tín hiệu trong dải cho phép. Không mặc định breakout có regulator hoặc chuyển mức chỉ vì có chân VCC.
2. Cấp nguồn core board qua đường nguồn được nhà sản xuất module cho phép. Chưa xác minh mạch nguồn USB-C thì dùng một đường cấp nguồn logic đã kiểm tra, không đấu song song USB và nguồn ngoài một cách mặc định.
3. Nguồn motor nối vào VM của AT8870, motor nối OUT1/OUT2. Không lấy nguồn motor từ GPIO, chân 3.3V hoặc USB–UART. Dòng motor phải đi qua dây/đầu nối phù hợp, tránh đường breadboard tín hiệu.
4. Nối GND chung MCU/module/driver. Tắt nguồn khi thay dây; thử cảm biến với nguồn motor tắt trước.

| Thiết bị | Nối F405 | Mục đích |
|---|---|---|
| ST-Link SWDIO / SWCLK | PA13 / PA14 | Nạp và debug, chọn Serial Wire |
| ST-Link GND | GND | Mốc chung |
| ST-Link NRST nếu có | NRST / NR | Reset khi nạp/debug |
| ST-Link VTref nếu có | Rail logic 3.3V | Tham chiếu điện áp; kiểm tra đúng pinout ST-Link, không nhầm chân cấp nguồn |
| USB–UART RX / TX | PC10 / PC11 | TX MCU → RX adapter, RX MCU ← TX adapter |
| USB–UART GND | GND | Khi bo đã cấp nguồn riêng chỉ nối TX/RX/GND, không cần VCC adapter |

Dùng USB–UART **tín hiệu 3.3V**, không chỉ nhìn vị trí jumper nguồn VCC của adapter. Nếu không dùng UART, để `BENCH_USE_UART=0` và xem `bench::live` qua ST-Link. Firmware không có USB CDC; USB-C trên core board không tự thay giao tiếp UART này. Tham khảo [ST-Link UM1075](https://www.st.com/resource/en/user_manual/um1075-stlinkv2-incircuit-debuggerprogrammer-for-stm8-and-stm32-stmicroelectronics.pdf) để phân biệt SWD, nguồn và điện áp tham chiếu.

### Dây từng module theo mẫu cấu hình

| Module muốn thử | Cờ đặt 1 | Nối tín hiệu | Kết quả cần xem |
|---|---|---|---|
| Một IR analog | `BENCH_TEST_IR` | PC1 → gate mạch phát; điện áp thu → PA3 | `ir_off[0]`, `ir_on[0]`, `ir_difference[0]` |
| MT6701 trái A/B | `BENCH_TEST_ENCODER_L` | A→PC6, B→PC7 | `encoder_total[0]` |
| MT6701 phải A/B | `BENCH_TEST_ENCODER_R` | A→PA0, B→PA1 | `encoder_total[1]` |
| AT8870 + motor trái | `BENCH_TEST_MOTOR_L` | IN1→PB4, IN2→PB5 | Đúng motor/chiều, `motor_active` rồi về 0 |
| AT8870 + motor phải | `BENCH_TEST_MOTOR_R` | IN1→PB6, IN2→PB7 | Như trên |
| ICM20602 SPI | `BENCH_TEST_GYRO` | CS→PB12, SCK→PB13, SDO/MISO→PB14, SDI/MOSI→PB15 | WHO=0x12, samples tăng, gyro XYZ đổi khi xoay |

Nguồn/GND từng breakout phải đấu thêm theo đúng module. Chân INT gyro và Z encoder chưa cần dùng. Tên PC6/PB12 là GPIO MCU, không phải số chân connector; kiểm tra nhãn trên core board và continuity khi chưa chắc chiều cắm.

IR phát phải có tầng MOSFET/transistor và điện trở hạn dòng; không cấp LED công suất trực tiếp bằng PC1. Module chỉ có DO không thay ADC analog; module LED luôn sáng cần cách điều khiển emitter trước khi dùng phép đo on/off. MT6701 chỉ lộ I²C/SSI chưa phù hợp driver A/B. Module mang tên MPU6050/6500/9250 cần driver riêng, không thay ICM20602 bằng cách bỏ kiểm tra ID.

### Từ project đến phiên test thật

1. Tạo project CubeIDE bench F405 riêng theo [MODULES_GUIDE.md](MODULES_GUIDE.md). Copy bốn file C++, thay nội dung file cấu hình bằng mẫu và chọn cờ. Giữ các thư viện HAL cần thiết dù module ngoài bị tắt bằng `nullptr`.
2. Cấu hình CubeMX, gọi `HardwareTests_Init()` một lần sau các `MX_*_Init()`; gọi `ModuleBench_SelectMonitor()` một lần nếu dùng mẫu mới. Trong `while(1)` chỉ gọi `HardwareTests_Poll()` liên tục, không delay. Chi tiết wrapper C/C++ ở mục 3 bên dưới.
3. Build bằng ARM toolchain của CubeIDE, nạp qua ST-Link rồi cho CPU Run. Executable CMake host trên PC không phải firmware nạp được.
4. Nếu dùng UART, đặt `BENCH_USE_UART=1`, mở COM1152008N1,no flow, gửi `h`. Ready mask bằng 1 cho riêng khối trái/IR đầu tiên là bình thường; không cần đủ motor/encoder mask3 hoặc IR mask31 khi chỉ cắm một module.
5. Test encoder bằng tay trước, rồi IR gần/xa, gyro đứng yên/xoay. Không cần nguồn motor cho các bài này. Nếu không dùng UART, mẫu đã tự mở monitor cho cảm biến được bật.
6. Test motor sau: nâng bánh, cấp VM đúng định mức, gửi một lệnh `1`/`2`/`3`/`4`. Xung mặc định 12%/300ms; gửi `s` hoặc SPACE để dừng. Nếu chưa quay, đo nguồn/PWM và kiểm tra ma sát trước khi tăng duty.
7. Kết thúc: dừng, ngắt nguồn motor trước khi tháo dây; ngắt nguồn logic trước khi thay module. Ghi lại dấu chiều, số đếm, điện áp và kết quả đo để chuyển sang cấu hình PID sau này.

| Muốn kiểm tra | Lệnh UART | Nếu không có UART: ghi vào `bench::command` |
|---|---|---:|
| IR | `i` | 105 |
| Encoder | `e` | 101 |
| Gyro | `g` | 103 |
| Zero gyro, giữ yên trong quá trình lấy mẫu | `c` | 99 |
| Motor trái +/- | `1` / `2` | 49 / 50 |
| Motor phải +/- | `3` / `4` | 51 / 52 |
| Dừng | `s` | 115 |

Ghi lệnh một lần, không ghi lặp trong while. Nếu debugger cần halt để sửa biến, chỉ halt khi các output đã dừng, đặt command rồi Resume. Không halt/single-step giữa xung motor/IR vì timeout cần CPU chạy; chuẩn bị cách cắt nguồn motor trực tiếp. Khác firmware PID, bộ bench này chưa có watchdog TIM6 cắt PWM khi main treo.

### Chẩn đoán khi chưa thấy hoạt động

| Hiện tượng | Kiểm tra |
|---|---|
| Compile báo thiếu kiểu/hàm HAL | CubeMX chưa đưa HAL TIM/ADC/SPI/UART vào build; tắt cờ module không loại mã driver |
| Không có log | Cờ UART, đúng COM, TX/RX nối chéo, GND,baud, MCU đang Run; không dùng RX_IT của firmware PID cùng UART polling bench |
| READY của khối cần test bằng 0 | Cờ/chân/handle chưa điền, ngoại vi chưa init, trùng PWM/emitter hoặc gyro sai ID/lỗi bus |
| Encoder không đổi | A/B đúng chân, đúng mode timer, nguồn, nam châm; xác nhận module thực sự xuất A/B |
| IR off/on giống nhau | LED có thật sự được PC1 điều khiển, đúng cực LED, đúng ngõ AO, vật nằm trong hướng phát/thu |
| IR sát 4095 | Điện áp/bão hòa/ánh sáng nền; chưa coi số đó là khoảng cách hợp lệ |
| WHO đúng nhưng samples không tăng | Xem `io_errors`, data-ready, cấu hình SPI; bộ bench hiện đã bật DATA_RDY trong ICM |
| Motor làm MCU reset hoặc SPI lỗi | Nguồn sụt, đường hồi dòng, tụ và dây tín hiệu gần dây motor |

Các mask ready cho biết driver được khởi tạo/chấp nhận cấu hình, không chứng minh toàn bộ dây và chất lượng tín hiệu đã đúng. Phải kiểm tra bằng chuyển động/vật phản xạ thực tế. Dòng motor, timing analog và sai số đo chưa được xác nhận trên bo thật.

## 1. File và nơi cần sửa

| File | Công dụng |
|---|---|
| `test_config.hpp` | Kiểu cấu hình và thông số test |
| `test_config.cpp` | **Điền toàn bộ chân/handle/kênh tại đây** |
| `hardware_tests.hpp` | API, biến `bench::live` và `bench::command` |
| `hardware_tests.cpp` | Motor, IR, encoder, driver ICM20602 và menu |

Ví dụ cú pháp (chỉ minh họa, KHÔNG phải sơ đồ đấu dây đã chọn):

```cpp
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1, htim2, htim5;

// Trong namespace bench, thay các định nghĩa có sẵn:
UART_HandleTypeDef* console = &huart2;
ADC_HandleTypeDef* ir_adc = &hadc1;
SPI_HandleTypeDef* imu_spi = &hspi1;
Pin imu_cs = {IMU_CS_GPIO_Port, IMU_CS_Pin};
Motor motors[2] = {
  {{&htim1,TIM_CHANNEL_1},{&htim1,TIM_CHANNEL_2}},
  {{&htim1,TIM_CHANNEL_3},{&htim1,TIM_CHANNEL_4}}
};
Encoder encoders[2] = {{&htim2,+1},{&htim5,+1}};
// Mỗi phần tử IR: { {port phát, pin phát}, kênh ADC thu, dấu tín hiệu }.
```

Có thể dùng tên GPIO do CubeMX sinh như `IR_LEFT_GPIO_Port`, `IR_LEFT_Pin`;
cũng có thể điền trực tiếp `GPIOA`, `GPIO_PIN_4`. Pin là bitmask `GPIO_PIN_x`, không phải số thứ tự x.
Không định nghĩa lại các handle HAL, chỉ dùng `extern` hoặc include header CubeMX tương ứng.
Không sửa cấu hình lúc đang chạy; dừng, build và reset lại sau khi sửa.

## 2. Cấu hình CubeMX bắt buộc

### Motor AT8870

- Mỗi motor cần một cầu H độc lập. Hai motor cần hai AT8870 (hoặc phần cứng tương đương).
- Mỗi IN1/IN2 dùng một ngõ PWM: tổng cộng 4 kênh riêng biệt.
- Timer PWM mode 1, up-counting, active HIGH; pulse khởi tạo 0; khoảng 20 kHz nếu đúng phiên bản driver hỗ trợ.
- IN1 có PWM, IN2=0 cho chiều +; IN1=0, IN2 có PWM cho chiều -; cả hai 0 để dừng/coast.
- Chiều '+' chỉ là quy ước điện; kiểm tra chiều cơ khí rồi đổi dây hoặc đổi cấu hình IN1/IN2.
- Đây không phải driver cho board motor có giao diện EN+DIR. Kiểm tra truth table đúng AT8870 đang có trước khi đấu.
- Cài giới hạn dòng VREF/ISEN và nguồn motor theo datasheet phần cứng. Module này không điều chỉnh dòng hay đo stall.

### Encoder MT6701

- Cấu hình **MT6701 xuất ABZ**, dùng A/B vào CH1/CH2 của hai timer khác nhau.
- Timer Encoder Mode TI12, Prescaler=0, ARR=65535 hoặc 4294967295 nếu timer hỗ trợ 32 bit.
- Không chia sẻ timer với PWM. Mức logic encoder phải phù hợp STM32.
- Nếu module đang ở I2C/SSI thì chỉ điền chân không đủ: cần đổi chế độ MT6701. Bản này không chứa driver SSI/I2C MT6701.
- `sign=+1/-1` đổi chiều đếm; không ảnh hưởng PWM.
- Count/vòng phụ thuộc cấu hình PPR của MT6701 và chế độ giải mã. Chưa quy đổi mm vì bạn chưa chốt PPR, bánh, tỉ số truyền/vị trí encoder.

### IR

- ADC1: 12 bit, right alignment, scan OFF, continuous OFF, software trigger, 1 conversion; clock trong giới hạn datasheet.
- Chân thu ở analog/no-pull, điện áp trong dải ADC; không đưa đầu thu 5 V trực tiếp vào ADC.
- Năm chân phát GPIO output push-pull, mặc định LOW, MOSFET active HIGH.
- Mỗi kênh phải có chân phát độc lập và kênh ADC riêng. Code từ chối pin phát hoặc ADC bị lặp giữa các kênh đã điền.
- Nếu thực tế chỉ có 3 MOSFET điều khiển nhóm 5 LED, cần điều chỉnh lịch theo nhóm trước khi dùng đủ 5 kênh. Bản này không tự giả định cách ghép nhóm.
- Thời gian chờ 80 us chỉ là điểm bắt đầu. Đo đầu thu để chọn settling phù hợp; dòng LED vẫn phải nằm trong giới hạn cả khi sáng kéo dài.

### ICM20602

- SPI master, full-duplex, 8-bit, MSB first, mode 0, NSS software; bắt đầu ở clock <=1 MHz.
- CS GPIO output HIGH từ lúc khởi tạo; không dùng chân CS chung với LED hay motor.
- Init kiểm tra WHO_AM_I=0x12, reset, đọc lại cấu hình.
- Gyro ±2000 deg/s, DLPF 92 Hz, nguồn mẫu 1 kHz. PWR_MGMT_2=0x38 tắt ba trục accel và giữ ba trục gyro hoạt động.
- Code đọc INT_STATUS để chỉ lấy mẫu mới, rồi burst 6 byte gyro XYZ từ thanh ghi 0x43. Không cần chân INT cho bài test này.
- Không dùng cho MPU6050/MPU6500 bằng cách chỉ bỏ kiểm tra ID: cần driver đúng chip.

### UART/debug

- UART 115200, 8 data bits, no parity, 1 stop bit, TX/RX, không flow control.
- Nếu không dùng UART, giữ `console=nullptr`, dùng biến Watch.
- Không chạy đồng thời firmware điều khiển cũ, DMA ADC cũ hoặc task khác cùng dùng các ngoại vi này.

## 3. Ghép vào main của CubeMX

Copy bốn file `.hpp/.cpp` vào project, thêm thư mục vào include path C++.
Bật C++11 hoặc mới hơn, bảo đảm bước link dùng trình liên kết C++ của toolchain.
Giữ nguyên phần khởi tạo HAL/clock và gọi các `MX_*_Init()` trước khi gọi test.

Nếu `main.c` vẫn là C, KHÔNG include header C++ vào đó. Khai báo wrapper:

```c
/* USER CODE BEGIN PFP */
void HardwareTests_Init(void);
void HardwareTests_Poll(void);
/* USER CODE END PFP */

/* Sau toàn bộ MX_GPIO_Init, MX_TIMx_Init, MX_ADC1_Init, MX_SPIx_Init, MX_USARTx_UART_Init: */
HardwareTests_Init();

while (1) {
    HardwareTests_Poll();
    /* Không HAL_Delay, không gọi controller cũ ở đây. */
}
```

Wrapper đã được định nghĩa `extern "C"` trong `.cpp`, nên liên kết được với main.c.
Nếu main.cpp, include `hardware_tests.hpp` và gọi `bench::init()` / `bench::poll()` trực tiếp.
Giữ nguyên SysTick của HAL để HAL_GetTick/HAL_Delay/timeout hoạt động.
Module dùng DWT CYCCNT để đo microsecond; không dùng SysTick ISR để chạy bài test.

## 4. Lệnh test

| Phím UART | Ghi số vào `bench::command` | Chức năng |
|---|---:|---|
| h | 104 | Menu và mask các khối sẵn sàng |
| s hoặc Space | 115 hoặc 32 | Dừng motor, tắt LED IR, dừng monitor/hiệu chuẩn |
| 1 | 49 | Motor trái chiều +, 12%, 300 ms |
| 2 | 50 | Motor trái chiều -, 12%, 300 ms |
| 3 | 51 | Motor phải chiều +, 12%, 300 ms |
| 4 | 52 | Motor phải chiều -, 12%, 300 ms |
| i | 105 | Monitor IR |
| e | 101 | Monitor encoder |
| g | 103 | Monitor gyro XYZ của ICM20602 |
| a | 97 | Monitor cả ba nhóm cảm biến; motor vẫn dừng |
| c | 99 | Lấy bias gyro từ 1000 mẫu mới, timeout 5 s |
| r | 114 | Dừng và khởi tạo lại tất cả khối |

Mỗi lệnh motor chỉ chạy MỘT motor. Lệnh motor mới khi đang chạy bị bỏ qua, không kéo dài xung.
Các lệnh cảm biến chuyển chế độ và dừng motor trước; khi motor đang chạy chỉ lệnh s/Space được xử lý.
`h` in ready mask: bit0=khối 0, bit1=khối 1... Motor/encoder đủ là 3; IR đủ là 31; IMU ready là 1.
UART log 5 Hz, không phải tần số lấy mẫu. DWT state machine IR phụ thuộc tốc độ main loop, không cam kết 1 kHz.

## 5. Trình tự đo và kết quả mong đợi

1. **Encoder trước:** gõ e, xoay từng bánh bằng tay. Chỉ count bên đó đổi; xoay ngược thì count đổi dấu. Raw có thể tràn, total tiếp tục liên tục. Không tự reset bộ đếm khi đang đo.
2. **IR:** gõ i, đưa tường trắng gần/xa từng mắt. Xem off, on và diff. Chọn signal_sign để diff dương khi phản xạ tăng. Off/on sát 0 hoặc 4095 có thể là bão hòa; diff âm giữ lại để chẩn đoán, không bị che bằng clamp.
3. **Gyro:** gõ g. WHO phải là 0x12, samples tăng. Để yên, gyro gần 0 nhưng có bias. Xoay quanh từng trục để kiểm tra đúng trục và dấu; quay ngược thì tốc độ góc đổi dấu. UART in raw XYZ; `live.gyro_dps[0..2]` là tốc độ góc X/Y/Z đã trừ bias, đơn vị deg/s. Không còn biến hoặc log accel.
4. **Bias:** để robot đứng yên, gõ c. Code không tự phát hiện robot đang bị xoay; kết quả chỉ hợp lệ nếu giữ yên trong cả quá trình. Bias chỉ lưu RAM, không ghi Flash. UART in gyro raw; xem `live.gyro_dps` để thấy giá trị đã trừ bias.
5. **Motor:** nhấc bánh khỏi mặt bàn, gõ từng lệnh 1..4. Kiểm tra đúng bánh/chiều và count trong Watch. Encoder luôn được lấy mẫu cả trong xung motor. Motor tự dừng sau 300 ms nếu main loop còn chạy. Nếu 12% chưa quay, kiểm tra điện/cơ trước; có thể tăng hằng số lên tối đa 20% sau khi xác nhận.

**Giới hạn thực tế:** timeout motor là phần mềm. Breakpoint/halt CPU hoặc firmware treo có thể để PWM tiếp tục chạy, tùy cấu hình debug freeze. Không đặt breakpoint trong lúc cấp motor/LED xung; để sẵn công tắc nguồn khi test. Đây là test phần cứng chứ không phải bảo vệ quá dòng độc lập.

IR dùng ADC polling có timeout ngắn và state machine chờ settling; IMU dùng SPI polling có timeout.
Không có PI/PID, tự lái, quy đổi khoảng cách IR hoặc góc yaw: mục đích là kiểm chứng từng tín hiệu thô trước.
Xem `bench::live.io_errors`; nếu tăng thì kiểm tra bus/config, không coi dữ liệu cũ là mẫu mới.
Nếu UART mất dữ liệu khi gửi dồn, dùng từng phím một; đây không phải giao thức điều khiển streaming.

## 6. Tài liệu đối chiếu

- ICM20602: https://invensense.tdk.com/wp-content/uploads/2020/11/DS-000176-ICM-20602-v1.1.pdf
- MT6701: https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf
- API và chân alternate function: reference manual/datasheet đúng STM32F405/package của bạn.

Kiểm thử trên máy tính dùng HAL giả chỉ xác minh logic; không thay thế build project CubeMX và đo trên board thật.
