# CubeMX và ghép mã C++ vào STM32F405RGT6

## 1. Project và clock

Chọn MCU **STM32F405RGT6 / LQFP64**, tạo project STM32CubeIDE, bật `Generate peripheral initialization as a pair of .c/.h files per peripheral`, giữ `Keep User Code when re-generating`. Các nhãn GPIO là tên gợi nhớ, không đổi chức năng chân.

Chưa xác nhận tần số thạch anh của core board: dùng HSI làm baseline bench. PLLM=16, PLLN=336, PLLP=2, PLLQ=7; SYSCLK=168MHz, AHB=/1, APB1=/4=42MHz, APB2=/2=84MHz. Timer APB1=84MHz, timer APB2=168MHz. Chọn cấu hình nguồn/Flash hợp lệ theo CubeMX; để công cụ sinh mã voltage scaling và latency. SYS Debug=`Serial Wire`, giữ PA13/PA14; HAL timebase=`SysTick`.

HSI có sai số; thay HSE đúng tần số thật sau khi kiểm tra module. Không chọn tần số crystal theo ảnh/vỏ PCB. USB chưa sử dụng trong project này; giữ PA11/PA12 trống.

## 2. Bảng tín hiệu thống nhất

Đây là tên GPIO MCU, **không phải số chân connector J97/J100**. Kiểm tra chiều cắm module/netlist trước khi sửa PCB.

| Tín hiệu | GPIO / ngoại vi | Giao thức / cấu hình | Tần số / thời gian |
|---|---|---|---|
| AT8870 trái IN1 | PB4 / AF2 TIM3_CH1 | PWM mode1, active high | 20kHz |
| AT8870 trái IN2 | PB5 / AF2 TIM3_CH2 | PWM mode1, active high | 20kHz |
| AT8870 phải IN1 | PB6 / AF2 TIM4_CH1 | PWM mode1, active high | 20kHz |
| AT8870 phải IN2 | PB7 / AF2 TIM4_CH2 | PWM mode1, active high | 20kHz |
| MT6701 trái A/B | PC6/PC7 / AF3 TIM8_CH1/2 | Encoder TI1 and TI2, x4 | Theo cơ khí/PPR, đọc counter 1kHz |
| MT6701 phải A/B | PA0/PA1 / AF2 TIM5_CH1/2 | Encoder TI1 and TI2, x4 | Theo cơ khí/PPR, đọc counter 1kHz |
| ICM CS | PB12 GPIO output | Software NSS, idle HIGH | Mỗi giao dịch SPI |
| ICM SCK/MISO/MOSI | PB13/PB14/PB15 / AF5 SPI2 | Master full duplex, mode0, MSB, 8bit | 656.25kHz ban đầu |
| ICM INT | PB8 input | Để dành, bản này polling data-ready bằng SPI | Không bật EXTI |
| Phát hông Q6, IR1/IR5 | PC1 GPIO output | Active HIGH, idle LOW | Quét nhóm trong frame IR |
| Phát chéo Q8, IR2/IR4 | PA5 GPIO output | Active HIGH, idle LOW | Như trên |
| Phát trước Q7, IR3 | PC2 GPIO output | Active HIGH, idle LOW | Như trên |
| Thu IR1 | PA3 ADC1_IN3 | Analog/no pull | Mỗi frame đọc off và on |
| Thu IR2 | PA4 ADC1_IN4 | Analog/no pull | Như trên |
| Thu IR3 | PB0 ADC1_IN8 | Analog/no pull | Như trên |
| Thu IR4 | PB1 ADC1_IN9 | Analog/no pull | Như trên |
| Thu IR5 | PC0 ADC1_IN10 | Analog/no pull | Như trên |
| Start / Mode | PC9 / PC8 GPIO input | Active LOW; no pull nếu có 10k ngoài | Debounce 20ms |
| Debug TX/RX | PC10/PC11 / AF7 USART3 | 115200, 8N1, ngắt TX/RX | Log 10Hz |
| Đồng hồ microsecond | TIM2 internal, không GPIO | 32bit, 1 count/us | 1MHz |
| Nhịp scheduler | TIM6 internal, không GPIO | Update interrupt | 1kHz |
| SWDIO/SWCLK | PA13/PA14 | Serial Wire | Theo ST-Link |

PB4 từng là chân JTAG: chọn Serial Wire để dùng PB4 cho PWM. PB8 và PC8 cùng số EXTI8: hiện cả hai không cần EXTI, tránh bật đồng thời.

## 3. Timer PWM

TIM3 và TIM4: clock Internal, CH1/CH2 PWM Generation, Counter Up, PSC=0, ARR=4199, CKD=DIV1, auto-reload preload enable. Kênh PWM mode1, pulse=0, polarity HIGH, Fast Mode disable. GPIO đúng bảng, AF push-pull, no pull. Không bật NVIC timer PWM.

`fPWM=84,000,000/((0+1)*(4199+1))=20,000Hz`. Bốn kênh đều bắt đầu CCR=0. Không dùng PWM complementary/dead-time của TIM1/TIM8 cho AT8870. Driver C++ giữ hai input bằng 0 trong ít nhất 100us khi đảo cực. Hai input 0 là coast; không dùng phanh chủ động trong bản này.

Nếu đổi clock, tính lại PSC/ARR. Nếu đổi ARR lên giá trị quá lớn, xem lại độ phân giải timer và kiểu `ARR+1` trong driver. Bản này dùng PWM TIM3/TIM4 16bit.

## 4. Timer encoder

TIM8: Combined Channels=`Encoder Mode`, mode=`TI1 and TI2`; PC6/PC7. PSC=0, ARR=65535. TIM5 tương tự ở PA0/PA1, ARR=4294967295. IC1/IC2 DirectTI, prescaler DIV1, polarity Rising/non-inverted; filter=0 ban đầu. TI12 vẫn đếm x4, không phải chỉ đếm hai cạnh lên.

Không dùng timer encoder làm PWM hoặc timer scheduler. Không bật interrupt từng cạnh encoder; phần cứng tự đếm. Có nhiễu thì xử lý nguồn/dây trước, thử filter mức nhỏ và đối chiếu số đếm ở tốc độ cao. Không mặc định filter lớn nhất. Tần số A/B phụ thuộc PPR và trục gắn nam châm; `1kHz` là tốc độ đọc counter, không phải giới hạn tần số encoder.

## 5. Timer thời gian

TIM2 Internal Clock: PSC=83, ARR=4294967295, Up, DIV1, không chọn channel GPIO, không NVIC. Counter 1MHz tràn khoảng 71.6 phút; code dùng hiệu unsigned xử lý một vòng tràn.

TIM6 Internal Clock: PSC=83, ARR=999, Up, enable `TIM6 global interrupt` (IRQ TIM6_DAC). Update 1kHz. **HAL timebase vẫn là SysTick**, không chọn TIM6 làm timebase của HAL.

Gợi ý ưu tiên NVIC: SysTick preemption 0, TIM6_DAC 1, USART3 3, PriorityGroup4. Giá trị nhỏ hơn có ưu tiên cao hơn. TIM6 callback chỉ phát tick và kiểm tra watchdog; không chạy SPI/ADC/PID ở ngắt. Không dùng RTOS trong bản này. Nếu thêm RTOS, cần thiết kế lại ownership và giới hạn ưu tiên syscall.

## 6. SPI gyro

SPI2 Full-Duplex Master, hardware NSS disable/software NSS, 8bit, MSB first, CPOL Low, CPHA 1Edge, prescaler64 → 42MHz/64=656.25kHz. TI mode/CRC disable. PB12 GPIO push-pull idle HIGH; không chọn hardware NSS.

Không bật SPI DMA/NVIC cho bản polling ngắn này. C++ đọc data-ready, rồi 6 byte gyro XYZ; bus timeout 1ms. ICM20602 được xác minh WHO_AM_I, reset và cấu hình gyro trong driver, không phải trong CubeMX. Accel standby, gyro ±2000dps, lọc 92Hz, nguồn mẫu 1kHz; chỉ một trục được dùng làm yaw theo `gyro_axis` và `gyro_sign`.

Driver bật nguồn sự kiện DATA_RDY bên trong ICM (`INT_ENABLE=0x01`), INT dạng latch và chỉ clear khi đọc `INT_STATUS`. Điều này khác với bật EXTI ở STM32: PB8 vẫn chỉ là GPIO input và MCU vẫn polling qua SPI.

Sau khi đo bus tốt có thể tăng SPI prescaler lên /32 hoặc /16 (tức clock nhanh hơn), trong giới hạn datasheet và đường mạch. Mục tiêu là giảm thời gian bus, không thay đổi ODR gyro.

## 7. ADC và phát IR

ADC1 Independent, 12bit, right alignment, PCLK2/4=21MHz. Scan disable, Continuous disable, Discontinuous disable, 1 regular conversion, software start, no external trigger, DMA disable. Rank1 ban đầu ADC_CHANNEL_3/PA3, sampling time 144 cycles. PA4/PB0/PB1/PC0 đặt GPIO Analog/no pull; code chuyển regular rank1 sang từng channel.

Không để CubeMX tự tạo 5 rank scan: adapter hiện đọc từng channel. Mỗi phép đổi xấp xỉ `(144+12)/21MHz=7.43us`, chưa tính HAL và trở kháng nguồn. 10 lần đổi/frame × 500 frame/s = khoảng 5000 conversion/s, khác với ADC clock 21MHz.

PC1/PA5/PC2 GPIO Output Push Pull, LOW, no pull, low speed. Trình tự thực tế:

1. Tắt tất cả, chờ 80us, đọc off IR1/IR5; bật hông, chờ 80us, đọc on IR1/IR5, tắt.
2. Chờ 80us, đọc off IR2/IR4; bật chéo, chờ 80us, đọc on IR2/IR4, tắt.
3. Chờ 80us, đọc off IR3; bật trước, chờ 80us, đọc on IR3, tắt.
4. Publish đủ 5 kết quả; frame kế bắt đầu không sớm hơn chu kỳ 2000us tính từ đầu frame trước.

80us là khởi điểm; đo điện áp thu thực tế rồi chỉnh. Số đo cùng frame vẫn lệch thời điểm vài trăm microsecond. Nếu ADC lỗi hoặc xung sáng kéo dài quá giới hạn, frame bị loại. IRQ STOP cắt cả LED; phần mềm loại frame dở, không publish số đo có xung bị cắt.

## 8. UART và nút

USART3 Asynchronous,115200,8bit,None parity,1stop,TX/RX,no flow control,oversampling16; bật USART3 global interrupt. DMA không cần. Không thêm `HAL_UART_Receive()` polling hoặc một driver khác lên cùng UART; app đã sở hữu RX_IT 1byte và TX_IT.

PC8/PC9 input no pull nếu đã có pull-up 10k ngoài; nếu bo không có thì dùng pull-up phù hợp. PB8 gyro INT chỉ input dự phòng. Không bật EXTI cho nút. Nút không tự arm robot.

## 9. Thêm source và hook callback

Copy/thêm linked folder `micromouse-control/include`, `src`, `platform` vào project CubeIDE. Chỉ build các `.cpp` của `src` và `platform`; thêm include path `include` và `platform`, cùng Core/Inc và HAL như project CubeMX. **Loại `tests/`, `build/`, bộ `hardware-tests/` cũ khỏi firmware build.**

Chọn C++17 cho compiler C++; linker dùng `arm-none-eabi-g++`, giữ cờ CPU/FPU/float ABI do CubeIDE sinh cho Cortex-M4F. Không đổi ABI riêng một file. Firmware không dùng exceptions/RTTI/heap, có thể tắt exceptions/RTTI nếu project hỗ trợ. Bảo đảm toolchain liên kết thư viện toán; không cần bật `_printf_float` vì log dùng số nguyên.

Nếu project tạo dạng C thuần và thiếu cấu hình C++, dùng chức năng Convert to C++ của IDE hoặc tạo project C++ mới rồi nhập các source. Giữ `main.c`; header bridge `.h` đã có `extern "C"`.

Trong `main.c`:

```c
/* USER CODE BEGIN Includes */
#include "micromouse_app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 -- đặt sau TẤT CẢ MX_*_Init */
if (!MM_Init()) {
    /* Không chạy motor; debug readiness/clock/UART, không tự thử lệnh chạy. */
}
/* USER CODE END 2 */

while (1) {
    /* USER CODE BEGIN 3 */
    MM_Poll();
    /* Không HAL_Delay(), printf blocking, đọc flash lâu ở đây. */
    /* USER CODE END 3 */
}
```

Đã cần init GPIO, TIM2/3/4/5/6/8, ADC1, SPI2, USART3 trước `MM_Init`. `MM_Init` tự start PWM/encoder/timer và RX_IT; không start lần thứ hai ngoài app.

Thêm vào vùng USER CODE 4 của `main.c` (hoặc ghép vào callback đã có; không định nghĩa trùng):

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) MM_ControlTickISR();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) MM_UartRxISR();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) MM_UartTxISR();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) MM_UartErrorISR();
}
```

Kiểm tra `stm32f4xx_it.c` có `TIM6_DAC_IRQHandler → HAL_TIM_IRQHandler(&htim6)` và `USART3_IRQHandler → HAL_UART_IRQHandler(&huart3)`. Thiếu TIM6 thì controller không chạy; thiếu UART IRQ thì lệnh không nhận/log mắc busy.

Phần lớn mã do CubeMX sinh giữ `extern "C"` trong tim.h/spi.h/adc.h/usart.h. Nếu tự viết header thiếu lớp bọc này, sẽ lỗi link handle/hàm dù compile C++ thành công.

## 10. Xác minh timing và watchdog trên bo

Quan sát `firmware::diagnostics.last_period_us`, `max_period_us`, `missed_ticks`, `last_control_work_us`, `max_control_work_us`. Period mục tiêu khoảng 1000us; work phải nhỏ hơn ngân sách đó. `maxwork` không phải `maxperiod`. Đếm `ir_frames` và `gyro_samples` trong 1 giây để biết tần số thật. Khi đã chạy motor, mất tick hoặc dt vượt ngưỡng sẽ dừng, không chạy bù nhiều lượt PID trên cùng mẫu cũ.

Watchdog TIM6 cần ngắt còn hoạt động. Có thể bổ sung IWDG khoảng 100–200ms để xử lý CPU treo toàn phần. Khi thêm: khởi tạo IWDG **sau** chuỗi init gyro có các delay ~300ms, hoặc cấp refresh phù hợp trong init; chỉ refresh ở main sau một lượt phục vụ thành công, không refresh vô điều kiện trong ISR. Tính timeout theo LSI thực và dung sai, thử trên bo. IWDG chưa được bật trong source này.

Debugger halt có thể để timer PWM tiếp tục chạy; chọn freeze các timer cần thiết trong DBGMCU hoặc cắt nguồn motor khi debug dừng bước. Không xem watchdog phần mềm là bảo đảm an toàn phần cứng.

## Nguồn kỹ thuật để đối chiếu

- [ST STM32F405/407 datasheet — chân, AF và ngoại vi](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf).
- [ST AN4013 — timer, encoder và cách chia clock](https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf).
- [TDK ICM20602 datasheet — SPI và thanh ghi gyro](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf).
- [Wemos AT8870 reference schematic](https://www.wemos.cc/en/latest/_static/files/sch_at8870_v1.0.0.pdf). Đối chiếu đúng biến thể AT8870 của linh kiện thật trước khi chốt VM/current limit; không áp thông số DRV8870 mặc định.

Các tần số vòng điều khiển, gain và giới hạn chuyển động trong project là lựa chọn thiết kế để bắt đầu đo/tune, không phải thông số bắt buộc do datasheet quy định.
