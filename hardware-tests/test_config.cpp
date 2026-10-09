#include "test_config.hpp"

// ==================== ĐIỀN CẤU HÌNH TẠI ĐÂY ====================
// Ví dụ khai báo handle CubeMX (bỏ comment và đổi tên khi đã có ngoại vi):
// extern UART_HandleTypeDef huart2;
// extern ADC_HandleTypeDef hadc1;
// extern SPI_HandleTypeDef hspi1;
// extern TIM_HandleTypeDef htim1, htim2, htim3, htim5;
// Nếu dùng header usart.h/adc.h/spi.h/tim.h sinh bởi CubeMX, có thể include
// các header đó thay cho các khai báo extern ở trên.

namespace bench {
UART_HandleTypeDef* console = nullptr; // Ví dụ: &huart2; UART 115200, 8N1.
ADC_HandleTypeDef* ir_adc = nullptr;   // Ví dụ: &hadc1; single conversion, 12 bit.
SPI_HandleTypeDef* imu_spi = nullptr; // Ví dụ: &hspi1; mode 0, 8 bit, MSB, <= 1 MHz.
Pin imu_cs = {nullptr, 0};             // Ví dụ: {IMU_CS_GPIO_Port, IMU_CS_Pin}.

// Hai AT8870, mỗi chip điều khiển một motor bằng IN1/IN2.
// Dùng 4 ngõ PWM: mỗi chân IN nối một kênh PWM, không phải cấu trúc EN+DIR.
// Ví dụ motor trái: {{&htim1,TIM_CHANNEL_1},{&htim1,TIM_CHANNEL_2}}.
// Timer phải đếm lên, PWM mode 1, polarity HIGH, PSC/ARR cho khoảng 20 kHz.
// Các kênh phải riêng biệt; code kiểm tra không trùng kênh giữa hai motor.
Motor motors[2] = {
    {{nullptr, 0}, {nullptr, 0}}, // [0] trái
    {{nullptr, 0}, {nullptr, 0}}  // [1] phải
};

// MT6701 phải xuất A/B, đưa A/B vào CH1/CH2 của timer encoder mode TI12.
// Ví dụ: {{&htim2,+1},{&htim5,-1}}. Đổi sign khi chiều đếm bị ngược.
// ARR chỉ dùng 65535 hoặc 0xFFFFFFFF. Không dùng chung timer với PWM.
// Không dùng driver này nếu module MT6701 hiện chỉ đưa ra SSI/I2C.
Encoder encoders[2] = {{nullptr, +1}, {nullptr, +1}};

// Thứ tự kênh do bạn đặt: ví dụ trái, chéo trái, trước, chéo phải, phải.
// signal_sign = +1 nếu ADC tăng khi bật LED; -1 nếu ADC giảm.
// Ví dụ: {{IR_L_GPIO_Port,IR_L_Pin},ADC_CHANNEL_0,-1}.
// MOSFET phát active HIGH. Mỗi emitter phải độc lập; nếu 5 LED chỉ dùng
// 3 MOSFET chung nhóm, cần thay lịch quét theo sơ đồ, không điền trùng pin.
Ir ir[5] = {
    {{nullptr, 0}, kUnusedAdc, +1},
    {{nullptr, 0}, kUnusedAdc, +1},
    {{nullptr, 0}, kUnusedAdc, +1},
    {{nullptr, 0}, kUnusedAdc, +1},
    {{nullptr, 0}, kUnusedAdc, +1}
};
}
