#include "board_config.hpp"
// CubeMX: Generate peripheral initialization as a pair of .c/.h files.
// Header CubeMX chuẩn có extern "C" khi include từ C++.
#include "tim.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"

namespace board {
const Motor motors[2] = {
    {&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, +1}, // PB4/PB5.
    {&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, +1}  // PB6/PB7.
};
const Encoder encoders[2] = {{&htim8,+1}, {&htim5,+1}}; // PC6/PC7 và PA0/PA1.
const Pin emitters[3] = {{GPIOC,GPIO_PIN_1}, {GPIOA,GPIO_PIN_5}, {GPIOC,GPIO_PIN_2}};
const IrReceiver receivers[5] = {
    {ADC_CHANNEL_3,  0, +1}, // IR1 hông trái, PA3; chung Q6 với IR5.
    {ADC_CHANNEL_4,  1, +1}, // IR2 chéo trái, PA4; chung Q8 với IR4.
    {ADC_CHANNEL_8,  2, +1}, // IR3 trước, PB0; Q7.
    {ADC_CHANNEL_9,  1, +1}, // IR4 chéo phải, PB1.
    {ADC_CHANNEL_10, 0, +1}  // IR5 hông phải, PC0.
};
const Pin imu_cs={GPIOB,GPIO_PIN_12};
const Pin start_button={GPIOC,GPIO_PIN_9}, mode_button={GPIOC,GPIO_PIN_8};
SPI_HandleTypeDef* const spi=&hspi2;
ADC_HandleTypeDef* const adc=&hadc1;
UART_HandleTypeDef* const uart=&huart3;
TIM_HandleTypeDef* const micros_timer=&htim2;
TIM_HandleTypeDef* const scheduler_timer=&htim6;

mm::Config control = [] {
    mm::Config c;
    // BẮT BUỘC điền số đo thật trước chạy kín vòng. 0 khóa lệnh di chuyển.
    // VD công thức: c.wheel[0].metres_per_count = mm::pi*0.025f / counts_per_wheel_turn;
    // counts_per_wheel_turn là SỐ ĐẾM x4 đo được, không mặc định PPR của MT6701.
    c.wheel[0].metres_per_count=0;
    c.wheel[1].metres_per_count=0;
    // Điền PI sau test chiều/quãng đường; xem docs/TUNING.md để tune từ Kp rồi Ki.
    c.wheel[0].pi_gain={0,0,0,0.15f,0.02f};
    c.wheel[1].pi_gain={0,0,0,0.15f,0.02f};
    c.geometry_verified=false;
    c.polarity_verified=false;
    return c;
}();
// Chưa có số đo, không tạo bảng khoảng cách giả. Đo riêng cả năm mắt thu.
mm::IrCurve ir_curves[5]{};
}
