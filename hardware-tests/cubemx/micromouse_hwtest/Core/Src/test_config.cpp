// MẪU CHO PROJECT TEST MODULE RỜI, KHÔNG THÊM VÀO FIRMWARE PID.
// Copy nội dung này THAY cho test_config.cpp trong project bench của bạn.
// Không compile cả file này và test_config.cpp cùng lúc: sẽ trùng định nghĩa biến.
#include "test_config.hpp"
#include "hardware_tests.hpp"
#include "tim.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"

// =========== CHỌN MODULE ĐANG ĐẤU DÂY: 1 = dùng, 0 = bỏ qua ===========
// Mặc định không sử dụng phần cứng ngoài nào, không tự quay motor.
#define BENCH_USE_UART       0 // 1: USART3 PC10/PC11; 0: xem bench::live bằng ST-Link.
#define BENCH_TEST_IR        1 // Một bộ IR: phát PC1 qua MOSFET, thu PA3/ADC1_IN3.
#define BENCH_TEST_ENCODER_L 0 // MT6701 A/B: PC6/PC7, TIM8.
#define BENCH_TEST_ENCODER_R 0 //MT6701 A/B: PA0/PA1, TIM5.
#define BENCH_TEST_MOTOR_L   0 // AT8870 IN1/IN2: PB4/PB5, TIM3 CH1/CH2.
#define BENCH_TEST_MOTOR_R   0 // AT8870 IN1/IN2: PB6/PB7, TIM4 CH1/CH2.
#define BENCH_TEST_GYRO      0// Chỉ ICM20602 SPI2, không tự nhận MPU6050/MPU6500.

// Giữ HAL ADC/TIM/SPI/UART trong build dù chỉ test một module. nullptr chỉ ngăn
// driver truy cập thiết bị lúc chạy; không loại mã driver khỏi quá trình compile.
// Dùng cấu hình CubeMX đầy đủ theo hướng dẫn, nhưng chưa cần cắm các module khác.
namespace bench {
#if BENCH_USE_UART
UART_HandleTypeDef* console=&huart3;
#else
UART_HandleTypeDef* console=nullptr;
#endif

#if BENCH_TEST_IR
ADC_HandleTypeDef* ir_adc=&hadc1;
#else
ADC_HandleTypeDef* ir_adc=nullptr;
#endif

#if BENCH_TEST_GYRO
SPI_HandleTypeDef* imu_spi=&hspi2;
Pin imu_cs={GPIOB,GPIO_PIN_12};
#else
SPI_HandleTypeDef* imu_spi=nullptr;
Pin imu_cs={nullptr,0};
#endif

Motor motors[2]={
#if BENCH_TEST_MOTOR_L
    {{&htim3,TIM_CHANNEL_1},{&htim3,TIM_CHANNEL_2}},
#else
    {{nullptr,0},{nullptr,0}},
#endif
#if BENCH_TEST_MOTOR_R
    {{&htim4,TIM_CHANNEL_1},{&htim4,TIM_CHANNEL_2}}
#else
    {{nullptr,0},{nullptr,0}}
#endif
};

Encoder encoders[2]={
#if BENCH_TEST_ENCODER_L
    {&htim8,+1}, // Đổi sign thành -1 nếu chiều tiến của bánh làm count giảm.
#else
    {nullptr,+1},
#endif
#if BENCH_TEST_ENCODER_R
    {&htim5,+1}
#else
    {nullptr,+1}
#endif
};

Ir ir[5]={
#if BENCH_TEST_IR
    {{GPIOC,GPIO_PIN_1},ADC_CHANNEL_3,+1}, // Một mắt tại vị trí logic IR[0].
#else
    {{nullptr,0},kUnusedAdc,+1},
#endif
    {{nullptr,0},kUnusedAdc,+1},
    {{nullptr,0},kUnusedAdc,+1},
    {{nullptr,0},kUnusedAdc,+1},
    {{nullptr,0},kUnusedAdc,+1}
};
}

// Helper tương thích hướng dẫn cũ. Với API trực tiếp mới, chọn bài bằng
// HardwareTests_TestIR/TestGyro/... trong main; không cần gọi helper này.
// Wrapper dùng được từ main.c và không tự chạy motor.
extern "C" void ModuleBench_SelectMonitor(void) {
#if BENCH_TEST_IR || BENCH_TEST_ENCODER_L || BENCH_TEST_ENCODER_R || BENCH_TEST_GYRO
    HardwareTests_TestAllSensors(); // Các khối nullptr vẫn bị bỏ qua.
#endif
}
