#pragma once
#include "main.h"  // STM32CubeMX sinh file này; phải có các kiểu HAL của STM32F4.
#include <cstdint>

namespace bench {
// Pin rỗng {nullptr, 0} nghĩa là chưa cấu hình. Không có chân mặc định ngầm.
struct Pin { GPIO_TypeDef* port; uint16_t pin; };
struct Pwm { TIM_HandleTypeDef* timer; uint32_t channel; };
struct Motor { Pwm in1; Pwm in2; };
struct Encoder { TIM_HandleTypeDef* timer; int sign; };
struct Ir { Pin emitter; uint32_t adc_channel; int signal_sign; };

// Chỉ sửa test_config.cpp khi chốt phần cứng. Các handle trỏ tới ngoại vi mà
// CubeMX đã khởi tạo; thay handle KHÔNG tự cấu hình GPIO alternate function.
extern UART_HandleTypeDef* console;
extern ADC_HandleTypeDef* ir_adc;
extern SPI_HandleTypeDef* imu_spi;
extern Pin imu_cs;
extern Motor motors[2];
extern Encoder encoders[2];
extern Ir ir[5];

constexpr unsigned kIrCount = 5;
constexpr uint32_t kUnusedAdc = 0xFFFFFFFFu;
constexpr uint32_t kIrSettleUs = 80; // Điểm bắt đầu; đo oscilloscope rồi chỉnh.
constexpr uint32_t kMotorPulseMs = 300;
constexpr unsigned kMotorDutyPercent = 12; // N20 có thể chưa thắng ma sát ở mức này.
constexpr uint32_t kReportMs = 200; // Log 5 Hz để dễ đọc trên terminal.
constexpr uint32_t kIoTimeoutMs = 2; // Không chờ bus vô hạn khi linh kiện lỗi.
}
