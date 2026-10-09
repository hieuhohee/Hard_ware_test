#pragma once
#include "main.h"
#include "mm/config.hpp"

namespace board {
struct Pin { GPIO_TypeDef* port; uint16_t number; };
struct Motor {
    TIM_HandleTypeDef* timer;
    uint32_t in1_channel, in2_channel;
    int sign; // +1 nếu IN1 PWM làm bánh tiến; -1 nếu cần đảo chiều điện.
};
struct Encoder { TIM_HandleTypeDef* timer; int sign; };
struct IrReceiver { uint32_t adc_channel; unsigned group; int signal_sign; };
extern const Motor motors[2];
extern const Encoder encoders[2];
extern const Pin emitters[3]; // group 0=hông, 1=chéo, 2=trước.
extern const IrReceiver receivers[5];
extern const Pin imu_cs, start_button, mode_button;
extern SPI_HandleTypeDef* const spi;
extern ADC_HandleTypeDef* const adc;
extern UART_HandleTypeDef* const uart;
extern TIM_HandleTypeDef* const micros_timer;
extern TIM_HandleTypeDef* const scheduler_timer;

extern mm::Config control;
extern mm::IrCurve ir_curves[5];
constexpr unsigned gyro_axis = 2; // 0=X,1=Y,2=Z; chọn trục vuông góc mặt sàn.
constexpr int gyro_sign = +1;     // Nhìn từ trên: quay trái phải cho gyro dương.
constexpr float gyro_scale = 1.0f; // Hiệu chỉnh scale sau bias, đo trên nhiều góc chuẩn.
constexpr uint32_t ir_settle_us = 80;
constexpr uint32_t ir_frame_period_us = 2000; // Mục tiêu 500 Hz, phải đo thực tế.
constexpr uint32_t max_led_on_us = 600;
constexpr uint32_t control_watchdog_ticks = 5; // TIM6 1 kHz => mất main ~5 ms thì ngắt PWM.
constexpr uint32_t reverse_coast_us = 100; // >=2 chu kỳ PWM; tránh đảo IN ngay một lượt ghi.
constexpr uint32_t bus_timeout_ms = 1;
constexpr uint32_t gyro_calibration_samples = 2000;
constexpr float gyro_calibration_std_max = 0.015f; // rad/s, đánh giá độ rung khi đứng yên.
constexpr float gyro_calibration_mean_max = 0.15f; // Không nhận đang xoay đều làm bias.
constexpr float bench_duty = 0.12f;
constexpr uint32_t bench_pulse_us = 300000;
}
