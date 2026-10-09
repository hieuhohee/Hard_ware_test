#pragma once
#include <cstdint>

namespace bench {
// Có thể xem cấu trúc này trong Live Expressions/Watch mà không cần UART.
// Dữ liệu chỉ phục vụ chẩn đoán, không phải bộ đệm đồng bộ cho ISR khác.
struct Live {
    uint32_t motor_ready_mask, encoder_ready_mask, ir_ready_mask;
    uint32_t imu_ready, imu_whoami, io_errors, imu_samples;
    uint32_t motor_active; // 0 = dừng; 1 = motor trái; 2 = motor phải.
    uint32_t ir_frames, gyro_calibrating, gyro_calibrated;
    uint32_t encoder_counter[2];
    int64_t encoder_total[2]; // Count có dấu sau cấu hình sign, tính từ lúc init.
    uint16_t ir_off[5], ir_on[5];
    int32_t ir_difference[5]; // Giữ cả giá trị âm để phát hiện sai cực tính/nhiễu.
    // Chỉ gyro XYZ: số thô, tốc độ góc đã trừ bias và bias lưu trong RAM.
    int16_t gyro_raw[3];
    float gyro_dps[3], gyro_bias_dps[3];
};
extern volatile Live live;
// Ghi mã ASCII vào command trong debugger: '1'=49, 'i'=105, 's'=115...
// Không tự chạy motor sau khởi động. Init/ poll chỉ gọi từ luồng main.
extern volatile uint8_t command;
void init();
void poll();
}

// Wrapper C để gọi từ main.c của CubeMX; các file triển khai vẫn là C++.
extern "C" void HardwareTests_Init(void);
extern "C" void HardwareTests_Poll(void);
