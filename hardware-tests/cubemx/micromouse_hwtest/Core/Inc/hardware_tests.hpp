#pragma once
#include <cstdint>
#include "hardware_tests_api.h" // API gọi trực tiếp, dùng cả từ C/C++.

namespace bench {
// Mã bước khởi tạo để phân biệt lỗi ID, lỗi HAL và lỗi đọc lại cấu hình.
// Xem imu_init_step trong Watch; các số giữ cố định để dễ đối chiếu tài liệu.
enum class ImuInitStep : uint32_t {
    NotConfigured=0, Interface=1, Identity=2, IdentityRejected=3,
    Reset=4, Configure=5, Verify=6, Ready=7, BusError=8, VerifyError=9
};
// Có thể xem cấu trúc này trong Live Expressions/Watch mà không cần UART.
// Dữ liệu chỉ phục vụ chẩn đoán, không phải bộ đệm đồng bộ cho ISR khác.
struct Live {
    uint32_t motor_ready_mask, encoder_ready_mask, ir_ready_mask;
    uint32_t imu_ready, imu_whoami, io_errors, imu_samples;
    uint32_t imu_init_step; // ImuInitStep; 7 mới là init thành công.
    uint32_t imu_hal_status; // HAL_OK=0, ERROR=1, BUSY=2, TIMEOUT=3.
    uint32_t imu_last_reg, imu_expected_value, imu_actual_value;
    uint32_t imu_id_reads, imu_id_matches; // Số giao dịch ID OK / số ID đúng 0x12.
    uint8_t imu_id_history[8]; // Tám mẫu ID lúc init; chỉ [0..imu_id_reads-1] hợp lệ.
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
