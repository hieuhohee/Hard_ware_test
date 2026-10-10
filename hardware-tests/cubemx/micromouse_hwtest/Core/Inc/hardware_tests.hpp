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
// Watch của extension STM32 có thể cố giải mã uint8_t/unsigned char thành
// chuỗi UTF-8, gây "Failed to decode cstring" với byte >= 0x80. Vì vậy các
// bản sao chẩn đoán dùng uint32_t để GDB hiển thị SỐ. Giá trị dữ liệu byte
// vẫn 0..255; buffer truyền/nhận HAL trong .cpp vẫn uint8_t đúng giao thức.
// Thay đổi này làm đổi layout/địa chỉ RAM: luôn dùng ELF vừa build/nạp.
struct SpiTrial {
    uint32_t mode, divider, hal_status, matches;
    uint32_t id[8], dummy[8]; // dummy là RX trong lúc MCU phát byte địa chỉ.
};
struct Live {
    uint32_t motor_ready_mask, encoder_ready_mask, ir_ready_mask;
    uint32_t imu_ready, imu_whoami, io_errors, imu_samples;
    uint32_t imu_init_step; // ImuInitStep; 7 mới là init thành công.
    uint32_t imu_hal_status; // HAL_OK=0, ERROR=1, BUSY=2, TIMEOUT=3.
    uint32_t imu_last_reg, imu_expected_value, imu_actual_value;
    uint32_t imu_id_reads, imu_id_matches; // Số giao dịch ID OK / số ID đúng 0x12.
    uint32_t imu_id_history[8]; // Tám mẫu ID lúc init; chỉ [0..imu_id_reads-1] hợp lệ.
    uint32_t spi_probe_done, spi_probe_selected; // 0: không chọn; 1..4: dòng được chọn.
    SpiTrial spi_trials[4]; // Mode0/64, mode0/256, mode3/64, mode3/256.
    uint32_t spi_cr1, spi_cr2, spi_sr; // Thanh ghi thực của MCU, không chỉ HAL Init.
    uint32_t gpiob_moder, gpiob_afrh, gpiob_idr, gpiob_odr;
    uint32_t spi_trace_size;
    uint32_t spi_tx[7], spi_rx[7]; // Giao dịch đọc gần nhất; cả byte dummy.
    uint32_t gpio_spi_done, gpio_spi_matches; // 0 chưa chạy, 1 xong, 2 lỗi DWT.
    uint32_t gpio_spi_id[8]; // ID đọc bằng GPIO độc lập HAL_SPI.
    uint32_t probe_registers[12], probe_values[12]; // Địa chỉ và dữ liệu map đọc.
    uint32_t power_probe_done;
    uint32_t power_probe_id[16]; // [0..7] sau wake, [8..15] sau DEVICE_RESET.
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
// Bài cô lập giao dịch đọc, không dùng kết quả này để báo gyro ready.
struct IdentityReadOnly {
    uint32_t done, reads, matches, io_errors;
    uint32_t id[32], dummy[32], hal_status[32];
    uint32_t registers[12], values[12];
};
extern volatile IdentityReadOnly readonly_probe;
// Kết quả chẩn đoán đường SPI, độc lập với imu_ready của driver gyro.
// result: 0 chưa chạy/thiếu config, 1 đang chạy, 2 HAL lỗi, 3 sai readback,
// 4 không khôi phục được divider, 5 roundtrip đạt nhưng ID sai, 6 ID cũng đúng.
struct SpiRoundTrip {
    uint32_t result, matches, hal_status, restore_ok;
    uint32_t original, restored, whoami, patterns[4], readback[4];
    uint32_t packet_size[3], packet_status[3];
    uint32_t tx[3][7], rx[3][7]; // Hai dummy khác nhau và một gói burst dài.
};
extern volatile SpiRoundTrip bus_probe;
struct I2cIdentity {
    // status: 0 chưa chạy/không hỗ trợ, 1 xong, 2 lỗi đường/timing,
    // 3 I2C đang bị disable, 4 không có địa chỉ ACK.
    uint32_t status, address, reads, matches;
    uint32_t ack[2], id[8];
};
extern volatile I2cIdentity i2c_probe;
// Ghi mã ASCII vào command trong debugger: '1'=49, 'i'=105, 's'=115...
// Không tự chạy motor sau khởi động. Init/ poll chỉ gọi từ luồng main.
extern volatile uint8_t command;
void init();
void poll();
}

// Wrapper C để gọi từ main.c của CubeMX; các file triển khai vẫn là C++.
extern "C" void HardwareTests_Init(void);
extern "C" void HardwareTests_Poll(void);
