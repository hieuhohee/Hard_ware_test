#pragma once
#include <cstdint>

namespace mm {
constexpr float pi = 3.14159265358979323846f;
constexpr float radians(float degrees) { return degrees * pi / 180.0f; }

// Tất cả phép tính điều khiển dùng SI: m, s, rad; duty nằm trong [-1,1].
// Hệ số bên dưới chỉ là giới hạn khởi đầu cho bench, KHÔNG là bộ PID đã tune.
struct PidConfig {
    float kp = 0, ki = 0, kd = 0;
    float integral_limit = 0.15f; // Giới hạn THÀNH PHẦN Ki*integral, cùng đơn vị output.
    float derivative_tau = 0.02f; // Lọc bậc một nhánh D; 0 = không lọc.
};
struct WheelConfig {
    float metres_per_count = 0; // PI*D_banh / counts_per_WHEEL_revolution, đo riêng L/R.
    PidConfig pi_gain{};        // kd phải bằng 0: lớp 1 là PI tốc độ.
    float kv = 0;              // Feedforward duty/(m/s); chưa đo thì giữ 0.
    float ks = 0;              // Duty thắng ma sát; chưa đo thì giữ 0.
};
struct Config {
    WheelConfig wheel[2]{};
    float track_m = 0.070f;    // Khoảng cách HIỆU DỤNG giữa hai vệt tiếp xúc, phải đo.
    bool geometry_verified = false;
    bool polarity_verified = false; // Bánh tiến => count tăng; quay trái => gyro dương.
    float speed_filter_tau = 0.015f;
    float duty_limit = 0.25f;
    float wheel_speed_limit = 0.20f;
    float straight_speed = 0.08f;
    float linear_accel = 0.20f;
    float distance_kp = 2.0f; // Vòng tạo tốc độ theo quãng đường còn lại, đơn vị 1/s.
    PidConfig heading{4.0f, 0.0f, 0.12f, 0.2f, 0.015f};
    float yaw_rate_limit = 1.5f;
    float yaw_accel = 5.0f;
    float heading_period_s = 0.004f; // Lớp 2: 250 Hz khi vòng nền đúng 1 kHz.
    PidConfig wall{1.5f, 0.0f, 0.0f, 0.02f, 0.04f}; // m lỗi ngang -> rad lệch hướng.
    float wall_period_s = 0.010f; // Lớp 3: 100 Hz.
    float wall_target_m = 0.060f; // Khoảng cách từ MẮT THU tới tường, phải đo lại.
    float wall_enter_m = 0.100f, wall_exit_m = 0.120f; // Hysteresis có/mất tường.
    float wall_offset_limit = radians(8);
    float wall_offset_slew = radians(30); // rad/s, tránh giật khi gặp cửa/cạnh tường.
    float distance_tolerance = 0.003f;
    float angle_tolerance = radians(1.5f);
    float stopped_speed = 0.008f, stopped_yaw_rate = radians(4);
    float settle_s = 0.15f;
    float command_timeout_s = 10;
    float max_control_dt = 0.003f;
    float gyro_stale_s = 0.006f, ir_stale_s = 0.020f;
    float stall_s = 0.60f;
    float stall_duty = 0.18f, stall_reference_speed = 0.025f, stall_measured_speed = 0.003f;
    float wrong_direction_s = 0.15f;
    // Bật khi IR3 đã có bảng khoảng cách. Mặc định không có chức năng tránh vật cản.
    bool front_guard = false;
    float front_stop_m = 0.045f; // Còn phải cộng quãng hãm + độ trễ khi nâng tốc độ.
};

// Có tối đa 8 điểm, x = tín hiệu on-off TĂNG, y = khoảng cách GIẢM.
// Không áp công thức 1/ADC cho mọi phototransistor/vật liệu tường.
struct IrCurve {
    unsigned count = 0;
    float signal[8]{};
    float distance_m[8]{};
    bool valid() const;
    bool convert(float value, float& metres) const;
};
}
