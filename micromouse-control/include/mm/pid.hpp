#pragma once
#include "config.hpp"
#include <cmath>

namespace mm {
inline float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float approach(float current, float target, float max_step) {
    return current + clamp(target-current, -max_step, max_step);
}
// PID nhận đạo hàm lỗi từ bên ngoài. Giữ hướng dùng -gyro, tránh derivative kick
// khi đổi góc đặt. Wheel PI truyền 0. reset() khi bắt đầu/dừng/chuyển mode.
class Pid {
public:
    void reset() { integral_ = derivative_ = 0; }
    float update(const PidConfig& c, float error, float error_rate, float dt,
                 float low, float high, float feedforward = 0) {
        if (!(dt > 0) || !std::isfinite(error) || !std::isfinite(error_rate)) return 0;
        const float alpha = dt/(c.derivative_tau + dt);
        derivative_ += alpha*(error_rate-derivative_);
        const float candidate = clamp(integral_ + c.ki*error*dt,
                                      -c.integral_limit, c.integral_limit);
        const float base = feedforward + c.kp*error + c.kd*derivative_;
        const float unsaturated = base + candidate;
        // Conditional integration: không tích thêm theo hướng đang bão hòa;
        // vẫn cho tích ngược để thoát bão hòa. Feedforward cũng tính vào giới hạn.
        if ((unsaturated <= high && unsaturated >= low) ||
            (unsaturated > high && c.ki*error < 0) ||
            (unsaturated < low && c.ki*error > 0)) integral_ = candidate;
        return clamp(base + integral_, low, high);
    }
    float integral() const { return integral_; }
private:
    float integral_ = 0, derivative_ = 0;
};
}
