#pragma once
#include "pid.hpp"
#include <cstdint>

namespace mm {
enum class Mode : uint8_t { Idle, Straight, Turn, WheelSpeed, Complete, Fault };
enum class Fault : uint8_t {
    None, Configuration, Timing, Sensor, GyroStale, IrStale, Obstacle,
    Timeout, Stall, WrongDirection, Emergency
};
enum class WallMode : uint8_t { Off, Auto, Left, Right };

struct Input {
    int64_t ticks[2]{}; // Đã áp sign trong driver, count x4 tích lũy, chưa đổi ra mét.
    bool encoder_ok = false;
    float gyro_rad_s = 0; // Trục thẳng đứng đã đổi dấu, trừ bias; không dùng accel.
    float gyro_age_s = 1;
    bool gyro_calibrated = false;
    bool gyro_ok = false;
    float distance_m[5]{}; // IR1,2,3,4,5; từ LUT đo thực tế.
    bool ir_valid[5]{};
    bool ir_frame_ok = false;
    bool front_saturated = false;
    float ir_age_s = 1;
};
struct State {
    Mode mode = Mode::Idle;
    Fault fault = Fault::None;
    float yaw = 0, yaw_reference = 0; // Unwrapped: rẽ liên tiếp không nhảy ở +/-pi.
    float distance = 0, distance_goal = 0;
    float speed[2]{}, wheel_reference[2]{}, duty[2]{};
    float forward_reference = 0, yaw_rate_reference = 0, wall_offset = 0;
    float command_elapsed = 0;
    bool left_wall = false, right_wall = false;
};

// Lõi thuần C++, không HAL, không cấp phát heap, không chờ bus. Chỉ một luồng
// main gọi. ISR không chạm đối tượng này; ISR chỉ ngắt PWM và báo sự kiện.
class Controller {
public:
    explicit Controller(const Config& config) : c_(config) {}
    void reset(const Input& input); // Motor phải dừng. Xóa góc tương đối về 0.
    void update(const Input& input, float dt);
    bool startStraight(float metres, WallMode walls = WallMode::Off);
    bool startTurn(float radians_relative); // Dương quay trái, âm quay phải; tại chỗ.
    bool startWheelSpeed(float left_m_s, float right_m_s, float duration_s);
    void stop();
    void fail(Fault reason);
    bool active() const;
    bool configured() const;
    const State& state() const { return s_; }
private:
    bool begin(Mode mode);
    void updateWall(const Input& input, float dt);
    void zeroOutput();
    const Config& c_;
    State s_{};
    Input last_{};
    bool initialized_ = false;
    int64_t previous_ticks_[2]{};
    double position_[2]{};
    Pid speed_pi_[2], heading_pid_, wall_pid_;
    WallMode wall_mode_ = WallMode::Off;
    unsigned wall_selection_ = 0;
    float base_heading_ = 0, heading_clock_ = 0, wall_clock_ = 0;
    float desired_omega_ = 0, settle_clock_ = 0, wall_error_ = 0;
    float stall_clock_[2]{}, reverse_clock_[2]{};
    float wheel_test_target_[2]{}, wheel_test_duration_ = 0;
};

// Timer encoder có thể 16 hoặc 32 bit. Giả thiết |delta| < nửa modulus/lần đọc.
int64_t encoderDelta(uint32_t now, uint32_t previous, uint32_t auto_reload);
}
