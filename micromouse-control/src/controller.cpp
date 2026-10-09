#include "mm/controller.hpp"
#include <algorithm>
#include <cmath>

namespace mm {
int64_t encoderDelta(uint32_t now, uint32_t previous, uint32_t arr) {
    const int64_t modulus = int64_t(arr) + 1;
    int64_t d = int64_t(now) - int64_t(previous);
    if (d > modulus/2) d -= modulus;
    if (d < -modulus/2) d += modulus;
    return d;
}
bool IrCurve::valid() const {
    if (count < 2 || count > 8) return false;
    for (unsigned i=0; i<count; ++i) {
        if (!std::isfinite(signal[i]) || !std::isfinite(distance_m[i]) ||
            signal[i]<0 || distance_m[i]<=0) return false;
        if (i && (signal[i]<=signal[i-1] || distance_m[i]>=distance_m[i-1])) return false;
    }
    return true;
}
bool IrCurve::convert(float value, float& metres) const {
    if (!valid() || !std::isfinite(value) || value < signal[0]) return false;
    // Tín hiệu yếu hơn điểm xa nhất: coi không có tường trong vùng đã hiệu chuẩn.
    // Mạnh hơn điểm gần nhất: kẹp về khoảng cách gần nhất (không ngoại suy).
    if (value >= signal[count-1]) { metres=distance_m[count-1]; return true; }
    for (unsigned i=1; i<count; ++i) if (value <= signal[i]) {
        const float t=(value-signal[i-1])/(signal[i]-signal[i-1]);
        metres=distance_m[i-1]+t*(distance_m[i]-distance_m[i-1]);
        return true;
    }
    return false;
}
static bool positive(float x) { return std::isfinite(x) && x>0; }
static bool gainValid(const PidConfig& p) {
    return std::isfinite(p.kp) && p.kp>=0 && std::isfinite(p.ki) && p.ki>=0 &&
           std::isfinite(p.kd) && p.kd>=0 && std::isfinite(p.integral_limit) &&
           p.integral_limit>=0 && std::isfinite(p.derivative_tau) && p.derivative_tau>=0;
}
bool Controller::configured() const {
    if (!c_.geometry_verified || !c_.polarity_verified) return false;
    for (const auto& w:c_.wheel) {
        if (!positive(w.metres_per_count) || !gainValid(w.pi_gain) ||
            w.pi_gain.kp<=0 || w.pi_gain.kd!=0 || !std::isfinite(w.kv) || w.kv<0 ||
            !std::isfinite(w.ks) || w.ks<0 || w.ks>=c_.duty_limit) return false;
    }
    const float positives[]={c_.track_m,c_.speed_filter_tau,c_.duty_limit,c_.wheel_speed_limit,
        c_.straight_speed,c_.linear_accel,c_.distance_kp,c_.yaw_rate_limit,c_.yaw_accel,
        c_.heading_period_s,c_.wall_period_s,c_.wall_target_m,c_.wall_enter_m,c_.wall_exit_m,
        c_.wall_offset_limit,c_.wall_offset_slew,c_.distance_tolerance,c_.angle_tolerance,
        c_.stopped_speed,c_.stopped_yaw_rate,c_.settle_s,c_.command_timeout_s,c_.max_control_dt,
        c_.gyro_stale_s,c_.ir_stale_s,c_.stall_s,c_.stall_duty,c_.stall_reference_speed,
        c_.stall_measured_speed,c_.wrong_direction_s,c_.front_stop_m};
    for(float v:positives) if(!positive(v)) return false;
    return gainValid(c_.heading) && c_.heading.kp>0 && gainValid(c_.wall) &&
           c_.duty_limit<=1 && c_.straight_speed<=c_.wheel_speed_limit &&
           c_.wall_enter_m<c_.wall_exit_m && c_.wall_offset_limit<pi/4;
}
bool Controller::active() const {
    return s_.mode==Mode::Straight || s_.mode==Mode::Turn || s_.mode==Mode::WheelSpeed;
}
void Controller::zeroOutput() {
    for(unsigned i=0;i<2;++i) {
        s_.duty[i]=s_.wheel_reference[i]=0;
        speed_pi_[i].reset(); stall_clock_[i]=reverse_clock_[i]=0;
    }
    s_.forward_reference=s_.yaw_rate_reference=desired_omega_=0;
    heading_pid_.reset(); wall_pid_.reset();
}
void Controller::stop() {
    zeroOutput();
    // Lệnh stop không xóa fault: phải reset có chủ ý sau khi tìm nguyên nhân.
    if(s_.mode!=Mode::Fault) { s_.mode=Mode::Idle; s_.fault=Fault::None; }
}
void Controller::fail(Fault reason) { zeroOutput(); s_.mode=Mode::Fault; s_.fault=reason; }
void Controller::reset(const Input& input) {
    s_=State{}; last_=input; initialized_=true;
    for(unsigned i=0;i<2;++i) { previous_ticks_[i]=input.ticks[i]; position_[i]=0; }
    zeroOutput();
}
bool Controller::begin(Mode mode) {
    if(active() || s_.mode==Mode::Fault || !initialized_) return false;
    if(!configured()) { fail(Fault::Configuration); return false; }
    if(!last_.encoder_ok || !last_.gyro_ok || !last_.gyro_calibrated ||
        !std::isfinite(last_.gyro_age_s) || last_.gyro_age_s>c_.gyro_stale_s) {
        fail(Fault::Sensor); return false;
    }
    // Không khởi động lệnh tiếp theo khi robot vẫn còn trôi hoặc đang bị cầm xoay.
    if(std::fabs(s_.speed[0])>c_.stopped_speed || std::fabs(s_.speed[1])>c_.stopped_speed ||
       std::fabs(last_.gyro_rad_s)>c_.stopped_yaw_rate) return false;
    zeroOutput(); s_.mode=mode; s_.fault=Fault::None;
    s_.command_elapsed=settle_clock_=heading_clock_=wall_clock_=wall_error_=0;
    s_.wall_offset=0; s_.left_wall=s_.right_wall=false; wall_selection_=0;
    wall_mode_=WallMode::Off; base_heading_=s_.yaw; s_.yaw_reference=s_.yaw;
    return true;
}
bool Controller::startStraight(float metres, WallMode walls) {
    if(!std::isfinite(metres) || std::fabs(metres)<c_.distance_tolerance || std::fabs(metres)>2) return false;
    // Bám tường khi lùi cần quy ước hình học khác: bản test này chỉ cho đi tiến có tường.
    if(metres<0 && walls!=WallMode::Off) return false;
    if(walls!=WallMode::Off && walls!=WallMode::Auto && walls!=WallMode::Left && walls!=WallMode::Right) return false;
    if(walls!=WallMode::Off && (!last_.ir_frame_ok || last_.ir_age_s>c_.ir_stale_s)) return false;
    if(!begin(Mode::Straight)) return false;
    s_.distance_goal=s_.distance+metres; wall_mode_=walls; return true;
}
bool Controller::startTurn(float angle) {
    if(!std::isfinite(angle) || std::fabs(angle)<c_.angle_tolerance || std::fabs(angle)>pi) return false;
    if(!begin(Mode::Turn)) return false;
    base_heading_=s_.yaw+angle; s_.yaw_reference=base_heading_; return true;
}
bool Controller::startWheelSpeed(float left, float right, float duration) {
    if(!std::isfinite(left) || !std::isfinite(right) || std::fabs(left)>c_.wheel_speed_limit ||
       std::fabs(right)>c_.wheel_speed_limit || !positive(duration) || duration>3) return false;
    if(!begin(Mode::WheelSpeed)) return false;
    wheel_test_target_[0]=left; wheel_test_target_[1]=right; wheel_test_duration_=duration; return true;
}
void Controller::updateWall(const Input& in, float dt) {
    // Không có đủ dữ liệu thì quay dần về hướng hành lang gốc, không giữ lỗi cũ.
    auto present=[&](unsigned i,bool old) {
        return in.ir_valid[i] && std::isfinite(in.distance_m[i]) && in.distance_m[i]>0 &&
            in.distance_m[i]<(old?c_.wall_exit_m:c_.wall_enter_m);
    };
    s_.left_wall=present(0,s_.left_wall); s_.right_wall=present(4,s_.right_wall);
    const bool left=s_.left_wall && wall_mode_!=WallMode::Right;
    const bool right=s_.right_wall && wall_mode_!=WallMode::Left;
    const unsigned selection=(left?1u:0u)|(right?2u:0u);
    float error=0, target=0;
    if(left && right) error=(in.distance_m[0]-in.distance_m[4])*0.5f;
    else if(left) error=in.distance_m[0]-c_.wall_target_m;
    else if(right) error=c_.wall_target_m-in.distance_m[4];
    if(selection!=wall_selection_) { wall_pid_.reset(); wall_error_=error; }
    if(selection) target=wall_pid_.update(c_.wall,error,(error-wall_error_)/dt,dt,
                                        -c_.wall_offset_limit,c_.wall_offset_limit);
    else wall_pid_.reset();
    wall_selection_=selection; wall_error_=error;
    s_.wall_offset=approach(s_.wall_offset,target,c_.wall_offset_slew*dt);
}
void Controller::update(const Input& in, float dt) {
    if(!initialized_) reset(in);
    last_=in;
    if(!std::isfinite(dt) || dt<=0 || dt>c_.max_control_dt) {
        if(active()) fail(Fault::Timing);
        return;
    }
    if(!in.encoder_ok || !in.gyro_ok || !std::isfinite(in.gyro_rad_s)) {
        if(active()) fail(Fault::Sensor);
        return;
    }
    for(unsigned i=0;i<2;++i) {
        const int64_t delta=in.ticks[i]-previous_ticks_[i]; previous_ticks_[i]=in.ticks[i];
        const float travel=float(delta)*c_.wheel[i].metres_per_count;
        position_[i]+=travel;
        const float alpha=dt/(c_.speed_filter_tau+dt);
        s_.speed[i]+=alpha*(travel/dt-s_.speed[i]);
    }
    s_.distance=float((position_[0]+position_[1])*0.5);
    if(in.gyro_calibrated && in.gyro_age_s<=c_.gyro_stale_s) s_.yaw+=in.gyro_rad_s*dt;
    if(!active()) return;
    if(!configured()) { fail(Fault::Configuration); return; }
    if(!in.gyro_calibrated || !std::isfinite(in.gyro_age_s) || in.gyro_age_s>c_.gyro_stale_s) { fail(Fault::GyroStale); return; }
    const bool need_ir=wall_mode_!=WallMode::Off || c_.front_guard;
    if(need_ir && (!in.ir_frame_ok || !std::isfinite(in.ir_age_s) || in.ir_age_s>c_.ir_stale_s)) { fail(Fault::IrStale); return; }
    if(c_.front_guard && s_.mode==Mode::Straight && s_.distance_goal>s_.distance &&
       (in.front_saturated || (in.ir_valid[2] && in.distance_m[2]<c_.front_stop_m))) { fail(Fault::Obstacle); return; }
    s_.command_elapsed+=dt;
    if(s_.command_elapsed>c_.command_timeout_s) { fail(Fault::Timeout); return; }

    if(s_.mode==Mode::WheelSpeed) {
        if(s_.command_elapsed>=wheel_test_duration_) { zeroOutput(); s_.mode=Mode::Complete; return; }
        for(unsigned i=0;i<2;++i) s_.wheel_reference[i]=approach(s_.wheel_reference[i],wheel_test_target_[i],c_.linear_accel*dt);
    } else {
        wall_clock_+=dt; heading_clock_+=dt;
        if(wall_clock_>=c_.wall_period_s) {
            if(s_.mode==Mode::Straight && wall_mode_!=WallMode::Off) updateWall(in,wall_clock_);
            wall_clock_=0;
        }
        s_.yaw_reference=base_heading_+s_.wall_offset;
        const float angle_error=s_.yaw_reference-s_.yaw;
        if(heading_clock_>=c_.heading_period_s) {
            // Góc mục tiêu tương đối, không wrap về [-pi,pi]: lệnh +90 luôn quay trái.
            float limit=c_.yaw_rate_limit;
            if(s_.mode==Mode::Turn) limit=std::min(limit,std::sqrt(2*c_.yaw_accel*std::fabs(angle_error)));
            desired_omega_=heading_pid_.update(c_.heading,angle_error,-in.gyro_rad_s,
                                               heading_clock_,-limit,limit);
            heading_clock_=0;
        }
        s_.yaw_rate_reference=approach(s_.yaw_rate_reference,desired_omega_,c_.yaw_accel*dt);
        float velocity=0;
        if(s_.mode==Mode::Straight) {
            const float remain=s_.distance_goal-s_.distance;
            const float limit=std::min(c_.straight_speed,std::sqrt(2*c_.linear_accel*std::fabs(remain)));
            velocity=clamp(c_.distance_kp*remain,-limit,limit);
        }
        s_.forward_reference=approach(s_.forward_reference,velocity,c_.linear_accel*dt);
        // Differential drive: omega>0 quay trái => bánh phải nhanh hơn bánh trái.
        s_.wheel_reference[0]=s_.forward_reference-s_.yaw_rate_reference*c_.track_m*0.5f;
        s_.wheel_reference[1]=s_.forward_reference+s_.yaw_rate_reference*c_.track_m*0.5f;
        const float peak=std::max(std::fabs(s_.wheel_reference[0]),std::fabs(s_.wheel_reference[1]));
        if(peak>c_.wheel_speed_limit) {
            // Co cả hai cùng tỷ lệ giữ độ cong, thay vì cắt độc lập làm đổi hướng.
            for(float& r:s_.wheel_reference) r*=c_.wheel_speed_limit/peak;
        }
        const bool reached=(s_.mode==Mode::Turn || std::fabs(s_.distance_goal-s_.distance)<c_.distance_tolerance) &&
            std::fabs(angle_error)<c_.angle_tolerance && std::fabs(in.gyro_rad_s)<c_.stopped_yaw_rate &&
            std::fabs(s_.speed[0])<c_.stopped_speed && std::fabs(s_.speed[1])<c_.stopped_speed;
        settle_clock_=reached?settle_clock_+dt:0;
        if(settle_clock_>=c_.settle_s) { zeroOutput(); s_.mode=Mode::Complete; return; }
    }
    for(unsigned i=0;i<2;++i) {
        const float ref=s_.wheel_reference[i];
        const float ff=c_.wheel[i].kv*ref+(std::fabs(ref)>0.002f?(ref>0?c_.wheel[i].ks:-c_.wheel[i].ks):0);
        s_.duty[i]=speed_pi_[i].update(c_.wheel[i].pi_gain,ref-s_.speed[i],0,dt,-c_.duty_limit,c_.duty_limit,ff);
        const bool stalled=std::fabs(s_.duty[i])>c_.stall_duty && std::fabs(ref)>c_.stall_reference_speed &&
                           std::fabs(s_.speed[i])<c_.stall_measured_speed;
        stall_clock_[i]=stalled?stall_clock_[i]+dt:0;
        const bool reverse=std::fabs(ref)>0.025f && std::fabs(s_.speed[i])>0.015f && ref*s_.speed[i]<0;
        reverse_clock_[i]=reverse?reverse_clock_[i]+dt:0;
        if(stall_clock_[i]>c_.stall_s) { fail(Fault::Stall); return; }
        if(reverse_clock_[i]>c_.wrong_direction_s) { fail(Fault::WrongDirection); return; }
    }
}
}
