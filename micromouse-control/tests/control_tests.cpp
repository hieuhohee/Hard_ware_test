#include "mm/controller.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>

#define CHECK(x) do { if(!(x)) { std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
static bool near(float a,float b,float tol) { return std::fabs(a-b)<tol; }
static mm::Config simulationConfig() {
    mm::Config c; c.geometry_verified=c.polarity_verified=true;
    // Chỉ dùng cho plant giả ở dưới, không phải bộ gain để copy lên robot.
    for(auto& w:c.wheel) { w.metres_per_count=0.00001f; w.pi_gain={1.0f,7.0f,0,0.25f,0.02f}; w.kv=2; }
    c.track_m=0.070f;
    return c;
}
static mm::Input healthyInput() {
    mm::Input in; in.encoder_ok=in.gyro_ok=in.gyro_calibrated=in.ir_frame_ok=true;
    in.gyro_age_s=in.ir_age_s=0;
    return in;
}
struct Robot {
    mm::Config cfg=simulationConfig();
    mm::Controller control{cfg};
    mm::Input in=healthyInput();
    double wheel_position[2]{};
    float velocity[2]{},yaw=0,x=0,y=0;
    Robot() { control.reset(in); }
    void step() {
        constexpr float dt=0.001f;
        const auto& state=control.state();
        for(unsigned i=0;i<2;++i) {
            // Hai motor khác nhau 8%, hằng số thời gian 50ms, encoder lượng tử hóa.
            const float free_speed=i==0?0.50f:0.46f;
            velocity[i]+=(state.duty[i]*free_speed-velocity[i])*dt/0.05f;
            wheel_position[i]+=velocity[i]*dt;
            in.ticks[i]=int64_t(std::llround(wheel_position[i]/cfg.wheel[i].metres_per_count));
        }
        in.gyro_rad_s=(velocity[1]-velocity[0])/cfg.track_m;
        yaw+=in.gyro_rad_s*dt;
        const float v=(velocity[0]+velocity[1])*0.5f;
        x+=v*std::cos(yaw)*dt; y+=v*std::sin(yaw)*dt;
        control.update(in,dt);
        for(float duty:control.state().duty) CHECK(std::isfinite(duty) && std::fabs(duty)<=cfg.duty_limit+1e-6f);
    }
    void finish() {
        for(unsigned i=0;i<11000 && control.active();++i) step();
        if(control.state().mode!=mm::Mode::Complete) {
            std::printf("Motion failed: state=%u fault=%u x=%f yaw=%f dist=%f\n",
                unsigned(control.state().mode),unsigned(control.state().fault),x,yaw,control.state().distance);
        }
        CHECK(control.state().mode==mm::Mode::Complete);
        CHECK(control.state().duty[0]==0 && control.state().duty[1]==0);
        for(unsigned i=0;i<300;++i) step(); // Để plant coast trước lệnh tiếp theo.
    }
};
int main() {
    using namespace mm;
    CHECK(encoderDelta(5,65530,65535)==11);
    CHECK(encoderDelta(65530,5,65535)==-11);
    CHECK(encoderDelta(3,0xfffffffeu,0xffffffffu)==5);
    CHECK(encoderDelta(0xfffffffeu,3,0xffffffffu)==-5);
    Pid pid; PidConfig gains{1,10,0,1,0};
    for(unsigned i=0;i<1000;++i) CHECK(pid.update(gains,10,0,0.001f,-0.2f,0.2f)<=0.2f);
    CHECK(pid.integral()==0); // Không wind-up khi motor đã bão hòa.
    CHECK(pid.update(gains,0,0,0.001f,-0.2f,0.2f)==0);

    IrCurve curve; curve.count=3;
    curve.signal[0]=100; curve.signal[1]=500; curve.signal[2]=1000;
    curve.distance_m[0]=0.12f; curve.distance_m[1]=0.06f; curve.distance_m[2]=0.025f;
    float distance=0;
    CHECK(curve.valid() && curve.convert(300,distance) && near(distance,0.09f,0.0001f));
    CHECK(!curve.convert(50,distance));
    CHECK(curve.convert(1500,distance) && near(distance,0.025f,0.0001f));
    curve.signal[1]=100; CHECK(!curve.valid());

    Config empty; Controller unconfigured(empty); auto healthy=healthyInput(); unconfigured.reset(healthy);
    CHECK(!unconfigured.startStraight(0.18f)); CHECK(unconfigured.state().fault==Fault::Configuration);
    Robot straight; CHECK(straight.control.startStraight(0.18f)); straight.finish();
    CHECK(near(straight.x,0.18f,0.006f) && std::fabs(straight.y)<0.005f);
    CHECK(straight.control.startStraight(-0.10f)); straight.finish();
    CHECK(near(straight.x,0.08f,0.008f));
    Robot turn; CHECK(turn.control.startTurn(radians(90))); turn.finish();
    CHECK(near(turn.yaw,radians(90),radians(2.5f)) && std::fabs(turn.x)<0.005f);
    CHECK(turn.control.startTurn(radians(-90))); turn.finish(); CHECK(std::fabs(turn.yaw)<radians(2.5f));
    for(unsigned i=0;i<4;++i) { CHECK(turn.control.startTurn(radians(90))); turn.finish(); }
    CHECK(near(turn.yaw,2*pi,radians(7))); // Góc liên tục qua +/-pi và 2*pi.
    Robot wheel; CHECK(wheel.control.startWheelSpeed(0.05f,0.05f,1)); wheel.finish();

    Robot stale; CHECK(stale.control.startStraight(0.18f)); stale.in.gyro_age_s=0.010f;
    stale.control.update(stale.in,0.001f); CHECK(stale.control.state().fault==Fault::GyroStale);
    CHECK(stale.control.state().duty[0]==0); CHECK(!stale.control.startTurn(radians(90)));
    stale.control.stop(); CHECK(stale.control.state().mode==Mode::Fault); // stop không xóa fault.
    Robot late; CHECK(late.control.startStraight(0.18f)); late.control.update(late.in,0.004f);
    CHECK(late.control.state().fault==Fault::Timing);
    Robot nan; CHECK(nan.control.startStraight(0.18f)); nan.in.gyro_rad_s=std::numeric_limits<float>::quiet_NaN();
    nan.control.update(nan.in,0.001f); CHECK(nan.control.state().fault==Fault::Sensor);

    Robot stuck; CHECK(stuck.control.startStraight(0.18f));
    for(unsigned i=0;i<3000 && stuck.control.active();++i) stuck.control.update(stuck.in,0.001f);
    CHECK(stuck.control.state().fault==Fault::Stall);
    Robot wrong; CHECK(wrong.control.startWheelSpeed(0.08f,0.08f,2));
    for(unsigned i=0;i<1500 && wrong.control.active();++i) {
        wrong.in.ticks[0]-=5; wrong.in.ticks[1]-=5; wrong.control.update(wrong.in,0.001f);
    }
    CHECK(wrong.control.state().fault==Fault::WrongDirection);
    Robot timeout; timeout.cfg.command_timeout_s=0.01f; CHECK(timeout.control.startTurn(radians(90)));
    for(unsigned i=0;i<20;++i) timeout.control.update(timeout.in,0.001f);
    CHECK(timeout.control.state().fault==Fault::Timeout);

    Robot wall; wall.in.ir_valid[0]=wall.in.ir_valid[4]=true;
    wall.in.distance_m[0]=0.04f; wall.in.distance_m[4]=0.08f;
    wall.control.update(wall.in,0.001f); CHECK(wall.control.startStraight(0.18f,WallMode::Auto));
    for(unsigned i=0;i<100;++i) wall.control.update(wall.in,0.001f);
    CHECK(wall.control.state().wall_offset<0); // Gần tường trái => đổi hướng phải.
    wall.in.ir_valid[0]=wall.in.ir_valid[4]=false;
    for(unsigned i=0;i<100;++i) wall.control.update(wall.in,0.001f);
    CHECK(std::fabs(wall.control.state().wall_offset)<0.001f);
    wall.in.ir_age_s=0.030f; wall.control.update(wall.in,0.001f);
    CHECK(wall.control.state().fault==Fault::IrStale);
    Robot one_wall; one_wall.in.ir_valid[4]=true; one_wall.in.distance_m[4]=0.035f;
    one_wall.control.update(one_wall.in,0.001f);
    CHECK(one_wall.control.startStraight(0.18f,WallMode::Right));
    for(unsigned i=0;i<100;++i) one_wall.control.update(one_wall.in,0.001f);
    CHECK(one_wall.control.state().wall_offset>0); // Gần tường phải => đổi hướng trái.
    Robot obstacle; obstacle.cfg.front_guard=true;
    CHECK(obstacle.control.startStraight(0.18f)); obstacle.in.ir_valid[2]=true;
    obstacle.in.distance_m[2]=0.020f; obstacle.control.update(obstacle.in,0.001f);
    CHECK(obstacle.control.state().fault==Fault::Obstacle);
    Robot saturated; saturated.cfg.front_guard=true; CHECK(saturated.control.startStraight(0.18f));
    saturated.in.front_saturated=true; saturated.control.update(saturated.in,0.001f);
    CHECK(saturated.control.state().fault==Fault::Obstacle);
    std::puts("PASS: PI saturation, LUT, 16/32-bit wrap, straight/reverse, +/-90, full rotation, wheel test, faults, wall fallback, front stop.");
}
