#include "micromouse_app.h"
#include "stm32_platform.hpp"
#include <cstdio>
#include <cmath>

namespace {
mm::Controller controller(board::control);
mm::Input input;
volatile uint32_t ticks=0, serviced_ticks=0;
volatile unsigned emergency=0; // 1=user stop,2=watchdog,3=RX overflow,4=UART error.
volatile bool tx_busy=false;
volatile uint8_t rx_head=0,rx_tail=0;
uint8_t rx_byte=0,rx_ring[16]{};
char tx_buffer[640]; // Giữ nguyên suốt HAL_UART_Transmit_IT; không là biến stack.
bool initialized=false,motion_armed=false,bench_armed=false,pulse_active=false;
bool uart_ok=true;
unsigned handled_emergency=0;
unsigned pulse_motor=0;
float pulse_output=0;
uint32_t pulse_started=0,last_control_us=0,last_telemetry_us=0;
bool calibration_was_running=false;
unsigned selected_test=0;
int last_command_result=0;

struct Button {
    bool raw=false,stable=false;
    uint32_t changed=0;
    // Nút active LOW, chỉ phát sự kiện cạnh nhấn sau 20ms. Giữ nút không tự lặp.
    bool pressed(board::Pin pin,uint32_t now) {
        const bool value=HAL_GPIO_ReadPin(pin.port,pin.number)==GPIO_PIN_RESET;
        if(value!=raw) { raw=value; changed=now; }
        if(raw!=stable && uint32_t(now-changed)>=20000u) {
            stable=raw; return stable;
        }
        return false;
    }
} start_button,mode_button;

bool busy() { return controller.active() || pulse_active || firmware::isCalibrating(); }
bool armOutput() {
    const uint32_t mask=__get_PRIMASK(); __disable_irq();
    // Không xóa sự kiện STOP vừa tới trong lúc main đang chuẩn bị lệnh.
    const bool ok=emergency==0 && uart_ok && firmware::motorHardwareReady();
    if(ok) { serviced_ticks=ticks; firmware::setDriveEnabled(true); }
    __set_PRIMASK(mask); return ok;
}
bool acceptMotion(bool accepted) {
    motion_armed=false; // Một lần arm chỉ nhận một lệnh, không tự chạy lệnh tiếp.
    if(!accepted) return false;
    if(!armOutput()) { controller.fail(mm::Fault::Emergency); return false; }
    return true;
}
bool readyToCommand() {
    return initialized && motion_armed && !busy() && emergency==0;
}
void transmit(const char* text) {
    if(tx_busy || !uart_ok) return;
    const int length=std::snprintf(tx_buffer,sizeof(tx_buffer),"%s",text);
    if(length<=0) return;
    tx_busy=true;
    if(HAL_UART_Transmit_IT(board::uart,reinterpret_cast<uint8_t*>(tx_buffer),
        uint16_t(length<int(sizeof(tx_buffer))?length:int(sizeof(tx_buffer))-1))!=HAL_OK) tx_busy=false;
}
void help() {
    transmit("c gyro zero | r reset controller | e arm motion | b arm raw pulse\r\n"
             "f straight 180mm | L left90 | R right90 | w straight180 + walls\r\n"
             "p PI test 0.05m/s 1s | 1 L+ | 2 L- | 3 R+ | 4 R- | s/SPACE STOP\r\n"
             "Mode button selects f/L/R/w; Start executes only after e. No auto-repeat.\r\n"
             "CSV: state,fault,cmd,tick,work_us,maxwork_us,period_us,maxperiod_us,missed,Lcount,Rcount,Lmmps,Rmmps,Lref,Rref,yaw_mdeg,Lduty1000,Rduty1000,IR1,IR2,IR3,IR4,IR5,cal,N,errors\r\n");
}
void execute(char c) {
    switch(c) {
    case 's': case ' ': MM_Stop(); last_command_result=1; break;
    case 'c': last_command_result=MM_CalibrateGyro(); break;
    case 'r': last_command_result=MM_ResetController(); break;
    case 'e': last_command_result=MM_ArmMotion(); break;
    case 'b': last_command_result=MM_ArmBench(); break;
    case 'f': last_command_result=MM_TestStraight(0.18f,0); break;
    case 'w': last_command_result=MM_TestStraight(0.18f,1); break;
    case 'L': last_command_result=MM_TestTurn90(1); break;
    case 'R': last_command_result=MM_TestTurn90(0); break;
    case 'p': last_command_result=MM_TestWheelSpeed(0.05f,0.05f,1.0f); break;
    case '1': case '2': case '3': case '4':
        last_command_result=MM_TestMotorPulse(unsigned(c-'1')/2,(c-'1')%2==0); break;
    case 'h': case '?': help(); break;
    default: break; // CR/LF và ký tự khác không tác động motor.
    }
}
void telemetry(uint32_t now) {
    if(tx_busy || !uart_ok || uint32_t(now-last_telemetry_us)<100000u) return;
    last_telemetry_us=now;
    const auto& s=controller.state(); const auto& d=firmware::diagnostics;
    // Số nguyên có hệ số rõ ràng, không cần newlib printf float, không chặn main.
    const int n=std::snprintf(tx_buffer,sizeof(tx_buffer),
        "%u,%u,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lld,%lld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%lu,%lu\r\n",
        unsigned(s.mode),unsigned(s.fault),last_command_result,
        (unsigned long)d.control_cycles,(unsigned long)d.last_control_work_us,(unsigned long)d.max_control_work_us,
        (unsigned long)d.last_period_us,(unsigned long)d.max_period_us,(unsigned long)d.missed_ticks,
        (long long)input.ticks[0],(long long)input.ticks[1],
        (long)(s.speed[0]*1000),(long)(s.speed[1]*1000),
        (long)(s.wheel_reference[0]*1000),(long)(s.wheel_reference[1]*1000),
        (long)(s.yaw*180000/mm::pi),(long)(s.duty[0]*1000),(long)(s.duty[1]*1000),
        (long)d.ir_difference[0],(long)d.ir_difference[1],(long)d.ir_difference[2],
        (long)d.ir_difference[3],(long)d.ir_difference[4],unsigned(d.calibrated),
        (unsigned long)d.calibration_count,(unsigned long)d.io_errors);
    if(n<=0) return;
    tx_busy=true;
    if(HAL_UART_Transmit_IT(board::uart,reinterpret_cast<uint8_t*>(tx_buffer),
        uint16_t(n<int(sizeof(tx_buffer))?n:int(sizeof(tx_buffer))-1))!=HAL_OK) tx_busy=false;
}
}

extern "C" int MM_Init() {
    initialized=false; ticks=serviced_ticks=0; emergency=0; handled_emergency=0;
    motion_armed=bench_armed=pulse_active=false;
    const bool ready=firmware::initializeHardware();
    input=firmware::readSensors(); controller.reset(input);
    last_control_us=last_telemetry_us=firmware::micros();
    rx_head=rx_tail=0; tx_busy=false;
    uart_ok=HAL_UART_Receive_IT(board::uart,&rx_byte,1)==HAL_OK;
    if(HAL_TIM_Base_Start_IT(board::scheduler_timer)!=HAL_OK) {
        firmware::allOutputsOff(); return 0;
    }
    initialized=ready && uart_ok; help(); return initialized?1:0;
}
extern "C" void MM_EmergencyStopISR() {
    firmware::allOutputsOff(); if(!emergency) emergency=1;
}
extern "C" void MM_ControlTickISR() {
    ++ticks;
    if(firmware::driveEnabled() && uint32_t(ticks-serviced_ticks)>=board::control_watchdog_ticks) {
        firmware::allOutputsOff(); emergency=2;
    }
}
extern "C" void MM_UartRxISR() {
    if(rx_byte=='s' || rx_byte==' ') MM_EmergencyStopISR();
    else {
        const uint8_t next=uint8_t((rx_head+1u)%16u);
        if(next==rx_tail) { firmware::allOutputsOff(); emergency=3; }
        else { rx_ring[rx_head]=rx_byte; __DMB(); rx_head=next; }
    }
    if(HAL_UART_Receive_IT(board::uart,&rx_byte,1)!=HAL_OK) MM_UartErrorISR();
}
extern "C" void MM_UartTxISR() { tx_busy=false; }
extern "C" void MM_UartErrorISR() { firmware::allOutputsOff(); emergency=4; }
extern "C" void MM_Stop() {
    firmware::allOutputsOff(); controller.stop(); firmware::abortIr(); firmware::cancelCalibration();
    pulse_active=motion_armed=bench_armed=false;
}
extern "C" int MM_ResetController() {
    if(!initialized || busy() || !uart_ok) return 0;
    firmware::allOutputsOff(); input=firmware::readSensors(); controller.reset(input);
    motion_armed=bench_armed=false;
    const uint32_t mask=__get_PRIMASK(); __disable_irq();
    // Xóa các lệnh còn nằm trong queue: không thực thi lại lệnh trước sự cố.
    rx_tail=rx_head; emergency=0; handled_emergency=0; serviced_ticks=ticks;
    __set_PRIMASK(mask); last_control_us=firmware::micros(); return 1;
}
extern "C" int MM_CalibrateGyro() {
    if(!initialized || busy() || emergency || controller.state().mode==mm::Mode::Fault) return 0;
    motion_armed=bench_armed=false;
    return firmware::beginCalibration()?1:0;
}
extern "C" int MM_ArmMotion() {
    if(!initialized || busy() || emergency || !uart_ok || !controller.configured() ||
       controller.state().mode==mm::Mode::Fault || !input.encoder_ok || !input.gyro_ok ||
       !input.gyro_calibrated || input.gyro_age_s>board::control.gyro_stale_s) return 0;
    motion_armed=true; bench_armed=false; return 1;
}
extern "C" int MM_TestStraight(float metres,int walls) {
    if(!readyToCommand() || walls<0 || walls>3) return 0;
    // Có frame ADC chưa đủ: cần LUT thật cho cặp hông trước khi bật bám tường.
    if(walls && (!board::ir_curves[0].valid() || !board::ir_curves[4].valid())) return 0;
    if(board::control.front_guard && !board::ir_curves[2].valid()) return 0;
    return acceptMotion(controller.startStraight(metres,mm::WallMode(walls)))?1:0;
}
extern "C" int MM_TestTurn(float degrees) {
    if(!readyToCommand()) return 0;
    return acceptMotion(controller.startTurn(mm::radians(degrees)))?1:0;
}
extern "C" int MM_TestTurn90(int left) { return MM_TestTurn(left?90.0f:-90.0f); }
extern "C" int MM_TestWheelSpeed(float left,float right,float duration) {
    if(!readyToCommand()) return 0;
    return acceptMotion(controller.startWheelSpeed(left,right,duration))?1:0;
}
extern "C" int MM_ArmBench() {
    if(!initialized || busy() || emergency || !uart_ok || !firmware::motorHardwareReady() ||
       controller.state().mode==mm::Mode::Fault) return 0;
    bench_armed=true; motion_armed=false; return 1;
}
extern "C" int MM_TestMotorPulse(unsigned motor,int forward) {
    if(!initialized || !bench_armed || busy() || motor>1 || emergency) return 0;
    bench_armed=false; pulse_motor=motor;
    pulse_output=(forward?1.0f:-1.0f)*board::bench_duty;
    pulse_started=firmware::micros();
    if(!armOutput()) return 0;
    pulse_active=true; return 1;
}
extern "C" void MM_Poll() {
    if(!initialized) { firmware::allOutputsOff(); return; }
    if(emergency && emergency!=handled_emergency) {
        const unsigned cause=emergency;
        handled_emergency=cause;
        MM_Stop();
        const uint32_t mask=__get_PRIMASK(); __disable_irq();
        rx_tail=rx_head;
        if(cause==1 && emergency==1) { emergency=0; handled_emergency=0; } // STOP thường không giữ fault.
        __set_PRIMASK(mask);
        if(cause>1) controller.fail(cause==2?mm::Fault::Timing:mm::Fault::Emergency);
        if(cause==4) uart_ok=false;
    }
    // Tối đa một lệnh/main pass, không để UART chiếm hết ngân sách điều khiển.
    if(rx_tail!=rx_head) {
        __DMB(); const char c=char(rx_ring[rx_tail]); rx_tail=uint8_t((rx_tail+1u)%16u); execute(c);
    }
    const uint32_t current_ticks=ticks;
    if(current_ticks!=serviced_ticks) {
        const uint32_t pending=current_ticks-serviced_ticks;
        serviced_ticks=current_ticks;
        const uint32_t started=firmware::micros();
        const uint32_t period=started-last_control_us;
        firmware::diagnostics.last_period_us=period;
        if(period>firmware::diagnostics.max_period_us) firmware::diagnostics.max_period_us=period;
        if(pending>1) firmware::diagnostics.missed_ticks+=pending-1;
        const float dt=float(period)*1e-6f; last_control_us=started;
        if(pending>1 && (controller.active() || pulse_active)) {
            firmware::allOutputsOff(); controller.fail(mm::Fault::Timing); pulse_active=false;
        }
        input=firmware::readSensors();
        if(calibration_was_running && !firmware::isCalibrating() && input.gyro_calibrated) controller.reset(input);
        calibration_was_running=firmware::isCalibrating();
        controller.update(input,dt);
        if(pulse_active) {
            if(uint32_t(firmware::micros()-pulse_started)>=board::bench_pulse_us) {
                pulse_active=false; firmware::allOutputsOff();
            } else firmware::setMotorOutputs(pulse_motor==0?pulse_output:0,pulse_motor==1?pulse_output:0);
        } else if(controller.active()) {
            const auto& s=controller.state(); firmware::setMotorOutputs(s.duty[0],s.duty[1]);
        } else if(firmware::driveEnabled()) firmware::allOutputsOff();
        if(controller.state().mode==mm::Mode::Fault) motion_armed=bench_armed=false;
        ++firmware::diagnostics.control_cycles;
        const uint32_t work=firmware::micros()-started;
        firmware::diagnostics.last_control_work_us=work;
        if(work>firmware::diagnostics.max_control_work_us) firmware::diagnostics.max_control_work_us=work;
    }
    firmware::pollIr();
    const uint32_t now=firmware::micros();
    if(mode_button.pressed(board::mode_button,now) && !busy()) selected_test=(selected_test+1)%4;
    if(start_button.pressed(board::start_button,now)) {
        if(busy()) MM_Stop();
        else { const char selections[]={'f','L','R','w'}; execute(selections[selected_test]); }
    }
    telemetry(now);
}
