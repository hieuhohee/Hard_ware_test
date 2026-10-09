#include "stm32_platform.hpp"
#include <cmath>
#include <cstring>

namespace firmware {
Diagnostics diagnostics{};
static volatile bool drive_enabled=false;
static volatile bool ir_interrupted=false;
static bool motor_ready=false, encoder_ready=false, gyro_ready=false, ir_ok=false;
static uint32_t old_counts[2]{}, last_gyro_us=0, last_ir_us=0;
static int64_t total_counts[2]{}, calibration_start_counts[2]{};
static float corrected_gyro=0;
static bool have_gyro=false, have_ir=false;
static uint32_t calibration_started=0;
static float calibration_mean=0, calibration_m2=0;
static int last_direction[2]{};
static uint32_t coast_started[2]{};
static bool coasting[2]{};
// IR tạm thời chỉ publish khi đủ 5 mắt. Controller không đọc nửa frame cũ/mới.
static uint16_t pending_off[5]{}, pending_on[5]{};
static unsigned ir_group=0, ir_index=0, ir_phase=0;
static uint32_t ir_deadline_start=0, ir_frame_start=0, led_started=0;
static bool led_on=false;
static void gpio(board::Pin p,bool value) { HAL_GPIO_WritePin(p.port,p.number,value?GPIO_PIN_SET:GPIO_PIN_RESET); }
uint32_t micros() { return __HAL_TIM_GET_COUNTER(board::micros_timer); }
static void ledsOff() { for(auto p:board::emitters) gpio(p,false); }
void allOutputsOff() {
    drive_enabled=false;
    for(const auto& m:board::motors) {
        __HAL_TIM_SET_COMPARE(m.timer,m.in1_channel,0);
        __HAL_TIM_SET_COMPARE(m.timer,m.in2_channel,0);
    }
    ledsOff();
    ir_interrupted=true; // Một ISR có thể vừa cắt xung IR đang đo: bỏ frame dở.
}
void setDriveEnabled(bool value) {
    if(!value) { allOutputsOff(); return; }
    drive_enabled=motor_ready;
}
bool driveEnabled() { return drive_enabled; }
bool motorHardwareReady() { return motor_ready; }
bool encoderHardwareReady() { return encoder_ready; }
bool gyroHardwareReady() { return gyro_ready; }
void setMotorOutputs(float left,float right) {
    const float duties[2]={left,right};
    // ISR watchdog/STOP không được chen vào giữa kiểm tra gate và ghi CCR.
    const uint32_t mask=__get_PRIMASK(); __disable_irq();
    if(!drive_enabled || !std::isfinite(left) || !std::isfinite(right)) {
        allOutputsOff(); __set_PRIMASK(mask); return;
    }
    const uint32_t now=micros();
    for(unsigned i=0;i<2;++i) {
        const auto& m=board::motors[i];
        const float duty=mm::clamp(duties[i]*float(m.sign),-1,1);
        const int direction=duty>0?1:(duty<0?-1:0);
        if(!std::isfinite(duty)) { allOutputsOff(); break; }
        // Cả hai đầu vào 0 = coast. Giữ tối thiểu 100 us khi đổi cực; thời gian
        // thực tế có thể là 1 tick điều khiển. Không dùng delay trong critical section.
        if(direction && last_direction[i] && direction!=last_direction[i] && !coasting[i]) {
            coast_started[i]=now; coasting[i]=true;
        }
        __HAL_TIM_SET_COMPARE(m.timer,m.in1_channel,0);
        __HAL_TIM_SET_COMPARE(m.timer,m.in2_channel,0);
        if(coasting[i] && uint32_t(now-coast_started[i])<board::reverse_coast_us) continue;
        coasting[i]=false;
        if(direction) {
            const uint32_t period=__HAL_TIM_GET_AUTORELOAD(m.timer)+1u;
            const uint32_t compare=uint32_t(float(period)*std::fabs(duty));
            __HAL_TIM_SET_COMPARE(m.timer,direction>0?m.in1_channel:m.in2_channel,compare);
            last_direction[i]=direction;
        }
    }
    __set_PRIMASK(mask);
}
static bool readReg(uint8_t reg,uint8_t* data,unsigned count) {
    if(count>6) return false;
    uint8_t tx[7]{},rx[7]{}; tx[0]=reg|0x80;
    gpio(board::imu_cs,false);
    const bool ok=HAL_SPI_TransmitReceive(board::spi,tx,rx,uint16_t(count+1),board::bus_timeout_ms)==HAL_OK;
    gpio(board::imu_cs,true);
    if(ok) std::memcpy(data,rx+1,count); else ++diagnostics.io_errors;
    return ok;
}
static bool writeReg(uint8_t reg,uint8_t value) {
    uint8_t tx[2]={uint8_t(reg&0x7f),value}; gpio(board::imu_cs,false);
    const bool ok=HAL_SPI_Transmit(board::spi,tx,2,board::bus_timeout_ms)==HAL_OK;
    gpio(board::imu_cs,true); if(!ok) ++diagnostics.io_errors; return ok;
}
static bool initGyro() {
    gpio(board::imu_cs,true); HAL_Delay(100); // Chỉ init khi PWM đã bằng 0.
    if(!readReg(0x75,&diagnostics.who_am_i,1) || diagnostics.who_am_i!=0x12) return false;
    if(!writeReg(0x6b,0x80)) return false;
    HAL_Delay(100);
    // Tắt I2C, chọn clock gyro, standby cả 3 accel, gyro DLPF92Hz/ODR1kHz/2000dps.
    // Bật DATA_RDY_INT ở chính sensor dù MCU không dùng EXTI: đọc INT_STATUS
    // mới có sự kiện data-ready được bật tường minh. Latch pin INT tới khi đọc status.
    const uint8_t regs[]={0x70,0x6b,0x6c,0x1a,0x19,0x1b,0x37,0x38};
    const uint8_t values[]={0x40,0x01,0x38,0x02,0x00,0x18,0x20,0x01};
    for(unsigned i=0;i<sizeof(regs);++i) if(!writeReg(regs[i],values[i])) return false;
    HAL_Delay(100);
    for(unsigned i=0;i<sizeof(regs);++i) {
        uint8_t read=0; if(!readReg(regs[i],&read,1) || read!=values[i]) return false;
    }
    return true;
}
bool initializeHardware() {
    allOutputsOff(); diagnostics=Diagnostics{};
    have_gyro=have_ir=false; ir_ok=false;
    if(HAL_TIM_Base_Start(board::micros_timer)!=HAL_OK) return false;
    motor_ready=encoder_ready=true;
    for(unsigned i=0;i<2;++i) {
        const auto& m=board::motors[i];
        const bool a=HAL_TIM_PWM_Start(m.timer,m.in1_channel)==HAL_OK;
        const bool b=HAL_TIM_PWM_Start(m.timer,m.in2_channel)==HAL_OK;
        motor_ready=motor_ready && a && b && (m.sign==1 || m.sign==-1);
        const auto& e=board::encoders[i]; const uint32_t arr=__HAL_TIM_GET_AUTORELOAD(e.timer);
        const bool ready=HAL_TIM_Encoder_Start(e.timer,TIM_CHANNEL_ALL)==HAL_OK;
        encoder_ready=encoder_ready && ready && (e.sign==1 || e.sign==-1) && (arr==65535u || arr==0xffffffffu);
        old_counts[i]=__HAL_TIM_GET_COUNTER(e.timer); total_counts[i]=0;
        last_direction[i]=0; coasting[i]=false;
    }
    gyro_ready=initGyro(); abortIr();
    // Cho test thô motor/encoder ngay cả khi gyro chưa gắn. Core chặn motion khi thiếu gyro.
    return motor_ready && encoder_ready;
}
bool beginCalibration() {
    if(drive_enabled || !gyro_ready || !encoder_ready) return false;
    diagnostics.calibrated=false; diagnostics.calibrating=true; diagnostics.calibration_failed=false;
    diagnostics.calibration_count=0; calibration_mean=calibration_m2=0;
    calibration_started=micros();
    for(unsigned i=0;i<2;++i) calibration_start_counts[i]=total_counts[i];
    return true;
}
bool isCalibrating() { return diagnostics.calibrating; }
void cancelCalibration() {
    if(diagnostics.calibrating) {
        diagnostics.calibrating=false; diagnostics.calibrated=false; diagnostics.calibration_failed=true;
    }
}
static void calibrationSample(float rate) {
    if(!diagnostics.calibrating) return;
    ++diagnostics.calibration_count;
    const float delta=rate-calibration_mean;
    calibration_mean+=delta/float(diagnostics.calibration_count);
    calibration_m2+=delta*(rate-calibration_mean); // Welford: variance ổn định số học.
    if(diagnostics.calibration_count>=board::gyro_calibration_samples) {
        diagnostics.gyro_std=std::sqrt(calibration_m2/float(diagnostics.calibration_count-1));
        diagnostics.calibrating=false;
        diagnostics.calibrated=diagnostics.gyro_std<board::gyro_calibration_std_max &&
                               std::fabs(calibration_mean)<board::gyro_calibration_mean_max;
        diagnostics.calibration_failed=!diagnostics.calibrated;
        if(diagnostics.calibrated) diagnostics.gyro_bias=calibration_mean;
    }
}
mm::Input readSensors() {
    const uint32_t now=micros();
    mm::Input in; in.encoder_ok=encoder_ready;
    for(unsigned i=0;i<2;++i) {
        const auto& e=board::encoders[i]; const uint32_t count=__HAL_TIM_GET_COUNTER(e.timer);
        total_counts[i]+=mm::encoderDelta(count,old_counts[i],__HAL_TIM_GET_AUTORELOAD(e.timer))*e.sign;
        old_counts[i]=count; in.ticks[i]=total_counts[i];
    }
    if(diagnostics.calibrating && (uint32_t(now-calibration_started)>5000000u ||
       total_counts[0]!=calibration_start_counts[0] || total_counts[1]!=calibration_start_counts[1])) cancelCalibration();
    if(gyro_ready) {
        uint8_t status=0, bytes[6]{};
        if(!readReg(0x3a,&status,1)) { gyro_ready=false; cancelCalibration(); }
        else if(status&1) {
            if(!readReg(0x43,bytes,6)) { gyro_ready=false; cancelCalibration(); }
            else {
                for(unsigned i=0;i<3;++i) {
                    const uint16_t u=uint16_t((uint16_t(bytes[2*i])<<8)|bytes[2*i+1]);
                    diagnostics.gyro_raw[i]=int16_t(u<32768u?int32_t(u):int32_t(u)-65536);
                }
                static_assert(board::gyro_axis<3,"gyro_axis must be X, Y or Z");
                const float raw=mm::radians(float(diagnostics.gyro_raw[board::gyro_axis])/16.4f)*float(board::gyro_sign);
                calibrationSample(raw);
                corrected_gyro=(raw-diagnostics.gyro_bias)*board::gyro_scale;
                ++diagnostics.gyro_samples; last_gyro_us=micros(); have_gyro=true;
            }
        }
    }
    in.gyro_ok=gyro_ready; in.gyro_rad_s=corrected_gyro;
    in.gyro_age_s=have_gyro?float(uint32_t(micros()-last_gyro_us))*1e-6f:1;
    in.gyro_calibrated=diagnostics.calibrated;
    in.ir_frame_ok=have_ir && ir_ok;
    in.ir_age_s=have_ir?float(uint32_t(micros()-last_ir_us))*1e-6f:1;
    for(unsigned i=0;i<5;++i) {
        in.ir_valid[i]=in.ir_frame_ok && diagnostics.ir_on[i]<4090 && diagnostics.ir_off[i]<4090 &&
            board::ir_curves[i].convert(float(diagnostics.ir_difference[i]),in.distance_m[i]);
    }
    in.front_saturated=diagnostics.ir_on[2]>=4090 || diagnostics.ir_off[2]>=4090;
    return in;
}
void abortIr() {
    ledsOff(); led_on=false; ir_phase=ir_group=ir_index=0; ir_frame_start=micros();
    ir_interrupted=false;
}
static bool adcRead(unsigned i,uint16_t& value) {
    ADC_ChannelConfTypeDef channel{};
    channel.Channel=board::receivers[i].adc_channel; channel.Rank=1;
    channel.SamplingTime=ADC_SAMPLETIME_144CYCLES;
    bool ok=HAL_ADC_ConfigChannel(board::adc,&channel)==HAL_OK;
    if(ok) ok=HAL_ADC_Start(board::adc)==HAL_OK;
    if(ok) ok=HAL_ADC_PollForConversion(board::adc,board::bus_timeout_ms)==HAL_OK;
    if(ok) value=uint16_t(HAL_ADC_GetValue(board::adc));
    HAL_ADC_Stop(board::adc);
    if(!ok) { ++diagnostics.io_errors; ir_ok=false; abortIr(); }
    return ok;
}
void pollIr() {
    if(ir_interrupted) { ir_ok=false; abortIr(); return; }
    const uint32_t now=micros();
    if(led_on && uint32_t(now-led_started)>board::max_led_on_us) {
        ++diagnostics.io_errors; ir_ok=false; abortIr(); return;
    }
    if(ir_phase==0) {
        if(uint32_t(now-ir_frame_start)<board::ir_frame_period_us) return;
        ir_frame_start=now; ledsOff(); ir_group=0; ir_phase=1; ir_deadline_start=now;
    } else if(ir_phase==1 && uint32_t(now-ir_deadline_start)>=board::ir_settle_us) {
        ir_index=0; ir_phase=2; // Đọc nền cho mọi mắt của nhóm trước khi bật nhóm đó.
    } else if(ir_phase==2 || ir_phase==4) {
        while(ir_index<5 && board::receivers[ir_index].group!=ir_group) ++ir_index;
        if(ir_index<5) {
            const unsigned i=ir_index++;
            if(!adcRead(i,ir_phase==2?pending_off[i]:pending_on[i])) return;
        } else if(ir_phase==2) {
            gpio(board::emitters[ir_group],true); led_on=true;
            led_started=micros(); ir_deadline_start=led_started; ir_phase=3;
        } else {
            ledsOff(); led_on=false;
            if(++ir_group==3) {
                for(unsigned i=0;i<5;++i) {
                    diagnostics.ir_off[i]=pending_off[i]; diagnostics.ir_on[i]=pending_on[i];
                    diagnostics.ir_difference[i]=(int32_t(pending_on[i])-int32_t(pending_off[i]))*board::receivers[i].signal_sign;
                }
                ++diagnostics.ir_frames; last_ir_us=micros(); have_ir=ir_ok=true; ir_phase=0;
            } else { ir_deadline_start=micros(); ir_phase=1; }
        }
    } else if(ir_phase==3 && uint32_t(now-ir_deadline_start)>=board::ir_settle_us) {
        ir_index=0; ir_phase=4;
    }
}
}
