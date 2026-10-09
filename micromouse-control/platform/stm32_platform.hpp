#pragma once
#include "board_config.hpp"
#include "mm/controller.hpp"

namespace firmware {
struct Diagnostics {
    uint32_t io_errors=0, ir_frames=0, gyro_samples=0, control_cycles=0;
    uint32_t last_control_work_us=0, max_control_work_us=0;
    uint32_t last_period_us=0, max_period_us=0, missed_ticks=0;
    uint16_t ir_off[5]{}, ir_on[5]{};
    int32_t ir_difference[5]{};
    int16_t gyro_raw[3]{};
    float gyro_bias=0, gyro_std=0;
    uint32_t calibration_count=0;
    bool calibrating=false, calibrated=false, calibration_failed=false;
    uint8_t who_am_i=0;
};
// Đọc watch variables bằng debugger khi cần xem chi tiết, UART chỉ in bản gọn.
extern Diagnostics diagnostics;
uint32_t micros();
bool initializeHardware();
mm::Input readSensors(); // Gọi mỗi chu kỳ điều khiển, encoder + gyro; không đọc accel.
void pollIr();           // State machine theo nhóm, gọi liên tục từ main.
void abortIr();
bool beginCalibration();
bool isCalibrating();
void cancelCalibration();
void allOutputsOff();    // Dùng được trong ISR: không HAL_Delay, không SPI/ADC.
void setMotorOutputs(float left, float right);
void setDriveEnabled(bool enable);
bool driveEnabled();
bool motorHardwareReady();
bool encoderHardwareReady();
bool gyroHardwareReady();
}
