#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// C bridge: include được từ main.c do CubeMX sinh. Implementation toàn bộ C++.
int MM_Init(void);             // Sau SystemClock_Config + tất cả MX_*_Init.
void MM_Poll(void);            // Gọi liên tục trong while(1), không HAL_Delay.
void MM_ControlTickISR(void);  // Từ callback TIM6, đúng 1 kHz.
void MM_UartRxISR(void);       // Từ UART3 RxCplt callback.
void MM_UartTxISR(void);       // Từ UART3 TxCplt callback.
void MM_UartErrorISR(void);    // Từ UART3 Error callback: dừng, yêu cầu reset MCU.
void MM_EmergencyStopISR(void);// Dùng cho nguồn STOP khác, không gọi HAL bus.

// Các hàm dưới CHỈ gọi ở main, không gọi trong ngắt. Return 1=nhận, 0=từ chối.
int MM_CalibrateGyro(void);    // Giữ robot đứng yên khoảng 2 giây, không blocking.
int MM_ArmMotion(void);       // Cần thông số thật, đúng chiều, gyro đã zero, không fault.
int MM_TestStraight(float distance_m, int wall_mode); // 0=off,1=auto,2=left,3=right.
int MM_TestTurn90(int left);  // left !=0 => +90 độ; left==0 => -90 độ, quay tại chỗ.
int MM_TestTurn(float angle_degrees);
int MM_TestWheelSpeed(float left_m_s, float right_m_s, float duration_s);
int MM_ArmBench(void);        // Chỉ mở test xung thô, chưa cần metres_per_count/PI.
int MM_TestMotorPulse(unsigned motor, int forward); // motor=0 trái,1 phải; 300 ms.
void MM_Stop(void);
int MM_ResetController(void); // Xóa fault/góc khi đang dừng, không re-init bus hỏng.
#ifdef __cplusplus
}
#endif
