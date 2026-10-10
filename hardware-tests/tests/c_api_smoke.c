/* Kiểm tra header dùng được từ main.c và link tới triển khai C++. */
#include "hardware_tests_api.h"

uint8_t bench_test_c_api_start_gyro(void)
{
    HardwareTests_Begin();
    return HardwareTests_TestGyro();
}
