/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hardware_tests_api.h"
#include "icm20602_minimal_c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_IDENTITY_ONLY 0/* 0: gyro C++; 1/2: probe C++; 3: ID C; 4: rà cấu hình C;
                              5: thử gyro C dù ID sai, xem icm_c_gyro. */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t test_started = 0; /* Watch hiển thị số, không kèm ký tự char. */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); 

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if GYRO_IDENTITY_ONLY >= 3
  MX_SPI2_Init(); /* Bài C chỉ khởi tạo GPIO/SPI, loại ảnh hưởng ADC/TIM/UART. */
#else
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
#endif
  /* USER CODE BEGIN 2 */
  test_started = 0; /* Giữ biến trạng thái trong ELF để Watch đọc đúng ở mode chẩn đoán. */
#if GYRO_IDENTITY_ONLY == 5
  test_started = IcmC_GyroBeginUnverified(); /* Chỉ kết quả readback, chưa xác nhận IMU. */
  IcmC_AnglesStartCalibration(2000); /* Giữ module yên, chờ ready=1 (2000 mẫu).
                                     Poll tự hiệu chuẩn rồi cập nhật roll/pitch/yaw. */
#elif GYRO_IDENTITY_ONLY >= 3
  IcmC_Begin(); /* Bài test C độc lập; không gọi Begin/driver C++ trước bài này. */
#if GYRO_IDENTITY_ONLY == 4
  IcmC_ConfigAudit(); /* Dò cấu hình một lần, khôi phục nếu không có ID12 ổn định. */
#endif
#elif GYRO_IDENTITY_ONLY
  HardwareTests_ReadOnlyGyroProbe(); /* Không gọi Begin trước bài chỉ đọc. */
#if GYRO_IDENTITY_ONLY == 2
  HardwareTests_ProbeGyroBus(); /* Ghi divider tạm thời rồi khôi phục, không bật gyro. */
  HardwareTests_ProbeGyroI2c(); /* Đối chiếu ID độc lập qua I2C, sau đó trả pin về SPI. */
#endif
#else
  HardwareTests_Begin();
  test_started = HardwareTests_TestIR();
  /* Chỉ khi init thường thất bại: dò mode/tốc độ một lần, giữ nguyên pin.
     Hàm dò không báo thành công chỉ vì HAL_OK: phải đúng ID và readback. */
  // if (!test_started && HardwareTests_ProbeGyroSpi())
  // {
  //   test_started = HardwareTests_TestGyro();
  // }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if GYRO_IDENTITY_ONLY == 5
    IcmC_GyroPollUnverified(); /* Bài bench: dữ liệu gyro, ID thực vẫn được giữ lại. */
#elif GYRO_IDENTITY_ONLY >= 3
    IcmC_Poll(); /* Chỉ kiểm tra ID, xem biến icm_c trong Watch. */
#else
    HardwareTests_Update();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
