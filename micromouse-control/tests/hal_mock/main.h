#pragma once
// HAL GIẢ, chỉ compile trên PC. TUYỆT ĐỐI không thêm tests/hal_mock vào CubeIDE.
#include <cstdint>
struct GPIO_TypeDef { uint16_t state=0; };
struct TIM_TypeDef { uint32_t CNT=0,ARR=65535,CCR[4]{}; };
struct TIM_HandleTypeDef { TIM_TypeDef* Instance; };
struct ADC_HandleTypeDef {};
struct SPI_HandleTypeDef {};
struct UART_HandleTypeDef {};
struct ADC_ChannelConfTypeDef { uint32_t Channel,Rank,SamplingTime; };
enum GPIO_PinState { GPIO_PIN_RESET,GPIO_PIN_SET };
enum HAL_StatusTypeDef { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT };
extern GPIO_TypeDef gpio_a,gpio_b,gpio_c;
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define GPIOC (&gpio_c)
constexpr uint16_t GPIO_PIN_1=2,GPIO_PIN_2=4,GPIO_PIN_5=32,GPIO_PIN_8=256,GPIO_PIN_9=512,GPIO_PIN_12=4096;
constexpr uint32_t TIM_CHANNEL_1=0,TIM_CHANNEL_2=4,TIM_CHANNEL_ALL=60;
constexpr uint32_t ADC_CHANNEL_3=3,ADC_CHANNEL_4=4,ADC_CHANNEL_8=8,ADC_CHANNEL_9=9,ADC_CHANNEL_10=10;
constexpr uint32_t ADC_SAMPLETIME_144CYCLES=6;
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[(c)/4]=(v))
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t);
void __DMB();
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
