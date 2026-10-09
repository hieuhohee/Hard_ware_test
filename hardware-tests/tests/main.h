#pragma once
// HAL giả dùng cho host test. KHÔNG thêm thư mục tests vào include path firmware.
#include <cstdint>
#include <cstring>
struct GPIO_TypeDef { uint16_t state = 0; };
struct TIM_TypeDef { uint32_t CNT=0, ARR=65535, CCR[4]{}; };
struct TIM_HandleTypeDef { TIM_TypeDef* Instance; };
struct UART_HandleTypeDef {};
struct ADC_HandleTypeDef {};
struct SPI_HandleTypeDef {};
struct ADC_ChannelConfTypeDef { uint32_t Channel, Rank, SamplingTime; };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
constexpr uint32_t TIM_CHANNEL_1=0, TIM_CHANNEL_2=4, TIM_CHANNEL_3=8, TIM_CHANNEL_4=12;
constexpr uint32_t TIM_CHANNEL_ALL=60, ADC_CHANNEL_15=15, ADC_SAMPLETIME_144CYCLES=6;
struct DwtType { uint32_t CYCCNT=0, CTRL=0; };
struct CoreDebugType { uint32_t DEMCR=0; };
extern DwtType fake_dwt;
extern CoreDebugType fake_debug;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_debug)
constexpr uint32_t CoreDebug_DEMCR_TRCENA_Msk=1, DWT_CTRL_CYCCNTENA_Msk=1;
extern uint32_t SystemCoreClock;
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[(c)/4]=(v))
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
uint32_t HAL_GetTick();
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
