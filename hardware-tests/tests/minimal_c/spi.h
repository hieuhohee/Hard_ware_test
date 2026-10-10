#ifndef ICM_C_TEST_SPI_H
#define ICM_C_TEST_SPI_H
/* HAL giả cho host test của source C thật; không thêm path này vào firmware. */
#include <stdint.h>
typedef struct { uint32_t MODER, AFR[2], OSPEEDR, PUPDR; } GPIO_TypeDef;
typedef struct { uint32_t CR1; } SPI_TypeDef;
typedef struct { uint32_t CR, PLLCFGR, CFGR; } RCC_TypeDef;
typedef struct { uint32_t CLKPolarity, CLKPhase, BaudRatePrescaler; } SPI_InitTypeDef;
typedef struct { SPI_TypeDef *Instance; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef fake_gpio;
extern SPI_TypeDef fake_spi;
extern RCC_TypeDef fake_rcc;
extern SPI_HandleTypeDef hspi2;
#define GPIOB (&fake_gpio)
#define SPI2 (&fake_spi)
#define RCC (&fake_rcc)
#define GPIO_PIN_12 (1u << 12)
#define SPI_BAUDRATEPRESCALER_64 5u
#define SPI_BAUDRATEPRESCALER_128 6u
#define SPI_BAUDRATEPRESCALER_256 7u
#define SPI_POLARITY_LOW 0u
#define SPI_POLARITY_HIGH 2u
#define SPI_PHASE_1EDGE 0u
#define SPI_PHASE_2EDGE 1u
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#endif
