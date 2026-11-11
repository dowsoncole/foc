#ifndef SPI3_TEST_HAL_H
#define SPI3_TEST_HAL_H
#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CR1; } SPI_TypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct {
  uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
    BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, NSSPMode;
} SPI_InitTypeDef;
typedef struct {
  SPI_TypeDef *Instance;
  SPI_InitTypeDef Init;
  uint32_t ErrorCode;
} SPI_HandleTypeDef;

extern SPI_TypeDef test_spi3;
extern GPIO_TypeDef test_gpiob;
extern uint32_t SystemCoreClock;
#define SPI3 (&test_spi3)
#define GPIOB (&test_gpiob)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 2U
#define SPI_MODE_MASTER 4U
#define SPI_DIRECTION_2LINES 0U
#define SPI_DATASIZE_8BIT 7U
#define SPI_POLARITY_LOW 0U
#define SPI_PHASE_2EDGE 1U
#define SPI_NSS_SOFT (1U << 9)
#define SPI_BAUDRATEPRESCALER_256 (7U << 3)
#define SPI_FIRSTBIT_MSB 0U
#define SPI_TIMODE_DISABLE 0U
#define SPI_CRCCALCULATION_DISABLE 0U
#define SPI_NSS_PULSE_DISABLE 0U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_SPI_ENABLE(h) ((h)->Instance->CR1 |= 0x40U)
#define __HAL_SPI_DISABLE(h) ((h)->Instance->CR1 &= ~0x40U)
#define __NOP() ((void)0)

uint32_t HAL_GetTick(void);
uint32_t HAL_SPI_GetError(const SPI_HandleTypeDef *h);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t state);
void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *config);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout);
#endif
