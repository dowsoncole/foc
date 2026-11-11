#ifndef ENCODER_TEST_HAL_H
#define ENCODER_TEST_HAL_H
#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { volatile uint32_t CR1, CR2, SR, DR; } SPI_TypeDef;
typedef struct { volatile uint32_t CR1, PSC, ARR, EGR, SR, DIER, CNT, BDTR; } TIM_TypeDef;
#define TIM_BDTR_MOE (1U << 15)
extern TIM_TypeDef test_tim1, test_tim16;
#define TIM1 (&test_tim1)
#define TIM16 (&test_tim16)
#define TIM_CR1_OPM (1U << 3)
#define TIM_CR1_URS (1U << 2)
#define TIM_CR1_CEN 1U
#define TIM_CR1_DIR (1U << 4)
#define TIM_EGR_UG 1U
#define TIM_DIER_UIE 1U
#define __HAL_RCC_TIM16_CLK_ENABLE() ((void)0)
typedef struct { volatile uint32_t CCR, CNDTR, CPAR, CMAR; } DMA_Channel_TypeDef;
typedef struct { volatile uint32_t ISR, IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t BSRR, IDR; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t Request, Direction, PeriphInc, MemInc,
  PeriphDataAlignment, MemDataAlignment, Mode, Priority; } DMA_InitTypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; DMA_InitTypeDef Init;
  uint32_t State, Lock, ErrorCode; void *Parent; } DMA_HandleTypeDef;
typedef struct { uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase,
  NSS, BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, NSSPMode; } SPI_InitTypeDef;
typedef struct { SPI_TypeDef *Instance; SPI_InitTypeDef Init;
  uint32_t State, Lock, ErrorCode; const uint8_t *pTxBuffPtr; uint8_t *pRxBuffPtr;
  uint16_t TxXferSize, RxXferSize, TxXferCount, RxXferCount;
  DMA_HandleTypeDef *hdmarx, *hdmatx; } SPI_HandleTypeDef;
typedef struct { uint32_t CYCCNT, CTRL; } test_dwt_t;
typedef struct { uint32_t DEMCR; } test_debug_t;
extern SPI_TypeDef test_spi;
extern DMA_Channel_TypeDef test_dma_rx, test_dma_tx;
extern DMA_TypeDef test_dma;
extern GPIO_TypeDef test_gpioa, test_gpioc, test_gpiod;
extern test_dwt_t test_dwt;
extern test_debug_t test_debug;
extern uint32_t SystemCoreClock, test_primask, test_resets;
extern HAL_StatusTypeDef test_sync_status;
#define SPI1 (&test_spi)
#define DMA1 (&test_dma)
#define DMA1_Channel1 (&test_dma_rx)
#define DMA1_Channel2 (&test_dma_tx)
#define GPIOA (&test_gpioa)
#define GPIOC (&test_gpioc)
#define GPIOD (&test_gpiod)
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __STATIC_FORCEINLINE static inline
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define __get_PRIMASK() test_primask
#define __disable_irq() ((void)(test_primask = 1U))
#define __enable_irq() ((void)(test_primask = 0U))
#define __NOP() ((void)test_dwt.CYCCNT++)
void test_barrier(void);
void test_spi_reset(void);
#define __DSB() test_barrier()
#define __HAL_RCC_SPI1_FORCE_RESET() test_spi_reset()
#define __HAL_RCC_SPI1_RELEASE_RESET() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_LINKDMA(h, field, dma) do { (h)->field = &(dma); (dma).Parent = (h); } while (0)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_RESET 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_INPUT 0U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_SPEED_FREQ_MEDIUM 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define SPI_CR1_SPE (1U << 6)
#define SPI_CR2_RXDMAEN (1U << 0)
#define SPI_CR2_TXDMAEN (1U << 1)
#define SPI_CR2_ERRIE (1U << 5)
#define SPI_CR2_RXNEIE (1U << 6)
#define SPI_CR2_TXEIE (1U << 7)
#define SPI_CR2_FRXTH (1U << 12)
#define SPI_SR_TXE (1U << 1)
#define SPI_SR_MODF (1U << 5)
#define SPI_SR_OVR (1U << 6)
#define SPI_SR_BSY (1U << 7)
#define SPI_SR_FRE (1U << 8)
#define SPI_SR_FRLVL (3U << 9)
#define SPI_SR_FTLVL (3U << 11)
#define DMA_CCR_EN 1U
#define DMA_CCR_TCIE 2U
#define DMA_CCR_HTIE 4U
#define DMA_CCR_TEIE 8U
#define DMA_ISR_TCIF1 (1U << 1)
#define DMA_ISR_HTIF1 (1U << 2)
#define DMA_ISR_TEIF1 (1U << 3)
#define DMA_ISR_TCIF2 (1U << 5)
#define DMA_ISR_HTIF2 (1U << 6)
#define DMA_ISR_TEIF2 (1U << 7)
#define DMA_IFCR_CTCIF1 DMA_ISR_TCIF1
#define DMA_IFCR_CHTIF1 DMA_ISR_HTIF1
#define DMA_IFCR_CTEIF1 DMA_ISR_TEIF1
#define DMA_IFCR_CTCIF2 DMA_ISR_TCIF2
#define DMA_IFCR_CHTIF2 DMA_ISR_HTIF2
#define DMA_IFCR_CTEIF2 DMA_ISR_TEIF2
#define HAL_SPI_STATE_READY 1U
#define HAL_SPI_STATE_BUSY_TX_RX 5U
#define HAL_DMA_STATE_READY 1U
#define HAL_DMA_STATE_BUSY 2U
#define HAL_UNLOCKED 0U
#define HAL_SPI_ERROR_NONE 0U
#define HAL_SPI_ERROR_DMA 16U
#define HAL_SPI_ERROR_FLAG 32U
#define HAL_DMA_ERROR_NONE 0U
#define SPI_MODE_MASTER 4U
#define SPI_DIRECTION_2LINES 0U
#define SPI_DATASIZE_8BIT (7U << 8)
#define SPI_POLARITY_HIGH 2U
#define SPI_PHASE_2EDGE 1U
#define SPI_NSS_SOFT (1U << 9)
#define SPI_BAUDRATEPRESCALER_16 (3U << 3)
#define SPI_BAUDRATEPRESCALER_32 (4U << 3)
#define SPI_BAUDRATEPRESCALER_128 (6U << 3)
#define SPI_FIRSTBIT_MSB 0U
#define SPI_TIMODE_DISABLE 0U
#define SPI_CRCCALCULATION_DISABLE 0U
#define SPI_NSS_PULSE_DISABLE 0U
#define DMA_REQUEST_SPI1_RX 10U
#define DMA_REQUEST_SPI1_TX 11U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_MEMORY_TO_PERIPH (1U << 4)
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE (1U << 7)
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_NORMAL 0U
#define DMA_PRIORITY_HIGH (2U << 12)
#define DMA_PRIORITY_MEDIUM (1U << 12)
#define DMA1_Channel1_IRQn 11U
#define DMA1_Channel2_IRQn 12U
#define SPI1_IRQn 35U
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t state);
void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *config);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout);
void HAL_NVIC_ClearPendingIRQ(uint32_t irq);
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(uint32_t irq);
void HAL_Delay(uint32_t ms);
#endif
