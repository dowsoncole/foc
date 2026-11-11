/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi1_encoder.c
  * @brief   MT6835 磁编码器的 SPI1 外设配置（L1 配置层）实现。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "spi1_encoder.h"

SPI_HandleTypeDef hspi1;

#if (MT6835_USE_SOFTWARE_SPI == 0U)
static DMA_HandleTypeDef s_hdma_spi1_rx;
static DMA_HandleTypeDef s_hdma_spi1_tx;
static uint32_t s_spi_cr1_config;
static uint32_t s_spi_cr2_config;
#endif
static volatile uint8_t s_async_busy;
static mt6835_spi_async_callback_t s_async_callback;
static void *s_async_context;

#if (MT6835_USE_SOFTWARE_SPI == 0U)
/* This driver exclusively owns SPI1 and DMA1 channels 1/2. HAL still configures
   them, but its DMA completion/abort paths wait on HAL_GetTick() in an IRQ or
   with PRIMASK set. SysTick cannot advance there. Own the bounded transaction
   lifecycle instead of mixing HAL-private callbacks with manual DMA rearming. */
#define MT6835_DMA_FLAGS (DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1 | \
                          DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2)
#define MT6835_SPI_ERRORS (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE)

static void MT6835_SPI_StopHardware(uint8_t reset)
{
  /* MODF can clear MSTR/SPE. Restore the initialized configuration rather
     than copying an already corrupted live CR1 back after reset. */
  uint32_t cr1 = s_spi_cr1_config;
  uint32_t cr2 = s_spi_cr2_config;
  SPI1->CR2 = cr2;
  CLEAR_BIT(s_hdma_spi1_rx.Instance->CCR,
            DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
  CLEAR_BIT(s_hdma_spi1_tx.Instance->CCR,
            DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
  __DSB();
  /* Disable channels BEFORE clearing flags (ES0431 section 2.3.1). */
  DMA1->IFCR = MT6835_DMA_FLAGS;
  s_hdma_spi1_rx.Instance->CNDTR = 0U;
  s_hdma_spi1_tx.Instance->CNDTR = 0U;
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
  if (reset != 0U)
  {
    /* Abort may see BSY stuck or a partly shifted frame. RCC reset has no
       flag/tick wait, and removes the shifter, FIFO and SPI error state. */
    MT6835_CSN_RELEASE();
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    SPI1->CR2 = cr2;
  }
  SPI1->CR1 = cr1;
  hspi1.TxXferCount = hspi1.RxXferCount = 0U;
  hspi1.State = HAL_SPI_STATE_READY;
  hspi1.Lock = HAL_UNLOCKED;
  s_hdma_spi1_rx.State = s_hdma_spi1_tx.State = HAL_DMA_STATE_READY;
  s_hdma_spi1_rx.Lock = s_hdma_spi1_tx.Lock = HAL_UNLOCKED;
  s_hdma_spi1_rx.ErrorCode = s_hdma_spi1_tx.ErrorCode = HAL_DMA_ERROR_NONE;
}

static void MT6835_SPI_StartDma(const uint8_t *tx, uint8_t *rx, uint16_t size)
{
  /* SPE is enabled while CS is HIGH, so establishing mode-3 idle cannot be
     mistaken for a selected clock. Keep the receiver disabled between frames
     so an idle-line disturbance cannot leave an old partial byte in the FIFO. */
  hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
  hspi1.pTxBuffPtr = tx;
  hspi1.pRxBuffPtr = rx;
  hspi1.TxXferSize = hspi1.RxXferSize = size;
  hspi1.TxXferCount = hspi1.RxXferCount = size;
  hspi1.State = HAL_SPI_STATE_BUSY_TX_RX;
  s_hdma_spi1_rx.State = s_hdma_spi1_tx.State = HAL_DMA_STATE_BUSY;
  DMA1->IFCR = MT6835_DMA_FLAGS;
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
  s_hdma_spi1_rx.Instance->CPAR = (uint32_t)(uintptr_t)&SPI1->DR;
  s_hdma_spi1_rx.Instance->CMAR = (uint32_t)(uintptr_t)rx;
  s_hdma_spi1_tx.Instance->CPAR = (uint32_t)(uintptr_t)&SPI1->DR;
  s_hdma_spi1_tx.Instance->CMAR = (uint32_t)(uintptr_t)tx;
  s_hdma_spi1_rx.Instance->CNDTR = size;
  s_hdma_spi1_tx.Instance->CNDTR = size;
  SET_BIT(SPI1->CR2, SPI_CR2_FRXTH);
  SET_BIT(s_hdma_spi1_rx.Instance->CCR, DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN);
  /* RX completion closes the entire transfer. TX needs only its ERROR IRQ. */
  SET_BIT(s_hdma_spi1_tx.Instance->CCR, DMA_CCR_TEIE | DMA_CCR_EN);
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}
#endif

/* 20 NOPs >=119 ns at 168 MHz, excluding GPIO/bus overhead. This covers
   MT6835 CS setup (100 ns), and CS hold (half an SCK period at /16).
   Fixed instruction count; independent of SysTick and interrupt priority. */
static void MT6835_SPI_CsDelay(void)
{
#if MT6701_SSI_ENABLE
  /* >= 1 us: covers SSI setup and half-clock hold at /128. */
  for (volatile uint32_t n = 0; n < 168U; ++n) __NOP();
#endif
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

#if (MT6835_USE_SOFTWARE_SPI != 0U)

/* Eight NOPs alone take >=47.6 ns at 168 MHz, above the 30 ns clock-level
   minimum. GPIO/bus/loop work adds time. Diagnostic only: measure total CPU
   occupancy before enabling motor output; this is not parallel DMA. */
#define MT6835_SOFT_SPI_DELAY() do { \
  __NOP(); __NOP(); __NOP(); __NOP(); \
  __NOP(); __NOP(); __NOP(); __NOP(); \
} while (0)

__STATIC_FORCEINLINE uint8_t MT6835_SoftSpiTransferByte(uint8_t tx)
{
  uint8_t rx = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++)
  {
    /* Mode 3：下降沿前准备发送数据，随后在上升沿采样输入。 */
    if ((tx & (uint8_t)(0x80U >> bit)) != 0U)
    {
      MT6835_SOFT_MOSI_GPIO_Port->BSRR = MT6835_SOFT_MOSI_Pin;
    }
    else
    {
      MT6835_SOFT_MOSI_GPIO_Port->BSRR = (uint32_t)MT6835_SOFT_MOSI_Pin << 16U;
    }

    MT6835_SOFT_SPI_DELAY();
    MT6835_SCK_GPIO_Port->BSRR = (uint32_t)MT6835_SCK_Pin << 16U;
    MT6835_SOFT_SPI_DELAY();
    MT6835_SCK_GPIO_Port->BSRR = MT6835_SCK_Pin;

    if (MT6835_SOFT_MISO_GPIO_Port->IDR & MT6835_SOFT_MISO_Pin)
    {
      rx |= (uint8_t)(0x80U >> bit);
    }

    MT6835_SOFT_SPI_DELAY();
  }

  return rx;
}

#endif

HAL_StatusTypeDef MX_SPI1_Encoder_Init(void)
{
  /* 片选必须在 SPI 外设使能之前就处于空闲高电平：CSN 的下降沿会触发
     MT6835 锁存角度寄存器（手册 §7.6.9），任何一次意外拉低都会让芯片
     进入一次未被消费的通讯。先配 GPIO、再初始化 SPI。 */
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
#if MT6701_SSI_ENABLE
  __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* CAL_EN 必须保持低电平，避免 MT6835 进入客户端自校准模式。
     先写输出锁存器，再切换为推挽输出，避免初始化瞬间出现高脉冲。 */
  HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin,
                    GPIO_PIN_RESET);
  gpio.Pin   = MT6835_CAL_EN_Pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MT6835_CAL_EN_GPIO_Port, &gpio);
  MT6835_CAL_EN_DISABLE();

  gpio.Pin   = MT6835_CSN_Pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = SPI1_ENC_OUTPUT_GPIO_SPEED;
  MT6835_CSN_RELEASE();
  HAL_GPIO_Init(MT6835_CSN_GPIO_Port, &gpio);

#if (MT6835_USE_SOFTWARE_SPI != 0U)
  /* Pin mapping is selected independently; normal is PA7 out / PA6 in. */
  gpio.Pin   = MT6835_SCK_Pin | MT6835_SOFT_MOSI_Pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin   = MT6835_SOFT_MISO_Pin;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* SPI Mode 3 的空闲电平为高。 */
  MT6835_SCK_GPIO_Port->BSRR = MT6835_SCK_Pin;
  return HAL_OK;
#else

  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  /* MT6835 = 模式 3：空闲高电平，第二个边沿（上升沿）采样。 */
#if MT6701_SSI_ENABLE
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
#else
  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
#endif
  hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
  /* 片选全程软件管理，硬件 NSS 关闭。 */
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI1_ENC_BAUDRATE_PRESCALER;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  s_spi_cr1_config = SPI1->CR1 & ~SPI_CR1_SPE;
  s_spi_cr2_config = SPI1->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN |
                                 SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);

  __HAL_RCC_DMA1_CLK_ENABLE();

  s_hdma_spi1_rx.Instance = DMA1_Channel1;
  s_hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
  s_hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  s_hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&s_hdma_spi1_rx) != HAL_OK)
  {
    return HAL_ERROR;
  }
  __HAL_LINKDMA(&hspi1, hdmarx, s_hdma_spi1_rx);

  s_hdma_spi1_tx.Instance = DMA1_Channel2;
  s_hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
  s_hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  s_hdma_spi1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&s_hdma_spi1_tx) != HAL_OK)
  {
    return HAL_ERROR;
  }
  __HAL_LINKDMA(&hspi1, hdmatx, s_hdma_spi1_tx);

  /* ADC/FOC 为优先级 1，编码器 DMA 为优先级 0，可抢占 ADC 以及时发布
     转子反馈。DMA 回调必须有界；完整 ADC ISR 预算包含这段抢占开销。 */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(SPI1_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);

  return HAL_OK;
#endif
}

HAL_StatusTypeDef MT6835_SPI_TransmitReceive(const uint8_t *tx, uint8_t *rx,
                                             uint16_t size, uint32_t timeout)
{
  if ((tx == NULL) || (rx == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (s_async_busy != 0U)
  {
    if (!mask) __enable_irq();
    return HAL_BUSY;
  }
  s_async_busy = 1U;
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
#endif
  MT6835_SPI_CsDelay();
  MT6835_CSN_SELECT();
  MT6835_SPI_CsDelay();
  if (!mask) __enable_irq();

#if (MT6835_USE_SOFTWARE_SPI != 0U)
  (void)timeout;
  MT6835_SCK_GPIO_Port->BSRR = MT6835_SCK_Pin;
  MT6835_SOFT_SPI_DELAY();
  MT6835_SOFT_SPI_DELAY();
  for (uint16_t i = 0U; i < size; i++)
  {
    rx[i] = MT6835_SoftSpiTransferByte(tx[i]);
  }
  HAL_StatusTypeDef status = HAL_OK;
#else
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, size, timeout);
#endif
  MT6835_SPI_CsDelay();
  MT6835_CSN_RELEASE();
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  MT6835_SPI_StopHardware(status != HAL_OK);
#endif
  s_async_busy = 0U;
  return status;
}

static void MT6835_SPI_AsyncFinish(HAL_StatusTypeDef status)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  /* A pending IRQ after abort/reset must not complete the next transaction or
     call the consumer twice. A synchronous transfer has no async callback. */
  if (!s_async_busy || !s_async_callback)
  {
    if (!mask) __enable_irq();
    return;
  }
  mt6835_spi_async_callback_t callback = s_async_callback;
  void *context = s_async_context;

  /* Finish CS hold and deselect BEFORE disabling SPE/resetting the shifter.
     Peripheral cleanup must not change SCK while the sensor is selected. */
  MT6835_SPI_CsDelay();
  MT6835_CSN_RELEASE();
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  MT6835_SPI_StopHardware(status != HAL_OK);
#endif
  s_async_callback = NULL;
  s_async_context = NULL;
  s_async_busy = 0U;
  if (!mask) __enable_irq();
  if (callback != NULL)
  {
    callback(status, context);
  }
}

HAL_StatusTypeDef MT6835_SPI_TransmitReceiveAsync(
    const uint8_t *tx, uint8_t *rx, uint16_t size,
    mt6835_spi_async_callback_t callback, void *context)
{
  if ((tx == NULL) || (rx == NULL) || (size == 0U) || (callback == NULL))
  {
    return HAL_ERROR;
  }
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (s_async_busy != 0U)
  {
    if (!mask) __enable_irq();
    return HAL_BUSY;
  }

  s_async_busy = 1U;
  s_async_callback = callback;
  s_async_context = context;

#if (MT6835_USE_SOFTWARE_SPI != 0U)
  MT6835_SCK_GPIO_Port->BSRR = MT6835_SCK_Pin;
#else
  /* Recover preexisting FIFO/error state before selecting the sensor. */
  uint32_t sr = SPI1->SR;
  if ((sr & (SPI_SR_BSY | SPI_SR_FRLVL | SPI_SR_FTLVL | MT6835_SPI_ERRORS)) ||
      hspi1.State != HAL_SPI_STATE_READY ||
      (s_hdma_spi1_rx.Instance->CCR & DMA_CCR_EN) ||
      (s_hdma_spi1_tx.Instance->CCR & DMA_CCR_EN) ||
      s_hdma_spi1_rx.Instance->CNDTR || s_hdma_spi1_tx.Instance->CNDTR)
  {
    MT6835_SPI_StopHardware(1U);
  }
  MT6835_SPI_StartDma(tx, rx, size);
#endif
  MT6835_SPI_CsDelay();
  MT6835_CSN_SELECT();
  MT6835_SPI_CsDelay();
#if (MT6835_USE_SOFTWARE_SPI != 0U)
  if (!mask) __enable_irq();
  for (uint16_t i = 0U; i < size; i++)
    rx[i] = MT6835_SoftSpiTransferByte(tx[i]);
  MT6835_SPI_AsyncFinish(HAL_OK);
#else
  /* Requests are enabled only after CS setup. No half-frame interrupt. */
  SET_BIT(SPI1->CR2, SPI_CR2_ERRIE | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
#endif
  if (!mask) __enable_irq();
  return HAL_OK;
}

HAL_StatusTypeDef MT6835_SPI_AbortAsync(void)
{
  if (s_async_busy == 0U)
  {
    return HAL_OK;
  }
  if (s_async_callback == NULL) return HAL_BUSY;
  MT6835_SPI_AsyncFinish(HAL_ERROR);
  return HAL_OK;
}

uint8_t MT6835_SPI_IsBusy(void)
{
  return s_async_busy;
}

HAL_StatusTypeDef MT6835_SPI_ResyncIdle(void)
{
#if (MT6835_USE_SOFTWARE_SPI != 0U)
  return HAL_ERROR;
#else
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (s_async_busy || hspi1.Instance != SPI1 ||
      !s_hdma_spi1_rx.Instance || !s_hdma_spi1_tx.Instance)
  {
    if (!mask) __enable_irq();
    return HAL_BUSY;
  }
  /* Software ownership is idle: stale BSY/CNDTR is precisely what needs
     resetting, not a reason to refuse recovery indefinitely. */
  MT6835_SPI_StopHardware(1U);
  __DSB();
  if (!mask) __enable_irq();
  return HAL_OK;
#endif
}

void SPI1_Encoder_DmaRxIrqHandler(void)
{
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  uint32_t flags = DMA1->ISR;
  if (!s_async_busy || !s_async_callback)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
    return;
  }
  if (flags & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2))
  {
    hspi1.ErrorCode = HAL_SPI_ERROR_DMA;
    MT6835_SPI_AsyncFinish(HAL_ERROR);
  }
  else if ((flags & DMA_ISR_TCIF1) && s_async_busy)
  {
    uint32_t sr = SPI1->SR;
    /* RX TC is the last received byte, not TX FIFO completion. Never wait
       for a stuck flag in the highest-priority interrupt. Reject and reset
       an inconsistent transaction; no partial buffer is published. */
    uint8_t complete = !(sr & (SPI_SR_BSY | SPI_SR_FRLVL | SPI_SR_FTLVL |
                               MT6835_SPI_ERRORS)) && (sr & SPI_SR_TXE) &&
                       !s_hdma_spi1_rx.Instance->CNDTR &&
                       !s_hdma_spi1_tx.Instance->CNDTR;
    if (!complete) hspi1.ErrorCode = HAL_SPI_ERROR_FLAG;
    MT6835_SPI_AsyncFinish(complete ? HAL_OK : HAL_ERROR);
  }
  else
  {
    DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  }
#endif
}

void SPI1_Encoder_DmaTxIrqHandler(void)
{
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  if (!s_async_busy || !s_async_callback)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2;
    return;
  }
  if (DMA1->ISR & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2))
  {
    hspi1.ErrorCode = HAL_SPI_ERROR_DMA;
    MT6835_SPI_AsyncFinish(HAL_ERROR);
  }
  else
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2;
#endif
}

void SPI1_Encoder_SpiIrqHandler(void)
{
#if (MT6835_USE_SOFTWARE_SPI == 0U)
  if (SPI1->SR & MT6835_SPI_ERRORS)
  {
    if (!s_async_busy)
    {
      MT6835_SPI_StopHardware(1U);
      return;
    }
    hspi1.ErrorCode = HAL_SPI_ERROR_FLAG;
    MT6835_SPI_AsyncFinish(HAL_ERROR);
  }
#endif
}
