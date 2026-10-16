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

#if (MT6835_USE_SOFTWARE_SPI != 0U)

/* 单个 NOP 保留数据建立/保持时间；GPIO BSRR 和循环本身构成主要时序。 */
#define MT6835_SOFT_SPI_DELAY()       __NOP()

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
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MT6835_CSN_GPIO_Port, &gpio);
  MT6835_CSN_RELEASE();

#if (MT6835_USE_SOFTWARE_SPI != 0U)
  /* 软件 SPI 使用当前交叉接线：PA6 输出、PA7 输入，PA5 为时钟。 */
  gpio.Pin   = MT6835_SCK_Pin | MT6835_SOFT_MOSI_Pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin   = MT6835_SOFT_MISO_Pin;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_NOPULL;
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
  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
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

  return HAL_OK;
#endif
}

HAL_StatusTypeDef MT6835_SPI_TransmitReceive(const uint8_t *tx, uint8_t *rx,
                                             uint16_t size, uint32_t timeout)
{
  uint16_t i;

  if ((tx == NULL) || (rx == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

#if (MT6835_USE_SOFTWARE_SPI != 0U)
  (void)timeout;
  MT6835_SCK_GPIO_Port->BSRR = MT6835_SCK_Pin;
  for (i = 0U; i < size; i++)
  {
    rx[i] = MT6835_SoftSpiTransferByte(tx[i]);
  }
  return HAL_OK;
#else
  return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, size, timeout);
#endif
}
