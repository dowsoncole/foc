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

HAL_StatusTypeDef MX_SPI1_Encoder_Init(void)
{
  /* 片选必须在 SPI 外设使能之前就处于空闲高电平：CSN 的下降沿会触发
     MT6835 锁存角度寄存器（手册 §7.6.9），任何一次意外拉低都会让芯片
     进入一次未被消费的通讯。先配 GPIO、再初始化 SPI。 */
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin   = MT6835_CSN_Pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MT6835_CSN_GPIO_Port, &gpio);
  MT6835_CSN_RELEASE();

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
}
