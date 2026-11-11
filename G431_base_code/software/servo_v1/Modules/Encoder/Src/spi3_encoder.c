#include "spi3_encoder.h"

SPI_HandleTypeDef hspi3;

/* 至少 1 us 的片选建立/保持时间，不占用定时器或修改 DWT。
   循环本身的开销只会延长等待；中断可以正常抢占。 */
static void Spi3Encoder_CsGuard(void)
{
  uint32_t cycles = SystemCoreClock / 1000000UL + 1U;
  while (cycles-- != 0U)
  {
    __NOP();
  }
}

HAL_StatusTypeDef MX_SPI3_Encoder_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(MT6701_SPI3_CSN_PORT, MT6701_SPI3_CSN_PIN, GPIO_PIN_SET);
  gpio.Pin = MT6701_SPI3_CSN_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MT6701_SPI3_CSN_PORT, &gpio);

  if (hspi3.Instance == SPI3)
  {
    (void)HAL_SPI_DeInit(&hspi3);
  }
  hspi3 = (SPI_HandleTypeDef){0};
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  /* 有限长度的全双工虚拟发送；RXONLY 主机会自由产生额外时钟，不适合 SSI。 */
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = MT6701_SPI3_PRESCALER;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  return HAL_SPI_Init(&hspi3);
}

HAL_StatusTypeDef SPI3_Encoder_ReadFrame(uint8_t frame[MT6701_SPI3_FRAME_BYTES],
                                       uint32_t timeout_ms)
{
  static const uint8_t dummy[MT6701_SPI3_FRAME_BYTES] = {0U, 0U, 0U};
  HAL_StatusTypeDef status;
  if (frame == NULL || hspi3.Instance != SPI3)
  {
    return HAL_ERROR;
  }
  /* 先让外设接管空闲低电平，再选择芯片，避免第一帧多出跳变沿。 */
  __HAL_SPI_ENABLE(&hspi3);
  Spi3Encoder_CsGuard();
  HAL_GPIO_WritePin(MT6701_SPI3_CSN_PORT, MT6701_SPI3_CSN_PIN, GPIO_PIN_RESET);
  Spi3Encoder_CsGuard();
  status = HAL_SPI_TransmitReceive(&hspi3, dummy, frame,
                                 MT6701_SPI3_FRAME_BYTES, timeout_ms);
  Spi3Encoder_CsGuard();
  HAL_GPIO_WritePin(MT6701_SPI3_CSN_PORT, MT6701_SPI3_CSN_PIN, GPIO_PIN_SET);
  if (status != HAL_OK)
  {
    __HAL_SPI_DISABLE(&hspi3);
  }
  return status;
}
