#include "mt6701_ssi.h"

SPI_HandleTypeDef hspi_mt6701;

/* 至少 1 us 的片选建立/保持时间，不占用定时器或修改 DWT。
   循环本身的开销只会延长等待；中断可以正常抢占。 */
static void Mt6701Ssi_CsGuard(void)
{
  uint32_t cycles = SystemCoreClock / 1000000UL + 1U;
  while (cycles-- != 0U)
  {
    __NOP();
  }
}

HAL_StatusTypeDef Mt6701Ssi_TransportInit(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(MT6701_SSI_CSN_PORT, MT6701_SSI_CSN_PIN, GPIO_PIN_SET);
  gpio.Pin = MT6701_SSI_CSN_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MT6701_SSI_CSN_PORT, &gpio);

  if (hspi_mt6701.Instance == SPI1)
  {
    (void)HAL_SPI_DeInit(&hspi_mt6701);
  }
  hspi_mt6701 = (SPI_HandleTypeDef){0};
  hspi_mt6701.Instance = SPI1;
  hspi_mt6701.Init.Mode = SPI_MODE_MASTER;
  /* 有限长度的全双工虚拟发送；RXONLY 主机会自由产生额外时钟，不适合 SSI。 */
  hspi_mt6701.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_mt6701.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_mt6701.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_mt6701.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_mt6701.Init.NSS = SPI_NSS_SOFT;
  hspi_mt6701.Init.BaudRatePrescaler = MT6701_SSI_PRESCALER;
  hspi_mt6701.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_mt6701.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_mt6701.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_mt6701.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  return HAL_SPI_Init(&hspi_mt6701);
}

HAL_StatusTypeDef Mt6701Ssi_ReadFrame(uint8_t frame[MT6701_SSI_FRAME_BYTES],
                                       uint32_t timeout_ms)
{
  static const uint8_t dummy[MT6701_SSI_FRAME_BYTES] = {0U, 0U, 0U};
  HAL_StatusTypeDef status;
  if (frame == NULL || hspi_mt6701.Instance != SPI1)
  {
    return HAL_ERROR;
  }
  /* 先让外设接管空闲低电平，再选择芯片，避免第一帧多出跳变沿。 */
  __HAL_SPI_ENABLE(&hspi_mt6701);
  Mt6701Ssi_CsGuard();
  HAL_GPIO_WritePin(MT6701_SSI_CSN_PORT, MT6701_SSI_CSN_PIN, GPIO_PIN_RESET);
  Mt6701Ssi_CsGuard();
  status = HAL_SPI_TransmitReceive(&hspi_mt6701, dummy, frame,
                                 MT6701_SSI_FRAME_BYTES, timeout_ms);
  Mt6701Ssi_CsGuard();
  HAL_GPIO_WritePin(MT6701_SSI_CSN_PORT, MT6701_SSI_CSN_PIN, GPIO_PIN_SET);
  if (status != HAL_OK)
  {
    __HAL_SPI_DISABLE(&hspi_mt6701);
  }
  return status;
}

#include "mt6701_ssi.h"


#define MT6701_SSI_TIMEOUT_MS 2U

volatile mt6701_ssi_t g_mt6701_ssi;
static uint32_t s_last_poll_tick;

/* MT6701 Rev.1.8 §7.8.2：前 18 位（14 位角度 + 4 位状态），
   MSB first，CRC6 多项式 x^6+x+1，采用初值 0、无反射、无末尾异或；需用实机完整帧核验。 */
static uint8_t Mt6701Ssi_Crc6(uint32_t data18)
{
  uint8_t crc = 0U;
  int32_t bit;
  for (bit = 17; bit >= 0; --bit)
  {
    uint8_t feedback = (uint8_t)(((data18 >> bit) ^ (crc >> 5U)) & 1U);
    crc = (uint8_t)((crc << 1U) & 0x3FU);
    if (feedback != 0U) crc ^= 0x03U;
  }
  return crc;
}

HAL_StatusTypeDef Mt6701Ssi_Init(void)
{
  HAL_StatusTypeDef status;
  g_mt6701_ssi = (mt6701_ssi_t){0};
  g_mt6701_ssi.enabled = 1U;
  g_mt6701_ssi.poll_period_ms = 1U;
  s_last_poll_tick = HAL_GetTick();
  status = Mt6701Ssi_TransportInit();
  g_mt6701_ssi.initialized = (status == HAL_OK);
  g_mt6701_ssi.last_hal_status = status;
  g_mt6701_ssi.last_hal_error = HAL_SPI_GetError(&hspi_mt6701);
  return status;
}

void Mt6701Ssi_Task(void)
{
  uint8_t rx[MT6701_SSI_FRAME_BYTES] = {0};
  uint32_t now = HAL_GetTick();
  uint32_t period = g_mt6701_ssi.poll_period_ms;
  HAL_StatusTypeDef status;

  if (g_mt6701_ssi.enabled == 0U)
  {
    g_mt6701_ssi.sample_valid = 0U;
    g_mt6701_ssi.communication_ok = 0U;
    g_mt6701_ssi.consecutive_good = 0U;
    s_last_poll_tick = now;
    return;
  }
  if (period == 0U) period = 1U;
  if ((uint32_t)(now - s_last_poll_tick) < period) return;
  s_last_poll_tick = now;
  g_mt6701_ssi.sample_valid = 0U;

  /* 上次传输出错时，下一次采样先复位 SPI3，避免残留 FIFO 造成持续错位。 */
  if (g_mt6701_ssi.initialized == 0U)
  {
    status = Mt6701Ssi_TransportInit();
    g_mt6701_ssi.last_hal_status = status;
    g_mt6701_ssi.last_hal_error = HAL_SPI_GetError(&hspi_mt6701);
    g_mt6701_ssi.initialized = (status == HAL_OK);
    if (status != HAL_OK) return;
  }

  status = Mt6701Ssi_ReadFrame(rx, MT6701_SSI_TIMEOUT_MS);
  g_mt6701_ssi.last_hal_error = HAL_SPI_GetError(&hspi_mt6701);
  Mt6701Ssi_ProcessFrame(rx, status);
}

void Mt6701Ssi_ProcessFrame(const uint8_t rx[3], HAL_StatusTypeDef status)
{
  uint32_t raw, angle;
  g_mt6701_ssi.sample_valid = 0U;
  g_mt6701_ssi.last_sample_tick_ms = HAL_GetTick();
  g_mt6701_ssi.frame_count++;
  g_mt6701_ssi.last_hal_status = status;
  if (status != HAL_OK)
  {
    g_mt6701_ssi.spi_error_count++;
    g_mt6701_ssi.initialized = 0U;
    g_mt6701_ssi.communication_ok = 0U;
    g_mt6701_ssi.consecutive_good = 0U;
    return;
  }

  g_mt6701_ssi.rx[0] = rx[0];
  g_mt6701_ssi.rx[1] = rx[1];
  g_mt6701_ssi.rx[2] = rx[2];
  raw = ((uint32_t)rx[0] << 16U) | ((uint32_t)rx[1] << 8U) | rx[2];
  g_mt6701_ssi.raw_frame = raw;
  g_mt6701_ssi.status = (uint8_t)((raw >> 6U) & 0x0FU);
  g_mt6701_ssi.magnetic_field_status = (uint8_t)((raw >> 6U) & 3U);
  g_mt6701_ssi.push_detected = (uint8_t)((raw >> 8U) & 1U);
  g_mt6701_ssi.loss_of_track = (uint8_t)((raw >> 9U) & 1U);
  g_mt6701_ssi.crc_received = (uint8_t)(raw & 0x3FU);
  g_mt6701_ssi.crc_calculated = Mt6701Ssi_Crc6(raw >> 6U);

  /* Angle=0/status=0 has a legitimate zero CRC. Do not reject the zero
     crossing; closed-loop feedback also applies step/freshness guards. */
  if (raw == 0xFFFFFFUL)
  {
    g_mt6701_ssi.blank_frame_count++;
  }
  else if (g_mt6701_ssi.crc_received != g_mt6701_ssi.crc_calculated)
  {
    g_mt6701_ssi.crc_error_count++;
  }
  else
  {
    angle = (raw >> 10U) & 0x3FFFU;
    if (g_mt6701_ssi.good_frame_count != 0U &&
        angle != g_mt6701_ssi.angle_raw)
    {
      g_mt6701_ssi.angle_change_count++;
    }
    g_mt6701_ssi.angle_raw = angle;
    g_mt6701_ssi.angle_deg = (float)angle * (360.0f / 16384.0f);
    g_mt6701_ssi.good_frame_count++;
    if (g_mt6701_ssi.status != 0U)
      g_mt6701_ssi.status_warning_count++;
    g_mt6701_ssi.last_good_tick_ms = g_mt6701_ssi.last_sample_tick_ms;
    g_mt6701_ssi.sample_valid = 1U;
    if (g_mt6701_ssi.consecutive_good < 3U)
      g_mt6701_ssi.consecutive_good++;
    g_mt6701_ssi.communication_ok = (g_mt6701_ssi.consecutive_good >= 3U);
    return;
  }
  g_mt6701_ssi.communication_ok = 0U;
  g_mt6701_ssi.consecutive_good = 0U;
}
