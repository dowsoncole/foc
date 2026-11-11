#include "mt6701_spi3_test.h"
#include "spi3_encoder.h"

#define MT6701_SPI3_TIMEOUT_MS 2U

volatile mt6701_spi3_test_t g_mt6701_spi3_test;
static uint32_t s_last_poll_tick;

/* MT6701 Rev.1.8/1.9 §7.8.2：前 18 位（14 位角度 + 4 位状态），
   MSB first，CRC6 多项式 x^6+x+1，初值 0，无反射，无末尾异或。 */
static uint8_t Mt6701Spi3Test_Crc6(uint32_t data18)
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

HAL_StatusTypeDef Mt6701Spi3Test_Init(void)
{
  HAL_StatusTypeDef status;
  g_mt6701_spi3_test = (mt6701_spi3_test_t){0};
  g_mt6701_spi3_test.enabled = 1U;
  g_mt6701_spi3_test.poll_period_ms = 10U;
  s_last_poll_tick = HAL_GetTick();
  status = MX_SPI3_Encoder_Init();
  g_mt6701_spi3_test.initialized = (status == HAL_OK);
  g_mt6701_spi3_test.last_hal_status = status;
  g_mt6701_spi3_test.last_hal_error = HAL_SPI_GetError(&hspi3);
  return status;
}

void Mt6701Spi3Test_Task(void)
{
  uint8_t rx[MT6701_SPI3_FRAME_BYTES] = {0};
  uint32_t now = HAL_GetTick();
  uint32_t period = g_mt6701_spi3_test.poll_period_ms;
  uint32_t raw;
  uint32_t angle;
  HAL_StatusTypeDef status;

  if (g_mt6701_spi3_test.enabled == 0U)
  {
    g_mt6701_spi3_test.sample_valid = 0U;
    g_mt6701_spi3_test.communication_ok = 0U;
    g_mt6701_spi3_test.consecutive_good = 0U;
    s_last_poll_tick = now;
    return;
  }
  if (period == 0U) period = 1U;
  if ((uint32_t)(now - s_last_poll_tick) < period) return;
  s_last_poll_tick = now;
  g_mt6701_spi3_test.sample_valid = 0U;

  /* 上次传输出错时，下一次采样先复位 SPI3，避免残留 FIFO 造成持续错位。 */
  if (g_mt6701_spi3_test.initialized == 0U)
  {
    status = MX_SPI3_Encoder_Init();
    g_mt6701_spi3_test.last_hal_status = status;
    g_mt6701_spi3_test.last_hal_error = HAL_SPI_GetError(&hspi3);
    g_mt6701_spi3_test.initialized = (status == HAL_OK);
    if (status != HAL_OK) return;
  }

  status = SPI3_Encoder_ReadFrame(rx, MT6701_SPI3_TIMEOUT_MS);
  g_mt6701_spi3_test.last_sample_tick_ms = HAL_GetTick();
  g_mt6701_spi3_test.frame_count++;
  g_mt6701_spi3_test.last_hal_status = status;
  g_mt6701_spi3_test.last_hal_error = HAL_SPI_GetError(&hspi3);
  if (status != HAL_OK)
  {
    g_mt6701_spi3_test.spi_error_count++;
    g_mt6701_spi3_test.initialized = 0U;
    g_mt6701_spi3_test.communication_ok = 0U;
    g_mt6701_spi3_test.consecutive_good = 0U;
    return;
  }

  g_mt6701_spi3_test.rx[0] = rx[0];
  g_mt6701_spi3_test.rx[1] = rx[1];
  g_mt6701_spi3_test.rx[2] = rx[2];
  raw = ((uint32_t)rx[0] << 16U) | ((uint32_t)rx[1] << 8U) | rx[2];
  g_mt6701_spi3_test.raw_frame = raw;
  g_mt6701_spi3_test.status = (uint8_t)((raw >> 6U) & 0x0FU);
  g_mt6701_spi3_test.magnetic_field_status = (uint8_t)((raw >> 6U) & 3U);
  g_mt6701_spi3_test.push_detected = (uint8_t)((raw >> 8U) & 1U);
  g_mt6701_spi3_test.loss_of_track = (uint8_t)((raw >> 9U) & 1U);
  g_mt6701_spi3_test.crc_received = (uint8_t)(raw & 0x3FU);
  g_mt6701_spi3_test.crc_calculated = Mt6701Spi3Test_Crc6(raw >> 6U);

  /* 全零可能是恰好零角度，也可能是断线拉低。保守记为空帧，不能凭它
     宣布通信成功；手动转动磁铁后用其他角度验证。 */
  if (raw == 0U || raw == 0xFFFFFFUL)
  {
    g_mt6701_spi3_test.blank_frame_count++;
  }
  else if (g_mt6701_spi3_test.crc_received != g_mt6701_spi3_test.crc_calculated)
  {
    g_mt6701_spi3_test.crc_error_count++;
  }
  else
  {
    angle = (raw >> 10U) & 0x3FFFU;
    if (g_mt6701_spi3_test.good_frame_count != 0U &&
        angle != g_mt6701_spi3_test.angle_raw)
    {
      g_mt6701_spi3_test.angle_change_count++;
    }
    g_mt6701_spi3_test.angle_raw = angle;
    g_mt6701_spi3_test.angle_deg = (float)angle * (360.0f / 16384.0f);
    g_mt6701_spi3_test.good_frame_count++;
    if (g_mt6701_spi3_test.status != 0U)
      g_mt6701_spi3_test.status_warning_count++;
    g_mt6701_spi3_test.last_good_tick_ms = g_mt6701_spi3_test.last_sample_tick_ms;
    g_mt6701_spi3_test.sample_valid = 1U;
    if (g_mt6701_spi3_test.consecutive_good < 3U)
      g_mt6701_spi3_test.consecutive_good++;
    g_mt6701_spi3_test.communication_ok = (g_mt6701_spi3_test.consecutive_good >= 3U);
    return;
  }
  g_mt6701_spi3_test.communication_ok = 0U;
  g_mt6701_spi3_test.consecutive_good = 0U;
}
