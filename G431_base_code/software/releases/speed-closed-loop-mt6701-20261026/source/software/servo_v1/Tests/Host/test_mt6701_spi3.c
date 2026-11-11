/* Actual SPI3 transport and test service, with injected SSI frames/HAL errors. */
#include "mt6701_spi3_test.h"
#include "spi3_encoder.h"
#include <assert.h>
#include <stdio.h>

SPI_TypeDef test_spi3;
GPIO_TypeDef test_gpiob;
uint32_t SystemCoreClock = 168000000U;
static uint32_t tick, frame = 0x400035U, cs = GPIO_PIN_SET;
static unsigned init_count, transfer_count;
static HAL_StatusTypeDef init_status = HAL_OK, transfer_status = HAL_OK;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_SPI_GetError(const SPI_HandleTypeDef *h) { return h->ErrorCode; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t state)
{
  assert(port == GPIOB && pin == GPIO_PIN_6);
  cs = state;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *config)
{
  assert(port == GPIOB && config->Pin == GPIO_PIN_6);
  assert(cs == GPIO_PIN_SET && config->Mode == GPIO_MODE_OUTPUT_PP);
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
  assert(h->Instance == SPI3 && cs == GPIO_PIN_SET);
  assert(h->Init.CLKPolarity == SPI_POLARITY_LOW);
  assert(h->Init.CLKPhase == SPI_PHASE_2EDGE);
  assert(h->Init.Direction == SPI_DIRECTION_2LINES);
  assert(h->Init.DataSize == SPI_DATASIZE_8BIT);
  assert(h->Init.NSS == SPI_NSS_SOFT);
  assert(h->Init.NSSPMode == SPI_NSS_PULSE_DISABLE);
  assert(h->Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256);
  init_count++;
  return init_status;
}
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h)
{
  assert(cs == GPIO_PIN_SET);
  h->Instance->CR1 = 0U;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout)
{
  assert(h->Instance == SPI3 && (h->Instance->CR1 & 0x40U));
  assert(cs == GPIO_PIN_RESET && size == 3U && timeout == 2U);
  assert(tx[0] == 0U && tx[1] == 0U && tx[2] == 0U);
  rx[0] = (uint8_t)(frame >> 16U);
  rx[1] = (uint8_t)(frame >> 8U);
  rx[2] = (uint8_t)frame;
  h->ErrorCode = (transfer_status == HAL_OK) ? 0U : 0x20U;
  transfer_count++;
  return transfer_status;
}
static void poll(uint32_t next_frame)
{
  frame = next_frame;
  tick += 10U;
  Mt6701Spi3Test_Task();
  assert(cs == GPIO_PIN_SET);
}
int main(void)
{
  assert(SPI3_Encoder_ReadFrame(NULL, 2U) == HAL_ERROR);
  assert(Mt6701Spi3Test_Init() == HAL_OK);
  Mt6701Spi3Test_Task();
  tick = 9U; Mt6701Spi3Test_Task();
  assert(transfer_count == 0U);

  /* Fixed CRC vectors computed by polynomial long division, no shared CRC code:
     payload 0x10000 -> CRC 0x35, payload 0x1001E -> CRC 0x17. */
  poll(0x400035U);
  assert(g_mt6701_spi3_test.sample_valid && !g_mt6701_spi3_test.communication_ok);
  assert(g_mt6701_spi3_test.angle_raw == 4096U);
  assert(g_mt6701_spi3_test.angle_deg == 90.0f);
  poll(0x400035U); poll(0x400035U);
  assert(g_mt6701_spi3_test.communication_ok);
  assert(g_mt6701_spi3_test.good_frame_count == 3U);

  /* Warning bits do not turn a valid response into a communication failure. */
  poll(0x400797U);
  assert(g_mt6701_spi3_test.communication_ok);
  assert(g_mt6701_spi3_test.angle_raw == 4097U);
  assert(g_mt6701_spi3_test.status == 14U);
  assert(g_mt6701_spi3_test.magnetic_field_status == 2U);
  assert(g_mt6701_spi3_test.push_detected && g_mt6701_spi3_test.loss_of_track);
  assert(g_mt6701_spi3_test.status_warning_count == 1U);
  assert(g_mt6701_spi3_test.angle_change_count == 1U);

  /* Flipped data bit: reject angle and drop communication immediately. */
  poll(0x400797U ^ 0x400U);
  assert(g_mt6701_spi3_test.crc_error_count == 1U);
  assert(!g_mt6701_spi3_test.sample_valid && !g_mt6701_spi3_test.communication_ok);
  assert(g_mt6701_spi3_test.angle_raw == 4097U);
  poll(0U); poll(0xFFFFFFU);
  assert(g_mt6701_spi3_test.blank_frame_count == 2U);
  assert(g_mt6701_spi3_test.good_frame_count == 4U);

  /* HAL failure must release CS and reset/reinitialize before the next frame. */
  transfer_status = HAL_TIMEOUT;
  poll(0x400035U);
  assert(g_mt6701_spi3_test.spi_error_count == 1U);
  assert(g_mt6701_spi3_test.last_hal_error == 0x20U);
  assert(!g_mt6701_spi3_test.initialized && !(SPI3->CR1 & 0x40U));
  transfer_status = HAL_OK;
  poll(0x400035U);
  assert(init_count == 2U && g_mt6701_spi3_test.initialized);
  poll(0x400035U); poll(0x400035U);
  assert(g_mt6701_spi3_test.communication_ok);

  unsigned before = transfer_count;
  g_mt6701_spi3_test.enabled = 0U;
  poll(0x400035U);
  assert(transfer_count == before && !g_mt6701_spi3_test.communication_ok);
  assert(!g_mt6701_spi3_test.sample_valid);
  g_mt6701_spi3_test.enabled = 1U;
  g_mt6701_spi3_test.poll_period_ms = 0U;
  Mt6701Spi3Test_Task();
  assert(transfer_count == before);
  tick++; Mt6701Spi3Test_Task();
  assert(transfer_count == before + 1U);

  /* Initialization failure retries on schedule; tick wrap keeps polling. */
  tick = UINT32_MAX - 4U;
  init_status = HAL_ERROR;
  assert(Mt6701Spi3Test_Init() == HAL_ERROR);
  before = transfer_count;
  tick += 10U; Mt6701Spi3Test_Task();
  assert(transfer_count == before && !g_mt6701_spi3_test.initialized);
  init_status = HAL_OK;
  poll(0xFFFC1FU);
  assert(g_mt6701_spi3_test.angle_raw == 16383U);
  assert(g_mt6701_spi3_test.sample_valid && cs == GPIO_PIN_SET);
  puts("MT6701 SPI3 tests passed: CRC/angle/status, blank frames, HAL recovery, polling, pause, tick wrap");
  return 0;
}
