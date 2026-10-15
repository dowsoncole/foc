#include "waveform_generator.h"

#include <math.h>

#define WAVEFORM_QUEUE_LENGTH 32U
#define WAVEFORM_FRAME_LENGTH 8U

static UART_HandleTypeDef *s_huart;
static volatile uint16_t s_queue[WAVEFORM_QUEUE_LENGTH];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static volatile uint32_t s_source_count;
static uint16_t s_phase_index;
static uint8_t s_frame_sequence;

HAL_StatusTypeDef WaveformGenerator_PortInit(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  huart->Instance = USART1;
  huart->Init.BaudRate = WAVEFORM_RS485_BAUDRATE;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  return (HAL_UART_Init(huart) == HAL_OK) ? HAL_OK : HAL_ERROR;
}

static uint8_t WaveformGenerator_Crc8(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0U;
  uint32_t i;

  for (i = 0U; i < length; ++i)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x80U) != 0U) ?
          (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

HAL_StatusTypeDef WaveformGenerator_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  s_huart = huart;
  s_head = 0U;
  s_tail = 0U;
  s_source_count = 0U;
  s_phase_index = 0U;
  s_frame_sequence = 0U;
  return HAL_OK;
}

void WaveformGenerator_SampleTick(void)
{
  uint8_t next_head;

  if (s_huart == NULL)
  {
    return;
  }

  if (++s_source_count < WAVEFORM_DECIMATION)
  {
    return;
  }
  s_source_count = 0U;

  next_head = (uint8_t)((s_head + 1U) % WAVEFORM_QUEUE_LENGTH);
  if (next_head == s_tail)
  {
    return; /* Drop newest sample rather than corrupting an in-flight frame. */
  }
  /* One complete cycle per second keeps the first scope test easy to identify.
     Store only the phase in the ISR; floating-point sine evaluation stays in
     the background task and cannot extend the ADC/FOC interrupt. */
  s_queue[s_head] = s_phase_index;
  s_head = next_head;
  s_phase_index = (uint16_t)((s_phase_index + 1U) % 1000U);
}

void WaveformGenerator_Task(void)
{
  uint8_t frame[WAVEFORM_FRAME_LENGTH];
  uint16_t sample;
  float phase;
  int32_t sample_value;
  uint32_t transmit_start;
  uint8_t tail;

  if ((s_huart == NULL) || (s_tail == s_head) ||
      (HAL_UART_GetState(s_huart) != HAL_UART_STATE_READY))
  {
    return;
  }

  tail = s_tail;
  sample = s_queue[tail];
  phase = (float)sample * (2.0f * 3.14159265358979323846f / 1000.0f);
  sample_value = (int32_t)(32767.0f * sinf(phase));
  frame[0] = 0xA5U;
  frame[1] = 0x5AU;
  frame[2] = 0x01U; /* waveform channel: sine */
  frame[3] = s_frame_sequence++;
  frame[4] = (uint8_t)((uint16_t)(int16_t)sample_value & 0xFFU);
  frame[5] = (uint8_t)((uint16_t)(int16_t)sample_value >> 8U);
  frame[6] = 0U; /* reserved for a future timestamp/divider */
  frame[7] = WaveformGenerator_Crc8(frame, 7U);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); /* RS485 transmit */
  transmit_start = HAL_GetTick();
  if (HAL_UART_Transmit(s_huart, frame, sizeof(frame), 2U) == HAL_OK)
  {
    s_tail = (uint8_t)((tail + 1U) % WAVEFORM_QUEUE_LENGTH);
  }
  while ((__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET) &&
         ((HAL_GetTick() - transmit_start) < 2U))
  {
    /* At 921600 baud this is under 100 us; DE must stay asserted to flush it. */
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); /* RS485 receive */
}
