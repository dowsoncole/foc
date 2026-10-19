#include "waveform_uart1_port.h"

typedef struct
{
  UART_HandleTypeDef *huart;
  uint32_t timeout_ms;
} waveform_uart1_context_t;

static waveform_uart1_context_t s_uart1_context;

static uint8_t WaveformUart1Port_Send(void *context,
                                     const uint8_t *data,
                                     uint16_t length)
{
  waveform_uart1_context_t *port = (waveform_uart1_context_t *)context;
  HAL_StatusTypeDef result;

  if ((port == NULL) || (port->huart == NULL) ||
      (data == NULL) || (length == 0U) ||
      (HAL_UART_GetState(port->huart) != HAL_UART_STATE_READY))
  {
    return 0U;
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  result = HAL_UART_Transmit(port->huart, data, length, port->timeout_ms);
  if (result == HAL_OK)
  {
    uint32_t start = HAL_GetTick();
    while ((__HAL_UART_GET_FLAG(port->huart, UART_FLAG_TC) == RESET) &&
           ((HAL_GetTick() - start) < port->timeout_ms))
    {
    }
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  return (result == HAL_OK) ? 1U : 0U;
}

HAL_StatusTypeDef WaveformUart1Port_Init(waveform_transport_t *transport,
                                        UART_HandleTypeDef *huart)
{
  if ((transport == NULL) || (huart == NULL)) return HAL_ERROR;

  huart->Instance = USART1;
  huart->Init.BaudRate = WAVEFORM_UART1_BAUDRATE;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart) != HAL_OK) return HAL_ERROR;

  s_uart1_context.huart = huart;
  s_uart1_context.timeout_ms = 2UL;
  transport->send = WaveformUart1Port_Send;
  transport->context = &s_uart1_context;
  return HAL_OK;
}
