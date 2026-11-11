#include "waveform_uart1_port.h"
#include <string.h>

typedef struct
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef dma;
  uint8_t frame[WAVEFORM_MAX_FRAME_LENGTH];
  uint8_t busy;
  uint8_t faulted;
  uint32_t started_ms;
} waveform_uart1_context_t;

static waveform_uart1_context_t s_uart1_context;
volatile uint32_t g_waveform_uart_completed;
volatile uint32_t g_waveform_uart_dma_errors;

void WaveformUart1Port_Service(void)
{
  waveform_uart1_context_t *port = &s_uart1_context;
  if (!port->busy) return;
  if ((__HAL_DMA_GET_FLAG(&port->dma, DMA_FLAG_TE3) != RESET) ||
      ((HAL_GetTick() - port->started_ms) > 100U))
  {
    CLEAR_BIT(port->huart->Instance->CR3, USART_CR3_DMAT);
    (void)HAL_DMA_Abort(&port->dma);
    if (!port->faulted) g_waveform_uart_dma_errors++;
    port->faulted = 1U;
  }
  if (!port->faulted &&
      __HAL_DMA_GET_FLAG(&port->dma, DMA_FLAG_TC3) == RESET) return;
  /* DMA completion only means the last byte reached TDR, not the wire. */
  if (__HAL_UART_GET_FLAG(port->huart, UART_FLAG_TC) == RESET) return;
  CLEAR_BIT(port->huart->Instance->CR3, USART_CR3_DMAT);
  if (!port->faulted)
  {
    (void)HAL_DMA_PollForTransfer(&port->dma, HAL_DMA_FULL_TRANSFER, 0U);
    g_waveform_uart_completed++;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  port->busy = 0U;
}

static uint8_t WaveformUart1Port_Send(void *context,
                                     const uint8_t *data,
                                     uint16_t length)
{
  waveform_uart1_context_t *port = (waveform_uart1_context_t *)context;
  if ((port == NULL) || (port->huart == NULL) ||
      (data == NULL) || (length == 0U) || (length > sizeof(port->frame)) ||
      port->faulted ||
      (HAL_UART_GetState(port->huart) != HAL_UART_STATE_READY))
  {
    return 0U;
  }

  WaveformUart1Port_Service();
  if (port->faulted) return 0U;
  if (port->busy) return 2U;
  /* The caller's frame is on its stack: DMA must own a persistent copy. */
  memcpy(port->frame, data, length);
  __HAL_UART_CLEAR_FLAG(port->huart, UART_CLEAR_TCF);
  if (HAL_DMA_Start(&port->dma, (uint32_t)port->frame,
                    (uint32_t)&port->huart->Instance->TDR, length) != HAL_OK)
  {
    g_waveform_uart_dma_errors++;
    return 0U;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  port->started_ms = HAL_GetTick();
  port->busy = 1U;
  SET_BIT(port->huart->Instance->CR3, USART_CR3_DMAT);
  return 1U;
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
  s_uart1_context.busy = s_uart1_context.faulted = 0U;
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  /* Encoder owns channels 1/2. Channel 3 is exclusively USART1 TX. */
  s_uart1_context.dma.Instance = DMA1_Channel3;
  s_uart1_context.dma.Init.Request = DMA_REQUEST_USART1_TX;
  s_uart1_context.dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_uart1_context.dma.Init.PeriphInc = DMA_PINC_DISABLE;
  s_uart1_context.dma.Init.MemInc = DMA_MINC_ENABLE;
  s_uart1_context.dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_uart1_context.dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_uart1_context.dma.Init.Mode = DMA_NORMAL;
  s_uart1_context.dma.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&s_uart1_context.dma) != HAL_OK) return HAL_ERROR;
  transport->send = WaveformUart1Port_Send;
  transport->context = &s_uart1_context;
  return HAL_OK;
}
