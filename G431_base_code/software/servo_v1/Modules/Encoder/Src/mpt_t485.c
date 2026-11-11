#include "mpt_t485.h"
#include "stm32g4xx_hal.h"
#include <string.h>
extern UART_HandleTypeDef huart1;
volatile mpt_t485_t g_mpt_t485={.enabled=1U};
static DMA_HandleTypeDef rx_dma, de_dma;
static uint32_t de_reset_word=(uint32_t)GPIO_PIN_8<<16;
static uint8_t rx[16];
static uint32_t started_ms, last_request_ms;
static uint32_t started_cycle;
static uint8_t busy;

uint8_t MptT485_Init(void)
{
  huart1.Instance=USART1;
  huart1.Init.BaudRate=2500000;
  huart1.Init.WordLength=UART_WORDLENGTH_8B;
  huart1.Init.StopBits=UART_STOPBITS_1;
  huart1.Init.Parity=UART_PARITY_NONE;
  huart1.Init.Mode=UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart1.Init.OverSampling=UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_Init(&huart1)!=HAL_OK) return 0;
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  /* SPI1 motor encoder owns DMA1 channels 1/2; waveform TX is disabled. */
  rx_dma.Instance=DMA1_Channel3;
  rx_dma.Init.Request=DMA_REQUEST_USART1_RX;
  rx_dma.Init.Direction=DMA_PERIPH_TO_MEMORY;
  rx_dma.Init.PeriphInc=DMA_PINC_DISABLE;
  rx_dma.Init.MemInc=DMA_MINC_ENABLE;
  rx_dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
  rx_dma.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
  rx_dma.Init.Mode=DMA_NORMAL;
  rx_dma.Init.Priority=DMA_PRIORITY_LOW;
  if(HAL_DMA_Init(&rx_dma)!=HAL_OK) return 0;
  /* TIM7 one-pulse update requests a single GPIO BSRR write through DMA4.
     UART TC does not have a GPIO DMA request. Match its 10-bit wire time
     using the actual peripheral clocks, then add 16 timer ticks of hold. */
  uint32_t timer_clock=HAL_RCC_GetPCLK1Freq();
  if((RCC->CFGR&RCC_CFGR_PPRE1)!=0U) timer_clock*=2U;
  uint32_t uart_clock=HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART1);
  if(!uart_clock || huart1.Init.OverSampling!=UART_OVERSAMPLING_16) return 0;
  uint32_t wire_ticks=(uint32_t)(((uint64_t)timer_clock*USART1->BRR*10U+
                                 uart_clock-1U)/uart_clock)+16U;
  if(wire_ticks>65535U) return 0;
  __HAL_RCC_TIM7_CLK_ENABLE();
  TIM7->CR1=TIM_CR1_OPM|TIM_CR1_URS;
  TIM7->PSC=0; TIM7->ARR=wire_ticks-1U;
  TIM7->EGR=TIM_EGR_UG; TIM7->SR=0;
  TIM7->DIER=TIM_DIER_UDE;
  de_dma.Instance=DMA1_Channel4;
  de_dma.Init.Request=DMA_REQUEST_TIM7_UP;
  de_dma.Init.Direction=DMA_MEMORY_TO_PERIPH;
  de_dma.Init.PeriphInc=DMA_PINC_DISABLE;
  de_dma.Init.MemInc=DMA_MINC_DISABLE;
  de_dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
  de_dma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
  de_dma.Init.Mode=DMA_NORMAL;
  de_dma.Init.Priority=DMA_PRIORITY_HIGH;
  if(HAL_DMA_Init(&de_dma)!=HAL_OK) return 0;
  g_mpt_t485.initialized=1;
  return 1;
}
static void finish(uint32_t error)
{
  uint32_t angle=0, turns=0;
  CLEAR_BIT(USART1->CR3,USART_CR3_DMAR);
  uint32_t n=sizeof(rx)-__HAL_DMA_GET_COUNTER(&rx_dma);
  (void)HAL_DMA_Abort(&rx_dma);
  busy=0;
  CLEAR_BIT(TIM7->CR1,TIM_CR1_CEN);
  (void)HAL_DMA_Abort(&de_dma);
  GPIOA->BSRR=de_reset_word;
  memcpy((void *)g_mpt_t485.frame,rx,sizeof(rx));
  g_mpt_t485.status=rx[1]; g_mpt_t485.alarm=rx[9];
  if(!error) error=MptT485_Decode(rx,n,&angle,&turns);
  if(error) {
    g_mpt_t485.valid=g_mpt_t485.consecutive_good=0;
    if(error==1) g_mpt_t485.crc_errors++;
    if(error==2) g_mpt_t485.format_errors++;
    if(error==3) g_mpt_t485.status_errors++;
    if(error==4) g_mpt_t485.timeouts++;
    if(error==5) g_mpt_t485.uart_errors++;
    return;
  }
  /* Publish atomically to the ADC-owned position controller. */
  uint32_t mask=__get_PRIMASK(); __disable_irq();
  g_mpt_t485.angle_raw=angle; g_mpt_t485.turns_raw=turns;
  g_mpt_t485.status=rx[1]; g_mpt_t485.alarm=rx[9];
  g_mpt_t485.angle_deg=angle*(360.0f/MPT_COUNTS_PER_TURN);
  /* [千问工作助理 2026-10-28] 记录器件真实响应延迟，取代按 2 ms 超时门限估算。
     HAL_GetTick() 为 1 ms 分辨率，故单帧延迟量化为 0/1/2 ms；用于确认
     1 ms 轮询下器件是否真能每周期返回新角度（数据手册 Update Rate=50 kHz）。 */
  {
    uint32_t latency=(uint32_t)(HAL_GetTick()-started_ms);
    if(latency>g_mpt_t485.resp_latency_max_ms) g_mpt_t485.resp_latency_max_ms=latency;
    g_mpt_t485.resp_latency_sum_ms+=latency;
    uint32_t latency_us=(uint32_t)(DWT->CYCCNT-started_cycle)/(SystemCoreClock/1000000U);
    g_mpt_t485.resp_latency_last_us=latency_us;
    if(latency_us>g_mpt_t485.resp_latency_max_us) g_mpt_t485.resp_latency_max_us=latency_us;
    if(g_mpt_t485.resp_latency_samples<0xFFFFFFFFU) g_mpt_t485.resp_latency_samples++;
  }
  /* Age the latched measurement from its request, not delayed foreground parsing. */
  g_mpt_t485.last_good_ms=started_ms;
  g_mpt_t485.good_frames++; g_mpt_t485.sequence++;
  if(g_mpt_t485.consecutive_good<1000U) g_mpt_t485.consecutive_good++;
  g_mpt_t485.valid=g_mpt_t485.consecutive_good>=10U;
  if(!mask) __enable_irq();
}
void MptT485_Task(void)
{
  uint32_t now=HAL_GetTick();
  if(!g_mpt_t485.initialized) return;
  if(busy) {
    uint32_t flags=USART1->ISR;
    if((flags&(USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE)) ||
       __HAL_DMA_GET_FLAG(&rx_dma,DMA_FLAG_TE3)!=RESET ||
       __HAL_DMA_GET_FLAG(&de_dma,DMA_FLAG_TE4)!=RESET) finish(5);
    else if((flags&USART_ISR_IDLE) && __HAL_DMA_GET_COUNTER(&rx_dma)<=sizeof(rx)-MPT_FRAME_LENGTH) finish(0);
    /* [千问工作助理 2026-10-28] 超时门限改用宏，与 1 ms 轮询周期解耦。
       原硬编码 2U 保持不变，线时间仅 48 us，仍有约 40 倍余量。 */
    else if((uint32_t)(now-started_ms)>=OUTPUT_POSITION_RESPONSE_TIMEOUT_MS) finish(4);
    else return;
  }
  if(!g_mpt_t485.enabled) { g_mpt_t485.valid=0; g_mpt_t485.consecutive_good=0; return; }
  if((uint32_t)(now-last_request_ms)<OUTPUT_POSITION_PERIOD_MS) return;
  CLEAR_BIT(USART1->CR1,USART_CR1_RE);
  USART1->ICR=USART_ICR_IDLECF|USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF|USART_ICR_PECF|USART_ICR_TCCF;
  USART1->RQR=USART_RQR_RXFRQ;
  memset(rx,0,sizeof(rx));
  if(HAL_DMA_Start(&rx_dma,(uint32_t)&USART1->RDR,(uint32_t)rx,sizeof(rx))!=HAL_OK) {
    g_mpt_t485.valid=0; g_mpt_t485.uart_errors++; return;
  }
  (void)HAL_DMA_Abort(&de_dma);
  TIM7->CNT=0; TIM7->SR=0;
  if(HAL_DMA_Start(&de_dma,(uint32_t)&de_reset_word,(uint32_t)&GPIOA->BSRR,1U)!=HAL_OK) {
    busy=1; finish(5); return;
  }
  SET_BIT(USART1->CR3,USART_CR3_DMAR);
  SET_BIT(USART1->CR1,USART_CR1_RE);
  GPIOA->BSRR=GPIO_PIN_8;
  uint32_t de_cycle=DWT->CYCCNT;
  while((uint32_t)(DWT->CYCCNT-de_cycle)<SystemCoreClock/1000000U) {}
  started_ms=last_request_ms=HAL_GetTick(); busy=1;
  /* Only the adjacent launch writes are atomic (<1 us), never the wire wait.
     DMA releases DE after the entire stop bit, even if the ADC ISR is active. */
  uint32_t mask=__get_PRIMASK(); __disable_irq();
  started_cycle=DWT->CYCCNT;
  USART1->TDR=0x1AU;
  SET_BIT(TIM7->CR1,TIM_CR1_CEN);
  if(!mask) __enable_irq();
  g_mpt_t485.requests++;
}
