/* Compile the actual SPI transport, protocol and encoder service against
   register stubs. Tests inject broken DMA/SPI state, not a model of the fix. */
#include "spi1_encoder.h"
#include "encoder.h"
#include "mt6835.h"
#include "timing_probes.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

SPI_TypeDef test_spi;
TIM_TypeDef test_tim1, test_tim16;
DMA_Channel_TypeDef test_dma_rx, test_dma_tx;
DMA_TypeDef test_dma;
GPIO_TypeDef test_gpioa, test_gpiob, test_gpioc, test_gpiod;
test_dwt_t test_dwt;
test_debug_t test_debug;
test_nvic_t test_nvic;
RCC_TypeDef test_rcc;
uint32_t SystemCoreClock = 168000000U, test_primask, test_resets;
HAL_StatusTypeDef test_sync_status = HAL_OK;
static unsigned callbacks;
static const uint32_t expected_pd2 = GPIO_PIN_2 << 16;
/* Mirror the outer TIM1 IRQ probe ownership. */
static void pwm_update(void) {
  TimingProbe_EncoderBegin();
  Encoder_OnPwmPeriodIsr();
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}
static HAL_StatusTypeDef callback_status;
static uint32_t callback_spi_state;

void test_barrier(void) {
  /* StopHardware has disabled DMA but not SPE at its barrier. The sensor
     must already be deselected before cleanup can change SPI clock state. */
  if((SPI1->CR1 & SPI_CR1_SPE) &&
     !(SPI1->CR2 & (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN)) &&
     !(DMA1_Channel1->CCR & DMA_CCR_EN) && !(DMA1_Channel2->CCR & DMA_CCR_EN))
    assert(GPIOA->BSRR == GPIO_PIN_4);
  DMA1->ISR &= ~DMA1->IFCR; DMA1->IFCR = 0U;
}
void test_spi_reset(void) {
  memset(SPI1, 0, sizeof(*SPI1)); SPI1->SR = SPI_SR_TXE; test_resets++;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint32_t pin, uint32_t state) {
  p->BSRR = state ? pin : pin << 16;
}
void HAL_GPIO_Init(GPIO_TypeDef *p, const GPIO_InitTypeDef *c) {
  if (p == GPIOA && c->Pin == GPIO_PIN_4) {
    assert(p->BSRR == GPIO_PIN_4);
    assert(c->Speed == SPI1_ENC_OUTPUT_GPIO_SPEED);
  }
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) {
  h->State = HAL_SPI_STATE_READY; h->Lock = HAL_UNLOCKED;
  SPI1->CR1 = h->Init.Mode | h->Init.CLKPolarity | h->Init.CLKPhase |
               h->Init.NSS | h->Init.BaudRatePrescaler;
  SPI1->CR2 = SPI_DATASIZE_8BIT | SPI_CR2_FRXTH;
  SPI1->SR = SPI_SR_TXE; return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h) {
  h->State = HAL_DMA_STATE_READY;
  h->Instance->CCR = h->Init.Direction | h->Init.MemInc | h->Init.Priority;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout) {
  (void)h; (void)tx; (void)timeout;
  assert(MT6835_SPI_IsBusy());
  assert(MT6835_SPI_AbortAsync() == HAL_BUSY);
  assert(GPIOA->BSRR == (GPIO_PIN_4 << 16));
  uint8_t other_tx[6] = {0}, other_rx[6] = {0};
  assert(MT6835_SPI_TransmitReceive(other_tx, other_rx, 6U, 1U) == HAL_BUSY);
  assert(GPIOA->BSRR == (GPIO_PIN_4 << 16));
  memset(rx, 0, size);
  if (test_sync_status != HAL_OK) SPI1->SR |= SPI_SR_BSY;
  return test_sync_status;
}
void HAL_NVIC_ClearPendingIRQ(uint32_t irq) { (void)irq; test_barrier(); }
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t p, uint32_t s) { (void)irq; (void)p; (void)s; }
void HAL_NVIC_EnableIRQ(uint32_t irq) { (void)irq; }
void HAL_Delay(uint32_t ms) { (void)ms; }

static void callback(HAL_StatusTypeDef status, void *context) {
  assert(context == &callbacks);
  assert(!MT6835_SPI_IsBusy());
  assert(GPIOA->BSRR == GPIO_PIN_4);
  callbacks++; callback_status = status; callback_spi_state = SPI1->CR1;
}
static void complete(void) {
  uint32_t probe_high = GPIOD->BSRR == GPIO_PIN_2;
  DMA1_Channel1->CNDTR = DMA1_Channel2->CNDTR = 0U;
  DMA1->ISR = DMA_ISR_TCIF1;
  SPI1->SR = SPI_SR_TXE;
  SPI1_Encoder_DmaRxIrqHandler();
  /* The real vector is not linked in this transport fixture. Its final
     idle-gated probe operation is mirrored here, AFTER all callback work.
     Catch an early falling edge anywhere inside the callback/driver. */
  if (probe_high) assert(GPIOD->BSRR == GPIO_PIN_2);
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}
static void transport_tests(void) {
  uint8_t tx[6] = {0xA0, 3}, rx[6] = {0};
  assert(MX_SPI1_Encoder_Init() == HAL_OK);
  assert(MT6835_SPI_TransmitReceiveAsync(NULL, rx, 6U, callback, &callbacks) == HAL_ERROR);
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  assert(!(DMA1_Channel1->CCR & DMA_CCR_HTIE));
  assert(!(DMA1_Channel2->CCR & (DMA_CCR_HTIE | DMA_CCR_TCIE)));
  uint32_t cs = GPIOA->BSRR;
  assert(MT6835_ReadAngle(&(mt6835_angle_t){0}, 1U) == HAL_BUSY);
  assert(GPIOA->BSRR == cs); /* Blocking caller must not deassert live DMA CS. */
  assert(MT6835_SPI_ResyncIdle() == HAL_BUSY);
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_BUSY);
  complete();
  assert(callbacks == 1 && callback_status == HAL_OK && !(callback_spi_state & SPI_CR1_SPE));
  assert(!(SPI1->CR2 & (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_ERRIE)));
  SPI1_Encoder_DmaRxIrqHandler(); SPI1_Encoder_DmaTxIrqHandler();
  assert(callbacks == 1);

  /* Simultaneous RX completion and TX error must report ERROR once. */
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  DMA1->ISR = DMA_ISR_TCIF1 | DMA_ISR_TEIF2;
  SPI1_Encoder_DmaRxIrqHandler(); SPI1_Encoder_DmaTxIrqHandler();
  assert(callbacks == 2 && callback_status == HAL_ERROR);
  assert(!DMA1_Channel1->CNDTR && !DMA1_Channel2->CNDTR);
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  complete(); assert(callbacks == 3 && callback_status == HAL_OK);

  /* Frozen SysTick / BSY stuck cannot hang abort with interrupts masked. */
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  test_primask = 1U; SPI1->SR = SPI_SR_BSY;
  uint32_t resets = test_resets;
  assert(MT6835_SPI_AbortAsync() == HAL_OK && test_primask == 1U);
  assert(test_resets == resets + 1U && callbacks == 4 && callback_status == HAL_ERROR);
  assert(MT6835_SPI_AbortAsync() == HAL_OK && callbacks == 4);
  test_primask = 0U;
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  complete(); assert(callbacks == 5 && callback_status == HAL_OK);

  /* Incomplete RX TC and peripheral OVR cannot publish partially filled RAM. */
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  DMA1->ISR = DMA_ISR_TCIF1;
  SPI1_Encoder_DmaRxIrqHandler();
  assert(callbacks == 6 && callback_status == HAL_ERROR);
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  SPI1->SR = SPI_SR_OVR;
  SPI1_Encoder_SpiIrqHandler();
  assert(callbacks == 7 && callback_status == HAL_ERROR);
  uint32_t configured_cr1 = SPI1->CR1;
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  SPI1->CR1 = 0U; SPI1->SR = SPI_SR_MODF;
  SPI1_Encoder_SpiIrqHandler();
  assert(callbacks == 8 && callback_status == HAL_ERROR && SPI1->CR1 == configured_cr1);
  /* Pending idle errors are cleared without another consumer callback. */
  SPI1->SR = SPI_SR_OVR; SPI1_Encoder_SpiIrqHandler();
  assert(callbacks == 8 && SPI1->SR == SPI_SR_TXE);

  /* Idle software ownership permits clearing stuck hardware, not refusing it. */
  SPI1->SR = SPI_SR_BSY | SPI_SR_FRLVL;
  DMA1_Channel1->CNDTR = 3U;
  hspi1.State = HAL_SPI_STATE_BUSY_TX_RX;
  assert(MT6835_SPI_ResyncIdle() == HAL_OK);
  assert(!DMA1_Channel1->CNDTR && SPI1->SR == SPI_SR_TXE);
  DMA1_Channel1->CCR |= DMA_CCR_EN; DMA1_Channel1->CNDTR = 3U;
  resets = test_resets;
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  assert(test_resets == resets + 1U && DMA1_Channel1->CNDTR == 6U);
  complete(); assert(callbacks == 9 && callback_status == HAL_OK);
  test_sync_status = HAL_TIMEOUT;
  assert(MT6835_SPI_TransmitReceive(tx, rx, 6U, 1U) == HAL_TIMEOUT);
  assert(!MT6835_SPI_IsBusy() && SPI1->SR == SPI_SR_TXE);
  test_sync_status = HAL_OK;
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  complete(); assert(callbacks == 10 && callback_status == HAL_OK);
  /* A TX-only transfer error must be handled even without an RX interrupt. */
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  DMA1->ISR = DMA_ISR_TEIF2; SPI1_Encoder_DmaTxIrqHandler();
  assert(callbacks == 11 && callback_status == HAL_ERROR && !MT6835_SPI_IsBusy());
  assert(MT6835_SPI_TransmitReceiveAsync(tx, rx, 6U, callback, &callbacks) == HAL_OK);
  complete(); assert(callbacks == 12 && callback_status == HAL_OK);
}
static void frame(uint32_t angle, uint8_t alarm, uint8_t bad_crc) {
  assert(hspi1.TxXferSize==6U);

  uint32_t payload = (angle << 3) | alarm;
  uint8_t *rx = hspi1.pRxBuffPtr;
  rx[0] = rx[1] = 0xff;
  rx[2] = (uint8_t)(payload >> 16); rx[3] = (uint8_t)(payload >> 8);
  rx[4] = (uint8_t)payload; rx[5] = MT6835_CalcCrc24(payload) ^ bad_crc;
  DWT->CYCCNT += 1000U; complete();

}
static void prime_angle_stream(void) {
  encoder_snapshot_t snapshot;
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
  assert(hspi1.TxXferSize==6U);
  frame(100000U,0,0);
  assert(Encoder_ReadSnapshot(&snapshot));
}
static void init_encoder_stream(void) {
  assert(Encoder_Init());
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_WAITING);
  g_encoder_logic_trigger.command = ENCODER_CAPTURE_START_COMMAND;
  Encoder_Service();
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
#endif
}
/* Source-only adaptation for the direct-update path; not run in this change. */
static void direct_update_trigger_tests(void) {
  init_encoder_stream();
  assert(ENCODER_SPI_SAMPLE_PHASE_US==0UL);
  assert(!(TIM16->CR1 & TIM_CR1_CEN) && !(TIM16->DIER & TIM_DIER_UIE));
  for (unsigned i=0; i<4; i++) {
    /* No software phase wait, including late entry or down-counting. */
    TIM1->CNT=i ? 1000U : 0U;
    TIM1->CR1=(i&1U) ? TIM_CR1_DIR : 0U;
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS;
    uint32_t triggers=g_encoder_diagnostics.trigger_count;
    uint32_t busy=g_encoder_diagnostics.busy_trigger_count;
    pwm_update();
    assert(MT6835_SPI_IsBusy() && g_encoder_diagnostics.trigger_count==triggers+1U);
    assert(!(TIM16->CR1 & TIM_CR1_CEN) && !(TIM16->DIER & TIM_DIER_UIE));
    pwm_update(); /* A busy transfer must not be restarted. */
    assert(MT6835_SPI_IsBusy() && g_encoder_diagnostics.busy_trigger_count==busy+1U);
    assert(hspi1.TxXferSize==6U); frame(100000U,0,0);
    assert(!MT6835_SPI_IsBusy());
  }
  TIM1->CR1=0U; TIM1->CNT=0U;
}
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC != 2
static void encoder_tests(void) {
  encoder_snapshot_t good, current;
  init_encoder_stream(); assert(Encoder_ConfigureMotor(1, 2, 0.0f));
  assert(Encoder_ConfigureFeedbackTolerance(3U)); /* Legacy three-period fixture. */
  prime_angle_stream();
  /* Remaining DWT fixtures denote direct TIM1 update sample instants. */
  DWT->CYCCNT = 0xffffd000U; pwm_update(); frame(100000U, 0, 0);
  assert(Encoder_ReadSnapshot(&good));
  DWT->CYCCNT = good.sample_cycle + CONTROL_PERIOD_TICKS;
  pwm_update(); frame(100010U, 0, 1);
  assert(Encoder_ReadSnapshot(&current));
  assert(current.sequence == good.sequence && current.sample_cycle == good.sample_cycle);
  assert(current.raw_angle == good.raw_angle);
  assert(g_encoder_diagnostics.crc_error_count == 1U);
  assert(g_encoder_diagnostics.spi_resync_count == 1U);
  DWT->CYCCNT = good.sample_cycle + (2U*CONTROL_PERIOD_TICKS);
  pwm_update(); frame(100020U, 0, 0);
  assert(Encoder_ReadSnapshot(&current) && current.sequence == good.sequence + 1U);

  good = current;
  for (uint32_t i = 1; i <= 3; i++) {
    DWT->CYCCNT = good.sample_cycle + i * CONTROL_PERIOD_TICKS;
    pwm_update(); frame(100030U, 0, 1);
  }
  assert(!Encoder_ReadSnapshot(&current));
  assert(current.sample_cycle == good.sample_cycle && current.sequence == good.sequence);
  DWT->CYCCNT += 1000U; pwm_update(); frame(100040U, 0, 0);
  assert(Encoder_ReadSnapshot(&current));

  /* CRC-valid plausible alarm frames remain usable and preserve raw status. */
  DWT->CYCCNT += CONTROL_PERIOD_TICKS; pwm_update(); frame(100050U, 2U, 0);
  assert(Encoder_ReadSnapshot(&current));
  assert(current.fault_flags == 0U && current.status_raw == 2U);
  assert(g_encoder_diagnostics.alarm_count == 1U);
  DWT->CYCCNT += CONTROL_PERIOD_TICKS; pwm_update(); frame(100060U, 0, 0);
  assert(Encoder_ReadSnapshot(&current));

  DWT->CYCCNT += CONTROL_PERIOD_TICKS; pwm_update();
  SPI1->SR = SPI_SR_BSY;
  DWT->CYCCNT += 17000U; Encoder_Service();
  assert(!MT6835_SPI_IsBusy() && !Encoder_ReadSnapshot(&current));
  assert(current.fault_flags == ENCODER_FAULT_COMMUNICATION);
  assert(g_encoder_diagnostics.transfer_error_count == 1U);
  DWT->CYCCNT += CONTROL_PERIOD_TICKS; pwm_update(); frame(100070U, 0, 0);
  assert(Encoder_ReadSnapshot(&current));
}
#endif
static void sensor_alarm_telemetry_tests(void) {
  TIM1->BDTR=0U;
  init_encoder_stream();
  assert(Encoder_ConfigureMotor(1,2,0)); prime_angle_stream();
  TIM1->BDTR=TIM_BDTR_MOE;
  encoder_snapshot_t sample;
  for (uint8_t alarm=1U; alarm<=7U; alarm++) {
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
    frame(100000U,alarm,0);
    assert(Encoder_ReadSnapshot(&sample));
    assert(sample.status_raw==alarm && sample.fault_flags==0U);
    assert(TIM1->BDTR & TIM_BDTR_MOE);
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
    assert(g_encoder_logic_trigger.state==ENCODER_CAPTURE_ARMED);
#endif
  }
  assert(g_encoder_diagnostics.alarm_count==7U);
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
  frame(1000000U,7U,0);
  assert(g_encoder_diagnostics.angle_step_error_count==1U);
  assert(g_encoder_snapshot.raw_angle==100000U);
  TIM1->BDTR=0U;
}
static void angle_only_stream_tests(void) {
  encoder_snapshot_t snapshot;
  init_encoder_stream(); assert(Encoder_ConfigureMotor(1,2,0));
  for(unsigned i=0;i<300;i++) {
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
    assert(hspi1.TxXferSize==6U);
    frame(100000U,0,0);
    assert(Encoder_ReadSnapshot(&snapshot));
    assert(!(snapshot.fault_flags & ENCODER_FAULT_ZERO_CONFIG));
  }
  assert(g_encoder_zero_guard.checks==0U &&
         g_encoder_zero_guard.verified_mask==0U &&
         g_encoder_zero_fault_capture.count==0U);
  puts("PASS angle-only stream without ZERO_POS polling or verification gate");
}
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
static void logic_trigger_tests(void) {
  assert(sizeof(encoder_logic_trigger_t) == 48U);
  assert(Encoder_Init());
  assert(g_encoder_logic_trigger.version == 1U &&
         g_encoder_logic_trigger.diagnostic_mode == ENCODER_FAULT_TRIGGER_DIAGNOSTIC);
  assert(!Encoder_PowerOutputPermitted());
  assert(GPIOA->BSRR == GPIO_PIN_4); /* SPI init deselects CS, not PA0. */
  TIM16->CR1 = 0U;
  for (unsigned i=0; i<100; i++) {
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS;
    pwm_update(); frame(100000U,0,0); Encoder_Service();
  }
  /* Diagnostic arming gates output permission, not background acquisition. */
  assert(!MT6835_SPI_IsBusy() && g_encoder_diagnostics.trigger_count==100U);
  assert(g_encoder_logic_trigger.state==ENCODER_CAPTURE_WAITING);
  assert(!(TIM16->CR1 & TIM_CR1_CEN));
  g_encoder_logic_trigger.command = 1U; Encoder_Service();
  assert(g_encoder_logic_trigger.rejected_commands == 1U &&
         g_encoder_logic_trigger.command == 0U);
  TIM1->BDTR = TIM_BDTR_MOE;
  g_encoder_logic_trigger.command = ENCODER_CAPTURE_START_COMMAND; Encoder_Service();
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_WAITING &&
         g_encoder_logic_trigger.rejected_commands == 2U);
  TIM1->BDTR = 0U;
  g_encoder_logic_trigger.command = ENCODER_CAPTURE_START_COMMAND; Encoder_Service();
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
  assert(Encoder_PowerOutputPermitted() == (ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2));
  assert(Encoder_ConfigureMotor(1,2,0)); prime_angle_stream();
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100000U,0,0);
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
  TIM1->BDTR = TIM_BDTR_MOE;
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100010U,0,1);
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
  /* Only CRC gets bounded tolerance in commissioning mode. */
  assert(TIM1->BDTR & TIM_BDTR_MOE);
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
  assert(g_encoder_diagnostics.crc_error_count == 1U);
  assert(g_encoder_fault_capture.last.reason == ENCODER_FAULT_CRC);
  encoder_snapshot_t retained;
  assert(Encoder_ReadSnapshot(&retained) && retained.raw_angle == 100000U);
  assert(!Encoder_ConfigureFeedbackTolerance(10U)); /* Reject while MOE on. */
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100000U,2U,0);
  assert(TIM1->BDTR & TIM_BDTR_MOE);
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
  assert(Encoder_ReadSnapshot(&retained) && retained.status_raw == 2U);
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
  SPI1->SR=SPI_SR_OVR; SPI1_Encoder_SpiIrqHandler();
  assert(!(TIM1->BDTR & TIM_BDTR_MOE));
  assert(!Encoder_PowerOutputPermitted());
#endif
  TIM1->BDTR = 0U;
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_FIRED &&
         g_encoder_logic_trigger.reason ==
           (ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2 ? ENCODER_FAULT_COMMUNICATION : ENCODER_FAULT_CRC) &&
         g_encoder_logic_trigger.previous_angle == 100000U &&
         g_encoder_logic_trigger.register_address == 0U);
  encoder_logic_trigger_t first = g_encoder_logic_trigger;
  for (unsigned i=0; i<20; i++) {
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100020U,0,(uint8_t)(i&1U));
    /* ADC pulse must not disturb the completed SPI pulse. */
    assert(GPIOD->BSRR == expected_pd2);

    TimingProbe_AdcBegin();
    assert(GPIOA->BSRR == GPIO_PIN_0);
    TimingProbe_AdcEnd();
    assert(GPIOA->BSRR == (GPIO_PIN_0 << 16));
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
    encoder_snapshot_t sample;
    assert(!Encoder_ReadSnapshot(&sample) && (sample.fault_flags & ENCODER_FAULT_COMMUNICATION));
    assert(!Encoder_PowerOutputPermitted());
#endif
  }
  encoder_logic_trigger_t frozen = g_encoder_logic_trigger;
  assert(memcmp(&first, &frozen, sizeof(first)) == 0);
  g_encoder_logic_trigger.command = ENCODER_CAPTURE_START_COMMAND; Encoder_Service();
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_FIRED &&
         g_encoder_logic_trigger.rejected_commands == first.rejected_commands+1U &&
         g_encoder_logic_trigger.cycle == first.cycle);

  init_encoder_stream(); pwm_update();
  SPI1->SR=SPI_SR_OVR; SPI1_Encoder_SpiIrqHandler();
  assert(g_encoder_logic_trigger.reason == ENCODER_FAULT_COMMUNICATION);
  init_encoder_stream(); assert(Encoder_ConfigureMotor(1,2,0)); prime_angle_stream();
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100000U,0,0);
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(1000000U,0,0);
  assert(g_encoder_logic_trigger.reason == ENCODER_FAULT_IMPLAUSIBLE);
  init_encoder_stream(); prime_angle_stream();
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100000U,2U,0);
  assert(g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED);
  assert(g_encoder_fault_capture.last_alarm.reason == ENCODER_FAULT_SENSOR_ALARM);
  init_encoder_stream(); prime_angle_stream();
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update();
  SPI1->SR=SPI_SR_BSY; DWT->CYCCNT+=17000U; Encoder_Service();
  assert(g_encoder_logic_trigger.reason == ENCODER_FAULT_COMMUNICATION);
  assert(GPIOD->BSRR == expected_pd2); /* Timeout closes the timing pulse. */
  puts("PASS OFF-only capture boot wait, explicit arm, invalid/live/repeated command rejection, first-event freeze and all fault classes");
}
#endif
static void feedback_tolerance_tests(void) {
  TIM1->BDTR=0U;
  init_encoder_stream();
  assert(Encoder_ConfigureMotor(1,2,0)); prime_angle_stream();
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; pwm_update(); frame(100000U,0,0);
  encoder_snapshot_t good, sample;
  assert(Encoder_ReadSnapshot(&good));
  assert(!Encoder_ConfigureFeedbackTolerance(0U));
  assert(!Encoder_ConfigureFeedbackTolerance(ENCODER_TOLERANCE_MAX_CYCLES+1U));
  DWT->CYCCNT=good.sample_cycle+4U*CONTROL_PERIOD_TICKS;
  assert(Encoder_ReadSnapshot(&sample)); /* Boot default is now ten periods. */
  assert(Encoder_ConfigureFeedbackTolerance(3U));
  /* No frame arrives: age comes from the original sample, not wall-clock
     reconfiguration time. The setter must not refresh timestamp or sequence. */
  DWT->CYCCNT=good.sample_cycle+3U*CONTROL_PERIOD_TICKS;
  assert(Encoder_ReadSnapshot(&sample));
  DWT->CYCCNT++;
  assert(!Encoder_ReadSnapshot(&sample));
  assert(Encoder_ConfigureFeedbackTolerance(10U));
  assert(Encoder_ReadSnapshot(&sample));
  assert(sample.sequence==good.sequence && sample.sample_cycle==good.sample_cycle);
  TIM1->BDTR=TIM_BDTR_MOE;
  assert(!Encoder_ConfigureFeedbackTolerance(1U));
  TIM1->BDTR=0U;
  DWT->CYCCNT=good.sample_cycle+10U*CONTROL_PERIOD_TICKS;
  assert(Encoder_ReadSnapshot(&sample));
  DWT->CYCCNT++;
  assert(!Encoder_ReadSnapshot(&sample));
  assert(Encoder_ConfigureFeedbackTolerance(ENCODER_MAX_SAMPLE_AGE_CYCLES));
}
static void communication_gpio_tests(void) {
  TIM1->BDTR=0U;
  init_encoder_stream(); prime_angle_stream();
  TimingProbe_AdcBegin();
  assert(GPIOA->BSRR == GPIO_PIN_0);
  TimingProbe_AdcEnd();
  assert(GPIOA->BSRR == (GPIO_PIN_0 << 16));
  TimingProbe_Tim1UpdateBegin();
  assert(GPIOD->BSRR == GPIO_PIN_2);
  TimingProbe_Tim1UpdateEnd();
  assert(GPIOD->BSRR == (GPIO_PIN_2 << 16));
  TimingProbe_EncoderDmaCallbackBegin();
  assert(GPIOB->BSRR == (GPIO_PIN_7 << 16));
  TimingProbe_EncoderDmaCallbackEnd();
  assert(GPIOB->BSRR == GPIO_PIN_7);
}
int main(void) {
  assert(g_encoder_logic_trigger.command ==
         (ENCODER_FAULT_TRIGGER_DIAGNOSTIC ? ENCODER_CAPTURE_START_COMMAND : 0U));
  GPIOD->BSRR = expected_pd2; /* Simulate boot GPIO initialization. */
  assert(ENCODER_MAX_SAMPLE_AGE_CYCLES==10U);
  transport_tests();
  assert(ENCODER_CONTROL_RATE_HZ==20000U && ENCODER_SPEED_WINDOW_SAMPLES==20U);
  assert(Encoder_Init());
  Encoder_RecordFocIsrCycles(CONTROL_PERIOD_TICKS/2U); /* Half-period stays within the actual carrier. */
  Encoder_RecordFocIsrCycles(CONTROL_PERIOD_TICKS-1U);
  assert(!g_encoder_diagnostics.foc_deadline_overrun_count);
  Encoder_RecordFocIsrCycles(CONTROL_PERIOD_TICKS);
  assert(g_encoder_diagnostics.foc_deadline_overrun_count==1U);
  Encoder_RecordFocIsrCycles(CONTROL_PERIOD_TICKS+1U);
  assert(g_encoder_diagnostics.foc_deadline_overrun_count==2U);
  direct_update_trigger_tests();
  feedback_tolerance_tests();
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC != 2
  encoder_tests();
#endif
  sensor_alarm_telemetry_tests();
  angle_only_stream_tests();
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  logic_trigger_tests();
#endif
  communication_gpio_tests();
  puts("PASS: SPI/DMA errors, frozen-tick abort, ownership, recovery, CRC/freshness, alarms and PA0/PD2/PB7 timing pulses");
  return 0;
}
