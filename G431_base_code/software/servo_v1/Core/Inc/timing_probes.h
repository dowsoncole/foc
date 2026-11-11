#ifndef TIMING_PROBES_H
#define TIMING_PROBES_H

#include "stm32g4xx_hal.h"

/* PA0/PD2 are active high. PB7 is active low:
   PA0: complete ADC/FOC IRQ, PD2: complete TIM1 update IRQ,
   PB7 low: Encoder_TransferComplete callback. */
__STATIC_FORCEINLINE void TimingProbe_Tim1UpdateBegin(void)
{
  GPIOD->BSRR = GPIO_PIN_2;
}

__STATIC_FORCEINLINE void TimingProbe_Tim1UpdateEnd(void)
{
  GPIOD->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
}

__STATIC_FORCEINLINE void TimingProbe_EncoderDmaCallbackBegin(void)
{
  GPIOB->BSRR = (uint32_t)GPIO_PIN_7 << 16U;
}

__STATIC_FORCEINLINE void TimingProbe_EncoderDmaCallbackEnd(void)
{
  GPIOB->BSRR = GPIO_PIN_7;
}

/* Legacy transaction-span hooks are retained for host compatibility. */
__STATIC_FORCEINLINE void TimingProbe_EncoderBegin(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_EncoderEnd(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_EncoderCrcError(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_EncoderCommunicationError(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_AdcBegin(void)
{
  GPIOA->BSRR = GPIO_PIN_0;
}

__STATIC_FORCEINLINE void TimingProbe_AdcEnd(void)
{
  GPIOA->BSRR = (uint32_t)GPIO_PIN_0 << 16U;
}

#endif
