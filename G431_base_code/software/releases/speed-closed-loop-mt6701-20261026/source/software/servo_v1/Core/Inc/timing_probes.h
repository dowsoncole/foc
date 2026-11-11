#ifndef TIMING_PROBES_H
#define TIMING_PROBES_H

#include "stm32g4xx_hal.h"

/* Idle low at boot. PD2 latches the first encoder CRC error; PA0 latches
   an encoder communication error. IRQ timing calls must not clear them. */
__STATIC_FORCEINLINE void TimingProbe_EncoderBegin(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_EncoderEnd(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_EncoderCrcError(void)
{
  GPIOD->BSRR = GPIO_PIN_2;
}

__STATIC_FORCEINLINE void TimingProbe_EncoderCommunicationError(void)
{
  GPIOA->BSRR = GPIO_PIN_0;
}

__STATIC_FORCEINLINE void TimingProbe_AdcBegin(void)
{
}

__STATIC_FORCEINLINE void TimingProbe_AdcEnd(void)
{
}

#endif
