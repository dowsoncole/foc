#ifndef CONTROL_TIMING_H
#define CONTROL_TIMING_H

/* TIM1 center-aligned period = 2*ARR (RM0440 time-base unit), not 2*(ARR+1).
   One carrier/ADC/control/encoder slot: 168 MHz / (2*4200) = 20 kHz = 50 us. */
#define CONTROL_TIMER_CLOCK_HZ        168000000UL
#define CONTROL_FREQUENCY_HZ          20000UL
#define CONTROL_PERIOD_S              (1.0f / (float)CONTROL_FREQUENCY_HZ)
#define CONTROL_PERIOD_TICKS          (CONTROL_TIMER_CLOCK_HZ / CONTROL_FREQUENCY_HZ)
#define CONTROL_PWM_PEAK_TICKS        (CONTROL_PERIOD_TICKS / 2UL)
#define CONTROL_SAMPLES_PER_MS        (CONTROL_FREQUENCY_HZ / 1000UL)
#define CONTROL_READY_SAMPLES         (5UL * CONTROL_SAMPLES_PER_MS)

#if CONTROL_TIMER_CLOCK_HZ % (2UL * CONTROL_FREQUENCY_HZ) != 0
#error "Center-aligned period must be an exact even number of timer ticks"
#endif
#if CONTROL_FREQUENCY_HZ % 1000UL != 0
#error "Control cadence must support an integer 1 ms speed window"
#endif

#endif
