/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm.h
  * @brief   TIM1 three-phase complementary PWM for the STSPIN32G4 power stage.
  *
  *          PE8  -> TIM1_CH1N   PE9  -> TIM1_CH1    -> GD INL1 / INH1  (phase A)
  *          PE10 -> TIM1_CH2N   PE11 -> TIM1_CH2    -> GD INL2 / INH2  (phase B)
  *          PE12 -> TIM1_CH3N   PE13 -> TIM1_CH3    -> GD INL3 / INH3  (phase C)
  *
  *          Pin map and GPIO configuration come from the STSPIN32G4 datasheet,
  *          "Table 7. MCU to gate driver internal connection": every gate signal
  *          is a push-pull alternate-function output, and the gate driver IC only
  *          tolerates that single GPIO configuration.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM1_PWM_H__
#define __TIM1_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* --------------------------------------------------------------------------*/
/* Timing                                                                    */
/* --------------------------------------------------------------------------*/
/* SystemClock_Config() sets SYSCLK = HSE 12 MHz / 1 * 28 / 2 = 168 MHz with
   APB2 prescaler 1, so TIM1 is clocked at 168 MHz. */
#define TIM1_PWM_TIMER_CLOCK_HZ     168000000UL

/* Center-aligned PWM: the counter runs 0 -> ARR -> 0, so one carrier period is
   2 x (ARR + 1) counter clocks and the repetition counter makes exactly one
   update event fall into each of them. Both the peak value loaded into ARR and
   the number of counts in a full period are named, because the compare
   registers are expressed against the latter. */
#define TIM1_PWM_FREQUENCY_HZ       20000UL
#define TIM1_PWM_COUNTS_PER_PERIOD  (TIM1_PWM_TIMER_CLOCK_HZ / TIM1_PWM_FREQUENCY_HZ)
#define TIM1_PWM_COUNTER_PERIOD     ((TIM1_PWM_COUNTS_PER_PERIOD / 2UL) - 1UL)

/* Dead time inserted between a main and its complementary output. TIM_BDTR_DTG
   values 0x00..0x7F are in units of tDTS (clock division DIV1 -> 1/168 MHz =
   5.952 ns), so 84 counts = 500 ns. That range ends at 127 counts; a longer
   dead time needs the 0x80..0xBF encoding (2 x tDTS steps). */
#define TIM1_PWM_DEADTIME_TDTS      84U

/* One tDTS is 1 / 168 MHz = 5.952 ns. */
#define TIM1_PWM_DEADTIME_NS        ((TIM1_PWM_DEADTIME_TDTS * 1000000000UL) / \
                                     TIM1_PWM_TIMER_CLOCK_HZ)

/* Application duty scale: the fraction of the carrier period for which the
   HIGH-SIDE switch of a half bridge is on. 0 = high side never on, 1000 = high
   side permanently on, 500 = 50 %. It divides TIM1_PWM_COUNTS_PER_PERIOD
   exactly, so one duty step is 8.4 compare counts. */
#define TIM1_PWM_DUTY_SCALE         1000U
#define TIM1_PWM_DUTY_NEUTRAL       (TIM1_PWM_DUTY_SCALE / 2U)

/* A 1 us sampling lead over the 20 kHz update event; one timer count is
   5.952 ns, so this is a 168-count advance of the ADC trigger. */
#define TIM1_PWM_ADC_TRIGGER_LEAD_NS 1000U

/* --------------------------------------------------------------------------*/
/* Register-level helpers                                                    */
/* --------------------------------------------------------------------------*/
/* The gate driver is fed by complementary outputs: TIM1_CH1..3 carry the
   high-side drive (datasheet table "MCU to gate driver internal connection":
   PE9 -> CH1 -> INH1, PE8 -> CH1N -> INL1) and the GD inverts the main signal
   internally for the low side.

   Duty of the main output in PWM mode 1, center-aligned, is the share of the
   counter period for which the counter is below the compare value. Two facts
   decide every constant below:

     1. The compare value is proportional to the duty, NOT its complement. A
        larger compare value means a longer on-time of the high side.
     2. A compare value above ARR never switches: the counter cannot pass ARR, so
        the output stays permanently active instead of extending the duty range.
        The usable compare band is therefore 0..ARR, which is half the nominal
        counter period of 2 x (ARR + 1) counts. Scaling the duty by the period
        instead of by the band is what makes the upper half of the scale
        saturate: any duty above 50 % would pin the output at nearly 100 %.

   Measured against the counter sequence itself (center-aligned, active while
   CNT < CCR, one carrier period = 2 x ARR = 8398 counter clocks):
     CCR    0 ->   0.0000 % high side     duty   0 ->   0.0000 %
     CCR 2100 ->  50.0000 %               duty 500 ->  50.0000 %
     CCR 2520 ->  60.0024 %               duty 600 ->  60.0024 %
     CCR 4199 ->  99.9881 %               duty1000 ->  99.9881 %
     CCR 4200 -> 100.0000 % (also saturates)
   The residual error of the integer divide is at most 0.0119 % of a period, so
   the 50 % point of the scale is exactly 50 % of the carrier. */

/* Compare value that means "high side permanently on", i.e. the top of the
   usable compare band. ARR + 1 would be one count past it and is clamped. */
#define TIM1_PWM_COMPARE_FULL_SCALE (TIM1_PWM_COUNTER_PERIOD + 1UL)

/* Compare value of the 50 % point, and the value the idle pattern loads. */
#define TIM1_PWM_COMPARE_NEUTRAL    TIM1_PWM_DUTY_TO_COMPARE(TIM1_PWM_DUTY_NEUTRAL)

/**
  * @brief  Convert a 0..TIM1_PWM_DUTY_SCALE high-side duty into the compare
  *         value the channels are loaded with.
  *
  * Scaled by the compare band: duty 0 loads 0 (high side never on) and duty
  * TIM1_PWM_DUTY_SCALE loads ARR + 1, which the point of use clamps to ARR
  * (high side permanently on). The +1 in the band makes the 50 % point land on
  * exactly half of ARR + 1 instead of one count below it.
  *
  * Do not invert this mapping, and do not scale it by TIM1_PWM_COUNTS_PER_PERIOD:
  * the compare value is proportional to the duty, and anything above ARR stops
  * switching instead of extending the range.
  */
#define TIM1_PWM_DUTY_TO_COMPARE(duty)                                        \
  (((uint32_t)(duty) * TIM1_PWM_COMPARE_FULL_SCALE) / TIM1_PWM_DUTY_SCALE)

/* --------------------------------------------------------------------------*/
/* Exported variables                                                        */
/* --------------------------------------------------------------------------*/
/* Non-static so the handle shows up in the debugger Watch window and so the
   interrupt layer can address it. Only tim1_pwm.c mutates it. */
extern TIM_HandleTypeDef htim1;

/* --------------------------------------------------------------------------*/
/* Exported functions                                                        */
/* --------------------------------------------------------------------------*/

/**
  * @brief  Configure and start TIM1: center-aligned 20 kHz complementary PWM on
  *         channels 1..3 with 500 ns dead time, plus channel 4 as an ADC
  *         trigger placed TIM1_PWM_ADC_TRIGGER_LEAD_NS ahead of the update
  *         event.
  *
  * The counter, the channel outputs and the CC4 interrupt all start here, but
  * MOE is left cleared so no gate signal reaches the power stage. Enabling the
  * power stage is a separate, explicit step: Tim1Pwm_EnableOutputs().
  *
  * Safe to call once, from main() after the system clock is configured.
  */
void     MX_TIM1_Init(void);

/**
  * @brief  GPIO configuration of the six gate outputs, generated in the MSP
  *         layer. Called at the end of MX_TIM1_Init().
  */
void     HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
  * @brief  Set one phase duty (0..TIM1_PWM_DUTY_SCALE, 500 = 50 %). Values
  *         above the scale are clamped. The compare register is written in a
  *         single store, so an ISR may call this while the timer is running.
  * @param  channel One of TIM_CHANNEL_1 / _2 / _3.
  * @retval HAL_OK, or HAL_ERROR for a channel that is not a PWM phase.
  */
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty);

/**
  * @brief  Read back the last duty written for a phase.
  * @retval Duty in 0..TIM1_PWM_DUTY_SCALE, or 0 for an invalid channel.
  */
uint16_t Tim1Pwm_GetPhaseDuty(uint32_t channel);

/**
  * @brief  Load the three phases with the 50 % bootstrap refresh pattern and
  *         clear the commanded duties. Call it before enabling the outputs.
  */
void     Tim1Pwm_SetNeutral(void);

/**
  * @brief  Release MOE so the six gate signals reach the power stage.
  * @note   The caller must have verified that the driver reports no fault and
  *         is ready to switch; see DrvMonitor_GetState().
  */
void     Tim1Pwm_EnableOutputs(void);

/**
  * @brief  Clear MOE: every gate output is driven to its inactive level at
  *         once, without waiting for a software fault path.
  */
void     Tim1Pwm_EmergencyStop(void);

/**
  * @brief  Report the MOE state.
  * @retval 1 when the gate signals are driving the power stage, 0 otherwise.
  */
uint8_t  Tim1Pwm_IsOutputEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_PWM_H__ */
