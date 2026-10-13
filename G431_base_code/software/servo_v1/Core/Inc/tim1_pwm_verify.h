/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.h
  * @brief   Small, self-contained check that the TIM1 gate signals really reach
  *          the three half bridges.
  *
  * Why a separate file: MX_TIM1_Init() can be verified from the debugger alone
  * (read TIM1->CR1 / BDTR / CCMR1), but "the power stage switches" is only
  * visible on the pins. This module drives a pattern that is trivial to judge on
  * a two- or four-channel oscilloscope and that is safe to run with the motor
  * already connected.
  *
  *     part 1  the three half bridges are driven with three distinct duties
  *             (60 % / 40 % / 50 %), so three probes show three clearly
  *             different pulse widths, pairwise in phase
  *     part 2  a slow 1 Hz open-loop V/f ramp on top of the 50 % neutral point,
  *             so a probe on one phase shows a pulse width sweeping up and down
  *             while the motor twitches gently around its rest position
  *
  * Both parts keep every phase at or below 60 % duty and keep the three average
  * phase voltages balanced, so the low side conducts a substantial part of every
  * carrier period and the gate-driver bootstrap capacitor stays charged. The
  * pattern never asks for duty 1000, which would leave a low side off for the
  * whole period.
  *
  * Run it only after the driver monitor reports DRV_STATE_READY and the bus
  * voltage is up; the caller owns that decision, see Tim1PwmVerify_Start().
  *
  * ---------------------------------------------------------------------------
  * Runbook
  * ---------------------------------------------------------------------------
  *   1. Bring-up order. Power the board and check DrvMonitor_GetState() ==
  *      DRV_STATE_READY in the debugger before anything else. Do not connect a
  *      load that cannot survive the motor's holding current yet.
  *   2. If the DC link is not current limited, run part 1 with the motor
  *      disconnected first: three probes on PE9, PE11, PE13 must show three
  *      different pulse widths, all in phase, all at 20 kHz.
  *   3. Probe one phase and its complementary pin (PE9 vs PE8, PE11 vs PE10,
  *      PE13 vs PE12) and zoom to one switching edge: the two traces must never
  *      be high together, and the gap must be about 500 ns.
  *   4. Reconnect the motor and repeat. The shaft should twitch at 1 Hz and stay
  *      near its rest position; a shaft that runs away means a wiring or
  *      direction problem, not a timer problem.
  *   5. Read the carrier back if a probe is not available. With part 1 running:
  *      TIM1->ARR = 4199, TIM1->BDTR has DTG = 84 and MOE = 1, TIM1->CR1 reads
  *      CMS = center-aligned 1 with CEN = 1, and TIM1->CCR1/2/3 = 2520 / 1680 /
  *      2100, i.e. 60 % / 40 % / 50 % high-side. The three values must be
  *      distinct and ordered 1 > 3 > 2. Equal or saturated compares, or an order
  *      that runs the other way, mean the duty mapping is wrong - the pins would
  *      then sit near a permanent level instead of switching.
  *   6. The sequence ends by itself after about 7.5 s and restores the neutral
  *      pattern. Tim1PwmVerify_IsRunning() going back to 0 is the pass signal;
  *      TIM1->BDTR losing MOE is the watchdog firing, which means something
  *      starved the main loop. The watchdog is checked once per TIM6 tick (1 Hz),
  *      so it can overshoot the deadline by up to one second.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM1_PWM_VERIFY_H__
#define __TIM1_PWM_VERIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* --------------------------------------------------------------------------*/
/* Tunables                                                                  */
/* --------------------------------------------------------------------------*/
/* Part 1: deliberately unbalanced duties, chosen far enough apart to be obvious
   on a scope and close enough to 50 % to avoid a large DC current. */
#define TIM1_PWM_VERIFY_STATIC_A_DUTY   (TIM1_PWM_DUTY_NEUTRAL + 100U)  /* 60 % */
#define TIM1_PWM_VERIFY_STATIC_B_DUTY   (TIM1_PWM_DUTY_NEUTRAL - 100U)  /* 40 % */
#define TIM1_PWM_VERIFY_STATIC_C_DUTY   (TIM1_PWM_DUTY_NEUTRAL)         /* 50 % */

/* Part 2: the open-loop V/f step. 1 Hz keeps the shaft movement slow enough to
   watch, and the modulation depth caps the applied voltage at about a tenth of
   the DC link, which a connected motor survives without a current limit. */
#define TIM1_PWM_VERIFY_ELEC_HZ         1U
#define TIM1_PWM_VERIFY_MOD_DEPTH       100U
#define TIM1_PWM_VERIFY_TABLE_SECTORS   6U
#define TIM1_PWM_VERIFY_TABLE_STEPS     (TIM1_PWM_VERIFY_TABLE_SECTORS * 2U)

/* --------------------------------------------------------------------------*/
/* Exported functions                                                        */
/* --------------------------------------------------------------------------*/

/**
  * @brief  Prepare and start the verification, then arm a bounded watchdog.
  *
  * Loads the part 1 pattern, starts the millisecond time base of the sequence
  * and enables the gate outputs. The watchdog guarantees that the pattern stops
  * even if nothing ever calls Tim1PwmVerify_Task() again.
  *
  * @param  timeout_ms  Maximum duration of the run. When it expires the outputs
  *                     are stopped from inside the tick interrupt, so this is a
  *                     real bound and not just a hint. 0 is rejected.
  * @retval HAL_OK when the outputs were enabled, HAL_ERROR when the vector
  *         table is empty or the outputs stayed gated by MOE.
  */
HAL_StatusTypeDef Tim1PwmVerify_Start(uint32_t timeout_ms);

/**
  * @brief  One step of the verification sequence. Call it from the main loop.
  *
  * Non-blocking: it compares HAL_GetTick() against the next step instant and
  * returns immediately in between. Also stops the sequence when the outputs are
  * no longer enabled, which is how a cleared MOE ends the test.
  *
  * @param  timeout_ms  Same value that was passed to Tim1PwmVerify_Start().
  * @retval HAL_OK while the sequence is running or has finished normally,
  *         HAL_ERROR when it was aborted by the timeout or by a gated output.
  */
HAL_StatusTypeDef Tim1PwmVerify_Task(uint32_t timeout_ms);

/**
  * @brief  Stop the verification and restore the neutral idle pattern.
  *         Safe to call at any time, including after a normal finish.
  */
void              Tim1PwmVerify_Stop(void);

/**
  * @brief  Report whether the sequence is still producing output patterns.
  * @retval 1 while running, 0 once it finished, aborted or was stopped.
  */
uint8_t           Tim1PwmVerify_IsRunning(void);

/**
  * @brief  Watchdog hook for the TIM6 tick interrupt.
  *
  * Enforces the deadline given to Tim1PwmVerify_Start() and clears MOE when it
  * expires, so the pattern stops even if the main loop dies. Call it from the
  * TIM6 vector next to the driver-monitor heartbeat; it costs one comparison
  * while the watchdog is disarmed.
  */
void              Tim1PwmVerify_TickIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_PWM_VERIFY_H__ */
