/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.h
  * @brief   Single-phase MOSFET gate-drive measurement waveform.
  *
  * The phase selected in g_measurement_manager receives a fixed 10 % high-side
  * PWM waveform by default. The other two phases are commanded to 0 % high-side
  * duty, which keeps their low-side MOSFETs on and clamps their phase terminals
  * to PGND. Select U, V or W by changing config.mos_driver.phase.
  *
  * Run with the motor disconnected and a current-limited DC bus. This waveform
  * deliberately applies a non-zero line-to-line voltage and is intended for
  * probing one half bridge, not for rotating the motor.
  *
  * ---------------------------------------------------------------------------
  * Runbook
  * ---------------------------------------------------------------------------
  *   1. Bring-up order. Power the board and check DrvMonitor_GetState() ==
  *      DRV_STATE_READY in the debugger before anything else. Do not connect a
  *      load that cannot survive the motor's holding current yet.
  *   2. Probe the selected phase gate-to-source voltage and its U/V/W terminal.
  *      The phase terminal must switch at 20 kHz with the configured duty.
  *   3. Check that the other two phase terminals stay close to PGND.
  *   4. For the default U/10 % configuration, TIM1->CCR1 is 420 and CCR2/3 are
  *      0. TIM1->ARR is 4199, BDTR.DTG is 84 and BDTR.MOE is 1 while running.
  *   5. The waveform remains active until the MOE field is cleared or
  *      Tim1PwmVerify_Stop() is called.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM1_PWM_VERIFY_H__
#define __TIM1_PWM_VERIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "measurement_manager.h"

/* --------------------------------------------------------------------------*/
/* Exported functions                                                        */
/* --------------------------------------------------------------------------*/

/**
  * @brief  Prepare and start the verification, then arm a bounded watchdog.
  *
  * Loads the selected-phase pattern and arms the millisecond deadline
  * and enables the gate outputs. The watchdog guarantees that the pattern stops
  * even if nothing ever calls Tim1PwmVerify_Task() again.
  *
  * @param  manager     Unified measurement object. Its MOS driver configuration
  *                     selects the phase, duty and duration.
  * @retval HAL_OK when the outputs were enabled, HAL_ERROR when the vector
  *         table is empty or the outputs stayed gated by MOE.
  */
HAL_StatusTypeDef Tim1PwmVerify_Start(measurement_manager_t *manager);

/**
  * @brief  Service the active measurement. Call it from the main loop.
  *
  * Non-blocking: it compares HAL_GetTick() against the next step instant and
  * returns immediately in between. Also stops the sequence when the outputs are
  * no longer enabled, which is how a cleared MOE ends the test.
  *
  * @param  manager     Unified measurement object passed to Start().
  * @retval HAL_OK while the sequence is running or has finished normally,
  *         HAL_ERROR when it was aborted by the timeout or by a gated output.
  */
HAL_StatusTypeDef Tim1PwmVerify_Task(measurement_manager_t *manager);

/**
  * @brief  Stop the verification and gate off the outputs.
  *         Safe to call at any time, including after a normal finish.
  */
void              Tim1PwmVerify_Stop(measurement_manager_t *manager);

/**
  * @brief  Report whether the sequence is still producing output patterns.
  * @retval 1 while running, 0 once it finished, aborted or was stopped.
  */
uint8_t           Tim1PwmVerify_IsRunning(const measurement_manager_t *manager);

/**
  * @brief  TIM6 hook retained for the measurement manager.
  *
  * Continuous measurement deliberately does not clear MOE here. Call it from
  * the TIM6 vector next to the driver-monitor heartbeat so future measurement
  * modules can add time-based handling without changing the interrupt wiring.
  */
void              Tim1PwmVerify_TickIsr(measurement_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_PWM_VERIFY_H__ */
