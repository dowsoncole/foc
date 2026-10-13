/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.c
  * @brief   TIM1 gate-signal verification: a scope pattern followed by a slow
  *          open-loop V/f ramp on a motor that is already connected.
  *
  * Execution model
  *   Tim1PwmVerify_Start()  arms the sequence and a bounded watchdog
  *   Tim1PwmVerify_Task()   advances it, one non-blocking step per call
  *   Tim1PwmVerify_TickIsr() (TIM6, 1 Hz) gates the outputs off and raises a flag
  *                          when the deadline expires; the main loop then does
  *                          the bookkeeping. The 1 Hz granularity means the
  *                          deadline can overshoot by up to one tick, which is
  *                          negligible against the 10 s bound of a 8 s sequence.
  *   HAL_TIM_PeriodElapsedCallback() (TIM1, one update per carrier period)
  *                          is deliberately left to the driver monitor until the
  *                          current loop does something with that event
  *
  * The verification uses the same public duty interface the current loop will
  * use (Tim1Pwm_SetPhaseDuty), so a passing test also exercises the mapping from
  * "high-side duty" to the compare register that the carrier actually loads.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim1_pwm_verify.h"
#include "tim1_pwm.h"

/* USER CODE BEGIN 0 */

/* Dwell time of each part 1 pattern and of each ramp step. */
#define TIM1_PWM_VERIFY_PART1_MS   2000U
#define TIM1_PWM_VERIFY_RAMP_MS    500U

/* Six-sector base vectors of a two-level inverter, in the (1, -1, 0) style of
   the classic table. A positive entry raises the phase duty above the neutral
   point, a negative one lowers it, and every row sums to zero, so the three
   average phase voltages always add up to zero. */
static const int8_t s_sector_table[TIM1_PWM_VERIFY_TABLE_STEPS][3] =
{
  {  1, -1,  0 },
  {  1,  0, -1 },
  {  0,  1, -1 },
  { -1,  1,  0 },
  { -1,  0,  1 },
  {  0, -1,  1 },
  /* The second half runs the same vectors backwards, which turns the staircase
     waveform into a symmetric up-down sweep instead of a sawtooth. */
  {  0, -1,  1 },
  { -1,  0,  1 },
  { -1,  1,  0 },
  {  0,  1, -1 },
  {  1,  0, -1 },
  {  1, -1,  0 },
};

/* --------------------------------------------------------------------------*/
/* Module state                                                              */
/* --------------------------------------------------------------------------*/
/* Sequence state. Owned by the main loop (Tim1PwmVerify_Start/Task/Stop) and by
   nothing else: the tick ISR only raises the two flags below, so this state
   needs no volatile qualifier and no critical section. */
static uint8_t  s_running;
static uint8_t  s_failed;
static uint8_t  s_part;
static uint8_t  s_step;
static uint32_t s_part_start_ms;
static uint32_t s_step_start_ms;
static uint32_t s_start_ms;

/* Deadline handed to the TIM6 tick interrupt, 0 when unarmed, and the flag it
   raises when the deadline expires. These two are the only variables shared with
   interrupt context, hence volatile. */
static volatile uint32_t s_watchdog_deadline_ms;
static volatile uint8_t  s_watchdog_fired;

/* USER CODE END 0 */

/* --------------------------------------------------------------------------*/
/* Internal helpers                                                          */
/* --------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/**
  * @brief  Drive the three half bridges with the part 1 pattern.
  *
  * Three distinct duties at the same phase, so the differential voltage between
  * any two phases stays at one fifth of the DC link at most: three probes show
  * three different pulse widths, and the motor can neither accelerate nor hold a
  * large current. Every phase stays between 40 % and 60 % duty, so each low side
  * still conducts most of every carrier period and the bootstrap capacitors stay
  * charged. On a low-resistance or small winding this is the pattern that draws
  * the most current - lower the two offsets in tim1_pwm_verify.h if the DC link
  * is current limited or the winding is unusually stiff.
  */
static void Tim1PwmVerify_ApplyStaticPattern(void)
{
  (void)Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_1, TIM1_PWM_VERIFY_STATIC_A_DUTY);
  (void)Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_2, TIM1_PWM_VERIFY_STATIC_B_DUTY);
  (void)Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_3, TIM1_PWM_VERIFY_STATIC_C_DUTY);
}

/**
  * @brief  Apply one sector vector as a duty offset around the neutral point.
  *
  * The offset is deliberately small: the demonstration is about proving that the
  * three gate signals move, not about producing torque. At +-TIM1_PWM_VERIFY_MOD_DEPTH
  * every phase stays between 40 % and 60 % duty, so no low side is ever off for a
  * whole carrier period.
  */
static void Tim1PwmVerify_ApplySector(const int8_t *sector)
{
  uint32_t phase;

  for (phase = 0U; phase < 3U; phase++)
  {
    int32_t duty = (int32_t)TIM1_PWM_DUTY_NEUTRAL +
                   ((int32_t)sector[phase] * (int32_t)TIM1_PWM_VERIFY_MOD_DEPTH);

    /* Every entry of the table keeps this inside a narrow band around 50 %, but
       clamping makes the range explicit rather than implied. */
    if (duty < 0)
    {
      duty = 0;
    }
    else if (duty > (int32_t)TIM1_PWM_DUTY_SCALE)
    {
      duty = (int32_t)TIM1_PWM_DUTY_SCALE;
    }

    (void)Tim1Pwm_SetPhaseDuty((uint32_t)(TIM_CHANNEL_1 << phase),
                               (uint16_t)duty);
  }
}

/**
  * @brief  End the sequence, disarm the watchdog and restore the idle pattern.
  */
static void Tim1PwmVerify_Finish(void)
{
  s_running = 0U;

  /* Disarm first: the deadline must never outlive the sequence. */
  s_watchdog_deadline_ms = 0U;
  s_watchdog_fired       = 0U;

  Tim1Pwm_SetNeutral();
}

/* USER CODE END 1 */

/* --------------------------------------------------------------------------*/
/* Public API                                                                */
/* --------------------------------------------------------------------------*/

/* USER CODE BEGIN 2 */

/**
  * @brief  Prepare and start the verification, then arm a bounded watchdog.
  *
  * Arm the watchdog only after the outputs are known to be enabled, so a start
  * that fails halfway cannot leave a deadline behind for a later tick to fire.
  */
HAL_StatusTypeDef Tim1PwmVerify_Start(uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    return HAL_ERROR;
  }

  /* The sequence is only as long as the table it walks. The watchdog flag is
     cleared before the deadline is armed below, never after: an expiry in
     between would otherwise be thrown away. */
  s_part           = 0U;
  s_step           = 0U;
  s_failed         = 0U;
  s_watchdog_fired = 0U;
  s_start_ms       = HAL_GetTick();
  s_part_start_ms  = s_start_ms;
  s_step_start_ms  = s_start_ms;

  Tim1PwmVerify_ApplyStaticPattern();

  /* The pattern above is already loaded; releasing MOE decides whether it is
     visible. Everything downstream is measured against that moment. */
  Tim1Pwm_EnableOutputs();

  if (Tim1Pwm_IsOutputEnabled() == 0U)
  {
    Tim1Pwm_SetNeutral();
    return HAL_ERROR;
  }

  s_running = 1U;

  /* Arm the watchdog last, so a start that fails cannot leave a deadline
     behind that a later tick would fire. */
  s_watchdog_deadline_ms = s_start_ms + timeout_ms;

  return HAL_OK;
}

/**
  * @brief  One non-blocking step of the verification sequence.
  */
HAL_StatusTypeDef Tim1PwmVerify_Task(uint32_t timeout_ms)
{
  uint32_t now;

  if (s_running == 0U)
  {
    return (s_failed != 0U) ? HAL_ERROR : HAL_OK;
  }

  /* Either the watchdog cleared MOE (its flag is set) or something else gated
     the outputs. Both mean the power stage is no longer being driven, and the
     sequence must not try to resume it. */
  if ((s_watchdog_fired != 0U) || (Tim1Pwm_IsOutputEnabled() == 0U))
  {
    s_failed = 1U;
    Tim1PwmVerify_Finish();
    return HAL_ERROR;
  }

  now = HAL_GetTick();

  /* Belt and braces: the tick interrupt owns the hard deadline, this check
     keeps the sequence bounded even if that interrupt is not running. */
  if ((now - s_start_ms) >= timeout_ms)
  {
    s_failed = 1U;
    Tim1PwmVerify_Finish();
    return HAL_ERROR;
  }

  if (s_part == 0U)
  {
    /* Part 1: hold the static pattern long enough to probe all six outputs. */
    if ((now - s_part_start_ms) >= TIM1_PWM_VERIFY_PART1_MS)
    {
      s_part          = 1U;
      s_step          = 0U;
      s_step_start_ms = now;
      Tim1PwmVerify_ApplySector(s_sector_table[0]);
    }

    return HAL_OK;
  }

  /* Part 2: one sector per step at TIM1_PWM_VERIFY_ELEC_HZ. */
  if ((now - s_step_start_ms) >= TIM1_PWM_VERIFY_RAMP_MS)
  {
    s_step_start_ms = now;
    s_step++;

    if (s_step >= TIM1_PWM_VERIFY_TABLE_STEPS)
    {
      Tim1PwmVerify_Finish();
      return HAL_OK;
    }

    Tim1PwmVerify_ApplySector(s_sector_table[s_step]);
  }

  return HAL_OK;
}

/**
  * @brief  Stop the verification and restore the neutral idle pattern.
  */
void Tim1PwmVerify_Stop(void)
{
  Tim1PwmVerify_Finish();
}

/**
  * @brief  Report whether the sequence is still producing output patterns.
  */
uint8_t Tim1PwmVerify_IsRunning(void)
{
  return s_running;
}

/**
  * @brief  TIM6 tick hook: enforce the verification deadline.
  *
  * The driver monitor owns the TIM6 tick itself; this runs alongside it and does
  * nothing at all while the watchdog is disarmed, which is the normal state of
  * the application.
  *
  * Deliberately minimal: it disarms the deadline, gates the outputs off and
  * raises a flag for the main loop. All sequence bookkeeping stays in
  * Tim1PwmVerify_Task(), so no non-volatile module state is ever touched from
  * interrupt context.
  */
void Tim1PwmVerify_TickIsr(void)
{
  uint32_t deadline = s_watchdog_deadline_ms;

  if ((deadline != 0U) &&
      ((int32_t)(HAL_GetTick() - deadline) >= 0))
  {
    /* Disarm before acting: no later tick may act again. */
    s_watchdog_deadline_ms = 0U;
    s_watchdog_fired       = 1U;

    /* Cut the drive immediately. The neutral pattern and the sequence state are
       restored by the main loop on its next Tim1PwmVerify_Task() call. */
    Tim1Pwm_EmergencyStop();
  }
}

/* USER CODE END 2 */
