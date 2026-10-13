/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm.c
  * @brief   TIM1 three-phase complementary PWM initialization and duty control.
  *
  *          Configuration summary
  *            counter   : center-aligned mode 1, ARR = 4199, RCR = 1
  *                        -> 168 MHz / (2 x 4200) = 20 kHz carrier,
  *                           one update event per carrier period
  *            channels 1..3 : PWM mode 1, direct (INH) + complementary (INL),
  *                        dead time 500 ns from TIM_BDTR
  *            channel 4     : PWM mode 2 on OC4REF, "sample soon after the
  *                        update event" trigger for the ADC/current loop
  *            MOE           : left cleared, the gate outputs stay inactive until
  *                        Tim1Pwm_EnableOutputs() is called explicitly
  *
  *          Half-bridge idle pattern
  *            While the power stage is disabled the phases are loaded with the
  *            neutral compare value, which produces a steady 50 % duty cycle.
  *            The low-side switch stays on long enough in every period to
  *            refresh the gate-driver bootstrap capacitor, and both switches of
  *            a half bridge are never on at the same time.
  *
  *            The compare value is proportional to the high-side duty, and ARR
  *            is the top of the usable compare band: a compare value above ARR
  *            stops the channel from switching instead of extending the duty, so
  *            both clamped endpoints mean "permanently on" for the high side and
  *            "permanently on" for the low side respectively. A phase that must
  *            not be driven must NOT be loaded with the top of the band (that
  *            keeps the high side on for the whole period); use the neutral
  *            pattern, which keeps both switches moving.
  *            Tim1Pwm_SetPhaseDuty() is the only entry point and it maps the
  *            duty in the right direction.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim1_pwm.h"
#include "stm32g4xx_ll_tim.h"

/* USER CODE BEGIN 0 */

/* The dead time is encoded, not a plain count: TIM_BDTR_DTG values 0x00..0x7F
   are in units of tDTS, so the value must stay inside that first range. */
#define TIM1_PWM_DTG_IN_RANGE(dtg)  ((dtg) <= 0x7FU)

/* Compile-time check: the 16-bit compare registers must be able to hold a whole
   PWM period, otherwise the duty mapping would truncate. */
#define TIM1_PWM_PERIOD_FITS_U16    (TIM1_PWM_COUNTS_PER_PERIOD <= 0xFFFFUL)

#if !TIM1_PWM_PERIOD_FITS_U16
#error "TIM1_PWM_FREQUENCY_HZ is too low for a 16-bit compare register"
#endif

static void Tim1Pwm_WriteCompare(uint32_t channel, uint32_t compare);
static uint32_t Tim1Pwm_NsToCounts(uint32_t nanoseconds);
static int32_t  Tim1Pwm_FindPhase(uint32_t channel);

/* Commanded duty per phase, kept in step with the loaded compare values so
   Tim1Pwm_SetNeutral() and the verification can restore a known state without
   reading the timer back. Written from thread and interrupt context, hence
   volatile. */
static volatile uint16_t s_phase_duty[3] = {TIM1_PWM_DUTY_NEUTRAL,
                                            TIM1_PWM_DUTY_NEUTRAL,
                                            TIM1_PWM_DUTY_NEUTRAL};

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* --------------------------------------------------------------------------*/
/* Internal helpers                                                          */
/* --------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/**
  * @brief  Map a PWM channel identifier onto the phase table index.
  * @retval 0..2 for TIM_CHANNEL_1..3, -1 for any other channel.
  */
static int32_t Tim1Pwm_FindPhase(uint32_t channel)
{
  int32_t phase = -1;

  switch (channel)
  {
    case TIM_CHANNEL_1:
      phase = 0;
      break;

    case TIM_CHANNEL_2:
      phase = 1;
      break;

    case TIM_CHANNEL_3:
      phase = 2;
      break;

    default:
      phase = -1;
      break;
  }

  return phase;
}

/**
  * @brief  Load one compare register.
  *
  * The duty mapping is scaled by ARR + 1, so duty 0 can land one count past the
  * compare band. Clamping to ARR keeps that case defined and monotonic: ARR is
  * the largest value that still switches, and it means "high side permanently
  * on". A value left above ARR would instead stop the channel from switching
  * altogether and pin the output - the exact failure this clamp prevents.
  */
static void Tim1Pwm_WriteCompare(uint32_t channel, uint32_t compare)
{
  if (compare > TIM1_PWM_COUNTER_PERIOD)
  {
    compare = TIM1_PWM_COUNTER_PERIOD;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint16_t)compare);
}

/**
  * @brief  Convert a time in nanoseconds into timer counts, rounded down.
  */
static uint32_t Tim1Pwm_NsToCounts(uint32_t nanoseconds)
{
  return ((nanoseconds * (TIM1_PWM_TIMER_CLOCK_HZ / 1000000UL)) / 1000UL);
}

/* USER CODE END 1 */

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sOcConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  uint32_t compare;
  uint32_t phase;

  /* USER CODE BEGIN TIM1_Init 0 */

  /* The dead time constant is combined from the define here so that its range
     is part of the compiled code rather than of a comment. */
  const uint32_t deadtime = (uint32_t)TIM1_PWM_DEADTIME_TDTS;

  if (TIM1_PWM_DTG_IN_RANGE(deadtime) == 0UL)
  {
    /* Not encodable in the tDTS-count range of TIM_BDTR_DTG. */
    Error_Handler();
  }

  /* Timing assumes the 168 MHz PLL configuration of SystemClock_Config(), where
     APB2 is not divided, so the TIM1 kernel clock equals PCLK2. A different
     clock tree would silently shift both the carrier and the dead time, so it is
     checked instead of assumed. */
  if (HAL_RCC_GetPCLK2Freq() != TIM1_PWM_TIMER_CLOCK_HZ)
  {
    Error_Handler();
  }

  /* USER CODE END TIM1_Init 0 */

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = TIM1_PWM_COUNTER_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Break and dead time. Both break inputs stay disabled: BKIN and BKIN2 are
     the READY and NFAULT pins of the gate driver (see the datasheet table "MCU
     to gate driver internal connection"), and on this MCU PE15 carries no TIM1
     alternate function at all. Those two pins are read by drv_monitor instead,
     and the driver's own VDS / thermal protections stay in charge. Enabling a
     break input needs an explicit decision about which pin to give up.

     OSSR/OSSI off: while MOE is cleared the six pins are left to the gate
     driver's own input pull resistors instead of being driven to a level whose
     meaning for the GD cannot be read off this datasheet. The driver is designed
     to tolerate that input configuration. Turning OSSR on would instead drive
     OC/OCN to their idle state, which is a deliberate brake/coast choice and
     belongs with the power-stage enable logic, not here. */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = deadtime;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0U;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0U;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Channels 1..3: PWM mode 1, output active high, complementary output also
     active high. A higher compare value therefore means a longer direct
     on-time, which is the high-side switch of the half bridge. */
  sOcConfig.OCMode = TIM_OCMODE_PWM1;
  sOcConfig.Pulse = 0U;
  sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOcConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sOcConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sOcConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sOcConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  for (phase = 0U; phase < 3U; phase++)
  {
    /* Pulse is overwritten right below; a throw-away value keeps the loop
       identical for every channel. */
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sOcConfig,
                                  (uint32_t)(TIM_CHANNEL_1 << phase)) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Channel 4 only feeds the ADC trigger, its pin is not routed to the gate
     driver. PWM mode 2 with the compare just below the peak asserts OC4REF
     TIM1_PWM_ADC_TRIGGER_LEAD_NS before the update event is generated, which is
     the sampling instant the current loop will want. */
  sOcConfig.OCMode = TIM_OCMODE_PWM2;
  sOcConfig.Pulse = (uint32_t)TIM1_PWM_COUNTER_PERIOD -
                    Tim1Pwm_NsToCounts(TIM1_PWM_ADC_TRIGGER_LEAD_NS);
  sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOcConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sOcConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  /* The peripheral clock and the NVIC priority were set up by HAL_TIM_PWM_MspInit(),
     which HAL_TIM_PWM_Init() called above. Only the pin configuration runs at
     this point, exactly where CubeMX generates HAL_TIM_MspPostInit(). */
  HAL_TIM_MspPostInit(&htim1);

  /* Start counting with the update interrupt live: the current loop will need
     the period boundary. The six outputs are still gated by MOE. */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable the direct and the complementary output of every phase. */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                 LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                 LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

  /* CC4 fires at the sampling instant; its handler is the entry point of the
     current loop. */
  LL_TIM_EnableIT_CC4(TIM1);

  /* Neutral 50 % pattern: both switches of each half bridge are driven, the
     bootstrap capacitor of the gate driver keeps its charge and the motor sees
     no net voltage. */
  for (phase = 0U; phase < 3U; phase++)
  {
    s_phase_duty[phase] = TIM1_PWM_DUTY_NEUTRAL;
    compare = TIM1_PWM_DUTY_TO_COMPARE(TIM1_PWM_DUTY_NEUTRAL);
    Tim1Pwm_WriteCompare((uint32_t)(TIM_CHANNEL_1 << phase), compare);
  }

  /* MOE stays cleared: the power stage is enabled by Tim1Pwm_EnableOutputs()
     once the driver has been reported healthy. */
}

/* USER CODE BEGIN 2 */

/**
  * @brief  Set one phase high-side duty (0..TIM1_PWM_DUTY_SCALE, 500 = 50 %).
  * @note   Both endpoints are saturated. Duty 1000 leaves the low side off for
  *         the whole period, and duty 0 leaves it on for the whole period; either
  *         one stops the bootstrap capacitor of that half bridge from being
  *         refreshed, and duty 0 additionally brakes the motor through the low
  *         side. Keep a running phase inside the scale, use the neutral pattern
  *         as the idle state, and never use either endpoint as an idle value.
  */
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty)
{
  int32_t phase = Tim1Pwm_FindPhase(channel);

  if (phase < 0)
  {
    return HAL_ERROR;
  }

  if (duty > TIM1_PWM_DUTY_SCALE)
  {
    duty = TIM1_PWM_DUTY_SCALE;
  }

  /* The register is written first: it is the state the power stage actually
     sees. The shadow value is bookkeeping only, so a read-back racing with a
     concurrent update can never misreport what the pins are doing. */
  Tim1Pwm_WriteCompare(channel, TIM1_PWM_DUTY_TO_COMPARE(duty));
  s_phase_duty[phase] = duty;

  return HAL_OK;
}

/**
  * @brief  Read back the last duty written for a phase.
  */
uint16_t Tim1Pwm_GetPhaseDuty(uint32_t channel)
{
  int32_t  phase = Tim1Pwm_FindPhase(channel);
  uint16_t duty  = 0U;

  if (phase >= 0)
  {
    duty = s_phase_duty[phase];
  }

  return duty;
}

/**
  * @brief  Load the phase-aligned neutral pattern (all three phases at 50 %).
  *
  * This is the safe idle state of the power stage: no net voltage on the motor,
  * and both switches of every half bridge keep moving, so the low side still
  * conducts half of every carrier period and the gate-driver bootstrap capacitor
  * stays charged.
  */
void Tim1Pwm_SetNeutral(void)
{
  uint32_t phase;

  for (phase = 0U; phase < 3U; phase++)
  {
    s_phase_duty[phase] = TIM1_PWM_DUTY_NEUTRAL;
  }

  /* All three phases receive the same compare value, so they switch together
     and the motor sees a zero vector. */
  for (phase = 0U; phase < 3U; phase++)
  {
    Tim1Pwm_WriteCompare((uint32_t)(TIM_CHANNEL_1 << phase),
                         TIM1_PWM_COMPARE_NEUTRAL);
  }
}

/**
  * @brief  Release MOE so the six gate signals reach the power stage.
  */
void Tim1Pwm_EnableOutputs(void)
{
  __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
  * @brief  Clear MOE: every gate output goes to its inactive level immediately.
  * @note   The unconditional form is required: __HAL_TIM_MOE_DISABLE() only acts
  *         when the CCxE and CCxNE bits of all channels are already cleared, so
  *         on a running stage it would silently do nothing.
  */
void Tim1Pwm_EmergencyStop(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

/**
  * @brief  Report the MOE state.
  */
uint8_t Tim1Pwm_IsOutputEnabled(void)
{
  return ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
}

/**
  * @brief  Sampling-instant callback for TIM1 channel 4.
  *
  * This is where the current loop will run: the compare of CC4 is placed
  * TIM1_PWM_ADC_TRIGGER_LEAD_NS before the update event, and the ADC is meant to
  * be started from OC4REF. The name is fixed by the HAL, so the period-boundary
  * callback stays HAL_TIM_PeriodElapsedCallback, which the driver monitor
  * already owns for the TIM6 heartbeat.
  *
  * Deliberately still a stub: nothing consumes the sampling instant until the
  * ADC is configured, and an empty ISR body at the carrier rate costs only the
  * interrupt entry. HAL_TIM_IRQHandler() calls the callback unconditionally, so
  * no definition is needed here yet - add one when the current loop needs it.
  */

/* USER CODE END 2 */
