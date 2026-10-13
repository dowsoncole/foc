/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.c
  * @brief   Fixed single-phase MOSFET gate-drive measurement waveform.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim1_pwm_verify.h"
#include "tim1_pwm.h"

static HAL_StatusTypeDef Tim1PwmVerify_GetChannel(mos_driver_phase_t phase,
                                                   uint32_t *channel)
{
  if (channel == NULL)
  {
    return HAL_ERROR;
  }

  switch (phase)
  {
    case MOS_DRIVER_PHASE_U:
      *channel = TIM_CHANNEL_1;
      break;

    case MOS_DRIVER_PHASE_V:
      *channel = TIM_CHANNEL_2;
      break;

    case MOS_DRIVER_PHASE_W:
      *channel = TIM_CHANNEL_3;
      break;

    default:
      return HAL_ERROR;
  }

  return HAL_OK;
}

/* Duty 0 leaves the high side off and its complementary low side on. */
static HAL_StatusTypeDef Tim1PwmVerify_ApplyPattern(
    const mos_driver_measurement_t *measurement)
{
  uint32_t selected_channel;

  if ((measurement == NULL) ||
      (measurement->duty == 0U) ||
      (measurement->duty >= TIM1_PWM_DUTY_SCALE) ||
      (Tim1PwmVerify_GetChannel(measurement->phase, &selected_channel) != HAL_OK))
  {
    return HAL_ERROR;
  }

  if ((Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_1, 0U) != HAL_OK) ||
      (Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_2, 0U) != HAL_OK) ||
      (Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_3, 0U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return Tim1Pwm_SetPhaseDuty(selected_channel, measurement->duty);
}

static void Tim1PwmVerify_GateOff(measurement_manager_t *manager,
                                  measurement_state_t state)
{
  Tim1Pwm_EmergencyStop();
  Tim1Pwm_SetNeutral();
  manager->config.mos_driver.deadline_ms = 0U;
  manager->config.mos_driver.moe_applied = 0U;
  manager->state = state;
}

HAL_StatusTypeDef Tim1PwmVerify_Start(measurement_manager_t *manager)
{
  mos_driver_measurement_t *measurement;

  if ((manager == NULL) ||
      (manager->module != MEASUREMENT_MODULE_MOS_DRIVER))
  {
    return HAL_ERROR;
  }

  measurement = &manager->config.mos_driver;

  if (Tim1PwmVerify_ApplyPattern(measurement) != HAL_OK)
  {
    manager->state = MEASUREMENT_STATE_FAILED;
    return HAL_ERROR;
  }

  if (measurement->moe_enable != 0U)
  {
    Tim1Pwm_EnableOutputs();
  }
  else
  {
    Tim1Pwm_EmergencyStop();
  }

  if ((measurement->moe_enable != 0U) &&
      (Tim1Pwm_IsOutputEnabled() == 0U))
  {
    Tim1Pwm_SetNeutral();
    manager->state = MEASUREMENT_STATE_FAILED;
    return HAL_ERROR;
  }

  measurement->moe_applied = measurement->moe_enable;
  measurement->phase_applied = measurement->phase;
  measurement->duty_applied = measurement->duty;
  /* Continuous output is the default; MOE is changed only by configuration or
     an explicit Tim1PwmVerify_Stop() call. */
  measurement->deadline_ms = 0U;
  manager->state = MEASUREMENT_STATE_RUNNING;

  return HAL_OK;
}

HAL_StatusTypeDef Tim1PwmVerify_Task(measurement_manager_t *manager)
{
  if ((manager == NULL) ||
      (manager->module != MEASUREMENT_MODULE_MOS_DRIVER))
  {
    return HAL_ERROR;
  }

  if (manager->state == MEASUREMENT_STATE_TIMEOUT)
  {
    return HAL_ERROR;
  }

  if (manager->state != MEASUREMENT_STATE_RUNNING)
  {
    return (manager->state == MEASUREMENT_STATE_FAILED) ? HAL_ERROR : HAL_OK;
  }

  /* Apply phase/duty edits made through the unified measurement object while
     the debugger or a host command is changing the active test. */
  if ((manager->config.mos_driver.phase !=
       manager->config.mos_driver.phase_applied) ||
      (manager->config.mos_driver.duty !=
       manager->config.mos_driver.duty_applied))
  {
    if (Tim1PwmVerify_ApplyPattern(&manager->config.mos_driver) != HAL_OK)
    {
      manager->state = MEASUREMENT_STATE_FAILED;
      return HAL_ERROR;
    }

    manager->config.mos_driver.phase_applied =
        manager->config.mos_driver.phase;
    manager->config.mos_driver.duty_applied =
        manager->config.mos_driver.duty;
  }

  /* MOE is explicit configuration and can be changed while running. */
  if (manager->config.mos_driver.moe_enable !=
      manager->config.mos_driver.moe_applied)
  {
    if (manager->config.mos_driver.moe_enable != 0U)
    {
      Tim1Pwm_EnableOutputs();
    }
    else
    {
      Tim1Pwm_EmergencyStop();
    }
    manager->config.mos_driver.moe_applied =
        manager->config.mos_driver.moe_enable;
  }
  else if ((manager->config.mos_driver.moe_enable != 0U) &&
           (Tim1Pwm_IsOutputEnabled() == 0U))
  {
    /* A hardware fault or another owner cleared MOE; do not re-enable it
       automatically just because the requested configuration is still 1. */
    manager->state = MEASUREMENT_STATE_FAILED;
    return HAL_ERROR;
  }

  return HAL_OK;
}

void Tim1PwmVerify_Stop(measurement_manager_t *manager)
{
  if (manager != NULL)
  {
    Tim1PwmVerify_GateOff(manager, MEASUREMENT_STATE_IDLE);
  }
}

uint8_t Tim1PwmVerify_IsRunning(const measurement_manager_t *manager)
{
  return ((manager != NULL) &&
          (manager->state == MEASUREMENT_STATE_RUNNING)) ? 1U : 0U;
}

void Tim1PwmVerify_TickIsr(measurement_manager_t *manager)
{
  if ((manager == NULL) ||
      (manager->module != MEASUREMENT_MODULE_MOS_DRIVER) ||
      (manager->state != MEASUREMENT_STATE_RUNNING))
  {
    return;
  }

  /* Continuous measurement deliberately has no automatic MOE shutdown. */
}

/* USER CODE END 2 */
