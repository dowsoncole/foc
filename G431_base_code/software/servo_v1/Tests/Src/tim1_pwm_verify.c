/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.c
  * @brief   固定的单相 MOSFET 栅极驱动测量波形（L5 测试/验证层）。
  *          本层只使用 L4 三相驱动层的业务接口，自身不碰任何寄存器。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim1_pwm_verify.h"
#include "tim1_pwm.h"

/**
  * @brief  把被测相枚举映射成 TIM1 通道号。
  * @retval HAL_OK；channel 为 NULL 或 phase 非法时返回 HAL_ERROR。
  */
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

/* 占空比 0 意味着高侧关断，而其互补的低侧导通。 */
/**
  * @brief  装载测量波形：三相先清零，再给选中相设定目标占空比。
  *         未被选中的两相保持 0 %，低侧导通并把相端钳到 PGND。
  * @retval HAL_OK；参数非法或写入失败时返回 HAL_ERROR。
  */
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

/**
  * @brief  关断栅极输出并回到中性点，同时把测量状态置为指定值。
  */
static void Tim1PwmVerify_GateOff(measurement_manager_t *manager,
                                  measurement_state_t state)
{
  Tim1Pwm_EmergencyStop();
  Tim1Pwm_SetNeutral();
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

  /* 自举电容预充电延时（DS13630 §5.1, §5.2.4）。
   *
   * MX_TIM1_Init() 之后 TIM1 已在运行，且 CCRx=0（三相低侧全部导通）。
   * 低侧 MOSFET 把 OUTx 钳到 PGND，于是自举电容可以通过内部二极管充电：
   *   VCC → [内部二极管] → BOOTx → [Cboot] → OUTx ≈ GND
   *
   * STSPIN32G4 内部的 VCC buck 变换器从上电到稳定需要约 5-10 ms，
   * 自举电容充电还要再加约 10 ms。配置的 startup_delay_ms 在使能 MOE
   * 之前留出了余量。
   *
   * 没有这个延时的话，MOE 首次置起时高侧栅极驱动电压不足，MOSFET 会工作
   * 在线性区、Rds(on) 很大，从而触发 VDS 监测保护（DS13630 §5.3）。
   */
  HAL_Delay(measurement->startup_delay_ms);

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
  /* 默认是连续输出；MOE 只会因配置变更或显式调用 Tim1PwmVerify_Stop()
     而改变。 */
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

  if (manager->state != MEASUREMENT_STATE_RUNNING)
  {
    return (manager->state == MEASUREMENT_STATE_FAILED) ? HAL_ERROR : HAL_OK;
  }

  /* 把通过统一测量对象做出的 phase/duty 修改应用下去——调试器或上位机命令
     可能在测量运行途中改变当前测试。 */
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

  /* MOE 是显式配置项，可以在运行途中改变。 */
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
    /* MOE 是被硬件故障或另一个持有者清掉的；不能仅仅因为请求的配置仍是 1
       就自动把它重新使能。 */
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

/* USER CODE END 2 */
