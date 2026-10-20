/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c3_bus.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim6_heartbeat.h"
#include "tim1_pwm.h"
#include "adc1_current_sampling.h"
#include "current_sampling.h"
#include "stspin32g4.h"
#include "drv_monitor.h"
#include "foc_interface.h"
#include "foc_control_manager.h"
#include "foc_current_tuning.h"
#include "current_loop_test.h"
#include "servo_manager.h"
#include "waveform_generator.h"
#include "waveform_uart1_port.h"
#include "encoder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* OPAMP/ADC 已启动后，给模拟信号链和驱动器状态一个独立稳定时间；
   这段时间 TIM1/CH4/ADC 继续运行，但 MOE 仍保持关闭。 */
#define CURRENT_SAMPLING_MOE_GUARD_MS  20U

#define FOC_TWO_PI                            6.2831853072f

/* Bootstrap precharge is followed by a neutral-vector soak.  During this
   window ADC/algorithm interrupts keep running, but precharge_in_progress
   prevents them from overwriting the startup CCR vector. */
#define FOC_TEST_NEUTRAL_SETTLE_MS             20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ADC JEOS 回调使用的唯一 FOC 控制器实例。PID 参数、限幅和 PWM 周期均由
   该实例的 algorithm.parameters/timing 管理，避免在中断中散落全局变量。 */
foc_controller_t g_foc_controller;
servo_manager_t g_servo_manager;
UART_HandleTypeDef huart1;
/* Commissioning mailbox: write design, then apply_request=1 while output OFF.
   R/L deliberately have no fabricated motor defaults. status: 1=OK, 2=busy,
   3=invalid design. Last accepted gains survive control mode switches. */
struct
{
  foc_current_design_t design;
  foc_current_gains_t gains;
  volatile uint32_t apply_request, status;
  uint8_t valid;
} g_current_tuning = {
  .design = { .dc_bus_v = 24.0f, .bandwidth_hz = 200.0f,
    .sample_period_s = 0.00005f, .delay_s = 0.000075f,
    .phase_margin_deg = 60.0f, .integral_limit_pu = 0.02f }
};
/* Zero-initialized capture stays in ZI, avoiding a 14 KB flash copy image. */
current_loop_test_t g_current_loop_test;
/* 全局模式控制对象：统一管理电压开环、电流测试环、正常模式及公共运行状态。 */
foc_control_manager_t g_global_mode_control =
{
  .requested_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP,
  .active_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP,
  .switch_count = 0UL,
  .voltage_open_loop =
  {
    .d_voltage_v = 0.20f, .q_voltage_v = 0.0f,
    .dc_bus_voltage_v = 24.0f, .electrical_frequency_hz = 2.0f,
    .voltage_ramp_rate_v_per_s = 0.5f
  },
  .current_open_loop =
  {
    .d_current_reference_a = 0.10f, .q_current_reference_a = 0.0f,
    .dc_bus_voltage_v = 24.0f, .electrical_frequency_hz = 0.0f
  },
  .normal_loop =
  {
    .d_current_reference_a = 0.0f, .q_current_reference_a = 0.0f,
    .electrical_angle_rad = 0.0f,
    .angle_valid = 0U, .reference_valid = 0U
  },
  .runtime = { .fault_flags = FOC_FAULT_NONE }
};

static uint8_t MainFoc_IsPowerStageSafe(void *context)
{
  (void)context;
  return DrvMonitor_IsPowerStageSafeFast();
}

static void MainFoc_ReadFeedback(void *context, foc_input_t *input)
{
  (void)context;
  if (input == NULL)
  {
    return;
  }

  /* 每个 ADC 周期重新判定，禁止沿用上一周期的有效标志。 */
  input->current_valid = 0U;
  input->angle_valid = 0U;
  input->reference_valid = 0U;

  {
    float phase_u_a;
    float phase_v_a;
    float phase_w_a;
    CurrentSampling_ReadFeedback_A(&phase_u_a, &phase_v_a, &phase_w_a);
    input->phase_a = phase_u_a;
    input->phase_b = phase_v_a;
    input->current_valid =
        (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_DONE) ?
        1U : 0U;
  }

  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    input->d_reference =
        g_global_mode_control.current_open_loop.d_current_reference_a;
    input->q_reference =
        g_global_mode_control.current_open_loop.q_current_reference_a;
    input->electrical_angle_rad =
        g_global_mode_control.current_open_loop.electrical_angle_rad;
    input->reference_valid = 1U;
    input->angle_valid = 1U;
    if (g_current_loop_test.state == CURRENT_TEST_ARMED ||
        g_current_loop_test.state == CURRENT_TEST_RUNNING)
    {
      input->reference_valid = CurrentTest_Reference(&g_current_loop_test,
          g_foc_controller.algorithm.parameters.sample_period_s,
          &input->d_reference, &input->q_reference);
    }
  }
  else if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_NORMAL)
  {
    float encoder_electrical_angle_rad;
    input->d_reference = g_global_mode_control.normal_loop.d_current_reference_a;
    input->q_reference = g_global_mode_control.normal_loop.q_current_reference_a;
    input->reference_valid = g_global_mode_control.normal_loop.reference_valid;
    if (Encoder_ReadElectricalAngleForFoc(&encoder_electrical_angle_rad) != 0U)
    {
      input->electrical_angle_rad = encoder_electrical_angle_rad;
      input->angle_valid = 1U;
      /* 保留在模式对象中作为调试遥测，不再把它当作反馈数据源。 */
      g_global_mode_control.normal_loop.electrical_angle_rad =
          encoder_electrical_angle_rad;
      g_global_mode_control.normal_loop.angle_valid = 1U;
    }
    else
    {
      input->angle_valid = 0U;
      g_global_mode_control.normal_loop.angle_valid = 0U;
    }
  }
}

static uint8_t MainFoc_WriteVoltage(void *context, const foc_output_t *output)
{
  (void)context;
  if (output != NULL)
  {
    return (Tim1Pwm_SetPhaseVoltage(output->phase_a_duty,
                                     output->phase_b_duty,
                                     output->phase_c_duty) == HAL_OK) ? 1U : 0U;
  }
  return 0U;
}

static void MainFoc_DisableOutput(void *context)
{
  (void)context;
  Tim1Pwm_EmergencyStop();
}

static uint8_t MainServo_IsCalibrationDone(void *context)
{
  (void)context;
  return (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_DONE) ?
      1U : 0U;
}

static uint8_t MainServo_IsCalibrationError(void *context)
{
  (void)context;
  return (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_ERROR) ?
      1U : 0U;
}

static uint8_t MainServo_IsOutputEnabled(void *context)
{
  (void)context;
  return Tim1Pwm_IsOutputEnabled();
}

static void MainServo_PrepareOutput(void *context)
{
  (void)context;
  Tim1Pwm_SetNeutral();
}

static void MainServo_EnableOutput(void *context)
{
  uint32_t interrupt_mask = __get_PRIMASK();

  (void)context;

  /* Servo Manager 是正常打开 MOE 的唯一入口。临界区防止在
     “检查 feedback_ready”和“置 MOE”之间被 ADC ISR 打断。
     临界区内只读内存/引脚并写一个寄存器，不做通讯或等待。 */
  __disable_irq();
  if ((g_foc_controller.running != 0U) &&
      (g_foc_controller.feedback_ready != 0U) &&
      (MainFoc_IsPowerStageSafe(NULL) != 0U) &&
      (Tim1Pwm_IsOutputEnabled() == 0U))
  {
    Tim1Pwm_EnableOutputs();
  }
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

static void MainServo_NotifyFault(void *context,
                                  servo_fault_mask_t active_faults,
                                  servo_fault_mask_t latched_faults)
{
  (void)context;
  (void)active_faults;
  (void)latched_faults;
  /* 故障已保存在 g_servo_manager 中。未来 CAN/UART 协议层在此
     发布事件，禁止在 ADC ISR 中直接发送或打印。 */
}

static void MainFoc_ConfigureModeParameters(foc_control_mode_t mode)
{
  foc_parameters_t *parameters = &g_foc_controller.algorithm.parameters;

  parameters->d_controller.integral = 0.0f;
  parameters->q_controller.integral = 0.0f;
  if (mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    parameters->voltage_limit = 0.02f;
    parameters->modulation.mode = FOC_MODULATION_LINEAR_ONLY;
    parameters->current_limit = 0.5f;
    parameters->d_controller.kp = 0.05f;
    parameters->d_controller.ki = 2.0f;
    parameters->d_controller.integral_min = -0.02f;
    parameters->d_controller.integral_max = 0.02f;
    parameters->q_controller = parameters->d_controller;
    if (g_current_tuning.valid)
    {
      parameters->d_controller = g_current_tuning.gains.d;
      parameters->q_controller = g_current_tuning.gains.q;
    }
  }
  else
  {
    parameters->voltage_limit = FOC_MODULATION_SIX_STEP_MAX;
    parameters->modulation.mode = FOC_MODULATION_OVERMODULATION;
    parameters->current_limit = 10.0f;
    parameters->d_controller.kp = 0.1f;
    parameters->d_controller.ki = 5.0f;
    parameters->d_controller.integral_min = -0.95f;
    parameters->d_controller.integral_max = 0.95f;
    parameters->q_controller = parameters->d_controller;
  }
  FOC_AlgorithmReset(&g_foc_controller.algorithm);
}

/* Background policy only; all excitation and capture math lives in Tests.
   Never apply a multi-field PI update while the ADC ISR is using the PI. */
static void MainCurrentTest_Service(void)
{
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  uint8_t off = !r->output_request && !r->output_armed &&
      !r->precharge_in_progress && !g_foc_controller.running &&
      !Tim1Pwm_IsOutputEnabled();
  if (g_current_tuning.apply_request)
  {
    foc_current_gains_t gains;
    g_current_tuning.apply_request = 0;
    if (!off || g_global_mode_control.active_mode != FOC_CONTROL_MODE_CURRENT_OPEN_LOOP ||
        g_current_loop_test.state == CURRENT_TEST_ARMED)
      g_current_tuning.status = 2;
    else
    {
      /* Timing comes from firmware, not an independently editable design value. */
      g_current_tuning.design.sample_period_s = g_foc_controller.timing.pwm_period_s;
      if (FOC_CurrentDesign(&g_current_tuning.design, &gains) &&
          g_current_tuning.design.integral_limit_pu <= g_foc_controller.algorithm.parameters.voltage_limit)
      {
        g_current_tuning.gains = gains;
        g_current_tuning.valid = 1;
        g_global_mode_control.current_open_loop.dc_bus_voltage_v = g_current_tuning.design.dc_bus_v;
        MainFoc_ConfigureModeParameters(FOC_CONTROL_MODE_CURRENT_OPEN_LOOP);
        g_current_tuning.status = 1;
      }
      else g_current_tuning.status = 3;
    }
  }
  if (g_current_loop_test.command)
  {
    uint32_t command = g_current_loop_test.command;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    g_current_loop_test.command = 0;
    if (command == 2)
    {
      r->output_request = r->output_armed = 0;
      FOC_ControllerStop(&g_foc_controller);
      CurrentTest_Abort(&g_current_loop_test, 0);
    }
    else if (command == 3 && off)
      g_current_loop_test.state = CURRENT_TEST_IDLE;
    else if (command == 1 && off &&
        g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP &&
        g_global_mode_control.current_open_loop.electrical_frequency_hz == 0.0f)
    {
      if (CurrentTest_Arm(&g_current_loop_test,
          g_foc_controller.algorithm.parameters.current_limit,
          g_foc_controller.timing.pwm_period_s))
      {
        g_current_loop_test.parameters_at_arm = g_foc_controller.algorithm.parameters;
        g_current_loop_test.dc_bus_at_arm_v = g_global_mode_control.current_open_loop.dc_bus_voltage_v;
      }
    }
    else CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
    if (!mask) __enable_irq();
  }
  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP &&
      (g_current_loop_test.state == CURRENT_TEST_DONE ||
       g_current_loop_test.state == CURRENT_TEST_ABORTED))
    r->output_request = 0;
}

/* 模式切换只在主循环执行，并强制经过 MOE=0、控制器停止、PI 清零的边界。
   requested_mode 可由协议或调试器动态写入，禁止直接改 active_mode。 */
static void MainFoc_ServiceModeSwitch(void)
{
  foc_control_mode_t requested = g_global_mode_control.requested_mode;
  foc_control_runtime_t *runtime = &g_global_mode_control.runtime;
  uint32_t interrupt_mask;

  if ((requested >= FOC_CONTROL_MODE_COUNT) ||
      (requested == g_global_mode_control.active_mode))
  {
    if (requested >= FOC_CONTROL_MODE_COUNT)
    {
      g_global_mode_control.requested_mode = g_global_mode_control.active_mode;
    }
    return;
  }

  runtime->output_request = 0U;
  runtime->output_armed = 0U;
  runtime->precharge_in_progress = 1U;
  ServoManager_RequestOff(&g_servo_manager);
  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_NORMAL)
  {
    ServoManager_Service(&g_servo_manager);
  }
  FOC_ControllerStop(&g_foc_controller);
  Tim1Pwm_EmergencyStop();

  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  FOC_ControllerClearFaults(&g_foc_controller);
  MainFoc_ConfigureModeParameters(requested);
  if (g_current_loop_test.state == CURRENT_TEST_ARMED ||
      g_current_loop_test.state == CURRENT_TEST_RUNNING)
    CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
  g_global_mode_control.voltage_open_loop.applied_d_voltage_v = 0.0f;
  g_global_mode_control.voltage_open_loop.applied_q_voltage_v = 0.0f;
  g_global_mode_control.voltage_open_loop.voltage_ramp_complete = 0U;
  g_global_mode_control.current_open_loop.electrical_angle_rad = 0.0f;
  g_global_mode_control.current_open_loop.cycle_count = 0UL;
  runtime->running = 0U;
  runtime->output_enabled = 0U;
  runtime->fault_flags = FOC_FAULT_NONE;
  g_global_mode_control.active_mode = requested;
  g_global_mode_control.switch_count++;
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
  runtime->precharge_in_progress = 0U;
}

/* 把控制层的完整同步快照交给波形业务层。这里不选择打印通道；预设模式和
   自定义通道均由 g_waveform_generator 统一解释，避免 main.c 散落打印策略。 */
static void MainWaveform_Publish(const foc_controller_t *controller,
                                 float phase_u_a,
                                 float phase_v_a,
                                 float phase_w_a)
{
  waveform_source_snapshot_t source = {0};
  float dc_bus_voltage_v = 0.0f;

  if (controller == NULL) return;
  source.phase_u_current_a = phase_u_a;
  source.phase_v_current_a = phase_v_a;
  source.phase_w_current_a = phase_w_a;
  source.d_current_feedback_a = controller->algorithm.output.d_current;
  source.q_current_feedback_a = controller->algorithm.output.q_current;
  source.d_voltage_output_pu = controller->algorithm.output.d_voltage;
  source.q_voltage_output_pu = controller->algorithm.output.q_voltage;
  source.phase_u_duty = controller->algorithm.output.phase_a_duty;
  source.phase_v_duty = controller->algorithm.output.phase_b_duty;
  source.phase_w_duty = controller->algorithm.output.phase_c_duty;

  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP)
  {
    dc_bus_voltage_v = g_global_mode_control.voltage_open_loop.dc_bus_voltage_v;
    source.electrical_angle_rad =
        g_global_mode_control.voltage_open_loop.electrical_angle_rad;
  }
  else if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    source.d_current_reference_a =
        controller->input.d_reference;
    source.q_current_reference_a =
        controller->input.q_reference;
    source.electrical_angle_rad =
        g_global_mode_control.current_open_loop.electrical_angle_rad;
    dc_bus_voltage_v = g_global_mode_control.current_open_loop.dc_bus_voltage_v;
  }
  else
  {
    source.d_current_reference_a =
        g_global_mode_control.normal_loop.d_current_reference_a;
    source.q_current_reference_a =
        g_global_mode_control.normal_loop.q_current_reference_a;
    source.electrical_angle_rad =
        g_global_mode_control.normal_loop.electrical_angle_rad;
  }
  source.dc_bus_voltage_v = dc_bus_voltage_v;
  WaveformGenerator_PublishFromIsr(g_current_sampling.sample_sequence, &source);
}

static void MainFoc_OnAdcConversion(void *context)
{
  foc_controller_t *controller = (foc_controller_t *)context;
  foc_control_runtime_t *runtime = &g_global_mode_control.runtime;
  float phase_u_a;
  float phase_v_a;
  float phase_w_a;

  CurrentSampling_ReadFeedback_A(&phase_u_a, &phase_v_a, &phase_w_a);
  if (controller == NULL)
  {
    return;
  }

  if (g_global_mode_control.active_mode != FOC_CONTROL_MODE_NORMAL)
  {
    uint32_t pin_levels;
    pin_levels = GD_READY_GPIO_Port->IDR;
    runtime->power_stage_pin_levels = pin_levels;
    runtime->power_stage_safe =
        (((pin_levels & GD_READY_Pin) != 0U) &&
         ((pin_levels & GD_NFAULT_Pin) != 0U)) ? 1U : 0U;
    if (runtime->power_stage_safe == 0U)
    {
      if (runtime->unsafe_sample_count == 0UL)
      {
        runtime->first_unsafe_pin_levels = pin_levels;
        runtime->fault_status_capture_pending = 1U;
      }
      runtime->unsafe_sample_count++;
      if ((pin_levels & GD_READY_Pin) == 0U)
      {
        runtime->ready_low_count++;
      }
      if ((pin_levels & GD_NFAULT_Pin) == 0U)
      {
        runtime->nfault_low_count++;
      }
    }
  }

  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP)
  {
    /* 纯算法测试不能被功率级状态冻结：即使 READY/nFAULT 不允许打开 MOE，
       仍持续计算角度、SVPWM 和 CCR。硬件安全门只控制引脚是否真正输出。 */
    if (FOC_AlgorithmVoltageStep(&controller->algorithm,
          &g_global_mode_control.voltage_open_loop,
          controller->timing.pwm_period_s) == 0U)
    {
      runtime->fault_flags |= FOC_FAULT_ALGORITHM;
      runtime->running = 0U;
      runtime->output_enabled = 0U;
      Tim1Pwm_EmergencyStop();
      return;
    }

    MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);

    runtime->running = 1U;
    if ((runtime->output_armed == 0U) ||
        (runtime->precharge_in_progress != 0U))
    {
      runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
      return;
    }

    if (MainFoc_WriteVoltage(NULL, &controller->algorithm.output) == 0U)
    {
      runtime->fault_flags |= FOC_FAULT_PWM_WRITE;
      runtime->running = 0U;
      runtime->output_enabled = 0U;
      Tim1Pwm_EmergencyStop();
      return;
    }

    if (runtime->power_stage_safe == 0U)
    {
      runtime->fault_flags |= FOC_FAULT_POWER_STAGE;
      runtime->output_enabled = 0U;
      Tim1Pwm_EmergencyStop();
      return;
    }

    if ((runtime->fault_flags == FOC_FAULT_NONE) &&
        (Tim1Pwm_IsOutputEnabled() == 0U))
    {
      Tim1Pwm_EnableOutputs();
    }
    runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
    return;
  }

  controller->algorithm.parameters.sample_period_s =
      Tim1Pwm_OnCurrentSample(g_current_sampling.sample_sequence);
  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    /* 电流测试环：采样真实相电流，Park 后进行 dq PI；角度由测试对象推进。
       预充电阶段不允许 PI 积分，避免 MOE 打开前积分器积累到限幅。 */
    if ((runtime->output_armed == 0U) ||
        (runtime->precharge_in_progress != 0U))
    {
      runtime->running = controller->running;
      runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
      MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);
      return;
    }

    FOC_ControllerStep(controller);
    CurrentTest_Observe(&g_current_loop_test, &controller->algorithm.output,
        controller->algorithm.parameters.sample_period_s,
        g_current_sampling.sample_sequence, Tim1Pwm_IsOutputEnabled());
    if (!controller->running && (g_current_loop_test.state == CURRENT_TEST_ARMED ||
        g_current_loop_test.state == CURRENT_TEST_RUNNING))
      CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
    if (g_current_loop_test.state == CURRENT_TEST_DONE ||
        g_current_loop_test.state == CURRENT_TEST_ABORTED)
    {
      /* Capture is frozen before stopping (Stop clears algorithm telemetry). */
      runtime->output_request = runtime->output_armed = 0;
      FOC_ControllerStop(controller);
    }
    controller->timing.last_sample_sequence = g_current_sampling.sample_sequence;
    runtime->fault_flags = controller->fault_flags;
    runtime->running = controller->running;
    runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
    MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);
    if ((controller->running == 0U) ||
        (controller->algorithm.output.valid == 0U))
    {
      runtime->output_armed = 0U;
      return;
    }

    g_global_mode_control.current_open_loop.measured_d_current_a =
        controller->algorithm.output.d_current;
    g_global_mode_control.current_open_loop.measured_q_current_a =
        controller->algorithm.output.q_current;
    g_global_mode_control.current_open_loop.output_d_voltage_v =
        controller->algorithm.output.d_voltage *
        g_global_mode_control.current_open_loop.dc_bus_voltage_v;
    g_global_mode_control.current_open_loop.output_q_voltage_v =
        controller->algorithm.output.q_voltage *
        g_global_mode_control.current_open_loop.dc_bus_voltage_v;
    g_global_mode_control.current_open_loop.electrical_angle_rad += FOC_TWO_PI *
        g_global_mode_control.current_open_loop.electrical_frequency_hz *
        controller->algorithm.parameters.sample_period_s;
    if (g_global_mode_control.current_open_loop.electrical_angle_rad >= FOC_TWO_PI)
    {
      g_global_mode_control.current_open_loop.electrical_angle_rad -= FOC_TWO_PI;
    }
    else if (g_global_mode_control.current_open_loop.electrical_angle_rad < 0.0f)
    {
      g_global_mode_control.current_open_loop.electrical_angle_rad += FOC_TWO_PI;
    }
    g_global_mode_control.current_open_loop.cycle_count++;
    return;
  }

  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_NORMAL)
  {
    FOC_ControllerStep(controller);
    controller->timing.last_sample_sequence = g_current_sampling.sample_sequence;
    MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t neutral_settle_start;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  if (MX_I2C3_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_TIM6_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_TIM1_Init() != HAL_OK)
  {
    Error_Handler();
  }
  {
    waveform_transport_t waveform_transport;
    const waveform_generator_config_t waveform_config =
    {
      .source_rate_hz = 20000UL,
      .output_rate_hz = 1000UL,
      .initial_mode = WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ
    };
    if (WaveformUart1Port_Init(&waveform_transport, &huart1) != HAL_OK)
    {
      Error_Handler();
    }
    if (WaveformGenerator_Init(&waveform_transport, &waveform_config) == 0U)
    {
      Error_Handler();
    }
  }
  /* 上电先保持硬件关断。MOE 只在零点校准完成、驱动状态安全且 FOC
     即将启动时释放，避免 ADC 偏置采集阶段把电压施加到电机。 */
  Tim1Pwm_EmergencyStop();
  /* USER CODE BEGIN 2 */
  /* ------------------------------------------------------------------------
   * Layer 0 上电初始化：在任何实时任务开始之前一次性跑完
   * ------------------------------------------------------------------------
   * DrvMonitor_Init() 会清除上电锁存故障（必须做：不写 CLEAR，芯片一直处于
   * 不动作状态），并启动 1 s 心跳。此处允许阻塞重试，因为此时还没有任何
   * 实时任务在运行，阻塞不会破坏 PWM 或采样时序。
   */
  if (DrvMonitor_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* ------------------------------------------------------------------------
   * 电流采样外设配置（模块 F：Modules/Current_Sampling/）
   * ------------------------------------------------------------------------
   * 顺序有意义：两个运放必须先于 ADC 配好。ADC1 采的是运放输出脚
   * （PA2 = ADC1_IN3，PB1 = ADC1_IN12），运放没起来时那两个节点上没有有效
   * 信号，先校准 ADC 只会把噪声当成基线记下来。
   *
   * ADC 使用 TIM1_CH4 硬件触发。MX_ADC1_Init() 只配置和校准，
   * CurrentSampling_Start() 负责按“运放先启动、ADC 后启动”的顺序接通信号链。
   * ADC 运行时不需要在 TIM1_CC ISR 里手工发起转换；CC4 的 OC4REF 会自动触发
   * 注入序列，JDR1/JDR2 由硬件更新。
   *
   * 相别已确认：rank 1 = U 相 = foc_input_t.phase_a，rank 2 = V 相 =
   * foc_input_t.phase_b，W 相由 -(ia + ib) 推出。
   * CH4 提前量已改为注入序列耗时的一半（5.714 µs），使序列以更新事件为中心。
   *
    * 零电流偏置由异步校准状态机完成，FOC 输入统一换算成物理安培。 */
  if (MX_OPAMP1_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_OPAMP3_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_ADC1_Init(0U) != HAL_OK)
  {
    Error_Handler();
  }

  {
    foc_parameters_t foc_parameters =
    {
      .sample_period_s = 1.0f / 20000.0f,
      .pwm_period_s = 1.0f / 20000.0f,
      /* 首次闭环试验限制为 2 % 母线电压和 0.5 A，避免极性或参数错误时
         PI 直接把桥臂推到高占空比。 */
      .voltage_limit = 0.02f,
      .modulation = { .mode = FOC_MODULATION_LINEAR_ONLY,
                      .duty_min = 0.0f, .duty_max = 1.0f },
      .current_limit = 0.5f,
      .d_controller = { .kp = 0.05f, .ki = 2.0f,
                        .integral_min = -0.02f, .integral_max = 0.02f },
      .q_controller = { .kp = 0.05f, .ki = 2.0f,
                        .integral_min = -0.02f, .integral_max = 0.02f }
    };
    const foc_port_t foc_port =
    {
      .is_power_stage_safe = MainFoc_IsPowerStageSafe,
      .read_feedback = MainFoc_ReadFeedback,
      .write_voltage = MainFoc_WriteVoltage,
      .disable_output = MainFoc_DisableOutput,
      .context = NULL
    };
    const servo_manager_port_t servo_port =
    {
      .is_calibration_done = MainServo_IsCalibrationDone,
      .is_calibration_error = MainServo_IsCalibrationError,
      .is_power_stage_safe = MainFoc_IsPowerStageSafe,
      .is_output_enabled = MainServo_IsOutputEnabled,
      .prepare_output = MainServo_PrepareOutput,
      .enable_output = MainServo_EnableOutput,
      .disable_output = MainFoc_DisableOutput,
      .notify_fault = MainServo_NotifyFault,
      .context = NULL
    };
    /* 保留 16 倍过采样；每周期根据下一组 duty 同步规划 CCR4。
       完整序列窗口仍需保留；不能仅因触发点可移动就开放全母线六步。 */
    if (Tim1Pwm_ConfigureCurrentSampling(ADC1_CS_SEQUENCE_CONV_NS,
          &foc_parameters.modulation.duty_min,
          &foc_parameters.modulation.duty_max) != HAL_OK)
    {
      Error_Handler();
    }
    FOC_ControllerInit(&g_foc_controller, &foc_parameters, &foc_port);
    g_current_loop_test.requested = (current_test_config_t) {
      .kind = CURRENT_TEST_STEP, .axis = 0, .amplitude_a = 0.05f,
      .settle_s = 0.005f, .step_delay_s = 0.004f, .average_count = 1
    };
    ServoManager_Init(&g_servo_manager, &g_foc_controller, &servo_port);
    CurrentSampling_SetConversionCallback(MainFoc_OnAdcConversion,
                                           &g_foc_controller);
  }

  if (CurrentSampling_Start() != HAL_OK)
  {
    Error_Handler();
  }

  if (MainFoc_IsPowerStageSafe(NULL) == 0U)
  {
    Error_Handler();
  }
  /* 所有模式都常驻固件，启动时统一完成零点校准，之后可安全动态切换。 */
  HAL_Delay(CURRENT_SAMPLING_MOE_GUARD_MS);
  CurrentSampling_RequestCalibration();

  /* MT6835 正式采集：TIM1 周期边界启动 SPI1 DMA，DMA 中断优先级低于
     ADC/FOC。电角度在上层完成方向、极对数和零位配置前保持无效。 */
  if (Encoder_Init() == 0U)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    foc_control_runtime_t *runtime = &g_global_mode_control.runtime;
    CurrentSampling_Service();
    MainFoc_ServiceModeSwitch();
    MainCurrentTest_Service();

    if ((g_global_mode_control.active_mode != FOC_CONTROL_MODE_NORMAL) &&
        (runtime->output_request != 0U) &&
        (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_ERROR))
    {
      runtime->output_request = 0U;
      runtime->fault_flags |= FOC_FAULT_CURRENT_INVALID;
    }
    if ((g_global_mode_control.active_mode != FOC_CONTROL_MODE_NORMAL) &&
        (runtime->output_request == 0U) &&
        ((runtime->output_armed != 0U) ||
         (Tim1Pwm_IsOutputEnabled() != 0U)))
    {
      if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
      {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        runtime->output_armed = 0U;
        FOC_ControllerStop(&g_foc_controller);
        if (g_current_loop_test.state == CURRENT_TEST_RUNNING ||
            g_current_loop_test.state == CURRENT_TEST_ARMED)
          CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_MOE_OFF);
        if (!mask) __enable_irq();
        runtime->running = 0U;
        g_global_mode_control.current_open_loop.output_d_voltage_v = 0.0f;
        g_global_mode_control.current_open_loop.output_q_voltage_v = 0.0f;
      }
      else
      {
        Tim1Pwm_EmergencyStop();
        g_global_mode_control.voltage_open_loop.applied_d_voltage_v = 0.0f;
        g_global_mode_control.voltage_open_loop.applied_q_voltage_v = 0.0f;
        g_global_mode_control.voltage_open_loop.voltage_ramp_complete = 0U;
      }
      runtime->output_armed = 0U;
      runtime->output_enabled = 0U;
    }
    else if ((g_global_mode_control.active_mode != FOC_CONTROL_MODE_NORMAL) &&
             (runtime->output_request != 0U) &&
             (runtime->output_armed == 0U) &&
             (runtime->precharge_in_progress == 0U) &&
             (runtime->fault_flags == FOC_FAULT_NONE) &&
             ((g_global_mode_control.active_mode ==
               FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP) ||
              (CurrentSampling_GetCalibrationState() ==
               CURRENT_SAMPLING_CAL_DONE)))
    {
      uint8_t startup_safe = 1U;
      runtime->precharge_in_progress = 1U;
      if (MainFoc_IsPowerStageSafe(NULL) == 0U)
      {
        startup_safe = 0U;
      }
      else
      {
        Tim1Pwm_BootstrapPrecharge();
      }
      neutral_settle_start = HAL_GetTick();
      while ((startup_safe != 0U) &&
             ((HAL_GetTick() - neutral_settle_start) <
              FOC_TEST_NEUTRAL_SETTLE_MS))
      {
        if (MainFoc_IsPowerStageSafe(NULL) == 0U)
        {
          startup_safe = 0U;
          break;
        }
      }

      if (startup_safe != 0U)
      {
        if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
        {
          g_global_mode_control.current_open_loop.electrical_angle_rad = 0.0f;
          g_global_mode_control.current_open_loop.cycle_count = 0UL;
          FOC_ControllerClearFaults(&g_foc_controller);
          runtime->output_armed = 1U;
          FOC_ControllerStart(&g_foc_controller);
        }
        else
        {
          runtime->output_armed = 1U;
        }
      }
      else
      {
        Tim1Pwm_EmergencyStop();
        runtime->output_request = 0U;
        runtime->output_enabled = 0U;
        runtime->fault_flags |= FOC_FAULT_POWER_STAGE;
      }
      runtime->precharge_in_progress = 0U;
    }

    if ((g_global_mode_control.active_mode != FOC_CONTROL_MODE_NORMAL) &&
        (runtime->fault_status_capture_pending != 0U) &&
        (runtime->fault_status_valid == 0U))
    {
      uint8_t status_raw;
      if (STSPIN32G4_ReadRegister(STSPIN32G4_REG_STATUS, &status_raw, 10U) ==
          HAL_OK)
      {
        runtime->first_fault_status_raw = status_raw;
        runtime->fault_status_valid = 1U;
        runtime->fault_status_capture_pending = 0U;
      }
    }
    WaveformGenerator_Task();
    Encoder_Service();
    /* --------------------------------------------------------------------
     * 后台任务层：无截止时间，可阻塞数十 µs
     * --------------------------------------------------------------------
     * DrvMonitor_Task() 在 TIM6 心跳未到时立即返回，所以每轮都调用的开销
     * 近乎为零。全工程的所有 I2C 通信都发生在这一次调用内部，绝不在中断里。
     */
    DrvMonitor_Task();

    /* 正式模式下，状态转换、Servo ON/OFF、MOE 和故障锁存由状态机管理；
       开环测试模式在 ADC 回调中直接运行算法与 PWM，不进入状态管理。 */
    if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_NORMAL)
    {
      ServoManager_Service(&g_servo_manager);
    }

    /* TODO: 协议栈（CAN/UART）、参数存储、温度记录、上位机遥测 ……
     * 都属于这一层。 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* X1 为 8 MHz：8 MHz / 2 = 4 MHz VCO 输入，× 84 = 336 MHz VCO，/ 2 = 168 MHz
     SYSCLK。VCO 输入取 4 MHz 是 ST 推荐的最低 PLL 抖动点。
     SYSCLK 刻意保持 168 MHz，因为所有派生时序常量都依赖它：20 kHz TIM1 载波、
     500 ns 死区、10 kHz TIM6 计数时钟（分频 16800）以及 I2C 时序字。改主频会
     连带让这些常量全部失效。 */
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC12 的复位时钟源为 NONE。只打开 ADC12 总线时钟还不够，必须显式
     选择内核时钟，否则 ADEN/JADSTART 可以置位，但转换永远不会完成，
     表现为 JEOS 不置位、JDR 不更新、采样序号保持 0。ADC12 源为
     SYSCLK=168 MHz，ADC 配置使用异步分频 4 得到 42 MHz。 */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInitStruct.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error source line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
