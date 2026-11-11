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
#include "speed_commission.h"
#include "waveform_generator.h"
#include "waveform_uart1_port.h"
#include "encoder.h"
#include "mt6701_spi3_test.h"
#include "mt6701_ssi.h"
#include "mpt_t485.h"
#include "output_position.h"
#include "encoder_calibration.h"
#include "user_control.h"
#include "encoder_calibration_store.h"
#include "spi1_encoder.h"
#include <math.h>

#ifndef FOC_TARGET_TIMING_BENCHMARK
#define FOC_TARGET_TIMING_BENCHMARK 0
#endif
#if FOC_TARGET_TIMING_BENCHMARK
#include "../../Tests/Target/foc_timing_benchmark.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* OPAMP/ADC 已启动后，给模拟信号链和驱动器状态一个独立稳定时间；
   这段时间 TIM1/CH4/ADC 继续运行，但 MOE 仍保持关闭。 */
#define CURRENT_SAMPLING_MOE_GUARD_MS  20U

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
/* 1 = HSE 12 MHz PLL, 2 = HSI 16 MHz fallback PLL; both target 168 MHz.
   Exposed for commissioning: fallback must not be mistaken for HSE success. */
volatile uint32_t g_system_clock_source;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void TimingProbeOutputs_Init(void);
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
  /* apply_request=2: independently optimized D/Q gains, same pu units.
     Write both controllers and design.dc_bus_v before submitting the request. */
  foc_pi_t requested_d, requested_q;
  volatile uint32_t apply_request, status;
  uint8_t valid;
} g_current_tuning = {
  .design = { .dc_bus_v = 24.0f, .bandwidth_hz = 200.0f,
    .sample_period_s = CONTROL_PERIOD_S, .delay_s = 1.5f * CONTROL_PERIOD_S,
    .phase_margin_deg = 60.0f, .integral_limit_pu = 0.02f }
};
/* Zero-initialized capture stays in ZI, avoiding a 14 KB flash copy image. */
current_loop_test_t g_current_loop_test;
/* Independent wall-clock evidence for commissioning; CYCCNT at 168 MHz. */
struct
{
  volatile uint32_t first_cycle, last_cycle, samples;
  volatile uint32_t step_cycles_max;
} g_current_test_timing;
volatile uint32_t g_speed_profile[5]; /* start, feedback done, algorithm done, PWM done, total */
static float s_adc_phase_u_a, s_adc_phase_v_a;
/* Frozen on measured-current trip; reset clears it. Units follow ADC calibration. */
struct
{
  volatile uint32_t count, sample_sequence;
  volatile float phase_u_a, phase_v_a, phase_w_a;
  volatile uint32_t raw_u, raw_v;
} g_current_trip;
/* First >5 A sampled phase peak in a continuous run: diagnostic only, never a
   limiter. Common DWT time base permits comparison with encoder bad frames.
   Timer CCR reads are preload-register snapshots, not an independent waveform. */
struct
{
  volatile uint32_t count, cycle, sample_sequence, raw_u, raw_v;
  volatile float phase_u_a, phase_v_a, phase_w_a;
  volatile uint32_t adc_isr, tim_cnt, tim_cr1, ccr1, ccr2, ccr3, ccr4;
  volatile uint32_t encoder_cycle, encoder_fault;
} g_current_peak_capture;
/* PWM-OFF diagnostic capture. No excitation; raw summed ADC codes.
   command=1 starts a fresh contiguous 1024-conversion capture after calibration. */
struct
{
  volatile uint32_t command, count, first_sequence, last_sequence;
  uint32_t offset_u, offset_v, ratio, profile;
  uint16_t raw[1024][2];
} g_adc_noise;

#if FOC_TARGET_IRQ_PROFILE
/* OFF-only timing fixture; uses the idle ADC-noise buffer, no extra trace RAM.
   No ServoManager task is run in this fixture and no MOE setter is called. */
volatile struct { uint32_t armed, count, done, maximum_cycles; } g_foc_irq_profile;
void FocIrqProfile_Record(uint32_t cycles)
{
  if (!g_foc_irq_profile.armed || !g_foc_controller.running ||
      !g_foc_controller.algorithm.output.valid) return;
  uint32_t n = g_foc_irq_profile.count;
  uint32_t *trace = (uint32_t *)(void *)g_adc_noise.raw;
  trace += n * 8U;
  trace[0] = g_speed_profile[0] - g_tim1_pwm_adc_entry_cycle;
  trace[1] = g_speed_profile[1];
  trace[2] = g_speed_profile[2] - g_speed_profile[1];
  trace[3] = g_speed_profile[3] - g_speed_profile[2];
  trace[4] = g_speed_profile[4] - g_speed_profile[3];
  trace[5] = cycles - (g_speed_profile[0] - g_tim1_pwm_adc_entry_cycle) - g_speed_profile[4];
  trace[6] = cycles;
  trace[7] = g_current_sampling.sample_sequence;
  if (cycles > g_foc_irq_profile.maximum_cycles) g_foc_irq_profile.maximum_cycles = cycles;
  g_foc_irq_profile.count = n + 1U;
  if (n + 1U == 128U) { g_foc_irq_profile.armed = 0U; g_foc_irq_profile.done = 1U; }
}
#endif
/* 全局模式控制对象：统一管理电压开环、电流测试环、正常模式及公共运行状态。 */
foc_control_manager_t g_global_mode_control =
{
  .requested_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP,
  .active_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP,
  .switch_count = 0UL,
  .voltage_open_loop =
  {
    .d_voltage_v = 1.0f, .q_voltage_v = 0.0f,
    .dc_bus_voltage_v = 24.0f, .electrical_frequency_hz = 10.0f,
    .electrical_angle_rad = 0.0f,
    .voltage_ramp_rate_v_per_s = 0.5f
  },
  .current_open_loop =
  {
    .requested_stage = FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP,
    .active_stage = FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP,
    .d_voltage_check_v = 0.0f,
    .applied_d_voltage_check_v = 0.0f,
    .voltage_check_ramp_v_per_s = 0.20f,
    .d_current_reference_a = 0.0f, .q_current_reference_a = 0.0f,
    .applied_d_current_reference_a = 0.0f,
    .applied_q_current_reference_a = 0.0f,
    .current_reference_ramp_a_per_s = 0.20f,
    /* Default to fixed-angle D/Q PI with zero references and output disabled.
       A fixed stator angle does not mechanically lock the rotor. */
    .dc_bus_voltage_v = 24.0f, .electrical_frequency_hz = 0.0f,
    .electrical_angle_rad = 0.0f
  },
  .normal_loop =
  {
    .d_current_reference_a = 0.0f, .q_current_reference_a = 0.0f,
    .electrical_angle_rad = 0.0f,
    .angle_valid = 0U, .reference_valid = 0U
  },
  /* Boot speed sequencing owns output: align first, then close the speed loop. */
  .runtime = { .output_request = 0U, .fault_flags = FOC_FAULT_NONE }
};

static float MainFoc_Slew(float value, float target, float step)
{
  if (step <= 0.0f) return target;
  if (value < target)
  {
    value += step;
    if (value > target) value = target;
  }
  else if (value > target)
  {
    value -= step;
    if (value < target) value = target;
  }
  return value;
}

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
    /* Converted once by the ADC callback, also used for phase protection. */
    input->phase_a = s_adc_phase_u_a;
    input->phase_b = s_adc_phase_v_a;
    input->current_valid =
        (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_DONE) ?
        1U : 0U;
  }

  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    if ((g_user_control.status.state == USER_STARTING ||
         g_user_control.status.state == USER_RUNNING) &&
        (g_user_control.status.active_mode == USER_MODE_VOLTAGE ||
         g_user_control.status.active_mode == USER_MODE_OPEN_CURRENT ||
         g_user_control.status.active_mode == USER_MODE_TORQUE))
    {
      float angle = 0.0f;
      uint8_t valid = g_user_control.status.active_mode != USER_MODE_TORQUE ||
          Encoder_ReadElectricalAngleForFoc(&angle);
      input->reference_valid = UserControl_FastReference(input,
          g_foc_controller.algorithm.parameters.sample_period_s, angle, valid);
      /* Bounded commissioning reuses the user enable/power-stage gates.
         Only a stationary, encoder-free current frame may accept the test;
         the MCU recorder stops output after target_count samples. */
      if (g_current_loop_test.state == CURRENT_TEST_ARMED ||
          g_current_loop_test.state == CURRENT_TEST_RUNNING)
      {
        if (g_user_control.status.active_mode == USER_MODE_OPEN_CURRENT &&
            g_user_control.open_current.electrical_hz == 0.0f &&
            input->reference_valid)
          input->reference_valid = CurrentTest_Reference(&g_current_loop_test,
              g_foc_controller.algorithm.parameters.sample_period_s,
              &input->d_reference, &input->q_reference);
        else
        {
          CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
          input->reference_valid = 0U;
        }
      }
      if (g_user_control.status.active_mode == USER_MODE_VOLTAGE)
      {
        g_speed_voltage_alignment.d_voltage_v = g_user_control.voltage.ud_v;
        g_speed_voltage_alignment.q_voltage_v = g_user_control.voltage.uq_v;
      }
      return;
    }
    if (g_speed_test.state != 0U)
    {
      input->reference_valid = SpeedCommission_FastReference(input);
      if (SpeedCommission_OuterPending())
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
      g_speed_profile[1] = DWT->CYCCNT-g_speed_profile[0];
      return;
    }
    foc_current_test_t *test = &g_global_mode_control.current_open_loop;
    float reference_step = test->current_reference_ramp_a_per_s *
        g_foc_controller.algorithm.parameters.sample_period_s;
    test->applied_d_current_reference_a = MainFoc_Slew(
        test->applied_d_current_reference_a,
        test->d_current_reference_a, reference_step);
    if (test->active_stage == FOC_CURRENT_STAGE_D_CURRENT_P_CHECK)
    {
      test->applied_q_current_reference_a = 0.0f;
    }
    else
    {
      test->applied_q_current_reference_a = MainFoc_Slew(
          test->applied_q_current_reference_a,
          test->q_current_reference_a, reference_step);
    }
    input->d_reference = test->applied_d_current_reference_a;
    input->q_reference = test->applied_q_current_reference_a;
    /* Fixed stator frame: no position feedback and no virtual rotation. */
    input->electrical_angle_rad = 0.0f;
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
    g_speed_profile[1] = DWT->CYCCNT - g_speed_profile[0];
  }
}

static uint8_t MainFoc_WriteVoltage(void *context, const foc_output_t *output)
{
  (void)context;
  if (output != NULL)
  {
    g_speed_profile[2] = DWT->CYCCNT-g_speed_profile[0];
    uint8_t ok=(Tim1Pwm_SetPhaseVoltage(output->phase_a_duty,
                                     output->phase_b_duty,
                                     output->phase_c_duty) == HAL_OK) ? 1U : 0U;
    g_speed_profile[3] = DWT->CYCCNT-g_speed_profile[0];
    return ok;
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

static void MainFoc_ConfigureCurrentStageParameters(foc_current_stage_t stage)
{
  foc_parameters_t *parameters = &g_foc_controller.algorithm.parameters;

  parameters->voltage_limit = 0.02f;
  parameters->modulation.mode = FOC_MODULATION_LINEAR_ONLY;
  parameters->current_limit = 0.5f;
  parameters->d_controller.integral = 0.0f;
  parameters->q_controller.integral = 0.0f;
  if (stage == FOC_CURRENT_STAGE_D_CURRENT_P_CHECK)
  {
    /* 第一次接环只闭 D 轴比例环。增益从原值的 1/10 起步，Q 轴输出强制为 0，
       用于区分 D 环自身稳定性和两轴耦合问题。 */
    parameters->d_controller.kp = 0.005f;
    parameters->d_controller.ki = 0.0f;
    parameters->d_controller.integral_min = 0.0f;
    parameters->d_controller.integral_max = 0.0f;
    parameters->q_controller.kp = 0.0f;
    parameters->q_controller.ki = 0.0f;
    parameters->q_controller.integral_min = 0.0f;
    parameters->q_controller.integral_max = 0.0f;
  }
  else if (stage == FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP)
  {
    /* 2026-10-27: retuned after physical U/V current-scale calibration.
       Actual 20 kHz, fixed frame, +/-0.3 A step and 10..1200 Hz sine;
       D/Q gains selected for response/noise balance, with CH4 U validation.
       Units: kp pu/A, ki pu/(A*s), integral += ki * error * period_s.
       See docs/log/2026-10-27_电流环时域与频域优化.md. */
    parameters->d_controller.kp = 0.05f;
    parameters->d_controller.ki = 100.0f;
    parameters->d_controller.integral_min = -0.01f;
    parameters->d_controller.integral_max = 0.01f;
    parameters->q_controller = parameters->d_controller;
    if (g_current_tuning.valid)
    {
      parameters->d_controller = g_current_tuning.gains.d;
      parameters->q_controller = g_current_tuning.gains.q;
    }
  }
  else
  {
    parameters->d_controller.kp = 0.0f;
    parameters->d_controller.ki = 0.0f;
    parameters->d_controller.integral_min = 0.0f;
    parameters->d_controller.integral_max = 0.0f;
    parameters->q_controller = parameters->d_controller;
  }
  FOC_AlgorithmReset(&g_foc_controller.algorithm);
}

static void MainFoc_ConfigureModeParameters(foc_control_mode_t mode)
{
  foc_parameters_t *parameters = &g_foc_controller.algorithm.parameters;
  /* Voltage open loop uses a generated angle, independent of encoder data. */
  g_encoder_feedback_required = (mode != FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP);

  if (mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
  {
    MainFoc_ConfigureCurrentStageParameters(
        g_global_mode_control.current_open_loop.active_stage);
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

/* Board adapters for the isolated calibration business module. */
static speed_test_config_t *s_calibration_previous_speed_config;
static uint8_t s_calibration_restore_speed_config;
static uint8_t MainEncoderCalibration_Start(void)
{
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  if (r->output_request || r->output_armed || r->precharge_in_progress ||
      g_foc_controller.running || Tim1Pwm_IsOutputEnabled() ||
      r->fault_flags || g_foc_controller.fault_flags || g_speed_test.fault ||
      g_speed_test.command || g_speed_run.enable ||
      CurrentSampling_GetCalibrationState() != CURRENT_SAMPLING_CAL_DONE)
    return 0U;
  *s_calibration_previous_speed_config = g_speed_test.requested;
  s_calibration_restore_speed_config = 1U;
  g_speed_test.requested.kind = SPEED_VOLTAGE_ALIGNMENT_KIND;
  g_speed_test.command = 1U;
  return 1U;
}
static void MainEncoderCalibration_Stop(void)
{
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  r->output_request = r->output_armed = 0U;
  g_speed_run.enable = 0U;
  g_speed_test.command = 0U;
  FOC_ControllerStop(&g_foc_controller);
  Tim1Pwm_EmergencyStop();
  if (g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND)
  {
    if (g_encoder_calibration.fault == ENCODER_CAL_CANCELLED && !g_speed_test.fault)
    {
      /* Cancellation never committed a new zero; the previously loaded
         configuration is still installed in Encoder. Release the executor. */
      g_speed_test.calibration_ok = g_encoder_calibration.valid;
      g_speed_test.state = 0U;
      if (s_calibration_restore_speed_config) {
        g_speed_test.requested = *s_calibration_previous_speed_config;
        s_calibration_restore_speed_config = 0U;
      }
    }
    else
    {
      g_speed_test.calibration_ok = 0U;
      g_speed_test.state = 4U;
      if (!g_speed_test.fault) g_speed_test.fault = 7U;
    }
  }
}
static uint32_t MainEncoderCalibration_Fault(void)
{
  return g_speed_test.fault || g_global_mode_control.runtime.fault_flags ||
      g_foc_controller.fault_flags;
}
static uint8_t MainEncoderCalibration_Save(int32_t direction, uint32_t poles,
    float offset, const encoder_calibration_config_t *recipe)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  /* Start the Flash pause only at an idle SPI boundary. A transfer left
     pending across an erase would otherwise look like a communication stall. */
  uint8_t saved = MT6835_SPI_IsBusy() ? 2U :
      EncoderCalibrationStore_Save(direction, poles, offset,
                                    hypotf(recipe->d_voltage_v, recipe->q_voltage_v),
                                    recipe->duration_s);
  if (!mask) __enable_irq();
  return saved;
}
static uint8_t MainEncoderCalibration_Apply(int32_t direction, uint32_t poles,
                                           float offset)
{
  if (Tim1Pwm_IsOutputEnabled() || g_foc_controller.running ||
      !Encoder_ConfigureMotor((int8_t)direction, (uint16_t)poles, offset)) return 0U;
  g_speed_test.direction = direction;
  g_speed_test.pole_pairs = poles;
  g_speed_test.electrical_offset_rad = offset;
  g_speed_test.calibration_ok = 1U;
  /* Position arm requires an idle executor. Keep the completed calibration
     in its own object, and release only a successful OFF voltage executor. */
  if (g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND &&
      g_speed_test.state == 3U && !g_speed_test.fault)
    g_speed_test.state = 0U;
  if (s_calibration_restore_speed_config)
  {
    g_speed_test.requested = *s_calibration_previous_speed_config;
    s_calibration_restore_speed_config = 0U;
  }
  return 1U;
}

/* Background policy only; all excitation and capture math lives in Tests.
   Never apply a multi-field PI update while the ADC ISR is using the PI. */
static void MainCurrentTest_Service(void)
{
  static uint32_t speed_run_was_enabled;
  static uint8_t continuous_limits_applied;
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  uint8_t off = !r->output_request && !r->output_armed &&
      !r->precharge_in_progress && !g_foc_controller.running &&
      !Tim1Pwm_IsOutputEnabled();
  SpeedResponse_Service(off);
  foc_current_test_t *current_test = &g_global_mode_control.current_open_loop;
  /* Restore bounded-test voltage limits only after all power outputs are off. */
  if (off && continuous_limits_applied)
  {
    MainFoc_ConfigureCurrentStageParameters(current_test->active_stage);
    continuous_limits_applied = 0U;
  }
  uint32_t sampled_test_command = g_speed_test.command;
  uint8_t start_encoder_debug =
      sampled_test_command == ENCODER_DEBUG_START_COMMAND ||
      (sampled_test_command == 1U &&
       g_speed_test.requested.kind == ENCODER_DEBUG_OPEN_LOOP_KIND);
  uint8_t start_voltage_zero = sampled_test_command == 1U &&
      g_speed_test.requested.kind == SPEED_VOLTAGE_ALIGNMENT_KIND;
  if (start_encoder_debug || start_voltage_zero)
  {
    /* Only the ENCODER readiness/calibration gates are omitted. Never clear
       a latched controller/runtime/driver/ADC fault to force this start. */
    uint32_t block = 0U;
    if (!off) block |= SPEED_START_OUTPUT_ACTIVE;
    if (r->fault_flags) block |= SPEED_START_RUNTIME_FAULT;
    if (g_foc_controller.fault_flags) block |= SPEED_START_CONTROLLER_FAULT;
    if (!MainFoc_IsPowerStageSafe(NULL)) block |= SPEED_START_POWER_UNSAFE;
    if (CurrentSampling_GetCalibrationState() != CURRENT_SAMPLING_CAL_DONE)
      block |= SPEED_START_ADC_NOT_READY;
    if (g_global_mode_control.active_mode != FOC_CONTROL_MODE_CURRENT_OPEN_LOOP ||
        g_global_mode_control.requested_mode != FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
      block |= SPEED_START_MODE_INVALID;
    if (current_test->active_stage != FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP ||
        current_test->requested_stage != FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP)
      block |= SPEED_START_STAGE_INVALID;
    if (g_current_loop_test.state != CURRENT_TEST_IDLE ||
        g_speed_test.state == 1U || g_speed_test.state == 2U)
      block |= SPEED_START_TEST_BUSY;
    if (g_speed_run.enable || g_current_loop_test.command ||
        g_current_tuning.apply_request || g_speed_test.calibration_command)
      block |= SPEED_START_COMMAND_PENDING;
    if (start_voltage_zero)
    {
      encoder_snapshot_t sample = {0};
      if (!Encoder_ReadSnapshot(&sample) || !sample.angle_valid ||
          !sample.speed_valid || !Encoder_PowerOutputPermitted())
        block |= SPEED_START_ENCODER_INVALID;
      g_speed_run.start_block_mask = block;
    }
    g_encoder_debug_start_block_mask = block;
    if (block)
    {
      g_speed_test.command = 0U;
      sampled_test_command = 0U;
      g_speed_test.fault = 12U; /* Debug start interlock refused. */
      g_speed_test.state = 4U;
      g_speed_run.enable = 0U;
      start_encoder_debug = 0U;
      start_voltage_zero = 0U;
    }
  }
  /* Read the debugger-written command once for edge detection. Reading it
     again at the end could remember a newly written 1 without processing it,
     permanently losing the start edge. */
  uint32_t sampled_speed_enable = g_speed_run.enable;
  if (sampled_speed_enable > 1U)
  {
    g_speed_run.enable = sampled_speed_enable = 0U;
  }
  uint8_t start_continuous = sampled_speed_enable && !speed_run_was_enabled;
  if (start_continuous)
  {
    encoder_snapshot_t start_encoder={0};
    uint32_t block=0U;
    if (!off) block |= SPEED_START_OUTPUT_ACTIVE;
    else if (!Encoder_ConfigureFeedbackTolerance(g_speed_run.encoder_tolerance_cycles))
      block |= SPEED_START_ENCODER_TOLERANCE_INVALID;
    if (r->fault_flags) block |= SPEED_START_RUNTIME_FAULT;
    if (g_foc_controller.fault_flags) block |= SPEED_START_CONTROLLER_FAULT;
    if (!MainFoc_IsPowerStageSafe(NULL)) block |= SPEED_START_POWER_UNSAFE;
    if (CurrentSampling_GetCalibrationState()!=CURRENT_SAMPLING_CAL_DONE)
      block |= SPEED_START_ADC_NOT_READY;
    if (g_global_mode_control.active_mode!=FOC_CONTROL_MODE_CURRENT_OPEN_LOOP ||
        g_global_mode_control.requested_mode!=FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
      block |= SPEED_START_MODE_INVALID;
    if (current_test->active_stage!=FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP ||
        current_test->requested_stage!=FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP)
      block |= SPEED_START_STAGE_INVALID;
    if (g_current_loop_test.state!=CURRENT_TEST_IDLE) block |= SPEED_START_TEST_BUSY;
    if (g_current_loop_test.command || g_current_tuning.apply_request || g_speed_test.command)
      block |= SPEED_START_COMMAND_PENDING;
    if (!g_speed_test.calibration_ok) block |= SPEED_START_CALIBRATION_MISSING;
    if (!Encoder_ReadSnapshot(&start_encoder) || !start_encoder.angle_valid ||
        !start_encoder.speed_valid || !start_encoder.electrical_angle_valid)
      block |= SPEED_START_ENCODER_INVALID;
    g_speed_run.start_block_mask=block;
    if (block)
    {
      g_speed_run.enable = 0U;
      g_speed_run.fault = 10U; /* Start interlock refused; never clear faults here. */
      g_speed_run.state = 4U;
      start_continuous = 0U;
    }
    else
    {
      g_speed_test.requested.kind = SPEED_CONTINUOUS_KIND;
      g_speed_test.requested.speed_limit_rad_s = SPEED_RUN_OVERSPEED_RAD_S;
      g_speed_test.requested.current_limit_a = g_speed_run_overload.request==1U ?
          SPEED_RUN_OVERLOAD_IQ_LIMIT_A : SPEED_RUN_IQ_LIMIT_A;
      g_speed_test.command = 1U;
      sampled_test_command = 1U;
    }
  }
  uint8_t stop_encoder_debug =
      g_speed_test.active.kind == ENCODER_DEBUG_OPEN_LOOP_KIND &&
      sampled_test_command == 2U;
  if ((g_speed_test.state == 1U || g_speed_test.state == 2U) &&
      (r->fault_flags || g_foc_controller.fault_flags ||
       ((g_speed_test.active.kind == SPEED_CONTINUOUS_KIND ||
         g_speed_test.active.kind == ENCODER_DEBUG_OPEN_LOOP_KIND) &&
        (stop_encoder_debug ||
         (g_speed_test.active.kind == SPEED_CONTINUOUS_KIND && !g_speed_run.enable) ||
         ((speed_run_was_enabled ||
           g_speed_test.active.kind == ENCODER_DEBUG_OPEN_LOOP_KIND) && !r->output_request)))))
  {
    g_speed_run.enable = 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    r->output_request = r->output_armed = 0U;
    FOC_ControllerStop(&g_foc_controller);
    r->running = 0U;
    r->output_enabled = Tim1Pwm_IsOutputEnabled();
    if (stop_encoder_debug) g_speed_test.command = 0U;
    g_speed_test.state = 3U;
    if (r->fault_flags || g_foc_controller.fault_flags)
    {
      g_speed_test.state = 4U;
      /* ADC feedback faults may preempt this background branch. Keep their
         first cause instead of overwriting it with generic controller fault. */
      if (!g_speed_test.fault) g_speed_test.fault = 11U;
    }
    if (!mask) __enable_irq();
  }
  SpeedCommission_ServiceCommand(off, sampled_test_command);
  SpeedCommission_PublishRunTelemetry();
  if (off && g_speed_test.state == 1U &&
      (g_speed_test.active.kind == 1U || g_speed_test.active.kind == SPEED_ALIGNMENT_SWEEP_KIND ||
       g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND))
  {
    foc_parameters_t *p = &g_foc_controller.algorithm.parameters;
    p->current_limit = g_speed_test.active.current_limit_a;
    p->voltage_limit = g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND ?
        SPEED_VOLTAGE_ALIGNMENT_LIMIT_PU : 0.05f;
    p->d_controller.integral_min = p->q_controller.integral_min = -0.05f;
    p->d_controller.integral_max = p->q_controller.integral_max = 0.05f;
    continuous_limits_applied = 1U; /* Restore commissioning limits after alignment. */
    if (start_voltage_zero)
      r->output_request = 1U; /* Use the existing precharge/driver/ADC gates. */
  }
  if (start_encoder_debug && g_speed_test.state == 1U &&
      g_speed_test.active.kind == ENCODER_DEBUG_OPEN_LOOP_KIND)
  {
    MainFoc_ConfigureCurrentStageParameters(current_test->active_stage);
    foc_parameters_t *p = &g_foc_controller.algorithm.parameters;
    p->current_limit = g_speed_test.active.current_limit_a;
    p->voltage_limit = ENCODER_DEBUG_VOLTAGE_LIMIT_PU;
    p->d_controller.integral_min = p->q_controller.integral_min =
        -ENCODER_DEBUG_VOLTAGE_LIMIT_PU;
    p->d_controller.integral_max = p->q_controller.integral_max =
        ENCODER_DEBUG_VOLTAGE_LIMIT_PU;
    continuous_limits_applied = 1U;
    r->output_request = 1U; /* Same precharge/driver/ADC checks as other tests. */
  }
  if (start_continuous && g_speed_test.state == 1U &&
      g_speed_test.active.kind == SPEED_CONTINUOUS_KIND)
  {
    /* Voltage headroom stays inside the ADC-profile-derived duty window.
       The sampled phase-amplitude trip is configured separately. */
    MainFoc_ConfigureCurrentStageParameters(current_test->active_stage);
    foc_parameters_t *p = &g_foc_controller.algorithm.parameters;
    p->voltage_limit = SPEED_RUN_VOLTAGE_LIMIT_PU;
    p->modulation.mode = SPEED_RUN_MODULATION_MODE;
    /* Service validated and latched the one-shot permission before state=1.
       Do not reread a debugger-mutable overload request in the live loop. */
    p->current_limit = g_speed_test.active.current_limit_a;
    p->d_controller.integral_min = -SPEED_RUN_VOLTAGE_LIMIT_PU;
    p->d_controller.integral_max = SPEED_RUN_VOLTAGE_LIMIT_PU;
    p->q_controller.integral_min = -SPEED_RUN_VOLTAGE_LIMIT_PU;
    p->q_controller.integral_max = SPEED_RUN_VOLTAGE_LIMIT_PU;
    g_speed_run.phase_peak_a = 0.0f;
    g_current_peak_capture.count = 0U;
    continuous_limits_applied = 1U;
    r->output_request = 1U; /* Existing precharge/start interlocks remain in force. */
  }
  if (g_speed_test.state == 3U || g_speed_test.state == 4U)
  {
    r->output_request = 0U;
    /* Clear only a request observed at this service entry. An asynchronous
       OFF->ON write AFTER sampling must survive for the next iteration. */
    if (sampled_speed_enable) g_speed_run.enable = 0U;
  }
  if (g_speed_test.active.kind == SPEED_CONTINUOUS_KIND || start_continuous)
  {
    if (g_speed_run.fault != 10U || start_continuous)
      g_speed_run.state = g_speed_test.state;
    if (g_speed_run.fault != 10U || start_continuous)
      g_speed_run.fault = g_speed_test.fault;
  }
  speed_run_was_enabled = sampled_speed_enable && g_speed_run.enable;

  /* 电流环投运阶段只能在 MOE=0、控制器停止时切换。第一阶段完全旁路 PI；
     用户确认 Ud/Id 方向正确后，先令 output_request=0，再请求 DQ_PI。 */
  if (current_test->requested_stage >= FOC_CURRENT_STAGE_COUNT)
  {
    current_test->requested_stage = current_test->active_stage;
  }
  else if ((current_test->requested_stage != current_test->active_stage) && off)
  {
    current_test->active_stage = current_test->requested_stage;
    current_test->stage_switch_count++;
    current_test->applied_d_voltage_check_v = 0.0f;
    current_test->applied_d_current_reference_a = 0.0f;
    current_test->applied_q_current_reference_a = 0.0f;
    current_test->electrical_angle_rad = 0.0f;
    current_test->cycle_count = 0UL;
    MainFoc_ConfigureCurrentStageParameters(current_test->active_stage);
  }
  if (g_current_tuning.apply_request)
  {
    foc_current_gains_t gains = {0};
    uint32_t request = g_current_tuning.apply_request;
    uint8_t accepted = 0U;
    g_current_tuning.apply_request = 0;
    if (!off || g_global_mode_control.active_mode != FOC_CONTROL_MODE_CURRENT_OPEN_LOOP ||
        g_current_loop_test.state == CURRENT_TEST_ARMED)
      g_current_tuning.status = 2;
    else
    {
      /* Timing comes from firmware, not an independently editable design value. */
      g_current_tuning.design.sample_period_s = g_foc_controller.timing.pwm_period_s;
      if (request == 1U)
      {
        accepted = FOC_CurrentDesign(&g_current_tuning.design, &gains) &&
            g_current_tuning.design.integral_limit_pu <=
              g_foc_controller.algorithm.parameters.voltage_limit;
      }
      else if (request == 2U)
      {
        float limit = g_foc_controller.algorithm.parameters.voltage_limit;
        gains.d = g_current_tuning.requested_d;
        gains.q = g_current_tuning.requested_q;
        gains.d.integral = gains.q.integral = 0.0f;
        accepted = FOC_PI_IsValid(&gains.d) && FOC_PI_IsValid(&gains.q) &&
            gains.d.kp > 0.0f && gains.d.ki > 0.0f &&
            gains.q.kp > 0.0f && gains.q.ki > 0.0f &&
            gains.d.integral_min >= -limit && gains.d.integral_max <= limit &&
            gains.q.integral_min >= -limit && gains.q.integral_max <= limit &&
            gains.d.integral_min < 0.0f && gains.d.integral_max > 0.0f &&
            gains.q.integral_min < 0.0f && gains.q.integral_max > 0.0f &&
            isfinite(g_current_tuning.design.dc_bus_v) &&
            g_current_tuning.design.dc_bus_v > 0.0f;
        /* Direct gains have no analytic bandwidth certificate: fields stay 0. */
      }
      if (accepted)
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
        g_global_mode_control.current_open_loop.active_stage ==
          FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP &&
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
  g_global_mode_control.current_open_loop.applied_d_voltage_check_v = 0.0f;
  g_global_mode_control.current_open_loop.applied_d_current_reference_a = 0.0f;
  g_global_mode_control.current_open_loop.applied_q_current_reference_a = 0.0f;
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

#include "../../Servo/Src/user_control_dispatch.inc"

/* 把控制层的完整同步快照交给波形业务层。这里不选择打印通道；预设模式和
   自定义通道均由 g_waveform_generator 统一解释，避免 main.c 散落打印策略。 */
static void MainWaveform_Publish(const foc_controller_t *controller,
                                 float phase_u_a,
                                 float phase_v_a,
                                 float phase_w_a)
{
  /* Bounded motion tests use the timestamped RAM recorder. Avoid duplicating
     carrier-rate UART snapshot work while measuring rotating-frame timing. */
  if (OUTPUT_AXIS_ENABLE || g_speed_test.state != 0U) return;
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
    if (g_global_mode_control.current_open_loop.active_stage !=
        FOC_CURRENT_STAGE_D_VOLTAGE_CHECK)
    {
      source.d_current_reference_a = controller->input.d_reference;
      source.q_current_reference_a = controller->input.q_reference;
    }
    else
    {
      /* 固定电压方向测试没有电流指令，禁止上位机继续显示旧的 0.1 A
         指令并误认为 PI 正在闭环。Ud/Uq 可从电压输出通道观察。 */
      source.d_current_reference_a = 0.0f;
      source.q_current_reference_a = 0.0f;
    }
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

static void MainFoc_RecordPhasePeak(float u, float v, float w)
{
#if OUTPUT_AXIS_ENABLE
  OutputPosition_ObservePhaseCurrents(u,v,w);
#endif
  float peak=fabsf(u), pv=fabsf(v), pw=fabsf(w);
  if(pv>peak) peak=pv;
  if(pw>peak) peak=pw;
  if(peak>g_speed_run.phase_peak_a) g_speed_run.phase_peak_a=peak;
  if(g_speed_test.active.kind!=SPEED_CONTINUOUS_KIND || peak<=5.0f ||
     g_current_peak_capture.count) return;
  g_current_peak_capture.cycle=DWT->CYCCNT;
  g_current_peak_capture.sample_sequence=g_current_sampling.sample_sequence;
  g_current_peak_capture.raw_u=g_current_sampling.raw_u;
  g_current_peak_capture.raw_v=g_current_sampling.raw_v;
  g_current_peak_capture.phase_u_a=u;
  g_current_peak_capture.phase_v_a=v;
  g_current_peak_capture.phase_w_a=w;
  g_current_peak_capture.adc_isr=ADC1->ISR;
  g_current_peak_capture.tim_cnt=TIM1->CNT;
  g_current_peak_capture.tim_cr1=TIM1->CR1;
  g_current_peak_capture.ccr1=TIM1->CCR1;
  g_current_peak_capture.ccr2=TIM1->CCR2;
  g_current_peak_capture.ccr3=TIM1->CCR3;
  g_current_peak_capture.ccr4=TIM1->CCR4;
  g_current_peak_capture.encoder_cycle=g_encoder_snapshot.sample_cycle;
  g_current_peak_capture.encoder_fault=g_encoder_snapshot.fault_flags;
  g_current_peak_capture.count=1U;
}

/* Keep the large commissioning/recorder stack out of the production path. */
static __attribute__((noinline)) void MainFoc_OnAdcConversionSlow(void *context)
{
  foc_controller_t *controller = (foc_controller_t *)context;
  foc_control_runtime_t *runtime = &g_global_mode_control.runtime;
  float phase_u_a;
  float phase_v_a;
  float phase_w_a;

  if ((g_user_control.status.state == USER_STARTING ||
       g_user_control.status.state == USER_RUNNING) &&
      (g_user_control.enable == 0U ||
       g_user_control.mode != g_user_control.status.active_mode))
  {
    runtime->output_request = runtime->output_armed = 0U;
    FOC_ControllerStop(controller);
    return;
  }

  CurrentSampling_ReadFeedback_A(&phase_u_a, &phase_v_a, &phase_w_a);
  s_adc_phase_u_a=phase_u_a; s_adc_phase_v_a=phase_v_a;

  /* Production position/speed control does not use commissioning, ADC-noise
     capture or waveform-UART work. Bypass those branches in the carrier ISR;
     the controller still performs feedback validity, power-stage safety,
     FOC, PWM write and fault handling. Keep the diagnostic path available
     whenever an ADC-noise capture is explicitly armed. */
  if ((controller != NULL) &&
      (g_global_mode_control.active_mode == FOC_CONTROL_MODE_NORMAL) &&
      (g_adc_noise.command == 0U))
  {
    uint32_t step_start_cycles = DWT->CYCCNT;
    g_speed_profile[0] = step_start_cycles;
    controller->algorithm.parameters.sample_period_s =
        Tim1Pwm_OnCurrentSample(g_current_sampling.sample_sequence);
    FOC_ControllerStep(controller);
    g_speed_profile[4] = DWT->CYCCNT - step_start_cycles;
    controller->timing.last_sample_sequence =
        g_current_sampling.sample_sequence;
    return;
  }

  /* Keep sampled phase-peak telemetry even when the user bypasses the
     continuous-speed amplitude trip. Bounded experiments keep their limits. */
  float phase_trip_a = (g_speed_test.state == 1U || g_speed_test.state == 2U) ? 0.8f : 0.25f;
  if (g_speed_test.active.kind == ENCODER_DEBUG_OPEN_LOOP_KIND &&
      (g_speed_test.state == 1U || g_speed_test.state == 2U))
    phase_trip_a = ENCODER_DEBUG_PHASE_TRIP_A;
  if ((g_speed_test.active.kind == SPEED_CONTINUOUS_KIND || g_speed_test.active.kind == 1U ||
       g_speed_test.active.kind == SPEED_ALIGNMENT_SWEEP_KIND ||
       g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND) &&
      (g_speed_test.state == 1U || g_speed_test.state == 2U))
  {
    phase_trip_a = SPEED_RUN_PHASE_TRIP_A;
    if (runtime->output_armed)
    {
      MainFoc_RecordPhasePeak(phase_u_a, phase_v_a, phase_w_a);
    }
  }
  if (g_adc_noise.command == 1U && !runtime->output_request &&
      !runtime->output_armed && !Tim1Pwm_IsOutputEnabled() &&
      CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_DONE)
  {
    g_adc_noise.count = 0U;
    g_adc_noise.offset_u = g_current_sampling.offset_u;
    g_adc_noise.offset_v = g_current_sampling.offset_v;
    g_adc_noise.ratio = ADC1_CS_OVERSAMPLING_RATIO;
    g_adc_noise.profile = ADC1_CS_PROFILE;
    g_adc_noise.first_sequence = g_current_sampling.sample_sequence;
    g_adc_noise.command = 2U;
  }
  if (g_adc_noise.command == 2U)
  {
    if (runtime->output_request || runtime->output_armed || Tim1Pwm_IsOutputEnabled())
      g_adc_noise.command = 4U;
    else if (g_adc_noise.count < 1024U)
    {
      g_adc_noise.raw[g_adc_noise.count][0] = g_current_sampling.raw_u;
      g_adc_noise.raw[g_adc_noise.count][1] = g_current_sampling.raw_v;
      g_adc_noise.last_sequence = g_current_sampling.sample_sequence;
      if (++g_adc_noise.count == 1024U) g_adc_noise.command = 3U;
    }
  }
  if (controller == NULL)
  {
    return;
  }

  /* Continuous-mode amplitude trip is explicitly configurable; NaN/Inf and
     bounded-test thresholds still latch a fault before any voltage command. */
  if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP &&
      runtime->output_armed &&
      SpeedCommission_PhaseCurrentFault(phase_u_a, phase_v_a, phase_w_a,
                                        phase_trip_a))
  {
    g_current_trip.count++;
    g_current_trip.sample_sequence = g_current_sampling.sample_sequence;
    g_current_trip.phase_u_a = phase_u_a;
    g_current_trip.phase_v_a = phase_v_a;
    g_current_trip.phase_w_a = phase_w_a;
    g_current_trip.raw_u = g_current_sampling.raw_u;
    g_current_trip.raw_v = g_current_sampling.raw_v;
    runtime->output_request = runtime->output_armed = 0U;
    controller->fault_flags |= FOC_FAULT_CURRENT_INVALID;
    runtime->fault_flags |= FOC_FAULT_CURRENT_INVALID;
    FOC_ControllerStop(controller);
    CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
    runtime->running = runtime->output_enabled = 0U;
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
    /* 电流环分三步投运：固定 Ud 检查方向 -> 仅 D 轴低增益 P 环 ->
       低增益 D/Q PI。预充电期间所有阶段都不得写 CCR。 */
    g_global_mode_control.current_open_loop.electrical_angle_rad = 0.0f;
    g_global_mode_control.current_open_loop.electrical_frequency_hz = 0.0f;
    if ((runtime->output_armed == 0U) ||
        (runtime->precharge_in_progress != 0U))
    {
      runtime->running = controller->running;
      runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
      MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);
      return;
    }

    if (g_global_mode_control.current_open_loop.active_stage ==
        FOC_CURRENT_STAGE_D_VOLTAGE_CHECK)
    {
      foc_current_test_t *test = &g_global_mode_control.current_open_loop;
      float target_voltage = test->d_voltage_check_v;
      float maximum_voltage = controller->algorithm.parameters.voltage_limit *
                              test->dc_bus_voltage_v;
      float voltage_step = test->voltage_check_ramp_v_per_s *
                           controller->algorithm.parameters.sample_period_s;

      controller->feedback_ready = 0U;
      if ((target_voltage != target_voltage) ||
          (test->dc_bus_voltage_v <= 0.0f) ||
          (test->voltage_check_ramp_v_per_s !=
           test->voltage_check_ramp_v_per_s))
      {
        runtime->fault_flags |= FOC_FAULT_ALGORITHM;
        runtime->running = 0U;
        runtime->output_enabled = 0U;
        runtime->output_armed = 0U;
        controller->running = 0U;
        Tim1Pwm_EmergencyStop();
        return;
      }
      if ((runtime->power_stage_safe == 0U) ||
          (CurrentSampling_GetCalibrationState() != CURRENT_SAMPLING_CAL_DONE))
      {
        runtime->fault_flags |= (runtime->power_stage_safe == 0U) ?
            FOC_FAULT_POWER_STAGE : FOC_FAULT_CURRENT_INVALID;
        runtime->running = 0U;
        runtime->output_enabled = 0U;
        runtime->output_armed = 0U;
        controller->running = 0U;
        Tim1Pwm_EmergencyStop();
        return;
      }

      /* 电压方向测试仍受 current-open-loop 的 ±0.02 pu 安全限幅约束。
         Q 轴命令没有可写变量，下面固定传 0，避免 Watch 误操作。 */
      if (target_voltage > maximum_voltage) target_voltage = maximum_voltage;
      if (target_voltage < -maximum_voltage) target_voltage = -maximum_voltage;
      if (test->voltage_check_ramp_v_per_s <= 0.0f)
      {
        test->applied_d_voltage_check_v = target_voltage;
      }
      else if (test->applied_d_voltage_check_v < target_voltage)
      {
        test->applied_d_voltage_check_v += voltage_step;
        if (test->applied_d_voltage_check_v > target_voltage)
          test->applied_d_voltage_check_v = target_voltage;
      }
      else if (test->applied_d_voltage_check_v > target_voltage)
      {
        test->applied_d_voltage_check_v -= voltage_step;
        if (test->applied_d_voltage_check_v < target_voltage)
          test->applied_d_voltage_check_v = target_voltage;
      }

      if ((FOC_AlgorithmVoltageCommandStep(&controller->algorithm,
              test->applied_d_voltage_check_v, 0.0f,
              test->dc_bus_voltage_v, test->electrical_angle_rad) == 0U) ||
          (FOC_TransformCurrentDQ(phase_u_a, phase_v_a,
              test->electrical_angle_rad,
              &controller->algorithm.output.d_current,
              &controller->algorithm.output.q_current) == 0U))
      {
        runtime->fault_flags |= FOC_FAULT_ALGORITHM;
        runtime->running = 0U;
        runtime->output_enabled = 0U;
        runtime->output_armed = 0U;
        controller->running = 0U;
        Tim1Pwm_EmergencyStop();
        return;
      }
      if (MainFoc_WriteVoltage(NULL, &controller->algorithm.output) == 0U)
      {
        runtime->fault_flags |= FOC_FAULT_PWM_WRITE;
        runtime->running = 0U;
        runtime->output_enabled = 0U;
        runtime->output_armed = 0U;
        controller->running = 0U;
        Tim1Pwm_EmergencyStop();
        return;
      }

      controller->feedback_ready = 1U;
      controller->timing.cycle_count++;
      controller->timing.last_sample_sequence =
          g_current_sampling.sample_sequence;
      test->measured_d_current_a = controller->algorithm.output.d_current;
      test->measured_q_current_a = controller->algorithm.output.q_current;
      test->output_d_voltage_v = controller->algorithm.output.d_voltage *
                                 test->dc_bus_voltage_v;
      test->output_q_voltage_v = controller->algorithm.output.q_voltage *
                                 test->dc_bus_voltage_v;
      test->cycle_count++;
      runtime->running = 1U;
      runtime->output_enabled = Tim1Pwm_IsOutputEnabled();
      MainWaveform_Publish(controller, phase_u_a, phase_v_a, phase_w_a);
      return;
    }

    /* 第二/三阶段才调用正式控制器；具体启用 P 还是 PI 由阶段参数决定。 */
    uint32_t step_start_cycles = DWT->CYCCNT;
    g_speed_profile[0]=step_start_cycles;
    if (g_current_loop_test.state == CURRENT_TEST_ARMED)
    {
      g_current_test_timing.first_cycle = step_start_cycles;
      g_current_test_timing.samples = 0U;
      g_current_test_timing.step_cycles_max = 0U;
    }
    if ((g_speed_test.active.kind == SPEED_VOLTAGE_ALIGNMENT_KIND && g_speed_test.state != 0U) ||
        ((g_user_control.status.state == USER_STARTING || g_user_control.status.state == USER_RUNNING) &&
         g_user_control.status.active_mode == USER_MODE_VOLTAGE))
    {
      g_speed_voltage_alignment.dc_bus_voltage_v =
          g_global_mode_control.current_open_loop.dc_bus_voltage_v;
      FOC_ControllerVoltageStep(controller, &g_speed_voltage_alignment);
      g_global_mode_control.current_open_loop.electrical_angle_rad =
          controller->input.electrical_angle_rad;
    }
    else FOC_ControllerStep(controller);
    uint32_t step_cycles = DWT->CYCCNT - step_start_cycles;
    g_speed_profile[4]=step_cycles;
    if (step_cycles > g_current_test_timing.step_cycles_max)
      g_current_test_timing.step_cycles_max = step_cycles;
    g_current_test_timing.last_cycle = step_start_cycles;
    g_current_test_timing.samples++;
    CurrentTest_Observe(&g_current_loop_test, &controller->algorithm.output,
        controller->algorithm.parameters.sample_period_s,
        g_current_sampling.sample_sequence, Tim1Pwm_IsOutputEnabled());
    if (!controller->running && (g_current_loop_test.state == CURRENT_TEST_ARMED ||
        g_current_loop_test.state == CURRENT_TEST_RUNNING))
      CurrentTest_Abort(&g_current_loop_test, CURRENT_TEST_INVALID);
    if (g_speed_test.state == 3U || g_speed_test.state == 4U ||
        g_current_loop_test.state == CURRENT_TEST_DONE ||
        g_current_loop_test.state == CURRENT_TEST_ABORTED)
    {
      if (g_current_loop_test.state == CURRENT_TEST_DONE ||
          g_current_loop_test.state == CURRENT_TEST_ABORTED)
        g_user_control.enable = 0U;
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
    /* Current commissioning intentionally holds theta_e = 0 at every sample. */
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

static void MainFoc_OnAdcConversion(void *context)
{
  foc_controller_t *c=(foc_controller_t *)context;
  foc_control_runtime_t *r=&g_global_mode_control.runtime;
  foc_current_test_t *t=&g_global_mode_control.current_open_loop;
  uint32_t pins=GD_READY_GPIO_Port->IDR;
  if(c==NULL || g_global_mode_control.active_mode!=FOC_CONTROL_MODE_CURRENT_OPEN_LOOP ||
     g_speed_test.active.kind!=SPEED_CONTINUOUS_KIND || g_speed_test.state!=2U ||
     t->active_stage!=FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP ||
     g_current_loop_test.state!=CURRENT_TEST_IDLE || g_adc_noise.command!=0U ||
     !r->output_armed || r->precharge_in_progress ||
     (pins&(GD_READY_Pin|GD_NFAULT_Pin))!=(GD_READY_Pin|GD_NFAULT_Pin))
  {
    MainFoc_OnAdcConversionSlow(context);
    return;
  }
  float u,v,w;
  CurrentSampling_ReadFeedback_A(&u,&v,&w);
  if(SpeedCommission_PhaseCurrentFault(u,v,w,SPEED_RUN_PHASE_TRIP_A))
  {
    /* Exceptional samples use the unchanged capture/latching path. */
    MainFoc_OnAdcConversionSlow(context);
    return;
  }
  s_adc_phase_u_a=u; s_adc_phase_v_a=v;
  r->power_stage_pin_levels=pins;
  r->power_stage_safe=1U;
  t->electrical_angle_rad=t->electrical_frequency_hz=0.0f;
  c->algorithm.parameters.sample_period_s=Tim1Pwm_OnCurrentSample(g_current_sampling.sample_sequence);
  uint32_t start=DWT->CYCCNT;
  g_speed_profile[0]=start;
  FOC_ControllerStep(c);
  uint32_t cycles=DWT->CYCCNT-start;
  /* R16, 2026-10-29: retain the same sampled peak/RMS telemetry, but execute
     it AFTER the voltage/PWM commit. R15 captured a BEFORE_GUARD rejection
     (remaining 248 ticks, required 337). Statistics must not consume this
     control deadline. Fault trip checks above and the PWM guard stay intact.
     First-peak diagnostic CCRs now observe the newly committed preloads. */
  MainFoc_RecordPhasePeak(u,v,w);
  g_speed_profile[4]=cycles;
  if(cycles>g_current_test_timing.step_cycles_max) g_current_test_timing.step_cycles_max=cycles;
  g_current_test_timing.last_cycle=start;
  g_current_test_timing.samples++;
  if(g_speed_test.state==3U || g_speed_test.state==4U)
  {
    r->output_request=r->output_armed=0U;
    FOC_ControllerStop(c);
  }
  c->timing.last_sample_sequence=g_current_sampling.sample_sequence;
  r->fault_flags=c->fault_flags;
  r->running=c->running;
  r->output_enabled=Tim1Pwm_IsOutputEnabled();
  if(!c->running || !c->algorithm.output.valid) { r->output_armed=0U; return; }
  t->measured_d_current_a=c->algorithm.output.d_current;
  t->measured_q_current_a=c->algorithm.output.q_current;
  t->output_d_voltage_v=c->algorithm.output.d_voltage*t->dc_bus_voltage_v;
  t->output_q_voltage_v=c->algorithm.output.q_voltage*t->dc_bus_voltage_v;
  t->cycle_count++;
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
  /* Below ADC/TIM1/SPI DMA, above SysTick; never blocks the carrier loop. */
  HAL_NVIC_SetPriority(PendSV_IRQn, 14U, 0U);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

#if FOC_TARGET_TIMING_BENCHMARK
  FOC_TargetTimingBenchmark();
  while (1) { /* Test image: no motor peripherals or output enable. */ }
#endif

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if MT6701_SPI3_TEST_ENABLE && !MT6701_SSI_ENABLE && !ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  /* 临时 SSI 通信验证：PB6 CSN / PB3 CLK / PB4 MISO，默认每 10 ms 采样。 */
  (void)Mt6701Spi3Test_Init();
#endif
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
  #if OUTPUT_AXIS_ENABLE
  if (!MptT485_Init()) Error_Handler();
  #else
  {
    waveform_transport_t waveform_transport;
    const waveform_generator_config_t waveform_config =
    {
      .source_rate_hz = CONTROL_FREQUENCY_HZ,
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
  #endif
  /* 上电先保持硬件关断。MOE 只在零点校准完成、驱动状态安全且 FOC
     即将启动时释放，避免 ADC 偏置采集阶段把电压施加到电机。 */
  Tim1Pwm_EmergencyStop();
  /* USER CODE BEGIN 2 */
  /* 调试执行窗口探针：PA0=ADC/FOC，PD2=TIM1更新，PB7=SPI DMA完成回调。 */
  TimingProbeOutputs_Init();
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
   * CH4 触发点由 PWM 规划器按约 0.716 us 的完整 U/V 序列动态安排。
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
      .sample_period_s = CONTROL_PERIOD_S,
      .pwm_period_s = CONTROL_PERIOD_S,
      /* 首次闭环试验限制为 2 % 母线电压和 0.5 A，避免极性或参数错误时
         PI 直接把桥臂推到高占空比。 */
      .voltage_limit = 0.02f,
      .modulation = { .mode = FOC_MODULATION_LINEAR_ONLY,
                      .duty_min = 0.0f, .duty_max = 1.0f },
      .current_limit = 0.5f,
      /* 初始化先给零增益；ControllerInit 后按 active_stage 装入对应参数，
         防止任何启动路径意外用到旧的高增益。 */
      .d_controller = { .kp = 0.0f, .ki = 0.0f,
                        .integral_min = 0.0f, .integral_max = 0.0f },
      .q_controller = { .kp = 0.0f, .ki = 0.0f,
                        .integral_min = 0.0f, .integral_max = 0.0f }
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
    /* U/V 采用紧邻单次转换；每周期根据下一组 duty 同步规划 CCR4。
       完整序列窗口仍需保留；不能仅因触发点可移动就开放全母线六步。 */
    if (Tim1Pwm_ConfigureCurrentSampling(ADC1_CS_SEQUENCE_CONV_NS,
          &foc_parameters.modulation.duty_min,
          &foc_parameters.modulation.duty_max) != HAL_OK)
    {
      Error_Handler();
    }
    FOC_ControllerInit(&g_foc_controller, &foc_parameters, &foc_port);
    MainFoc_ConfigureModeParameters(g_global_mode_control.active_mode);
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
  /* 每次上电在 MOE=0 下重新测量 U/V 零偏；模拟链先独立稳定 20 ms，
     之后采满 20 帧约需 1 ms。采集失败时不使用历史常量继续启动。 */
  HAL_Delay(CURRENT_SAMPLING_MOE_GUARD_MS);
  CurrentSampling_RequestCalibration();
  while (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_COLLECTING)
  {
    CurrentSampling_Service();
  }
  if (CurrentSampling_GetCalibrationState() != CURRENT_SAMPLING_CAL_DONE)
  {
    Error_Handler();
  }

  /* MT6835 正式采集：TIM1 周期边界启动 SPI1 DMA，TIM1/DMA 优先级高于
     ADC/FOC。电角度在上层完成方向、极对数和零位配置前保持无效。 */
  if (Encoder_Init() == 0U)
  {
    Error_Handler();
  }
  /* Seed this assembly's direction/pole pairs. The legacy offset is not
     accepted for boot speed operation: voltage zeroing must replace it. */
  /* Consume the default enable=1 into a pending request BEFORE the OFF-only
     calibration loader. This is sequencing, not a change to the boot default.
     Preserve an operator's early enable=0 instead of overriding it. */
  uint32_t boot_speed_requested = g_speed_run.enable;
  g_speed_run.enable = 0U;
  uint8_t board_calibration_ok = SpeedCommission_LoadBoardCalibration();
  const encoder_calibration_port_t calibration_port = {
    .start = MainEncoderCalibration_Start,
    .stop = MainEncoderCalibration_Stop,
    .backend_fault = MainEncoderCalibration_Fault,
    .now_ms = HAL_GetTick,
    .load = EncoderCalibrationStore_Load,
    .save = MainEncoderCalibration_Save,
    .apply = MainEncoderCalibration_Apply
  };
  EncoderCalibration_Init(&calibration_port, SPEED_BOARD_ENCODER_DIRECTION,
                           SPEED_BOARD_ENCODER_POLE_PAIRS);
  uint8_t stored_encoder_zero = board_calibration_ok && EncoderCalibration_Load(1U);
#if FOC_TARGET_IRQ_PROFILE
  /* Execute the real carrier ISR, feedback/outer-loop code and PWM planner
     against live ADC/encoder data, while the bridge remains physically OFF.
     This is a stationary scheduling fixture, not energized calibration. */
  g_speed_run.enable = 1U;
  g_speed_run.target_rpm = 0.0f;
  g_speed_test.active = g_speed_test.requested;
  g_speed_test.active.kind = SPEED_CONTINUOUS_KIND;
  g_speed_test.active.speed_limit_rad_s = SPEED_RUN_OVERSPEED_RAD_S;
  g_speed_test.active.current_limit_a = FLT_MAX;
  MainFoc_ConfigureCurrentStageParameters(FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP);
  g_foc_controller.algorithm.parameters.voltage_limit = SPEED_RUN_VOLTAGE_LIMIT_PU;
  g_foc_controller.algorithm.parameters.modulation.mode = SPEED_RUN_MODULATION_MODE;
  g_global_mode_control.runtime.output_armed = 1U;
  g_speed_test.state = 1U;
  FOC_ControllerStart(&g_foc_controller);
  g_foc_irq_profile.armed = 1U;
  while (1)
  {
    MptT485_Task();
    DrvMonitor_Task();
    Encoder_Service();
  }
#endif
  speed_test_config_t boot_speed_config = g_speed_test.requested;
  s_calibration_previous_speed_config = &boot_speed_config;
  /* User-selected automatic acquisition. Submit the command, never forge
     ARMED or clear fault evidence. Encoder_Service validates it while OFF. */
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  if (board_calibration_ok ||
      g_global_mode_control.active_mode == FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP)
  {
    g_encoder_logic_trigger.command = ENCODER_CAPTURE_START_COMMAND;
    /* Arm diagnostic capture before the default voltage output starts. */
    Encoder_Service();
  }
#endif
  /* Boot sequence: 1 wait feedback, 2 voltage-zero, 3 wait fresh calibrated
     feedback, 0 finished/cancelled. One attempt, never automatic retry. */
  volatile uint8_t boot_speed_pending =
      board_calibration_ok && SPEED_RUN_BOOT_AUTO_START && (boot_speed_requested == 1U) ?
          (stored_encoder_zero ? 3U : 1U) : 0U;
  uint32_t boot_speed_start_ms = HAL_GetTick();
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 1
  boot_speed_pending = 0U; /* OFF-only capture mode must never request motion. */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    foc_control_runtime_t *runtime = &g_global_mode_control.runtime;
    #if OUTPUT_AXIS_ENABLE
    uint8_t position_boot_cancelled = g_output_position.command == 2U;
    MptT485_Task();
    OutputPosition_Service(!runtime->output_request && !runtime->output_armed &&
        !runtime->precharge_in_progress && !g_foc_controller.running &&
        !Tim1Pwm_IsOutputEnabled());
    #endif
    CurrentSampling_Service();
    MainFoc_ServiceModeSwitch();
    MainUser_Service(&boot_speed_pending);
    if (boot_speed_pending)
    {
      /* Serialize completion handoff against ADC fault/state updates. */
      uint32_t boot_mask = __get_PRIMASK();
      __disable_irq();
      encoder_snapshot_t boot_encoder = {0};
      uint8_t feedback_ready = Encoder_ReadSnapshot(&boot_encoder) &&
          boot_encoder.angle_valid && boot_encoder.speed_valid &&
          isfinite(boot_encoder.mechanical_speed_rad_s) &&
          Encoder_PowerOutputPermitted();
#if OUTPUT_AXIS_ENABLE
      feedback_ready = feedback_ready && g_mpt_t485.enabled && g_mpt_t485.valid &&
          (uint32_t)(HAL_GetTick()-g_mpt_t485.last_good_ms)<=OUTPUT_POSITION_MAX_AGE_MS;
#endif
      uint8_t output_off = !runtime->output_request && !runtime->output_armed &&
          !runtime->precharge_in_progress && !g_foc_controller.running &&
          !Tim1Pwm_IsOutputEnabled();
      uint32_t wait_limit = boot_speed_pending == 2U ?
          (uint32_t)(ENCODER_CALIBRATION_MAX_SECONDS * 1000.0f) + 3000UL :
          SPEED_RUN_BOOT_READY_TIMEOUT_MS;
      if (runtime->fault_flags || g_foc_controller.fault_flags ||
#if OUTPUT_AXIS_ENABLE
          position_boot_cancelled || g_output_position.fault ||
          !g_output_position.commissioned ||
#endif
          g_speed_test.state == 4U || g_speed_test.fault ||
          g_encoder_logic_trigger.state == ENCODER_CAPTURE_FIRED ||
          g_speed_run.enable ||
          (g_speed_test.command && !EncoderCalibration_Busy()) ||
          g_encoder_calibration.state == ENCODER_CAL_FAILED ||
          (uint32_t)(HAL_GetTick() - boot_speed_start_ms) >= wait_limit)
      {
        /* Failed/cancelled boot must not fall back to the stored old zero. */
        boot_speed_pending = 0U;
        g_speed_run.enable = 0U;
        runtime->output_request = 0U;
        g_speed_test.command = 0U;
        g_speed_test.calibration_ok = 0U;
        if (!g_speed_test.fault) g_speed_test.fault = 16U; /* Boot sequence aborted. */
        g_speed_test.state = 4U;
        g_speed_run.fault = g_speed_test.fault;
        g_speed_run.state = 4U;
      }
      else if (boot_speed_pending == 1U && output_off && feedback_ready &&
               g_speed_test.state == 0U)
      {
        g_encoder_calibration.command = 1U;
        boot_speed_pending = 2U;
        boot_speed_start_ms = HAL_GetTick();
      }
      else if (boot_speed_pending == 2U && output_off &&
               g_encoder_calibration.state == ENCODER_CAL_DONE &&
               g_speed_test.state == 0U && g_speed_test.calibration_ok)
      {
        /* Service has committed the energized angle while OFF. Restore speed
           gains/config, not the voltage recipe's zero PI gains or 15 A limit. */
        g_speed_test.requested = boot_speed_config;
        g_speed_test.state = 0U;
        boot_speed_pending = 3U;
        boot_speed_start_ms = HAL_GetTick();
      }
      else if (boot_speed_pending == 3U && output_off && feedback_ready &&
               boot_encoder.electrical_angle_valid && g_speed_test.calibration_ok)
      {
        boot_speed_pending = 0U;
#if OUTPUT_AXIS_ENABLE
        /* Ordinary position arm captures the current angle and starts the
           speed inner loop at zero. Never submit the old 500 rpm default. */
        g_output_position.command = 1U;
#else
        g_speed_run.enable = 1U; /* Ordinary continuous-start checks still apply. */
#endif
      }
      if (!boot_mask) __enable_irq();
    }
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
        g_global_mode_control.current_open_loop.applied_d_voltage_check_v = 0.0f;
        g_global_mode_control.current_open_loop.applied_d_current_reference_a = 0.0f;
        g_global_mode_control.current_open_loop.applied_q_current_reference_a = 0.0f;
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
        #if OUTPUT_AXIS_ENABLE
        /* Keep output feedback fresh during the blocking neutral soak. */
        MptT485_Task();
        #endif
        if (MainFoc_IsPowerStageSafe(NULL) == 0U)
        {
          startup_safe = 0U;
          break;
        }
      }

      #if OUTPUT_AXIS_ENABLE
      if (g_output_position.state == 1U && !OutputPosition_Check())
        startup_safe = 0U;
      #endif
      if (startup_safe != 0U)
      {
        if (g_global_mode_control.active_mode == FOC_CONTROL_MODE_CURRENT_OPEN_LOOP)
        {
          g_global_mode_control.current_open_loop.electrical_angle_rad = 0.0f;
          g_global_mode_control.current_open_loop.cycle_count = 0UL;
          g_global_mode_control.current_open_loop.applied_d_voltage_check_v = 0.0f;
          g_global_mode_control.current_open_loop.applied_d_current_reference_a = 0.0f;
          g_global_mode_control.current_open_loop.applied_q_current_reference_a = 0.0f;
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
    #if !OUTPUT_AXIS_ENABLE
    WaveformUart1Port_Service();
    WaveformGenerator_Task();
    #endif
    Encoder_Service();
/* Encoder SSI is sampled by TIM1/DMA; no foreground SPI owner. */
#if MT6701_SPI3_TEST_ENABLE && !MT6701_SSI_ENABLE && !ENCODER_FAULT_TRIGGER_DIAGNOSTIC
    Mt6701Spi3Test_Task();
#endif
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
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* 外部 HSE 12 MHz / 3 = 4 MHz VCO 输入，× 84 = 336 MHz VCO，/ 2 = 168 MHz
     SYSCLK，使用板载 12 MHz 晶振。
     SYSCLK 刻意保持 168 MHz，因为所有派生时序常量都依赖它：10 kHz TIM1 载波、
     500 ns 死区、10 kHz TIM6 计数时钟（分频 16800）以及 I2C 时序字。改主频会
     连带让这些常量全部失效。 */
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    g_system_clock_source = 2U;
  }
  else
  {
    g_system_clock_source = 1U;
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

/**
  * @brief  初始化补充示波器探针，并将已停用的旧探针保持低电平。
  *         PA0/PD2 已由 MX_GPIO_Init 初始化；此处初始化 PB7。
  *         PA15/PB9 已停用，固定为低电平，避免误判。
  */
static void TimingProbeOutputs_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_7 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &gpio);
}

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
