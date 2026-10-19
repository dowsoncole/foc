#include "foc_algorithm.h"
#include "foc_interface.h"
#include "servo_manager.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static foc_modulation_parameters_t full_bus = { FOC_MODULATION_OVERMODULATION, 0.0f, 1.0f };
static double worst_fundamental_error;
void TestPwmSampling(void);
void TestWaveformGenerator(void);

static foc_modulation_output_t compute(foc_modulation_parameters_t parameters,
                                       float amplitude, double angle)
{
  foc_modulation_output_t output;
  CHECK(FOC_ModulationCompute(&parameters, 1.0f,
          amplitude * (float)cos(angle), amplitude * (float)sin(angle), &output) != 0U);
  CHECK(output.valid != 0U);
  CHECK(output.phase_a_duty >= parameters.duty_min && output.phase_a_duty <= parameters.duty_max);
  CHECK(output.phase_b_duty >= parameters.duty_min && output.phase_b_duty <= parameters.duty_max);
  CHECK(output.phase_c_duty >= parameters.duty_min && output.phase_c_duty <= parameters.duty_max);
  CHECK(output.sector >= 1U && output.sector <= 6U);
  return output;
}

static void test_linear_and_sectors(void)
{
  int i;
  for (i = 0; i < 7200; ++i)
  {
    double angle = (i + 0.25) * 2.0 * PI / 7200.0;
    foc_modulation_output_t output = compute(full_bus, 0.4f, angle);
    CHECK(output.region == FOC_MODULATION_REGION_LINEAR);
    CHECK(output.sector == (uint8_t)(1 + (int)(angle / (PI / 3.0))));
    CHECK(fabs(output.alpha_voltage - 0.4 * cos(angle)) < 2.0e-7);
    CHECK(fabs(output.beta_voltage - 0.4 * sin(angle)) < 2.0e-7);
    /* 相电压差只由目标 αβ 决定，零序注入不能改变线电压。 */
    CHECK(fabs((output.phase_a_duty - output.phase_b_duty) -
        (0.6 * cos(angle) - 0.3464101615 * sin(angle))) < 3.0e-7);
  }
}

/* 独立 Fourier 测量：直接从 duty 重构相电压，不相信模块的遥测字段。 */
static void check_fundamental(foc_modulation_parameters_t parameters, float target)
{
  const int count = 32768;
  double in_phase = 0.0, quadrature = 0.0, harmonic5 = 0.0;
  double span = parameters.duty_max - parameters.duty_min;
  double expected = fmin(target, span * (double)FOC_MODULATION_SIX_STEP_MAX);
  int i;
  for (i = 0; i < count; ++i)
  {
    double angle = (i + 0.5) * 2.0 * PI / count;
    foc_modulation_output_t output = compute(parameters, target, angle);
    double alpha = (2.0 * output.phase_a_duty - output.phase_b_duty - output.phase_c_duty) / 3.0;
    double beta = (output.phase_b_duty - output.phase_c_duty) / sqrt(3.0);
    in_phase += alpha * cos(angle) + beta * sin(angle);
    quadrature += beta * cos(angle) - alpha * sin(angle);
    harmonic5 += alpha * cos(5.0 * angle) - beta * sin(5.0 * angle);
  }
  in_phase /= count;
  quadrature /= count;
  harmonic5 /= count;
  if (fabs(in_phase - expected) > worst_fundamental_error)
    worst_fundamental_error = fabs(in_phase - expected);
  CHECK(fabs(in_phase - expected) < 5.0e-5);
  CHECK(fabs(quadrature) < 2.0e-6);
  if (target > (float)(span * 0.625)) CHECK(fabs(harmonic5) > span * 0.01);
}

static void test_overmodulation_and_bounds(void)
{
  foc_modulation_parameters_t derated = { FOC_MODULATION_OVERMODULATION, 0.02f, 0.73f };
  foc_modulation_parameters_t linear = { FOC_MODULATION_LINEAR_ONLY, 0.0f, 1.0f };
  int i, j;
  for (i = 0; i <= 80; ++i)
  {
    float amplitude = 0.57f + 0.07f * (float)i / 80.0f;
    check_fundamental(full_bus, amplitude);
  }
  check_fundamental(derated, 0.71f * 0.60f);
  check_fundamental(derated, 0.71f * FOC_MODULATION_SIX_STEP_MAX);
  check_fundamental(derated, 0.9f);
  CHECK(compute(full_bus, 0.59f, 0.2).region == FOC_MODULATION_REGION_OVM1);
  CHECK(compute(full_bus, 0.62f, 0.2).region == FOC_MODULATION_REGION_OVM2);
  CHECK(compute(full_bus, 1.0f, 0.2).region == FOC_MODULATION_REGION_SIX_STEP);
  CHECK(compute(linear, 0.9f, 0.2).region == FOC_MODULATION_REGION_LINEAR);
  CHECK(compute(linear, 0.9f, 0.2).limited != 0U);
  for (i = 0; i < 6; ++i)
  {
    foc_modulation_output_t output = compute(full_bus, 1.0f, i * PI / 3.0);
    float a = output.phase_a_duty, b = output.phase_b_duty, c = output.phase_c_duty;
    CHECK((a == 0.0f || a == 1.0f) && (b == 0.0f || b == 1.0f) && (c == 0.0f || c == 1.0f));
    CHECK(a + b + c >= 1.0f && a + b + c <= 2.0f);
  }
  /* 区域边界、六个扇区边界及正反方向都连续（六步换相点除外）。 */
  for (j = 0; j < 2; ++j)
  {
    float boundary = (j == 0) ? FOC_MODULATION_LINEAR_MAX : 0.6056966996f;
    for (i = 0; i < 720; ++i)
    {
      double angle = i * PI / 360.0;
      foc_modulation_output_t before = compute(full_bus, boundary - 1.0e-6f, angle);
      foc_modulation_output_t after = compute(full_bus, boundary + 1.0e-6f, angle);
      CHECK(fabs(before.alpha_voltage - after.alpha_voltage) < 0.001);
      CHECK(fabs(before.beta_voltage - after.beta_voltage) < 0.001);
    }
  }
  for (i = 0; i < 6; ++i)
  {
    foc_modulation_output_t before = compute(full_bus, 0.625f, i * PI / 3.0 - 1.0e-6);
    foc_modulation_output_t after = compute(full_bus, 0.625f, i * PI / 3.0 + 1.0e-6);
    CHECK(fabs(before.alpha_voltage - after.alpha_voltage) < 2.0e-5);
    CHECK(fabs(before.beta_voltage - after.beta_voltage) < 2.0e-5);
  }
}

static void test_invalid(void)
{
  foc_modulation_output_t output;
  foc_modulation_parameters_t invalid = full_bus;
  CHECK(FOC_ModulationCompute(&full_bus, 1.0f, NAN, 0.0f, &output) == 0U && !output.valid);
  CHECK(FOC_ModulationCompute(&full_bus, 1.0f, 0.0f, INFINITY, &output) == 0U);
  CHECK(FOC_ModulationCompute(&full_bus, 1.0f, FLT_MAX, 0.0f, &output) == 0U);
  CHECK(FOC_ModulationCompute(&full_bus, NAN, 0.0f, 0.0f, &output) == 0U);
  CHECK(FOC_ModulationCompute(&full_bus, -1.0f, 0.0f, 0.0f, &output) == 0U);
  invalid.duty_min = invalid.duty_max;
  CHECK(FOC_ModulationCompute(&invalid, 1.0f, 0.0f, 0.0f, &output) == 0U);
  invalid.duty_min = 0.0f;
  invalid.duty_max = 1.0e-20f;
  CHECK(FOC_ModulationCompute(&invalid, 1.0f, 0.0f, 0.0f, &output) == 0U);
  CHECK(FOC_ModulationCompute(NULL, 1.0f, 0.0f, 0.0f, &output) == 0U);
  CHECK(FOC_ModulationCompute(&full_bus, 1.0f, 0.0f, 0.0f, NULL) == 0U);
  output = compute(full_bus, 0.0f, 0.0);
  CHECK(output.phase_a_duty == 0.5f && output.phase_b_duty == 0.5f && output.phase_c_duty == 0.5f);
}

static foc_parameters_t parameters(void)
{
  foc_parameters_t p;
  memset(&p, 0, sizeof(p));
  p.pwm_period_s = 0.00005f;
  p.voltage_limit = FOC_MODULATION_SIX_STEP_MAX;
  p.modulation = full_bus;
  p.current_limit = 100.0f;
  p.d_controller.kp = 0.1f;
  p.d_controller.ki = 5.0f;
  p.d_controller.integral_min = -1.0f;
  p.d_controller.integral_max = 1.0f;
  p.q_controller = p.d_controller;
  return p;
}

static foc_input_t feedback;
static int disable_count, write_count;
static uint8_t write_ok = 1U;
static uint8_t safe = 1U;
static uint8_t is_safe(void *context) { (void)context; return safe; }
static void read_feedback(void *context, foc_input_t *input) { (void)context; *input = feedback; }
static uint8_t write_voltage(void *context, const foc_output_t *output)
{
  (void)context;
  CHECK(output != NULL);
  ++write_count;
  return write_ok;
}
static void disable_output(void *context) { (void)context; ++disable_count; }
static uint8_t is_true(void *context) { (void)context; return 1U; }
static uint8_t is_false(void *context) { (void)context; return 0U; }
static void no_action(void *context) { (void)context; }

static void test_foc_and_faults(void)
{
  foc_parameters_t p = parameters();
  foc_algorithm_t algorithm;
  foc_voltage_open_loop_t open_loop =
  {
    .d_voltage_v = 0.2f,
    .q_voltage_v = 0.0f,
    .dc_bus_voltage_v = 1.0f,
    .electrical_frequency_hz = 2.0f,
    .electrical_angle_rad = 0.0f,
    .cycle_count = 0U,
    .applied_d_voltage_v = 0.0f,
    .applied_q_voltage_v = 0.0f,
    .voltage_ramp_rate_v_per_s = 0.0f,
    .voltage_ramp_complete = 0U
  };
  foc_controller_t controller;
  foc_port_t port = { is_safe, read_feedback, write_voltage, disable_output, NULL };
  int i;
  memset(&feedback, 0, sizeof(feedback));
  feedback.current_valid = feedback.angle_valid = feedback.reference_valid = 1U;
  feedback.q_reference = 100.0f;
  FOC_AlgorithmInit(&algorithm, &p);
  CHECK(FOC_AlgorithmVoltageStep(&algorithm, &open_loop, 0.00005f) != 0U);
  CHECK(algorithm.output.valid != 0U);
  CHECK(fabs(algorithm.output.fundamental_d_voltage - 0.2f) < 1.0e-6);
  CHECK(fabs(algorithm.output.fundamental_q_voltage) < 1.0e-6);
  CHECK(fabs(algorithm.output.phase_a_duty - 0.65f) < 1.0e-6);
  CHECK(fabs(algorithm.output.phase_b_duty - 0.35f) < 1.0e-6);
  CHECK(fabs(algorithm.output.phase_c_duty - 0.35f) < 1.0e-6);
  open_loop.electrical_angle_rad = (float)(PI / 2.0);
  CHECK(FOC_AlgorithmVoltageStep(&algorithm, &open_loop, 0.00005f) != 0U);
  CHECK(fabs(algorithm.output.fundamental_d_voltage - 0.2f) < 1.0e-6);
  CHECK(fabs(algorithm.output.fundamental_q_voltage) < 1.0e-6);
  open_loop.d_voltage_v = NAN;
  CHECK(FOC_AlgorithmVoltageStep(&algorithm, &open_loop, 0.00005f) == 0U);
  CHECK(algorithm.output.valid == 0U);
  for (i = 0; i < 10000; ++i) FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(algorithm.output.valid && algorithm.output.saturated);
  CHECK(fabs(algorithm.parameters.q_controller.integral) < 1.0e-7);
  CHECK(fabs(algorithm.output.fundamental_q_voltage - FOC_MODULATION_SIX_STEP_MAX) < 1.0e-6);
  /* 无饱和时允许积分；进入 OVM 并不意味着必须冻结积分。 */
  feedback.q_reference = 0.1f;
  FOC_AlgorithmReset(&algorithm);
  FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(algorithm.parameters.q_controller.integral > 0.0f);
  algorithm.parameters.sample_period_s = 0.0001f;
  FOC_AlgorithmReset(&algorithm);
  FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(fabs(algorithm.parameters.q_controller.integral - 0.00005f) < 1.0e-9);
  algorithm.parameters.sample_period_s = 0.0f;
  /* 已有饱和积分向内退回时仍可更新。 */
  algorithm.parameters.q_controller.integral = 0.9f;
  feedback.q_reference = -0.1f;
  FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(algorithm.parameters.q_controller.integral < 0.9f);
  CHECK(algorithm.parameters.q_controller.integral >= p.q_controller.integral_min);
  feedback.angle_valid = 0U;
  FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(!algorithm.output.valid && algorithm.parameters.q_controller.integral == 0.0f);
  feedback.angle_valid = 1U;
  feedback.q_reference = 6.2f;
  FOC_ControllerInit(&controller, &p, &port);
  FOC_ControllerStart(&controller);
  FOC_ControllerStep(&controller);
  CHECK(controller.feedback_ready && write_count == 1);
  CHECK(controller.algorithm.output.modulation_region == FOC_MODULATION_REGION_OVM2);
  write_ok = 0U;
  FOC_ControllerStep(&controller);
  CHECK(!controller.running && !controller.feedback_ready && disable_count == 1);
  CHECK(controller.fault_flags & FOC_FAULT_PWM_WRITE);
  write_ok = 1U;
  FOC_ControllerClearFaults(&controller);
  FOC_ControllerStart(&controller);
  controller.algorithm.parameters.modulation.duty_max = 0.0f;
  FOC_ControllerStep(&controller);
  CHECK(controller.fault_flags & FOC_FAULT_ALGORITHM);
  CHECK(!controller.running && !controller.feedback_ready && disable_count == 2);
  FOC_ControllerInit(&controller, &p, &port);
  FOC_ControllerStart(&controller);
  feedback.electrical_angle_rad = NAN;
  FOC_ControllerStep(&controller);
  CHECK(controller.fault_flags & FOC_FAULT_ALGORITHM);
  CHECK(disable_count == 3);
  feedback.electrical_angle_rad = 0.0f;
  FOC_ControllerInit(&controller, &p, &port);
  FOC_ControllerStart(&controller);
  feedback.current_valid = 0U;
  FOC_ControllerStep(&controller);
  CHECK(controller.fault_flags & FOC_FAULT_CURRENT_INVALID);
  CHECK(disable_count == 4);
  feedback.current_valid = 1U;
  FOC_ControllerInit(&controller, &p, &port);
  FOC_ControllerStart(&controller);
  safe = 0U;
  FOC_ControllerStep(&controller);
  CHECK(controller.fault_flags & FOC_FAULT_POWER_STAGE);
  CHECK(disable_count == 5);
  /* 积分自身限幅独立于输出电压，正负两端都应准确钳位。 */
  p.q_controller.kp = 0.0f;
  p.q_controller.integral_min = -0.01f;
  p.q_controller.integral_max = 0.01f;
  FOC_AlgorithmInit(&algorithm, &p);
  feedback.q_reference = 1.0f;
  for (i = 0; i < 1000; ++i) FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(algorithm.parameters.q_controller.integral == 0.01f);
  feedback.q_reference = -1.0f;
  for (i = 0; i < 1000; ++i) FOC_AlgorithmStep(&algorithm, &feedback);
  CHECK(algorithm.parameters.q_controller.integral == -0.01f);
}

static void test_servo_fault_mapping(void)
{
  foc_parameters_t p = parameters();
  foc_controller_t controller;
  servo_manager_t manager;
  foc_port_t port = { is_safe, read_feedback, write_voltage, disable_output, NULL };
  servo_manager_port_t servo_port = { is_true, is_false, is_true, is_false,
      no_action, no_action, disable_output, NULL, NULL };
  FOC_ControllerInit(&controller, &p, &port);
  ServoManager_Init(&manager, &controller, &servo_port);
  controller.fault_flags = FOC_FAULT_ALGORITHM | FOC_FAULT_PWM_WRITE;
  ServoManager_Service(&manager);
  CHECK(ServoManager_GetState(&manager) == SERVO_STATE_FAULT);
  CHECK(ServoManager_GetLatchedFaults(&manager) ==
      (SERVO_FAULT_FOC_ALGORITHM | SERVO_FAULT_PWM_WRITE));
  /* 只有显式复位且 Servo OFF、恢复条件有效时才允许清锁存。 */
  ServoManager_RequestFaultReset(&manager);
  ServoManager_Service(&manager);
  CHECK(ServoManager_GetState(&manager) == SERVO_STATE_READY);
  CHECK(ServoManager_GetLatchedFaults(&manager) == SERVO_FAULT_NONE);
}

int main(void)
{
  void TestCurrentLoop(void);
  TestCurrentLoop();
  test_linear_and_sectors();
  test_overmodulation_and_bounds();
  test_invalid();
  test_foc_and_faults();
  test_servo_fault_mapping();
  TestPwmSampling();
  TestWaveformGenerator();
  printf("PASS: linear/OVM1/OVM2/six-step, six sectors, Fourier gain, duty bounds, PI and faults.\n");
  printf("Worst fundamental error / Vdc: %.9f\n", worst_fundamental_error);
  printf("Six-step / linear fundamental gain: %.7f (+%.4f%%)\n",
      (2.0 / PI) / (1.0 / sqrt(3.0)), ((2.0 / PI) / (1.0 / sqrt(3.0)) - 1.0) * 100.0);
  return 0;
}
