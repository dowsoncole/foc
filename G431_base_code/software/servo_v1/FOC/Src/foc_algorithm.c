#include "foc_algorithm.h"

#include <math.h>
#include <string.h>

#define FOC_TWO_PI  6.2831853072f

static float foc_clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static uint8_t foc_modulate(foc_algorithm_t *algorithm, float d, float q,
                            float sin_angle, float cos_angle,
                            foc_modulation_output_t *modulation)
{
  return FOC_ModulationCompute(&algorithm->parameters.modulation,
                                algorithm->parameters.voltage_limit,
                                d * cos_angle - q * sin_angle,
                                d * sin_angle + q * cos_angle, modulation);
}

void FOC_AlgorithmInit(foc_algorithm_t *algorithm,
                       const foc_parameters_t *parameters)
{
  if ((algorithm == 0) || (parameters == 0))
  {
    return;
  }

  algorithm->parameters = *parameters;
  FOC_AlgorithmReset(algorithm);
}

void FOC_AlgorithmReset(foc_algorithm_t *algorithm)
{
  if (algorithm == 0)
  {
    return;
  }

  algorithm->parameters.d_controller.integral = 0.0f;
  algorithm->parameters.q_controller.integral = 0.0f;
  (void)memset(&algorithm->output, 0, sizeof(algorithm->output));
}

uint8_t FOC_InputIsValid(const foc_input_t *input)
{
  if (input == 0)
  {
    return 0U;
  }

  return ((input->current_valid != 0U) &&
          (input->angle_valid != 0U) &&
          (input->reference_valid != 0U) &&
          isfinite(input->phase_a) && isfinite(input->phase_b) &&
          isfinite(input->electrical_angle_rad) &&
          isfinite(input->d_reference) && isfinite(input->q_reference)) ? 1U : 0U;
}

uint8_t FOC_AlgorithmVoltageStep(foc_algorithm_t *algorithm,
                                 foc_voltage_open_loop_t *open_loop,
                                 float sample_period_s)
{
  float voltage_step;
  float d_voltage_pu;
  float q_voltage_pu;
  float sin_angle;
  float cos_angle;
  foc_modulation_output_t modulation;

  if ((algorithm == 0) || (open_loop == 0))
  {
    return 0U;
  }

  (void)memset(&algorithm->output, 0, sizeof(algorithm->output));
  if (!isfinite(open_loop->d_voltage_v) ||
      !isfinite(open_loop->q_voltage_v) ||
      !isfinite(open_loop->dc_bus_voltage_v) ||
      (open_loop->dc_bus_voltage_v <= 0.0f) ||
      !isfinite(open_loop->applied_d_voltage_v) ||
      !isfinite(open_loop->applied_q_voltage_v) ||
      !isfinite(open_loop->voltage_ramp_rate_v_per_s) ||
      (open_loop->voltage_ramp_rate_v_per_s < 0.0f) ||
      !isfinite(open_loop->electrical_frequency_hz) ||
      !isfinite(open_loop->electrical_angle_rad) ||
      !isfinite(sample_period_s) || (sample_period_s <= 0.0f))
  {
    return 0U;
  }

  if (open_loop->voltage_ramp_rate_v_per_s == 0.0f)
  {
    open_loop->applied_d_voltage_v = open_loop->d_voltage_v;
    open_loop->applied_q_voltage_v = open_loop->q_voltage_v;
  }
  else
  {
    voltage_step = open_loop->voltage_ramp_rate_v_per_s * sample_period_s;
    if (open_loop->applied_d_voltage_v < open_loop->d_voltage_v)
    {
      open_loop->applied_d_voltage_v = foc_clamp(
          open_loop->applied_d_voltage_v + voltage_step,
          open_loop->applied_d_voltage_v, open_loop->d_voltage_v);
    }
    else if (open_loop->applied_d_voltage_v > open_loop->d_voltage_v)
    {
      open_loop->applied_d_voltage_v = foc_clamp(
          open_loop->applied_d_voltage_v - voltage_step,
          open_loop->d_voltage_v, open_loop->applied_d_voltage_v);
    }

    if (open_loop->applied_q_voltage_v < open_loop->q_voltage_v)
    {
      open_loop->applied_q_voltage_v = foc_clamp(
          open_loop->applied_q_voltage_v + voltage_step,
          open_loop->applied_q_voltage_v, open_loop->q_voltage_v);
    }
    else if (open_loop->applied_q_voltage_v > open_loop->q_voltage_v)
    {
      open_loop->applied_q_voltage_v = foc_clamp(
          open_loop->applied_q_voltage_v - voltage_step,
          open_loop->q_voltage_v, open_loop->applied_q_voltage_v);
    }
  }
  open_loop->voltage_ramp_complete =
      ((open_loop->applied_d_voltage_v == open_loop->d_voltage_v) &&
       (open_loop->applied_q_voltage_v == open_loop->q_voltage_v)) ? 1U : 0U;

  d_voltage_pu = open_loop->applied_d_voltage_v /
                 open_loop->dc_bus_voltage_v;
  q_voltage_pu = open_loop->applied_q_voltage_v /
                 open_loop->dc_bus_voltage_v;

  sin_angle = sinf(open_loop->electrical_angle_rad);
  cos_angle = cosf(open_loop->electrical_angle_rad);
  if (foc_modulate(algorithm, d_voltage_pu, q_voltage_pu,
                   sin_angle, cos_angle, &modulation) == 0U)
  {
    return 0U;
  }

  algorithm->output.fundamental_d_voltage =
      modulation.fundamental_alpha * cos_angle +
      modulation.fundamental_beta * sin_angle;
  algorithm->output.fundamental_q_voltage =
      -modulation.fundamental_alpha * sin_angle +
      modulation.fundamental_beta * cos_angle;
  algorithm->output.alpha_voltage = modulation.alpha_voltage;
  algorithm->output.beta_voltage = modulation.beta_voltage;
  algorithm->output.d_voltage = modulation.alpha_voltage * cos_angle +
                                modulation.beta_voltage * sin_angle;
  algorithm->output.q_voltage = -modulation.alpha_voltage * sin_angle +
                                modulation.beta_voltage * cos_angle;
  algorithm->output.phase_a_voltage = modulation.alpha_voltage;
  algorithm->output.phase_b_voltage = -0.5f * modulation.alpha_voltage +
                                       0.8660254038f * modulation.beta_voltage;
  algorithm->output.phase_c_voltage = -0.5f * modulation.alpha_voltage -
                                       0.8660254038f * modulation.beta_voltage;
  algorithm->output.phase_a_duty = modulation.phase_a_duty;
  algorithm->output.phase_b_duty = modulation.phase_b_duty;
  algorithm->output.phase_c_duty = modulation.phase_c_duty;
  algorithm->output.fundamental_voltage_limit = modulation.fundamental_limit;
  algorithm->output.modulation_region = modulation.region;
  algorithm->output.sector = modulation.sector;
  algorithm->output.saturated = ((modulation.limited != 0U) ||
      (modulation.region != FOC_MODULATION_REGION_LINEAR)) ? 1U : 0U;
  algorithm->output.valid = 1U;

  open_loop->electrical_angle_rad += FOC_TWO_PI *
      open_loop->electrical_frequency_hz * sample_period_s;
  if (open_loop->electrical_angle_rad >= FOC_TWO_PI)
  {
    open_loop->electrical_angle_rad -= FOC_TWO_PI;
  }
  else if (open_loop->electrical_angle_rad < 0.0f)
  {
    open_loop->electrical_angle_rad += FOC_TWO_PI;
  }
  open_loop->cycle_count++;
  return 1U;
}

void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input)
{
  float alpha_current;
  float beta_current;
  float sin_angle;
  float cos_angle;
  float d_voltage;
  float q_voltage;
  float control_period_s;
  float d_error, q_error, old_d_integral, old_q_integral;
  float fundamental_d, fundamental_q;
  float reference_norm, reference_scale;
  foc_modulation_output_t modulation;

  if ((algorithm == 0) || (input == 0))
  {
    return;
  }

  /* 反馈无效时不进行坐标变换和 PI 计算，同时清除历史积分。 */
  if (FOC_InputIsValid(input) == 0U)
  {
    FOC_AlgorithmReset(algorithm);
    return;
  }

  control_period_s = (algorithm->parameters.sample_period_s != 0.0f) ?
                     algorithm->parameters.sample_period_s :
                     algorithm->parameters.pwm_period_s;
  if (!isfinite(control_period_s) || (control_period_s <= 0.0f) ||
      !isfinite(algorithm->parameters.current_limit) ||
      (algorithm->parameters.current_limit < 0.0f) ||
      (FOC_PI_IsValid(&algorithm->parameters.d_controller) == 0U) ||
      (FOC_PI_IsValid(&algorithm->parameters.q_controller) == 0U))
  {
    FOC_AlgorithmReset(algorithm);
    return;
  }

  /*
   * Clarke 变换：三相静止坐标系 (a,b,c) → 二相静止坐标系 (α,β)。
   *
   * 本工程采用两相电流采样，并假设无零序电流：
   *
   *     ia + ib + ic = 0
   *     ic = -(ia + ib)
   *
   * 令 α 轴与 a 相轴重合，三相轴之间相差 120°，则等幅值形式为：
   *
   *     iα = (2/3)*(ia - 0.5*ib - 0.5*ic)
   *        = ia
   *
   *     iβ = (2/3)*(sqrt(3)/2)*(ib - ic)
   *        = (ia + 2*ib) / sqrt(3)
   *
   * 因此：
   *     alpha_current = ia
   *     beta_current  = (ia + 2*ib) / sqrt(3)
   *
   * 0.5773502692f = 1/sqrt(3)。这里的 phase_a/phase_b 已经完成零点标定
   * 和物理量换算，单位为 A；d/q 反馈、给定和限幅也统一使用 A。
   *
   * 归一化注意：
   * - 当前是“等幅值”形式，使基波空间电流矢量的幅值与相电流幅值保持一致。
   * - 常见的 (2/3) 形式是另一种写法：
   *       iα = (2/3)*(ia - 0.5*ib - 0.5*ic)
   *       iβ = (2/3)*(sqrt(3)/2)*(ib - ic)
   *   在 ia+ib+ic=0 时与当前公式完全等价，仍属于等幅值归一化。
   * - 真正的“等功率”形式常写为：
   *       iα = sqrt(2/3)*(ia - 0.5*ib - 0.5*ic)
   *       iβ = sqrt(2/3)*(sqrt(3)/2)*(ib - ic)
   *   它使 α/β 坐标中的平方和与 abc 坐标中的平方和保持能量/功率一致，但在无零序时，电流矢量数值会比当前等幅值形式放大 sqrt(3/2)。
   *
   * 两种形式都可用，但必须与后续 Park 变换、反变换、PI 参数、
   * 电流限幅和功率计算使用同一套归一化。切换归一化时，电流参考和
   * 反馈、PI 增益、限幅阈值都要同比换算，不能只改这里的一个系数。
   */
  alpha_current = input->phase_a;
  beta_current = (input->phase_a +
                  (2.0f * input->phase_b)) * 0.5773502692f;
  sin_angle = sinf(input->electrical_angle_rad);
  cos_angle = cosf(input->electrical_angle_rad);

  algorithm->output.d_current = (alpha_current * cos_angle) +
                                (beta_current * sin_angle);
  algorithm->output.q_current = (-alpha_current * sin_angle) +
                                (beta_current * cos_angle);
  algorithm->output.saturated = 0U;

  old_d_integral = algorithm->parameters.d_controller.integral;
  old_q_integral = algorithm->parameters.q_controller.integral;
  /* Limit vector magnitude; independent clamps allowed sqrt(2)*Imax. */
  reference_norm = hypotf(input->d_reference, input->q_reference);
  if (!isfinite(reference_norm)) { FOC_AlgorithmReset(algorithm); return; }
  algorithm->output.reference_limited = reference_norm > algorithm->parameters.current_limit;
  reference_scale = algorithm->output.reference_limited ?
      algorithm->parameters.current_limit / reference_norm : 1.0f;
  algorithm->output.applied_d_reference = input->d_reference * reference_scale;
  algorithm->output.applied_q_reference = input->q_reference * reference_scale;
  d_error = algorithm->output.applied_d_reference - algorithm->output.d_current;
  q_error = algorithm->output.applied_q_reference - algorithm->output.q_current;
  d_voltage = FOC_PI_Step(&algorithm->parameters.d_controller,
                          d_error, control_period_s);
  q_voltage = FOC_PI_Step(&algorithm->parameters.q_controller,
                          q_error, control_period_s);
  if (foc_modulate(algorithm, d_voltage, q_voltage, sin_angle, cos_angle,
                    &modulation) == 0U)
  {
    FOC_AlgorithmReset(algorithm);
    return;
  }
  fundamental_d = modulation.fundamental_alpha * cos_angle +
                    modulation.fundamental_beta * sin_angle;
  fundamental_q = -modulation.fundamental_alpha * sin_angle +
                    modulation.fundamental_beta * cos_angle;

  /* 条件积分：若本次积分增量把目标推向基波能力之外，撤销本次积分。
     不能用每周期的畸变电压回算积分，否则会抵消 OVM 需要的谐波。
     向饱和区内回退的积分仍允许；积分自身的上下限继续有效。 */
  if ((modulation.limited != 0U) &&
      (((algorithm->parameters.d_controller.integral - old_d_integral) *
          (d_voltage - fundamental_d) +
        (algorithm->parameters.q_controller.integral - old_q_integral) *
          (q_voltage - fundamental_q)) > 0.0f))
  {
    algorithm->parameters.d_controller.integral = old_d_integral;
    algorithm->parameters.q_controller.integral = old_q_integral;
    d_voltage = algorithm->parameters.d_controller.kp * d_error + old_d_integral;
    q_voltage = algorithm->parameters.q_controller.kp * q_error + old_q_integral;
    if (foc_modulate(algorithm, d_voltage, q_voltage, sin_angle, cos_angle,
                      &modulation) == 0U)
    {
      FOC_AlgorithmReset(algorithm);
      return;
    }
  }

  algorithm->output.integral_limited =
      algorithm->parameters.d_controller.integral <= algorithm->parameters.d_controller.integral_min ||
      algorithm->parameters.d_controller.integral >= algorithm->parameters.d_controller.integral_max ||
      algorithm->parameters.q_controller.integral <= algorithm->parameters.q_controller.integral_min ||
      algorithm->parameters.q_controller.integral >= algorithm->parameters.q_controller.integral_max;
  algorithm->output.fundamental_d_voltage = modulation.fundamental_alpha * cos_angle +
                                             modulation.fundamental_beta * sin_angle;
  algorithm->output.fundamental_q_voltage = -modulation.fundamental_alpha * sin_angle +
                                             modulation.fundamental_beta * cos_angle;
  algorithm->output.alpha_voltage = modulation.alpha_voltage;
  algorithm->output.beta_voltage = modulation.beta_voltage;
  algorithm->output.d_voltage = modulation.alpha_voltage * cos_angle +
                                 modulation.beta_voltage * sin_angle;
  algorithm->output.q_voltage = -modulation.alpha_voltage * sin_angle +
                                 modulation.beta_voltage * cos_angle;
  algorithm->output.phase_a_voltage = modulation.alpha_voltage;
  algorithm->output.phase_b_voltage = -0.5f * modulation.alpha_voltage +
                                       0.8660254038f * modulation.beta_voltage;
  algorithm->output.phase_c_voltage = -0.5f * modulation.alpha_voltage -
                                       0.8660254038f * modulation.beta_voltage;
  algorithm->output.phase_a_duty = modulation.phase_a_duty;
  algorithm->output.phase_b_duty = modulation.phase_b_duty;
  algorithm->output.phase_c_duty = modulation.phase_c_duty;
  algorithm->output.fundamental_voltage_limit = modulation.fundamental_limit;
  algorithm->output.modulation_region = modulation.region;
  algorithm->output.sector = modulation.sector;
  algorithm->output.saturated = ((modulation.limited != 0U) ||
      (modulation.region != FOC_MODULATION_REGION_LINEAR)) ? 1U : 0U;
  algorithm->output.valid = 1U;
}
