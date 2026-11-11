#include "foc_algorithm.h"

#include <math.h>
#include <string.h>

#define FOC_TWO_PI  6.2831853072f

/* Bounded single-precision Taylor polynomial after quadrant reduction.
   The degree-11 sine remainder on [-pi/2,pi/2] is <6e-8 (before rounding).
   Avoid generic libm argument reduction in the 20 kHz rotating-frame path. */
static float foc_sine_reduced(float x)
{
  if (x > 3.141592654f) x -= FOC_TWO_PI;
  if (x > 1.570796327f) x = 3.141592654f - x;
  else if (x < -1.570796327f) x = -3.141592654f - x;
  float z=x*x;
  return x*(1.0f+z*(-1.0f/6.0f+z*(1.0f/120.0f+z*(-1.0f/5040.0f+
         z*(1.0f/362880.0f-z/39916800.0f)))));
}
static void foc_sincos(float x,float *s,float *c)
{
  if(x<0.0f || x>=FOC_TWO_PI) x=fmodf(x,FOC_TWO_PI);
  if(x<0.0f) x+=FOC_TWO_PI;
  *s=foc_sine_reduced(x);
  *c=foc_sine_reduced(x+1.570796327f);
}

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

uint8_t FOC_TransformCurrentDQ(float phase_a, float phase_b,
                               float electrical_angle_rad,
                               float *d_current, float *q_current)
{
  float alpha_current;
  float beta_current;
  float sin_angle;
  float cos_angle;

  if ((d_current == 0) || (q_current == 0) ||
      !isfinite(phase_a) || !isfinite(phase_b) ||
      !isfinite(electrical_angle_rad))
  {
    return 0U;
  }
  alpha_current = phase_a;
  beta_current = (phase_a + (2.0f * phase_b)) * 0.5773502692f;
  foc_sincos(electrical_angle_rad, &sin_angle, &cos_angle);
  *d_current = (alpha_current * cos_angle) +
               (beta_current * sin_angle);
  *q_current = (-alpha_current * sin_angle) +
               (beta_current * cos_angle);
  return 1U;
}

static uint8_t foc_voltage_command_step(foc_algorithm_t *algorithm,
                                        float d_voltage_v,
                                        float q_voltage_v,
                                        float dc_bus_voltage_v,
                                        float electrical_angle_rad,
                                        const foc_input_t *feedback)
{
  float sin_angle;
  float cos_angle;
  foc_modulation_output_t modulation;

  if ((algorithm == 0) || !isfinite(d_voltage_v) ||
      !isfinite(q_voltage_v) || !isfinite(dc_bus_voltage_v) ||
      (dc_bus_voltage_v <= 0.0f) || !isfinite(electrical_angle_rad))
  {
    return 0U;
  }

  (void)memset(&algorithm->output, 0, sizeof(algorithm->output));
  foc_sincos(electrical_angle_rad, &sin_angle, &cos_angle);
  if (feedback != 0)
  {
    float beta = (feedback->phase_a + 2.0f * feedback->phase_b) * 0.5773502692f;
    algorithm->output.d_current = feedback->phase_a * cos_angle + beta * sin_angle;
    algorithm->output.q_current = -feedback->phase_a * sin_angle + beta * cos_angle;
  }
  if (foc_modulate(algorithm, d_voltage_v / dc_bus_voltage_v,
                   q_voltage_v / dc_bus_voltage_v,
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
  return 1U;
}

uint8_t FOC_AlgorithmVoltageCommandStep(foc_algorithm_t *algorithm,
                                        float d_voltage_v,
                                        float q_voltage_v,
                                        float dc_bus_voltage_v,
                                        float electrical_angle_rad)
{
  return foc_voltage_command_step(algorithm, d_voltage_v, q_voltage_v,
                                  dc_bus_voltage_v, electrical_angle_rad, 0);
}

uint8_t FOC_AlgorithmVoltageFeedbackStep(foc_algorithm_t *algorithm,
                                         float d_voltage_v,
                                         float q_voltage_v,
                                         float dc_bus_voltage_v,
                                         const foc_input_t *feedback)
{
  if (algorithm == 0) return 0U;
  algorithm->output.valid = 0U;
  if (feedback == 0 || !isfinite(feedback->phase_a) ||
      !isfinite(feedback->phase_b)) return 0U;
  return foc_voltage_command_step(algorithm, d_voltage_v, q_voltage_v,
                                  dc_bus_voltage_v, feedback->electrical_angle_rad,
                                  feedback);
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

  if (FOC_AlgorithmVoltageCommandStep(algorithm,
        open_loop->applied_d_voltage_v, open_loop->applied_q_voltage_v,
        open_loop->dc_bus_voltage_v, open_loop->electrical_angle_rad) == 0U)
  {
    return 0U;
  }

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
      (algorithm->parameters.current_limit < 0.0f))
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
  foc_sincos(input->electrical_angle_rad, &sin_angle, &cos_angle);

  /* Inputs were validated above; reuse sin/cos for Park and inverse Park. */
  float beta_current = (input->phase_a + 2.0f * input->phase_b) * 0.5773502692f;
  algorithm->output.d_current = input->phase_a * cos_angle + beta_current * sin_angle;
  algorithm->output.q_current = -input->phase_a * sin_angle + beta_current * cos_angle;
  algorithm->output.saturated = 0U;

  old_d_integral = algorithm->parameters.d_controller.integral;
  old_q_integral = algorithm->parameters.q_controller.integral;
  /* Limit vector magnitude; independent clamps allowed sqrt(2)*Imax. */
  /* Scaled norm avoids overflow without generic hypotf library dispatch. */
  float ref_d=fabsf(input->d_reference), ref_q=fabsf(input->q_reference);
  float ref_hi=ref_d>ref_q ? ref_d:ref_q;
  float ref_lo=ref_d>ref_q ? ref_q:ref_d;
  if (ref_lo == 0.0f) reference_norm = ref_hi;
  else {
    float ratio = ref_lo / ref_hi;
    reference_norm = ref_hi * sqrtf(1.0f + ratio * ratio);
  }
  if (!isfinite(reference_norm)) { FOC_AlgorithmReset(algorithm); return; }
  algorithm->output.reference_limited = reference_norm > algorithm->parameters.current_limit;
  reference_scale = algorithm->output.reference_limited ?
      algorithm->parameters.current_limit / reference_norm : 1.0f;
  algorithm->output.applied_d_reference = input->d_reference * reference_scale;
  algorithm->output.applied_q_reference = input->q_reference * reference_scale;
  d_error = algorithm->output.applied_d_reference - algorithm->output.d_current;
  q_error = algorithm->output.applied_q_reference - algorithm->output.q_current;
  algorithm->output.d_current_error = d_error;
  algorithm->output.q_current_error = q_error;
  /* Validate each PI once, retaining explicit failure propagation. */
  if (!FOC_PI_TryStep(&algorithm->parameters.d_controller,
                      d_error, control_period_s, &d_voltage) ||
      !FOC_PI_TryStep(&algorithm->parameters.q_controller,
                      q_error, control_period_s, &q_voltage))
  {
    FOC_AlgorithmReset(algorithm);
    return;
  }
  algorithm->output.d_pi_command = d_voltage;
  algorithm->output.q_pi_command = q_voltage;
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
     向饱和区内回退的积分仍允许；积分自身的上下限继续有效。
     本周期使用已限幅的候选输出，撤销的积分在下一周期生效。这是有界的
     条件积分离散实现，避免饱和时再次运行整个 OVM 查表/调制而错过 PWM
     截止点。d/q_pi_command 保留生成本周期输出的候选值。 */
  if ((modulation.limited != 0U) &&
      (((algorithm->parameters.d_controller.integral - old_d_integral) *
          (d_voltage - fundamental_d) +
        (algorithm->parameters.q_controller.integral - old_q_integral) *
          (q_voltage - fundamental_q)) > 0.0f))
  {
    algorithm->parameters.d_controller.integral = old_d_integral;
    algorithm->parameters.q_controller.integral = old_q_integral;
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
