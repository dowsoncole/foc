#include "foc_algorithm.h"

#include <math.h>
#include <string.h>

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

static float foc_pi_step(foc_pi_t *controller, float error, float period)
{
  float integral = controller->integral + error * controller->ki * period;
  float output;

  controller->integral = foc_clamp(integral, controller->integral_min,
                                   controller->integral_max);
  output = controller->kp * error + controller->integral;
  return foc_clamp(output, controller->integral_min, controller->integral_max);
}

static void foc_limit_voltage(float *d_voltage, float *q_voltage, float limit,
                              uint8_t *saturated)
{
  float magnitude = sqrtf((*d_voltage * *d_voltage) +
                          (*q_voltage * *q_voltage));
  float scale;

  if (magnitude <= limit || magnitude <= 0.0f)
  {
    return;
  }

  scale = limit / magnitude;
  *d_voltage *= scale;
  *q_voltage *= scale;
  *saturated = 1U;
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

  if ((algorithm == 0) || (input == 0))
  {
    return;
  }

  alpha_current = input->phase_a;
  beta_current = (input->phase_a + (2.0f * input->phase_b)) * 0.5773502692f;
  sin_angle = sinf(input->electrical_angle_rad);
  cos_angle = cosf(input->electrical_angle_rad);

  algorithm->output.d_current = (alpha_current * cos_angle) +
                                (beta_current * sin_angle);
  algorithm->output.q_current = (-alpha_current * sin_angle) +
                                (beta_current * cos_angle);
  algorithm->output.saturated = 0U;

  if (input->enabled == 0U)
  {
    FOC_AlgorithmReset(algorithm);
    return;
  }

  control_period_s = (algorithm->parameters.pwm_period_s > 0.0f) ?
                     algorithm->parameters.pwm_period_s :
                     algorithm->parameters.sample_period_s;
  d_voltage = foc_pi_step(&algorithm->parameters.d_controller,
                          foc_clamp(input->d_reference,
                                    -algorithm->parameters.current_limit,
                                    algorithm->parameters.current_limit) -
                          algorithm->output.d_current,
                          control_period_s);
  q_voltage = foc_pi_step(&algorithm->parameters.q_controller,
                          foc_clamp(input->q_reference,
                                    -algorithm->parameters.current_limit,
                                    algorithm->parameters.current_limit) -
                          algorithm->output.q_current,
                          control_period_s);
  foc_limit_voltage(&d_voltage, &q_voltage,
                    algorithm->parameters.voltage_limit,
                    &algorithm->output.saturated);

  algorithm->output.d_voltage = d_voltage;
  algorithm->output.q_voltage = q_voltage;
  algorithm->output.alpha_voltage = (d_voltage * cos_angle) -
                                    (q_voltage * sin_angle);
  algorithm->output.beta_voltage = (d_voltage * sin_angle) +
                                   (q_voltage * cos_angle);
  algorithm->output.phase_a_voltage = algorithm->output.alpha_voltage;
  algorithm->output.phase_b_voltage = (-0.5f * algorithm->output.alpha_voltage) +
                                      (0.8660254038f * algorithm->output.beta_voltage);
  algorithm->output.phase_c_voltage = (-0.5f * algorithm->output.alpha_voltage) -
                                      (0.8660254038f * algorithm->output.beta_voltage);
}
