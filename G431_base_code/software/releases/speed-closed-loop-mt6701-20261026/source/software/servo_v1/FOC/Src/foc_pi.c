#include "foc_pi.h"

#include <math.h>
#include <stddef.h>

static float FOC_PI_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

uint8_t FOC_PI_IsValid(const foc_pi_t *controller)
{
  if (controller == NULL) return 0U;
  return (isfinite(controller->kp) && (controller->kp >= 0.0f) &&
          isfinite(controller->ki) && (controller->ki >= 0.0f) &&
          isfinite(controller->integral) &&
          isfinite(controller->integral_min) &&
          isfinite(controller->integral_max) &&
          (controller->integral_min <= 0.0f) &&
          (controller->integral_max >= 0.0f) &&
          (controller->integral_min <= controller->integral_max)) ? 1U : 0U;
}

uint8_t FOC_PI_TryStep(foc_pi_t *controller, float error, float period_s, float *output)
{
  float integral;
  if (output == NULL) return 0U;
  *output = 0.0f;
  if ((FOC_PI_IsValid(controller) == 0U) || !isfinite(error) ||
      !isfinite(period_s) || (period_s <= 0.0f)) return 0U;

  integral = controller->integral + error * controller->ki * period_s;
  controller->integral = FOC_PI_Clamp(integral,
      controller->integral_min, controller->integral_max);
  *output = controller->kp * error + controller->integral;
  return 1U;
}

float FOC_PI_Step(foc_pi_t *controller, float error, float period_s)
{
  float output = 0.0f;
  (void)FOC_PI_TryStep(controller, error, period_s, &output);
  return output;
}

void FOC_PI_Reset(foc_pi_t *controller)
{
  if (controller != NULL) controller->integral = 0.0f;
}
