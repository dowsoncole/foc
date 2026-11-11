#include "foc_pi.h"
#include "foc_pi_inline.h"

#include <math.h>
#include <stddef.h>

uint8_t FOC_PI_IsValid(const foc_pi_t *controller)
{
  return FocPi_IsValidInline(controller);
}

uint8_t FOC_PI_TryStep(foc_pi_t *controller, float error, float period_s, float *output)
{
  return FocPi_TryStepInline(controller, error, period_s, output);
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
