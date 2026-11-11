#ifndef FOC_PI_H
#define FOC_PI_H

#include <stdint.h>

typedef struct
{
  float kp;
  float ki;
  float integral;
  float integral_min;
  float integral_max;
} foc_pi_t;

uint8_t FOC_PI_IsValid(const foc_pi_t *controller);
/* Validate and update once. Failure is distinguishable from a valid zero output. */
uint8_t FOC_PI_TryStep(foc_pi_t *controller, float error, float period_s, float *output);
float FOC_PI_Step(foc_pi_t *controller, float error, float period_s);
void FOC_PI_Reset(foc_pi_t *controller);

#endif /* FOC_PI_H */
