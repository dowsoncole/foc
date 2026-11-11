#ifndef FOC_PI_INLINE_H
#define FOC_PI_INLINE_H

#include "foc_pi.h"
#include <math.h>
#include <stddef.h>

/* Shared implementation for the public PI API and the carrier-rate caller.
   Keep validation and arithmetic ordering identical; allow the FOC compiler
   to avoid two cross-object calls and the associated live-register spills. */
#if defined(__GNUC__) || defined(__clang__)
#define FOC_PI_INLINE static inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define FOC_PI_INLINE static __forceinline
#else
#define FOC_PI_INLINE static inline
#endif

FOC_PI_INLINE uint8_t FocPi_IsValidInline(const foc_pi_t *controller)
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

FOC_PI_INLINE uint8_t FocPi_TryStepInline(foc_pi_t *controller, float error,
                                         float period_s, float *output)
{
  float integral;
  if (output == NULL) return 0U;
  *output = 0.0f;
  if (!FocPi_IsValidInline(controller) || !isfinite(error) ||
      !isfinite(period_s) || period_s <= 0.0f) return 0U;

  integral = controller->integral + error * controller->ki * period_s;
  if (integral < controller->integral_min) integral = controller->integral_min;
  else if (integral > controller->integral_max) integral = controller->integral_max;
  controller->integral = integral;
  *output = controller->kp * error + controller->integral;
  return 1U;
}

#endif
