#include "foc_current_tuning.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define FOC_TUNING_TWO_PI  6.2831853071795864769f

static float FOC_CurrentMinimum(float a, float b)
{
  return (a < b) ? a : b;
}

uint8_t FOC_CurrentDesign(const foc_current_design_t *design,
                          foc_current_gains_t *gains)
{
  float requested_omega;
  float maximum_omega;
  float achieved_omega;
  float phase_margin_rad;
  float kp;
  float ki;

  if (gains == NULL) return 0U;
  (void)memset(gains, 0, sizeof(*gains));
  if ((design == NULL) || !isfinite(design->resistance_ohm) ||
      (design->resistance_ohm <= 0.0f) ||
      !isfinite(design->inductance_h) || (design->inductance_h <= 0.0f) ||
      !isfinite(design->dc_bus_v) || (design->dc_bus_v <= 0.0f) ||
      !isfinite(design->bandwidth_hz) || (design->bandwidth_hz <= 0.0f) ||
      !isfinite(design->sample_period_s) ||
      (design->sample_period_s <= 0.0f) ||
      !isfinite(design->delay_s) || (design->delay_s < 0.0f) ||
      !isfinite(design->phase_margin_deg) ||
      (design->phase_margin_deg <= 0.0f) ||
      (design->phase_margin_deg >= 90.0f) ||
      !isfinite(design->integral_limit_pu) ||
      (design->integral_limit_pu <= 0.0f)) return 0U;

  requested_omega = FOC_TUNING_TWO_PI * design->bandwidth_hz;
  /* Keep crossover below one tenth of the sample rate. */
  maximum_omega = FOC_TUNING_TWO_PI * 0.1f / design->sample_period_s;
  phase_margin_rad = (90.0f - design->phase_margin_deg) *
      (FOC_TUNING_TWO_PI / 360.0f);
  if (design->delay_s > 0.0f)
    maximum_omega = FOC_CurrentMinimum(maximum_omega,
        phase_margin_rad / design->delay_s);
  if (!isfinite(maximum_omega) || (maximum_omega <= 0.0f)) return 0U;
  achieved_omega = FOC_CurrentMinimum(requested_omega, maximum_omega);

  kp = design->inductance_h * achieved_omega / design->dc_bus_v;
  ki = design->resistance_ohm * achieved_omega / design->dc_bus_v;
  gains->d.kp = kp;
  gains->d.ki = ki;
  gains->d.integral = 0.0f;
  gains->d.integral_min = -design->integral_limit_pu;
  gains->d.integral_max = design->integral_limit_pu;
  gains->q = gains->d;
  gains->requested_bandwidth_hz = design->bandwidth_hz;
  gains->achieved_bandwidth_hz = achieved_omega / FOC_TUNING_TWO_PI;
  gains->estimated_phase_margin_deg = 90.0f -
      achieved_omega * design->delay_s * (360.0f / FOC_TUNING_TWO_PI);
  gains->bandwidth_limited = (achieved_omega < requested_omega) ? 1U : 0U;
  if ((FOC_PI_IsValid(&gains->d) == 0U) ||
      !isfinite(gains->estimated_phase_margin_deg))
  {
    (void)memset(gains, 0, sizeof(*gains));
    return 0U;
  }
  return 1U;
}
