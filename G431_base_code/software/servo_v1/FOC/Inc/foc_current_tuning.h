#ifndef FOC_CURRENT_TUNING_H
#define FOC_CURRENT_TUNING_H

#include <stdint.h>
#include "foc_pi.h"

typedef struct
{
  float resistance_ohm;
  float inductance_h;
  float dc_bus_v;
  float bandwidth_hz;
  float sample_period_s;
  float delay_s;
  float phase_margin_deg;
  float integral_limit_pu;
} foc_current_design_t;

typedef struct
{
  foc_pi_t d;
  foc_pi_t q;
  float requested_bandwidth_hz;
  float achieved_bandwidth_hz;
  float estimated_phase_margin_deg;
  uint8_t bandwidth_limited;
} foc_current_gains_t;

/* Design a pole-zero-canceling PI for Vdc/(L*s+R).  Gains produce a
   normalized bus-voltage command per ampere because the FOC output is pu. */
uint8_t FOC_CurrentDesign(const foc_current_design_t *design,
                          foc_current_gains_t *gains);

#endif /* FOC_CURRENT_TUNING_H */
