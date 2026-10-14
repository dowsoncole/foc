#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

#include <stdint.h>

typedef struct
{
  float kp;
  float ki;
  float integral;
  float integral_min;
  float integral_max;
} foc_pi_t;

typedef struct
{
  float sample_period_s;
  float voltage_limit;
  float current_limit;
  foc_pi_t d_controller;
  foc_pi_t q_controller;
} foc_parameters_t;

typedef struct
{
  float phase_a;
  float phase_b;
  float electrical_angle_rad;
  float d_reference;
  float q_reference;
  uint8_t enabled;
} foc_input_t;

typedef struct
{
  float d_current;
  float q_current;
  float d_voltage;
  float q_voltage;
  float alpha_voltage;
  float beta_voltage;
  float phase_a_voltage;
  float phase_b_voltage;
  float phase_c_voltage;
  uint8_t saturated;
} foc_output_t;

typedef struct
{
  foc_parameters_t parameters;
  foc_output_t output;
} foc_algorithm_t;

void FOC_AlgorithmInit(foc_algorithm_t *algorithm,
                       const foc_parameters_t *parameters);
void FOC_AlgorithmReset(foc_algorithm_t *algorithm);
void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input);

#endif
