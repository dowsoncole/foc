#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

#include <stdint.h>
#include "foc_modulation.h"

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
  /* 实际两次电流快照间隔；动态移动采样点时由 ADC 回调逐周期更新。 */
  float sample_period_s;
  /* 名义 PWM 周期；未提供实际 sample_period_s 时用作后备。 */
  float pwm_period_s;
  /* 基波相电压峰值 / Vdc。最大 2/pi，并受 modulation 的 duty 跨度降额。 */
  float voltage_limit;
  foc_modulation_parameters_t modulation;
  float current_limit;
  foc_pi_t d_controller;
  foc_pi_t q_controller;
} foc_parameters_t;

typedef struct
{
  int16_t phase_a;
  int16_t phase_b;
  float electrical_angle_rad;
  float d_reference;
  float q_reference;
  /* 三类输入独立表示有效性：只有全部为 1 才允许本周期运行 FOC。 */
  uint8_t current_valid;
  uint8_t angle_valid;
  uint8_t reference_valid;
} foc_input_t;

typedef struct
{
  float d_current;
  float q_current;
  float d_voltage;
  float q_voltage;
  float alpha_voltage;
  float beta_voltage;
  /* 过调制中实际电压含谐波；基波目标单独提供给调参与抗饱和。 */
  float fundamental_d_voltage;
  float fundamental_q_voltage;
  float fundamental_voltage_limit;
  /* SVPWM 计算时的三相投影，仅供遥测；PWM 硬件消费下方 duty。 */
  float phase_a_voltage;
  float phase_b_voltage;
  float phase_c_voltage;
  float phase_a_duty;
  float phase_b_duty;
  float phase_c_duty;
  foc_modulation_region_t modulation_region;
  uint8_t sector;
  uint8_t valid;
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
uint8_t FOC_InputIsValid(const foc_input_t *input);
void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input);

#endif
