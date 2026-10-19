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
  /* 两相反馈使用物理安培，必须与 d/q 电流给定和 current_limit 同量纲。 */
  float phase_a;
  float phase_b;
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

/* 开环电压算法的给定与运行状态。给定和斜坡状态使用物理伏特；算法内部
   除以 dc_bus_voltage_v 后才进入标幺调制。频率为电角频率；
   electrical_angle_rad 和 cycle_count 由 VoltageStep 自动更新。 */
typedef struct
{
  /* Command values.  The algorithm slews applied_* toward these targets so
     the first enabled PWM frame never contains a full voltage step. */
  float d_voltage_v;
  float q_voltage_v;
  float dc_bus_voltage_v;
  float electrical_frequency_hz;
  float electrical_angle_rad;
  uint32_t cycle_count;
  float applied_d_voltage_v;
  float applied_q_voltage_v;
  /* Per-axis voltage slew in V/s.  Zero keeps the legacy immediate-step
     behaviour, which is useful for deterministic host tests. */
  float voltage_ramp_rate_v_per_s;
  uint8_t voltage_ramp_complete;
} foc_voltage_open_loop_t;

/* 电流环台架测试参数与遥测。电流闭环使用真实 ADC 反馈，但电角度由本对象
   自行推进，因此它是“开环角度 + 闭环电流”，不依赖编码器。PI 输出仍为
   相对母线电压的标幺值，下面的 output_*_voltage_v 仅用于物理量观察。 */
typedef struct
{
  float d_current_reference_a;
  float q_current_reference_a;
  float dc_bus_voltage_v;
  float electrical_frequency_hz;
  float electrical_angle_rad;
  uint32_t cycle_count;
  float measured_d_current_a;
  float measured_q_current_a;
  float output_d_voltage_v;
  float output_q_voltage_v;
} foc_current_test_t;

void FOC_AlgorithmInit(foc_algorithm_t *algorithm,
                       const foc_parameters_t *parameters);
void FOC_AlgorithmReset(foc_algorithm_t *algorithm);
uint8_t FOC_InputIsValid(const foc_input_t *input);
void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input);

/* 开环电压测试入口：跳过电流采样、Park 反馈和 PI，把伏特给定除以母线电压
   得到标幺D/Q电压，再按电角度变换到 alpha/beta 并生成三相 duty。
   voltage_limit 仍是相对母线电压 Vdc 的标幺限制。 */
uint8_t FOC_AlgorithmVoltageStep(foc_algorithm_t *algorithm,
                                 foc_voltage_open_loop_t *open_loop,
                                 float sample_period_s);

#endif
