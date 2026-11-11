#ifndef FOC_MODULATION_H
#define FOC_MODULATION_H

#include <stdint.h>

#define FOC_MODULATION_LINEAR_MAX    0.5773502692f /* 1/sqrt(3) */
#define FOC_MODULATION_SIX_STEP_MAX  0.6366197724f /* 2/pi，基波相电压峰值 / Vdc */

typedef enum
{
  FOC_MODULATION_LINEAR_ONLY = 0,
  FOC_MODULATION_OVERMODULATION
} foc_modulation_mode_t;

typedef enum
{
  FOC_MODULATION_REGION_LINEAR = 0,
  FOC_MODULATION_REGION_OVM1,
  FOC_MODULATION_REGION_OVM2,
  FOC_MODULATION_REGION_SIX_STEP
} foc_modulation_region_t;

typedef struct
{
  foc_modulation_mode_t mode;
  /* 由功率级/采样层提供。0..1 才能达到全母线六步极限；收窄后按跨度降额。
     不得仅为提高利用率放宽低侧分流器采样窗口或自举刷新约束。 */
  float duty_min;
  float duty_max;
} foc_modulation_parameters_t;

typedef struct
{
  float phase_a_duty;
  float phase_b_duty;
  float phase_c_duty;
  /* 从占空比重构的当前 PWM 周期平均电压，均除以 Vdc。 */
  float alpha_voltage;
  float beta_voltage;
  /* 限幅后的目标基波，不包含过调制产生的 5/7 次等谐波。
     只对恒幅、匀速旋转的输入定义基波增益；不能当作本周期实际电压。 */
  float fundamental_alpha;
  float fundamental_beta;
  float fundamental_limit;
  foc_modulation_region_t region;
  uint8_t sector; /* 标准逆时针编号：1 对应 [0, pi/3)，边界可归任一邻区。 */
  uint8_t limited; /* 请求基波超出配置/硬件能力，与 OVM 状态分开。 */
  uint8_t valid;
} foc_modulation_output_t;

/* 纯算法，无 HAL、定时器或静态可变状态。
   输入 αβ 是目标基波相电压 / Vdc，voltage_limit 使用相同单位。
   OVM1：径向投影至六边形；OVM2：压缩相邻矢量的混合区，直到六步。
   查表补偿非线性基波增益，实时路径无 atan2、无数值积分。
   参数/输入非法返回 0，output.valid=0，调用方必须禁止输出。 */
uint8_t FOC_ModulationCompute(const foc_modulation_parameters_t *parameters,
                              float voltage_limit,
                              float alpha, float beta,
                              foc_modulation_output_t *output);

#endif
