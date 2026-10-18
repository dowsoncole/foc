#ifndef PWM_SAMPLING_H
#define PWM_SAMPLING_H

#include <stdint.h>

/* 纯计数域规划器。时间原点是 CNT=0、向上计数，完整周期为 2*peak。
   U/V 为固定低侧采样通道，不假设可以切换到不存在的 W 相传感器。 */
typedef struct
{
  uint32_t peak;
  uint32_t sequence_ticks;
  uint32_t guard_ticks;
} pwm_sampling_parameters_t;

typedef struct
{
  uint32_t trigger; /* 向上计数时的 CCR4，0 < trigger < peak */
  uint32_t low_window_start;
  uint32_t low_window_end;
  uint32_t sequence_end;
  uint8_t valid;
} pwm_sampling_plan_t;

/* 完整两路过采样序列必须位于 U/V 都已稳定的低侧窗口中；还要避开 W
   的两次开关边沿。优先让序列在峰值完成，窗口变窄时向后移动到最早可行点。
   只规划时间，不改变三相 duty，不改 ADC 过采样倍率。 */
uint8_t PwmSampling_Plan(const pwm_sampling_parameters_t *parameters,
                          uint32_t compare_u, uint32_t compare_v,
                          uint32_t compare_w, pwm_sampling_plan_t *plan);

/* 所有三相都不超过此 CCR 时，对任意相序保证存在完整采样窗口。
   上限由窗口长度决定，不依赖上一次 CCR4；不是全母线能力的承诺。 */
uint8_t PwmSampling_UniformLimit(const pwm_sampling_parameters_t *parameters,
                                  uint32_t *maximum_compare);

#endif
