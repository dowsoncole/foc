#include "foc_modulation.h"
#include "foc_modulation_lut.h"

#include <math.h>
#include <string.h>

static float modulation_clamp(float value, float low, float high)
{
  return (value < low) ? low : ((value > high) ? high : value);
}

/* 表按参数等间距、基波严格递增存放。二分查找逆增益，最多 7/8 次比较。 */
static float modulation_inverse_gain(const float *table, uint32_t last, float value)
{
  uint32_t low = 0U;
  uint32_t high = last;
  float fraction;

  while ((high - low) > 1U)
  {
    uint32_t middle = (low + high) / 2U;
    if (value < table[middle]) high = middle;
    else low = middle;
  }
  fraction = modulation_clamp((value - table[low]) / (table[high] - table[low]),
                               0.0f, 1.0f);
  return ((float)low + fraction) / (float)last;
}

static uint8_t modulation_sector(float a, float b, float c)
{
  if (a >= b)
  {
    if (b >= c) return 1U;
    return (a >= c) ? 6U : 5U;
  }
  if (a >= c) return 2U;
  return (b >= c) ? 3U : 4U;
}

/* OVM2 对归一化后的中间相 duty 做对称展开；最大/最小相保持 1/0。
   q = T2/(T1+T2)，q' = clamp(1/2 + (q-1/2)/(1-h), 0, 1)。
   h=0 为六边形轨迹，h->1 为最近有效矢量六步轨迹。
   换相点 q=1/2 保留两矢量等时间，不引入零矢量。 */
static float modulation_hold(float duty, float hold)
{
  if (hold >= 1.0f)
  {
    if (duty > (0.5f + 1.0e-6f)) return 1.0f;
    if (duty < (0.5f - 1.0e-6f)) return 0.0f;
    return 0.5f;
  }
  return modulation_clamp(0.5f + ((duty - 0.5f) / (1.0f - hold)), 0.0f, 1.0f);
}

uint8_t FOC_ModulationCompute(const foc_modulation_parameters_t *parameters,
                              float voltage_limit,
                              float alpha, float beta,
                              foc_modulation_output_t *output)
{
  float span, limit, magnitude, fundamental, normalized, radius, gain;
  float a, b, c, maximum, minimum, spread, zero, hold = 0.0f;
  float da, db, dc;

  if (output == 0) return 0U;
  (void)memset(output, 0, sizeof(*output));
  if ((parameters == 0) || !isfinite(alpha) || !isfinite(beta) ||
      !isfinite(voltage_limit) || (voltage_limit < 0.0f) ||
      !isfinite(parameters->duty_min) || !isfinite(parameters->duty_max) ||
      (parameters->duty_min < 0.0f) || (parameters->duty_max > 1.0f) ||
      (parameters->duty_min >= parameters->duty_max) ||
      ((parameters->mode != FOC_MODULATION_LINEAR_ONLY) &&
       (parameters->mode != FOC_MODULATION_OVERMODULATION)))
  {
    return 0U;
  }

  span = parameters->duty_max - parameters->duty_min;
  /* 小于任何实际 16 位 PWM 的分辨率，不能用作有效调制范围。 */
  if (span < 1.0e-6f) return 0U;
  limit = span * ((parameters->mode == FOC_MODULATION_LINEAR_ONLY) ?
                   FOC_MODULATION_LINEAR_MAX : FOC_MODULATION_SIX_STEP_MAX);
  limit = fminf(limit, voltage_limit);
  magnitude = sqrtf(alpha * alpha + beta * beta);
  if (!isfinite(magnitude)) return 0U;
  fundamental = fminf(magnitude, limit);
  output->fundamental_limit = limit;
  output->limited = (magnitude > limit) ? 1U : 0U;
  gain = (magnitude > 0.0f) ? (fundamental / magnitude) : 0.0f;
  output->fundamental_alpha = alpha * gain;
  output->fundamental_beta = beta * gain;
  normalized = fundamental / span;
  radius = normalized;
  output->region = FOC_MODULATION_REGION_LINEAR;

  if (normalized > FOC_MODULATION_LINEAR_MAX)
  {
    if (normalized < FOC_MODULATION_HEXAGON_FUNDAMENTAL)
    {
      radius = FOC_MODULATION_LINEAR_MAX +
               ((2.0f / 3.0f) - FOC_MODULATION_LINEAR_MAX) *
               modulation_inverse_gain(foc_ovm1_fundamental, 64U, normalized);
      output->region = FOC_MODULATION_REGION_OVM1;
    }
    else
    {
      radius = 2.0f / 3.0f;
      hold = modulation_inverse_gain(foc_ovm2_fundamental, 128U, normalized);
      output->region = FOC_MODULATION_REGION_OVM2;
      if (normalized >= FOC_MODULATION_SIX_STEP_MAX)
      {
        hold = 1.0f;
        output->region = FOC_MODULATION_REGION_SIX_STEP;
      }
    }
  }

  gain = (magnitude > 0.0f) ? (radius / magnitude) : 0.0f;
  a = alpha * gain;
  b = (-0.5f * alpha + 0.8660254038f * beta) * gain;
  c = (-0.5f * alpha - 0.8660254038f * beta) * gain;
  output->sector = modulation_sector(a, b, c);
  maximum = fmaxf(a, fmaxf(b, c));
  minimum = fminf(a, fminf(b, c));
  spread = maximum - minimum;

  /* OVM1 保角度径向限幅：T1+T2>1 时按其和归一化，T0=0。
     与 min/max 差值限幅等价；不是逐相截断引起的不可控角度偏移。 */
  gain = (spread > 1.0f) ? (1.0f / spread) : 1.0f;
  zero = -0.5f * (maximum + minimum);
  da = modulation_clamp(0.5f + (a + zero) * gain, 0.0f, 1.0f);
  db = modulation_clamp(0.5f + (b + zero) * gain, 0.0f, 1.0f);
  dc = modulation_clamp(0.5f + (c + zero) * gain, 0.0f, 1.0f);
  if (output->region >= FOC_MODULATION_REGION_OVM2)
  {
    da = modulation_hold(da, hold);
    db = modulation_hold(db, hold);
    dc = modulation_hold(dc, hold);
  }
  output->phase_a_duty = modulation_clamp(parameters->duty_min + span * da,
                                          parameters->duty_min, parameters->duty_max);
  output->phase_b_duty = modulation_clamp(parameters->duty_min + span * db,
                                          parameters->duty_min, parameters->duty_max);
  output->phase_c_duty = modulation_clamp(parameters->duty_min + span * dc,
                                          parameters->duty_min, parameters->duty_max);
  /* 等幅值 Clarke：去掉共模后重构实际电压，不能用基波目标冒充。 */
  output->alpha_voltage = (2.0f * output->phase_a_duty - output->phase_b_duty -
                            output->phase_c_duty) / 3.0f;
  output->beta_voltage = (output->phase_b_duty - output->phase_c_duty) *
                          0.5773502692f;
  output->valid = 1U;
  return 1U;
}
