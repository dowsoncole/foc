#include "pwm_sampling.h"

#include <string.h>

static uint8_t sampling_parameters_valid(const pwm_sampling_parameters_t *p)
{
  return ((p != 0) && (p->peak > 1U) && (p->peak <= 65535U) &&
          (p->sequence_ticks > 0U) && (p->sequence_ticks < 2U * p->peak) &&
          (p->guard_ticks < p->peak)) ? 1U : 0U;
}

uint8_t PwmSampling_UniformLimit(const pwm_sampling_parameters_t *p,
                                  uint32_t *maximum_compare)
{
  uint32_t reservation;
  if ((maximum_compare == 0) || (sampling_parameters_valid(p) == 0U)) return 0U;
  reservation = p->guard_ticks + (p->sequence_ticks + 1U) / 2U;
  if (reservation >= p->peak) return 0U;
  *maximum_compare = p->peak - reservation;
  return 1U;
}

uint8_t PwmSampling_Plan(const pwm_sampling_parameters_t *p,
                          uint32_t compare_u, uint32_t compare_v,
                          uint32_t compare_w, pwm_sampling_plan_t *plan)
{
  int32_t low, high, preferred, best = -1, best_distance = 0;
  int32_t edges[2];
  uint32_t edge_count = 0U, i;
  uint32_t largest;

  if (plan == 0) return 0U;
  (void)memset(plan, 0, sizeof(*plan));
  if ((sampling_parameters_valid(p) == 0U) ||
      (compare_u >= p->peak) || (compare_v >= p->peak) ||
      (compare_w > p->peak + 1U)) return 0U;

  largest = (compare_u > compare_v) ? compare_u : compare_v;
  low = (int32_t)(largest + p->guard_ticks);
  high = (int32_t)(2U * p->peak) - (int32_t)largest -
           (int32_t)p->guard_ticks - (int32_t)p->sequence_ticks;
  if (low < 1) low = 1;
  if (high >= (int32_t)p->peak) high = (int32_t)p->peak - 1;
  if (low > high) return 0U;

  plan->low_window_start = largest + p->guard_ticks;
  plan->low_window_end = 2U * p->peak - largest - p->guard_ticks;
  preferred = (int32_t)p->peak - (int32_t)p->sequence_ticks;
  /* CCR=0 或 CCR>ARR 为静态电平；CCR=ARR 只在峰值产生极窄切换，仍避开。 */
  if ((compare_w > 0U) && (compare_w <= p->peak))
  {
    edges[edge_count++] = (int32_t)compare_w;
    edges[edge_count++] = (int32_t)(2U * p->peak - compare_w);
  }

  /* 最多两个 W 边沿，将可用起点区间分为最多三段，无无界循环。 */
  for (i = 0U; i <= edge_count; ++i)
  {
    int32_t end = high;
    if (i < edge_count)
    {
      int32_t before_edge = edges[i] - (int32_t)p->guard_ticks -
                              (int32_t)p->sequence_ticks;
      if (before_edge < end) end = before_edge;
    }
    if (low <= end)
    {
      int32_t candidate = preferred;
      int32_t distance;
      if (candidate < low) candidate = low;
      if (candidate > end) candidate = end;
      distance = (candidate > preferred) ? candidate - preferred : preferred - candidate;
      if ((best < 0) || (distance < best_distance))
      {
        best = candidate;
        best_distance = distance;
      }
    }
    if (i < edge_count)
    {
      int32_t after_edge = edges[i] + (int32_t)p->guard_ticks;
      if (after_edge > low) low = after_edge;
    }
  }
  if (best < 0) return 0U;
  plan->trigger = (uint32_t)best;
  plan->sequence_end = plan->trigger + p->sequence_ticks;
  plan->valid = 1U;
  return 1U;
}
