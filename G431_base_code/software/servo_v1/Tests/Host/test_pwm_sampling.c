#include "pwm_sampling.h"
#include "foc_modulation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "SAMPLING FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

/* 逐 tick 检查真实时间区间，不重复规划器的分段选择过程。 */
static int trigger_safe(const pwm_sampling_parameters_t *p, unsigned u,
                        unsigned v, unsigned w, unsigned trigger)
{
  unsigned end = trigger + p->sequence_ticks;
  unsigned edge[2] = { w, 2U * p->peak - w };
  unsigned i;
  if (trigger == 0U || trigger >= p->peak) return 0;
  if (trigger < u + p->guard_ticks || trigger < v + p->guard_ticks) return 0;
  if (end > 2U * p->peak - u - p->guard_ticks ||
      end > 2U * p->peak - v - p->guard_ticks) return 0;
  if (w == 0U || w > p->peak) return 1;
  for (i = 0; i < 2; ++i)
  {
    int before = (int)edge[i] - (int)p->guard_ticks;
    int after = (int)edge[i] + (int)p->guard_ticks;
    if (!((int)end <= before || (int)trigger >= after)) return 0;
  }
  return 1;
}

static void exhaustive_reference(unsigned sequence)
{
  pwm_sampling_parameters_t p = { 32U, 0U, 3U };
  unsigned u, v, w, trigger;
  p.sequence_ticks = sequence;
  for (u = 0; u < p.peak; ++u)
    for (v = 0; v < p.peak; ++v)
      for (w = 0; w <= p.peak + 1U; ++w)
      {
        int best = -1, distance = 1000000;
        int preferred = (int)p.peak - (int)p.sequence_ticks;
        pwm_sampling_plan_t plan;
        unsigned found = PwmSampling_Plan(&p, u, v, w, &plan);
        for (trigger = 1; trigger < p.peak; ++trigger)
        {
          int d = abs((int)trigger - preferred);
          if (trigger_safe(&p, u, v, w, trigger) && d < distance)
          {
            best = (int)trigger;
            distance = d;
          }
        }
        CHECK((best >= 0) == (found != 0U));
        if (found)
        {
          CHECK(plan.valid != 0U);
          CHECK(plan.trigger == (unsigned)best);
          CHECK(plan.sequence_end == plan.trigger + p.sequence_ticks);
        }
        else CHECK(plan.valid == 0U);
      }
}

void TestPwmSampling(void)
{
  pwm_sampling_parameters_t p = { 4199U, 1920U, 169U };
  pwm_sampling_plan_t plan, low, high;
  foc_modulation_parameters_t parameters = { FOC_MODULATION_OVERMODULATION, 0.0f, 1.0f };
  unsigned upper, i;
  CHECK(PwmSampling_UniformLimit(&p, &upper));
  CHECK(upper == 3070U);
  parameters.duty_min = 85.0f / 4200.0f;
  parameters.duty_max = (float)upper / 4200.0f;
  for (i = 0; i < 7200U; ++i)
  {
    float angle = (float)i * 6.2831853072f / 7200.0f;
    foc_modulation_output_t output;
    unsigned u, v, w;
    CHECK(FOC_ModulationCompute(&parameters, 1.0f, cosf(angle), sinf(angle), &output));
    u = (unsigned)(output.phase_a_duty * 4200.0f + 0.5f);
    v = (unsigned)(output.phase_b_duty * 4200.0f + 0.5f);
    w = (unsigned)(output.phase_c_duty * 4200.0f + 0.5f);
    CHECK(PwmSampling_Plan(&p, u, v, w, &plan));
    CHECK(trigger_safe(&p, u, v, w, plan.trigger));
  }
  CHECK(PwmSampling_Plan(&p, 1000U, 1000U, 1000U, &low));
  CHECK(PwmSampling_Plan(&p, upper, upper, upper, &high));
  CHECK(low.trigger == 2279U && low.sequence_end == p.peak);
  CHECK(high.trigger == 3239U && high.sequence_end > p.peak);
  CHECK(PwmSampling_Plan(&p, 400U, 400U, 4000U, &plan));
  CHECK(trigger_safe(&p, 400U, 400U, 4000U, plan.trigger));
  CHECK(!PwmSampling_Plan(&p, 4199U, 0U, 0U, &plan));
  CHECK(!PwmSampling_Plan(&p, 3071U, 3071U, 0U, &plan));
  CHECK(!PwmSampling_Plan(NULL, 0U, 0U, 0U, &plan));
  CHECK(!PwmSampling_Plan(&p, 0U, 0U, 0U, NULL));
  CHECK(!PwmSampling_UniformLimit(&p, NULL));
  exhaustive_reference(9U);
  exhaustive_reference(31U);
  exhaustive_reference(55U);
  printf("PASS: dynamic ADC scheduling matches exhaustive timing search; 16x sequence retained.\n");
  printf("Current uniform bounds: %.8f..%.8f; fundamental ceiling %.8f Vdc.\n",
      parameters.duty_min, parameters.duty_max,
      (parameters.duty_max - parameters.duty_min) * FOC_MODULATION_SIX_STEP_MAX);
}
