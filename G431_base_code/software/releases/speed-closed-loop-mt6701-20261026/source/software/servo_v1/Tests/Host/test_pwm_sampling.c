#include "pwm_sampling.h"
#include "foc_modulation.h"
#include "control_timing.h"
#include "pwm_commit_diagnostics.h"

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
  printf("PASS: dynamic ADC scheduling matches exhaustive timing search; legacy 1920-tick fixture.\n");
  printf("Legacy fixture bounds: %.8f..%.8f; fundamental ceiling %.8f Vdc.\n",
      parameters.duty_min, parameters.duty_max,
      (parameters.duty_max - parameters.duty_min) * FOC_MODULATION_SIX_STEP_MAX);
  /* Actual rounded timing for 1x, 2x, 4x, 16x at 12.5-cycle S/H, 42 MHz.
     The 169-tick edge exclusion is unchanged for every ADC profile. */
  const unsigned sequences[4] = { 201U, 401U, 801U, 3201U };
  const unsigned uppers[4] = { 3929U, 3829U, 3629U, 2429U };
  for (unsigned profile = 0; profile < 4U; ++profile)
  {
    p.sequence_ticks = sequences[profile];
    CHECK(PwmSampling_UniformLimit(&p, &upper) && upper == uppers[profile]);
    parameters.duty_max = (float)upper / 4200.0f;
    for (i = 0; i < 7200U; ++i)
    {
      float angle = (float)i * 6.2831853072f / 7200.0f;
      foc_modulation_output_t output;
      CHECK(FOC_ModulationCompute(&parameters, 1.0f, cosf(angle), sinf(angle), &output));
      unsigned u = (unsigned)(output.phase_a_duty * 4200.0f + 0.5f);
      unsigned v = (unsigned)(output.phase_b_duty * 4200.0f + 0.5f);
      unsigned w = (unsigned)(output.phase_c_duty * 4200.0f + 0.5f);
      CHECK(PwmSampling_Plan(&p, u, v, w, &plan));
      CHECK(trigger_safe(&p, u, v, w, plan.trigger));
    }
    printf("PASS: actual sequence %u ticks, upper %u, all 7200 electrical angles.\n",
        sequences[profile], upper);
    /* Independent direct min/max reference for the unsaturated linear path. */
    parameters.mode = FOC_MODULATION_LINEAR_ONLY;
    for (i = 0; i < 7200U; ++i)
    {
      float angle = (float)i * 6.2831853072f / 7200.0f;
      float radius = 0.4f * (parameters.duty_max - parameters.duty_min);
      float alpha = radius * cosf(angle), beta = radius * sinf(angle);
      float a = alpha, b = -0.5f * alpha + 0.8660254038f * beta;
      float c = -0.5f * alpha - 0.8660254038f * beta;
      float common = -0.5f * (fmaxf(a, fmaxf(b, c)) + fminf(a, fminf(b, c)));
      float middle = (parameters.duty_max + parameters.duty_min) * 0.5f;
      foc_modulation_output_t output;
      CHECK(FOC_ModulationCompute(&parameters, 1.0f, alpha, beta, &output));
      CHECK(fabsf(output.phase_a_duty - (middle + a + common)) < 0.000001f);
      CHECK(fabsf(output.phase_b_duty - (middle + b + common)) < 0.000001f);
      CHECK(fabsf(output.phase_c_duty - (middle + c + common)) < 0.000001f);
      CHECK(fabsf(output.fundamental_alpha - alpha) < 0.000001f);
      CHECK(fabsf(output.fundamental_beta - beta) < 0.000001f);
      CHECK(!output.limited && output.region == FOC_MODULATION_REGION_LINEAR);
    }
    parameters.mode = FOC_MODULATION_OVERMODULATION;
  }
  puts("PASS: linear fast path matches independent min/max reference, 28800 vectors.");
  /* Current board: exact center-aligned 10 kHz, ARR=8400, CCR/ARR duty.
     Keep the 20 kHz fixtures above as legacy regression coverage. */
  CHECK(CONTROL_PERIOD_TICKS == 16800U && CONTROL_PWM_PEAK_TICKS == 8400U);
  CHECK(fabsf(CONTROL_PERIOD_S - 0.0001f) < 1e-10f);
  p.peak = CONTROL_PWM_PEAK_TICKS;
  p.sequence_ticks = 201U;
  p.guard_ticks = 169U;
  CHECK(PwmSampling_UniformLimit(&p, &upper) && upper == 8130U);
  parameters.duty_min = 85.0f / (float)p.peak;
  parameters.duty_max = (float)upper / (float)p.peak;
  for (i = 0; i < 7200U; ++i)
  {
    float angle = (float)i * 6.2831853072f / 7200.0f;
    foc_modulation_output_t output;
    CHECK(FOC_ModulationCompute(&parameters, 1.0f, cosf(angle), sinf(angle), &output));
    unsigned u = (unsigned)(output.phase_a_duty * (float)p.peak + 0.5f);
    unsigned v = (unsigned)(output.phase_b_duty * (float)p.peak + 0.5f);
    unsigned w = (unsigned)(output.phase_c_duty * (float)p.peak + 0.5f);
    CHECK(PwmSampling_Plan(&p, u, v, w, &plan));
    CHECK(trigger_safe(&p, u, v, w, plan.trigger));
  }
  CHECK(PwmSampling_Plan(&p, 4200U, 4200U, 4200U, &plan));
  CHECK(plan.trigger == 8199U && plan.sequence_end == 8400U);
  puts("PASS: 10 kHz/100 us, exact ARR/duty scaling and all 7200 sampling windows.");
  {
    volatile pwm_commit_fault_t capture = {0};
    pwm_commit_fault_t event = {0};
    CHECK(sizeof(event) == 88U);
    event.reason = PWM_COMMIT_FAULT_BEFORE_GUARD;
    event.cycle = 10U; /* Counter wrap remains an unsigned difference. */
    event.adc_entry_cycle = UINT32_MAX - 9U;
    event.position_before = 10164U;
    event.position_after = PWM_COMMIT_POSITION_UNREAD;
    event.guard_ticks = 337U;
    event.requested_ccr[3] = 5049U;
    PwmCommit_FreezeFirst(0, &event);
    PwmCommit_FreezeFirst(&capture, 0);
    CHECK(capture.count == 0U);
    PwmCommit_FreezeFirst(&capture, &event);
    CHECK(capture.count == 1U && capture.version == 1U);
    CHECK(capture.reason == PWM_COMMIT_FAULT_BEFORE_GUARD);
    CHECK(capture.position_before == 10164U && capture.position_after == UINT32_MAX);
    CHECK(capture.cycle - capture.adc_entry_cycle == 20U);
    CHECK(capture.requested_ccr[3] == 5049U);
    for (unsigned reason = 1; reason <= 4; ++reason)
    {
      event.reason = reason;
      event.cycle = 99U;
      event.requested_ccr[3] = 0U;
      PwmCommit_FreezeFirst(&capture, &event);
      CHECK(capture.reason == PWM_COMMIT_FAULT_BEFORE_GUARD);
      CHECK(capture.cycle == 10U && capture.requested_ccr[3] == 5049U);
    }
    puts("PASS: PWM first-fault snapshot publication, freeze and DWT wrap.");
  }
  {
    volatile pwm_commit_timing_t timing = {0};
    CHECK(sizeof(timing) == 20U);
    PwmCommit_RecordSuccess(NULL, 1000U, 2000U);
    PwmCommit_RecordSuccess(&timing, 900U, 3000U);
    PwmCommit_RecordSuccess(&timing, 700U, 3300U);
    PwmCommit_RecordSuccess(&timing, 1000U, 2800U);
    CHECK(timing.count == 3U && timing.minimum_remaining_ticks == 700U);
    CHECK(timing.maximum_adc_to_commit_cycles == 3300U);
    CHECK(timing.last_remaining_ticks == 1000U && timing.last_adc_to_commit_cycles == 2800U);
    timing.count = UINT32_MAX;
    PwmCommit_RecordSuccess(&timing, 800U, 2900U);
    CHECK(timing.count == UINT32_MAX && timing.minimum_remaining_ticks == 700U);
    puts("PASS: successful PWM commit margin extrema and count saturation.");
  }
}
