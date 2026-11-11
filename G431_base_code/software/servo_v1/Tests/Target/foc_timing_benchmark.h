#ifndef FOC_TIMING_BENCHMARK_H
#define FOC_TIMING_BENCHMARK_H

/* Test-only boot image. It runs before any motor peripheral is initialized,
   never starts PWM, and never enables MOE. Not compiled in production. */
#include <limits.h>
#include <string.h>
#define FOC_ModulationCompute Before_FOC_ModulationCompute
#define FOC_AlgorithmInit Before_FOC_AlgorithmInit
#define FOC_AlgorithmReset Before_FOC_AlgorithmReset
#define FOC_AlgorithmStep Before_FOC_AlgorithmStep
#define FOC_InputIsValid Before_FOC_InputIsValid
#define FOC_TransformCurrentDQ Before_FOC_TransformCurrentDQ
#define FOC_AlgorithmVoltageStep Before_FOC_AlgorithmVoltageStep
#define FOC_AlgorithmVoltageCommandStep Before_FOC_AlgorithmVoltageCommandStep
#define FOC_AlgorithmVoltageFeedbackStep Before_FOC_AlgorithmVoltageFeedbackStep
#define FOC_PI_IsValid Before_FOC_PI_IsValid
#define FOC_PI_TryStep Before_FOC_PI_TryStep
#define FOC_PI_Step Before_FOC_PI_Step
#define FOC_PI_Reset Before_FOC_PI_Reset
void Before_FOC_AlgorithmReset(foc_algorithm_t *algorithm);
__attribute__((noinline)) uint8_t Before_FOC_PI_TryStep(foc_pi_t *, float, float, float *);
__attribute__((noinline)) uint8_t Before_FOC_ModulationCompute(const foc_modulation_parameters_t *,
    float, float, float, foc_modulation_output_t *);
#include "../../../control_analysis/results/foc_hotpath_round2_20261027/baseline/foc_algorithm.c"
#include "../../../control_analysis/results/foc_hotpath_round2_20261027/baseline/foc_modulation.c"
#include "../../../control_analysis/results/foc_hotpath_round2_20261027/baseline/foc_pi.c"
#undef FOC_ModulationCompute
#undef FOC_AlgorithmInit
#undef FOC_AlgorithmReset
#undef FOC_AlgorithmStep
#undef FOC_InputIsValid
#undef FOC_TransformCurrentDQ
#undef FOC_AlgorithmVoltageStep
#undef FOC_AlgorithmVoltageCommandStep
#undef FOC_AlgorithmVoltageFeedbackStep
#undef FOC_PI_IsValid
#undef FOC_PI_TryStep
#undef FOC_PI_Step
#undef FOC_PI_Reset

typedef struct {
  uint32_t count, minimum, maximum, sum;
} foc_benchmark_cycles_t;
typedef struct {
  foc_benchmark_cycles_t before, after;
  uint32_t mismatches, valid_count, saturated_count, reserved;
} foc_benchmark_case_t;
volatile struct {
  uint32_t version, state, clock_hz, mismatch_count, case_count, iterations;
  foc_benchmark_case_t cases[6];
} g_foc_target_benchmark;
static foc_algorithm_t s_benchmark_before, s_benchmark_after;

static void FocBenchmark_Record(volatile foc_benchmark_cycles_t *p, uint32_t cycles)
{
  if (p->count == 0U || cycles < p->minimum) p->minimum = cycles;
  if (cycles > p->maximum) p->maximum = cycles;
  p->sum += cycles;
  p->count++;
}

static void FOC_TargetTimingBenchmark(void)
{
  void (* volatile before_step)(foc_algorithm_t *, const foc_input_t *) = Before_FOC_AlgorithmStep;
  void (* volatile after_step)(foc_algorithm_t *, const foc_input_t *) = FOC_AlgorithmStep;
  foc_parameters_t p = {0};
  foc_input_t in = {0};
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_foc_target_benchmark.version = 1U;
  g_foc_target_benchmark.state = 1U;
  g_foc_target_benchmark.clock_hz = SystemCoreClock;
  g_foc_target_benchmark.case_count = 6U;
  g_foc_target_benchmark.iterations = 10000U;
  p.sample_period_s = p.pwm_period_s = 0.0001f;
  p.current_limit = 60;
  p.d_controller = (foc_pi_t){0.02f, 10, 0, -0.5f, 0.5f};
  p.q_controller = p.d_controller;
  for (uint32_t test = 0U; test < 6U; ++test)
  {
    volatile foc_benchmark_case_t *result = &g_foc_target_benchmark.cases[test];
    p.modulation.mode = test >= 2U && test <= 3U ?
        FOC_MODULATION_OVERMODULATION : FOC_MODULATION_LINEAR_ONLY;
    p.modulation.duty_min = 0.0f;
    p.modulation.duty_max = 1.0f;
    p.voltage_limit = test == 2U ? 0.59f : (test == 3U ? 0.625f : 0.50f);
    p.d_controller.ki = p.q_controller.ki = test == 0U ? 0.0f : 10.0f;
    Before_FOC_AlgorithmInit(&s_benchmark_before, &p);
    FOC_AlgorithmInit(&s_benchmark_after, &p);
    for (uint32_t i = 0U; i < 10032U; ++i)
    {
      uint32_t a, b, start, mask;
      in.current_valid = in.angle_valid = in.reference_valid = 1U;
      in.electrical_angle_rad = (float)(i % 1024U) * (6.2831853072f / 1024.0f);
      in.phase_a = 0.25f; in.phase_b = -0.125f;
      in.d_reference = test == 0U ? 0.3f : 40.0f;
      in.q_reference = test == 0U ? 0.2f : 30.0f;
      if (test == 4U) in.electrical_angle_rad -= 12.5663706144f;
      if (test == 5U) in.angle_valid = 0U;
      mask = __get_PRIMASK();
      __disable_irq();
      /* Alternate order to avoid always rewarding the second invocation. */
      if (i & 1U)
      {
        start = DWT->CYCCNT; after_step(&s_benchmark_after, &in); b = DWT->CYCCNT - start;
        start = DWT->CYCCNT; before_step(&s_benchmark_before, &in); a = DWT->CYCCNT - start;
      }
      else
      {
        start = DWT->CYCCNT; before_step(&s_benchmark_before, &in); a = DWT->CYCCNT - start;
        start = DWT->CYCCNT; after_step(&s_benchmark_after, &in); b = DWT->CYCCNT - start;
      }
      if (!mask) __enable_irq();
      if (memcmp(&s_benchmark_before, &s_benchmark_after, sizeof(s_benchmark_before)) != 0)
      {
        result->mismatches++;
        g_foc_target_benchmark.mismatch_count++;
      }
      if (i >= 32U)
      {
        FocBenchmark_Record(&result->before, a);
        FocBenchmark_Record(&result->after, b);
        result->valid_count += s_benchmark_after.output.valid != 0U;
        result->saturated_count += s_benchmark_after.output.saturated != 0U;
      }
    }
  }
  g_foc_target_benchmark.state = 2U;
}
#endif
