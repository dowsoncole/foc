/* Link an independently compiled, symbol-renamed pre-change algorithm and
   modulation object. Compare complete state, not only duty or RMS error. */
#include "foc_algorithm.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void Before_FOC_AlgorithmInit(foc_algorithm_t *, const foc_parameters_t *);
void Before_FOC_AlgorithmStep(foc_algorithm_t *, const foc_input_t *);
uint8_t Before_FOC_ModulationCompute(const foc_modulation_parameters_t *,
    float, float, float, foc_modulation_output_t *);
uint8_t Before_FOC_AlgorithmVoltageFeedbackStep(foc_algorithm_t *,
    float, float, float, const foc_input_t *);
uint8_t Before_FOC_PI_TryStep(foc_pi_t *, float, float, float *);

static uint32_t random_state = 0x451f0cU;
static float random_float(float low, float high)
{
  random_state = random_state * 1664525U + 1013904223U;
  return low + (high - low) * (float)(random_state >> 8) / 16777216.0f;
}

static void same(const void *before, const void *after, size_t bytes,
                 const char *kind, unsigned index)
{
  if (memcmp(before, after, bytes) != 0)
  {
    const unsigned char *a = before, *b = after;
    size_t offset;
    for (offset = 0; offset < bytes && a[offset] == b[offset]; ++offset) {}
    fprintf(stderr, "FAIL %s sample %u byte %zu: %02x != %02x\n",
        kind, index, offset, a[offset], b[offset]);
    exit(1);
  }
}

int main(void)
{
  const float special[] = {0.0f, -0.0f, 1.0e-30f, -1.0e-30f,
      0.5f, -0.5f, FLT_MAX, -FLT_MAX, INFINITY, -INFINITY, NAN};
  unsigned i;
  for (i = 0; i < 100000U; ++i)
  {
    foc_pi_t a = {0.02f, 10, 0.1f, -0.5f, 0.5f}, b;
    float x, y;
    float error = random_float(-50, 50), period = 0.0001f;
    float value = special[(i / 9U) % 11U];
    switch (i % 9U)
    {
      case 0: a.kp = value; break;
      case 1: a.ki = value; break;
      case 2: a.integral = value; break;
      case 3: a.integral_min = value; break;
      case 4: a.integral_max = value; break;
      case 5: error = value; break;
      case 6: period = value; break;
      default: break;
    }
    b = a;
    if (Before_FOC_PI_TryStep(&a, error, period, &x) !=
        FOC_PI_TryStep(&b, error, period, &y)) return 4;
    same(&a, &b, sizeof(a), "PI state", i);
    same(&x, &y, sizeof(x), "PI output", i);
  }
  for (i = 0; i < 200000U; ++i)
  {
    foc_modulation_parameters_t p;
    foc_modulation_output_t a, b;
    float alpha = random_float(-2, 2), beta = random_float(-2, 2);
    float limit = random_float(0, 0.8f);
    p.mode = (i & 1U) ? FOC_MODULATION_LINEAR_ONLY : FOC_MODULATION_OVERMODULATION;
    p.duty_min = random_float(0, 0.25f);
    p.duty_max = random_float(0.6f, 1);
    if (i % 7U == 0U) alpha = special[(i / 7U) % 11U];
    if (i % 13U == 0U) beta = special[(i / 13U) % 11U];
    if (i % 17U == 0U) limit = special[(i / 17U) % 11U];
    if (i % 19U == 0U) p.duty_min = special[(i / 19U) % 11U];
    if (i % 23U == 0U) p.duty_max = special[(i / 23U) % 11U];
    memset(&a, 0xa5, sizeof(a)); memset(&b, 0xa5, sizeof(b));
    if (Before_FOC_ModulationCompute(&p, limit, alpha, beta, &a) !=
        FOC_ModulationCompute(&p, limit, alpha, beta, &b)) return 2;
    same(&a, &b, sizeof(a), "modulation", i);
  }
  for (unsigned mode = 0; mode < 2U; ++mode)
  {
    foc_parameters_t p = {0};
    foc_algorithm_t before, after;
    foc_input_t in = {0};
    p.sample_period_s = p.pwm_period_s = 0.0001f;
    p.voltage_limit = 0.56f;
    p.current_limit = 60;
    p.modulation.mode = (foc_modulation_mode_t)mode;
    p.modulation.duty_min = 0.02f; p.modulation.duty_max = 0.86f;
    p.d_controller = (foc_pi_t){0.01f, 10, 0, -0.5f, 0.5f};
    p.q_controller = p.d_controller;
    memset(&before, 0, sizeof(before)); memset(&after, 0, sizeof(after));
    Before_FOC_AlgorithmInit(&before, &p); FOC_AlgorithmInit(&after, &p);
    for (i = 0; i < 100000U; ++i)
    {
      in.current_valid = in.angle_valid = in.reference_valid = 1;
      in.phase_a = random_float(-30, 30); in.phase_b = random_float(-30, 30);
      in.d_reference = random_float(-20, 20); in.q_reference = random_float(-20, 20);
      in.electrical_angle_rad = random_float(-50, 50);
      if (i % 101U == 0U) in.current_valid = 0;
      if (i % 103U == 0U) in.angle_valid = 0;
      if (i % 107U == 0U) in.reference_valid = 0;
      if (i % 109U == 0U) in.phase_a = special[(i / 109U) % 11U];
      if (i % 113U == 0U) in.electrical_angle_rad = special[(i / 113U) % 11U];
      Before_FOC_AlgorithmStep(&before, &in); FOC_AlgorithmStep(&after, &in);
      same(&before, &after, sizeof(before), "current loop state", i);
      if (i % 5U == 0U)
      {
        uint8_t a = Before_FOC_AlgorithmVoltageFeedbackStep(&before, 1, -2, 24, &in);
        uint8_t b = FOC_AlgorithmVoltageFeedbackStep(&after, 1, -2, 24, &in);
        if (a != b) return 3;
        same(&before, &after, sizeof(before), "voltage loop state", i);
      }
    }
  }
  puts("PASS bit-exact baseline: 100000 PI cases, 200000 modulation vectors, 200000 current-loop steps, 40000 voltage steps; full output, PI state and faults.");
  return 0;
}
