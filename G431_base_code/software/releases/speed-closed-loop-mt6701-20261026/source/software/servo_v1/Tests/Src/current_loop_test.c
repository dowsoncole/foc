#include "current_loop_test.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CURRENT_TEST_TWO_PI 6.2831853071795864769f

static uint8_t CurrentTest_ConfigIsValid(const current_test_config_t *config,
                                         float current_limit_a,
                                         float sample_period_s)
{
  uint16_t count;
  if ((config == NULL) || (config->kind >= CURRENT_TEST_KIND_COUNT) ||
      (config->axis > 1U) || !isfinite(config->amplitude_a) ||
      (fabsf(config->amplitude_a) > current_limit_a) ||
      !isfinite(config->frequency_hz) ||
      !isfinite(config->settle_s) || (config->settle_s < 0.0f) ||
      !isfinite(config->step_delay_s) || (config->step_delay_s < 0.0f) ||
      !isfinite(current_limit_a) || (current_limit_a <= 0.0f) ||
      !isfinite(sample_period_s) || (sample_period_s <= 0.0f) ||
      (config->average_count == 0U)) return 0U;
  if ((config->kind == CURRENT_TEST_SINE) &&
      ((config->frequency_hz <= 0.0f) ||
       (config->frequency_hz >= 0.5f / sample_period_s))) return 0U;
  count = (config->capture_count == 0U) ?
      CURRENT_TEST_MAX_SAMPLES : config->capture_count;
  return (count <= CURRENT_TEST_MAX_SAMPLES) ? 1U : 0U;
}

void CurrentTest_Abort(current_loop_test_t *test,
                       current_test_abort_t reason)
{
  if (test == NULL) return;
  test->abort_reason = reason;
  test->state = CURRENT_TEST_ABORTED;
}

uint8_t CurrentTest_Arm(current_loop_test_t *test,
                        float current_limit_a,
                        float sample_period_s)
{
  current_test_config_t requested;
  if (test == NULL) return 0U;
  requested = test->requested;
  if (CurrentTest_ConfigIsValid(&requested, current_limit_a,
                                sample_period_s) == 0U)
  {
    CurrentTest_Abort(test, CURRENT_TEST_INVALID);
    return 0U;
  }

  test->active = requested;
  test->sample_period_s = sample_period_s;
  test->reference_ticks = 0UL;
  test->elapsed_s = 0.0f;
  test->sample_count = 0U;
  test->target_count = (requested.capture_count == 0U) ?
      CURRENT_TEST_MAX_SAMPLES : requested.capture_count;
  test->average_progress = 0U;
  test->saturation_count = 0UL;
  test->maximum_absolute_error_a = 0.0f;
  (void)memset(&test->accumulator, 0, sizeof(test->accumulator));
  test->abort_reason = CURRENT_TEST_ABORT_NONE;
  test->state = CURRENT_TEST_ARMED;
  return 1U;
}

uint8_t CurrentTest_Reference(current_loop_test_t *test,
                              float sample_period_s,
                              float *d_reference_a,
                              float *q_reference_a)
{
  float elapsed_s;
  float excitation_s;
  float reference = 0.0f;
  if ((test == NULL) || (d_reference_a == NULL) ||
      (q_reference_a == NULL) || !isfinite(sample_period_s) ||
      (sample_period_s <= 0.0f) ||
      ((test->state != CURRENT_TEST_ARMED) &&
       (test->state != CURRENT_TEST_RUNNING))) return 0U;

  if (test->state == CURRENT_TEST_ARMED) test->state = CURRENT_TEST_RUNNING;
  elapsed_s = test->elapsed_s;
  excitation_s = elapsed_s - test->active.settle_s -
      test->active.step_delay_s;
  if (excitation_s >= 0.0f)
  {
    if (test->active.kind == CURRENT_TEST_STEP)
      reference = test->active.amplitude_a;
    else if (test->active.kind == CURRENT_TEST_SINE)
      reference = test->active.amplitude_a * sinf(CURRENT_TEST_TWO_PI *
          test->active.frequency_hz * excitation_s);
    else
    {
      CurrentTest_Abort(test, CURRENT_TEST_INVALID);
      return 0U;
    }
  }
  *d_reference_a = (test->active.axis == 0U) ? reference : 0.0f;
  *q_reference_a = (test->active.axis == 1U) ? reference : 0.0f;
  test->reference_ticks++;
  test->elapsed_s += sample_period_s;
  return 1U;
}

void CurrentTest_Observe(current_loop_test_t *test,
                         const foc_output_t *output,
                         float sample_period_s,
                         uint32_t source_sequence,
                         uint8_t output_enabled)
{
  float error;
  float divisor;
  current_test_sample_t *sample;
  if ((test == NULL) || (test->state != CURRENT_TEST_RUNNING)) return;
  if ((output == NULL) || (output_enabled == 0U) ||
      (output->valid == 0U) || !isfinite(sample_period_s) ||
      (sample_period_s <= 0.0f) ||
      !isfinite(output->applied_d_reference) ||
      !isfinite(output->applied_q_reference) ||
      !isfinite(output->d_current) || !isfinite(output->q_current) ||
      !isfinite(output->d_voltage) || !isfinite(output->q_voltage))
  {
    CurrentTest_Abort(test, (output_enabled == 0U) ?
        CURRENT_TEST_MOE_OFF : CURRENT_TEST_ALGORITHM_INVALID);
    return;
  }

  test->accumulator.source_sequence = source_sequence;
  test->accumulator.time_s = test->elapsed_s - sample_period_s;
  test->accumulator.d_reference_a += output->applied_d_reference;
  test->accumulator.q_reference_a += output->applied_q_reference;
  test->accumulator.d_current_a += output->d_current;
  test->accumulator.q_current_a += output->q_current;
  test->accumulator.d_voltage_pu += output->d_voltage;
  test->accumulator.q_voltage_pu += output->q_voltage;
  test->average_progress++;
  if (output->saturated || output->reference_limited ||
      output->integral_limited) test->saturation_count++;
  error = (test->active.axis == 0U) ?
      output->applied_d_reference - output->d_current :
      output->applied_q_reference - output->q_current;
  if (fabsf(error) > test->maximum_absolute_error_a)
    test->maximum_absolute_error_a = fabsf(error);

  if (test->average_progress < test->active.average_count) return;
  if (test->sample_count >= test->target_count)
  {
    test->state = CURRENT_TEST_DONE;
    return;
  }
  divisor = (float)test->average_progress;
  sample = &test->samples[test->sample_count];
  *sample = test->accumulator;
  sample->d_reference_a /= divisor;
  sample->q_reference_a /= divisor;
  sample->d_current_a /= divisor;
  sample->q_current_a /= divisor;
  sample->d_voltage_pu /= divisor;
  sample->q_voltage_pu /= divisor;
  test->sample_count++;
  test->average_progress = 0U;
  (void)memset(&test->accumulator, 0, sizeof(test->accumulator));
  if (test->sample_count >= test->target_count)
    test->state = CURRENT_TEST_DONE;
}
