#include "current_loop_test.h"
#include "foc_current_tuning.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_CURRENT(condition) do { if (!(condition)) { \
  fprintf(stderr, "CURRENT LOOP FAIL line %d: %s\n", __LINE__, #condition); \
  exit(1); } } while (0)

void TestCurrentLoop(void)
{
  foc_pi_t pi = { 0.2f, 10.0f, 0.0f, -0.01f, 0.01f };
  foc_current_design_t design =
  {
    0.5f, 0.001f, 24.0f, 200.0f, 0.00005f, 0.000075f, 60.0f, 0.02f
  };
  foc_current_gains_t gains;
  current_loop_test_t test;
  foc_output_t output;
  float d_reference;
  float q_reference;
  unsigned int i;

  CHECK_CURRENT(FOC_PI_IsValid(&pi) != 0U);
  CHECK_CURRENT(FOC_PI_Step(&pi, 1.0f, 0.001f) > 0.0f);
  for (i = 0U; i < 100U; ++i) (void)FOC_PI_Step(&pi, 1.0f, 0.001f);
  CHECK_CURRENT(pi.integral == pi.integral_max);
  FOC_PI_Reset(&pi);
  CHECK_CURRENT(pi.integral == 0.0f);
  {
    float command = 123.0f;
    foc_pi_t invalid = pi;
    invalid.kp = NAN;
    CHECK_CURRENT(!FOC_PI_TryStep(&invalid, 1.0f, 0.001f, &command));
    CHECK_CURRENT(command == 0.0f && invalid.integral == 0.0f);
    CHECK_CURRENT(!FOC_PI_TryStep(&pi, NAN, 0.001f, &command));
    CHECK_CURRENT(!FOC_PI_TryStep(&pi, 1.0f, 0.0f, &command));
    CHECK_CURRENT(!FOC_PI_TryStep(NULL, 1.0f, 0.001f, &command));
    CHECK_CURRENT(!FOC_PI_TryStep(&pi, 1.0f, 0.001f, NULL));
    CHECK_CURRENT(FOC_PI_TryStep(&pi, 0.0f, 0.001f, &command));
    CHECK_CURRENT(command == 0.0f); /* Valid zero must not look like failure. */
  }

  CHECK_CURRENT(FOC_CurrentDesign(&design, &gains) != 0U);
  CHECK_CURRENT(gains.d.kp > 0.0f && gains.d.ki > 0.0f);
  CHECK_CURRENT(gains.achieved_bandwidth_hz <= design.bandwidth_hz);
  design.resistance_ohm = 0.0f;
  CHECK_CURRENT(FOC_CurrentDesign(&design, &gains) == 0U);

  (void)memset(&test, 0, sizeof(test));
  test.requested.kind = CURRENT_TEST_STEP;
  test.requested.axis = 0U;
  test.requested.amplitude_a = 0.05f;
  test.requested.step_delay_s = 0.0001f;
  test.requested.average_count = 1U;
  test.requested.capture_count = 4U;
  CHECK_CURRENT(CurrentTest_Arm(&test, 0.5f, 0.00005f) != 0U);
  (void)memset(&output, 0, sizeof(output));
  output.valid = 1U;
  for (i = 0U; i < 4U; ++i)
  {
    CHECK_CURRENT(CurrentTest_Reference(&test, 0.00005f,
        &d_reference, &q_reference) != 0U);
    output.applied_d_reference = d_reference;
    output.applied_q_reference = q_reference;
    output.d_current = d_reference;
    CurrentTest_Observe(&test, &output, 0.00005f, i, 1U);
  }
  CHECK_CURRENT(test.state == CURRENT_TEST_DONE);
  CHECK_CURRENT(test.sample_count == 4U);
  CHECK_CURRENT(test.samples[0].d_reference_a == 0.0f);
  CHECK_CURRENT(test.samples[2].d_reference_a == 0.05f);
  CHECK_CURRENT(fabsf(test.samples[3].time_s - 0.00015f) < 1e-8f);

  /* Dynamic trigger placement changes dt: accumulated time, not ticks*last_dt. */
  test.requested.step_delay_s = 0.00012f;
  CHECK_CURRENT(CurrentTest_Arm(&test, 0.5f, 0.00005f) != 0U);
  CHECK_CURRENT(CurrentTest_Reference(&test, 0.00010f, &d_reference, &q_reference));
  CHECK_CURRENT(d_reference == 0.0f);
  CHECK_CURRENT(CurrentTest_Reference(&test, 0.00004f, &d_reference, &q_reference));
  CHECK_CURRENT(d_reference == 0.0f);
  CHECK_CURRENT(CurrentTest_Reference(&test, 0.00003f, &d_reference, &q_reference));
  CHECK_CURRENT(d_reference == 0.05f);
}
