#ifndef CURRENT_LOOP_TEST_H
#define CURRENT_LOOP_TEST_H

#include <stdint.h>
#include "foc_algorithm.h"

#define CURRENT_TEST_MAX_SAMPLES 512U

typedef enum
{
  CURRENT_TEST_STEP = 0,
  CURRENT_TEST_SINE,
  CURRENT_TEST_KIND_COUNT
} current_test_kind_t;

typedef enum
{
  CURRENT_TEST_IDLE = 0,
  CURRENT_TEST_ARMED,
  CURRENT_TEST_RUNNING,
  CURRENT_TEST_DONE,
  CURRENT_TEST_ABORTED
} current_test_state_t;

typedef enum
{
  CURRENT_TEST_ABORT_NONE = 0,
  CURRENT_TEST_INVALID,
  CURRENT_TEST_MOE_OFF,
  CURRENT_TEST_ALGORITHM_INVALID
} current_test_abort_t;

typedef struct
{
  current_test_kind_t kind;
  uint8_t axis;                 /* 0 = d, 1 = q */
  float amplitude_a;
  float frequency_hz;           /* Sine test only. */
  float settle_s;
  float step_delay_s;
  uint16_t average_count;
  uint16_t capture_count;       /* Zero selects CURRENT_TEST_MAX_SAMPLES. */
} current_test_config_t;

typedef struct
{
  uint32_t source_sequence;
  float time_s;                /* Actual accumulated control time; block end if averaged. */
  float d_reference_a;
  float q_reference_a;
  float d_current_a;
  float q_current_a;
  float d_voltage_pu;
  float q_voltage_pu;
} current_test_sample_t;

typedef struct
{
  volatile uint32_t command;    /* 1=arm, 2=abort, 3=return to idle */
  volatile current_test_state_t state;
  volatile current_test_abort_t abort_reason;
  current_test_config_t requested;
  current_test_config_t active;
  foc_parameters_t parameters_at_arm;
  float dc_bus_at_arm_v;
  float sample_period_s;
  float elapsed_s;
  uint32_t reference_ticks;
  uint16_t sample_count;
  uint16_t target_count;
  uint16_t average_progress;
  uint32_t saturation_count;
  float maximum_absolute_error_a;
  current_test_sample_t accumulator;
  union {
    current_test_sample_t samples[CURRENT_TEST_MAX_SAMPLES];
    struct { float time_s, iq_command_a, speed_rad_s; } speed_prbs_samples[1365];
  }; /* Mutually exclusive current and speed commissioning recorders. */
} current_loop_test_t;

uint8_t CurrentTest_Arm(current_loop_test_t *test,
                        float current_limit_a,
                        float sample_period_s);
uint8_t CurrentTest_Reference(current_loop_test_t *test,
                              float sample_period_s,
                              float *d_reference_a,
                              float *q_reference_a);
void CurrentTest_Observe(current_loop_test_t *test,
                         const foc_output_t *output,
                         float sample_period_s,
                         uint32_t source_sequence,
                         uint8_t output_enabled);
void CurrentTest_Abort(current_loop_test_t *test,
                       current_test_abort_t reason);

#endif /* CURRENT_LOOP_TEST_H */
