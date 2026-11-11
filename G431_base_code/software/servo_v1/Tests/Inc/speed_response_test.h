#ifndef SPEED_RESPONSE_TEST_H
#define SPEED_RESPONSE_TEST_H
#include <stdint.h>
/* Explicit, bounded commissioning only. The existing current recorder RAM is
   borrowed while CURRENT_TEST_IDLE; no production control path is replaced. */
typedef struct {
  uint32_t kind, divider, samples; /* 0 step, 1 sine; outer ticks per row. */
  float bias_rpm, amplitude_rpm, frequency_hz, delay_s;
} speed_response_config_t;
typedef struct {
  volatile uint32_t command, state, count, fault; /* arm=1/cancel=2; 0..4 states */
  speed_response_config_t requested, active;
  uint32_t first_cycle, ticks, saturated_rows, last_cycle;
  volatile uint32_t version; /* Read-only: v2 records 100 ms pre-step after warmup. */
} speed_response_test_t;
extern volatile speed_response_test_t g_speed_response;
void SpeedResponse_Service(uint8_t output_off);
#endif
