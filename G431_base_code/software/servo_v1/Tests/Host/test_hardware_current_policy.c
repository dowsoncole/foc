/* Reuse the real speed-loop harness, with the board's default current policy. */
#define main legacy_regression_main
#include "test_speed_commission.c"
#undef main
int main(void) {
  assert(CURRENT_LIMIT_HARDWARE_ONLY==1);
  for(unsigned kind=1;kind<=10;kind++) {
    reset(kind); g_speed_test.active.kind=kind; g_speed_test.state=2;
    assert(!SpeedCommission_PhaseCurrentFault(1000,-1000,500,.25f));
    assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,.25f));
  }
  reset(2);g_speed_test.requested.amplitude=30;arm();run_ms(500);
  assert(g_speed_test.active.current_limit_a==FLT_MAX && last_input.q_reference==30);
  reset(3);g_speed_test.requested.amplitude=5;arm();run_ms(1000);
  assert(last_input.q_reference>.5f); /* Ignore legacy .5 A config. */
  reset(7);g_speed_run_overload.request=1;g_speed_run_guard.timeout_ms=1000;
  arm();run_ms(500);
  assert(!g_speed_run_overload.armed && g_speed_test.active.current_limit_a==FLT_MAX);
  assert(!SpeedCommission_PhaseCurrentFault(1000,-1000,500,.25f));
  puts("PASS hardware current policy: all phase thresholds bypassed, bounded/continuous Iq unclipped, I2t disabled, NaN rejected");
}
