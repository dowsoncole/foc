/* Compile the REAL speed_commission.c with clock/encoder stubs.
   These are software-state tests, not a motor or PI-performance simulation. */
#include "speed_commission.h"
#include "encoder_calibration.h"
#include "user_control.h"
#include "speed_overload_guard.h"
#include "encoder.h"
#include "current_loop_test.h"
#include "stm32g4xx_hal.h"
#include "mt6835_crc.h"
#include "encoder_frame_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

speed_test_dwt_t speed_test_dwt;
uint32_t SystemCoreClock=168000000U;
current_loop_test_t g_current_loop_test;
foc_controller_t g_foc_controller;
static encoder_snapshot_t encoder;
static uint8_t encoder_valid=1;
static uint8_t encoder_configure_ok=1;
static foc_input_t last_input;
static uint8_t calibration_save(int32_t d,uint32_t p,float a,
                                const encoder_calibration_config_t *c) {
  return (d==1 || d==-1) && p==2 && isfinite(a) && c->d_voltage_v>0;
}
static uint8_t calibration_apply(int32_t d,uint32_t p,float a) {
  if(!Encoder_ConfigureMotor((int8_t)d,(uint16_t)p,a)) return 0;
  g_speed_test.electrical_offset_rad=a;
  g_speed_test.calibration_ok=1;
  if(g_speed_test.active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND &&
     g_speed_test.state==3U && !g_speed_test.fault) g_speed_test.state=0U;
  return 1;
}
static uint32_t calibration_fault(void) { return g_speed_test.fault; }

uint8_t Encoder_ReadSnapshot(encoder_snapshot_t *s) { *s=encoder; return encoder_valid; }
uint8_t Encoder_ConfigureMotor(int8_t d,uint16_t p,float a) {
  return encoder_configure_ok && (d==1 || d==-1) && p==2 && isfinite(a);
}
static void reset(uint32_t kind) {
  static const encoder_calibration_port_t port={.save=calibration_save,
    .apply=calibration_apply,.backend_fault=calibration_fault};
  EncoderCalibration_Init(&port,-1,2);
  memset(&g_speed_test,0,sizeof(g_speed_test));
  memset(&g_speed_run,0,sizeof(g_speed_run));
  memset(&g_speed_run_guard,0,sizeof(g_speed_run_guard));
  memset(&g_speed_run_overload,0,sizeof(g_speed_run_overload));
  memset(&g_current_loop_test,0,sizeof(g_current_loop_test));
  memset((void *)&g_speed_response,0,sizeof(g_speed_response));
  memset(&encoder,0,sizeof(encoder));
  g_current_loop_test.state=CURRENT_TEST_IDLE;
  g_speed_test.calibration_ok=1;
  g_speed_test.requested=(speed_test_config_t){
    .kind=kind,.duration_s=2,.amplitude=10,.kp=.032f,.ki=1,
    .current_limit_a=.5f,.speed_limit_rad_s=kind==7 ? 950 : 20,
    .record_divider=8,.prbs_bit_ms=16};
  g_speed_run.enable=1;
  g_speed_run.target_rpm=500;
  encoder.angle_valid=encoder.speed_valid=encoder.electrical_angle_valid=1;
  encoder_valid=1;
  DWT->CYCCNT=0xFFF00000U; /* Start close to hardware counter wrap. */
}
static void arm(void) {
  g_speed_test.command=1;
  SpeedCommission_Service(1);
  assert(g_speed_test.state==1 && g_speed_test.fault==0);
}
static uint8_t tick(void) {
  foc_input_t in={0};
  DWT->CYCCNT+=CONTROL_PERIOD_TICKS; /* 50 us; intentional uint32 wrap. */
  encoder.sample_cycle=DWT->CYCCNT;
  encoder.multi_turn_count+=100;
  uint8_t ok=SpeedCommission_Reference(&in);
  last_input=in;
  SpeedCommission_PublishRunTelemetry();
  if(ok) assert(isfinite(in.q_reference) &&
                fabsf(in.q_reference)<=g_speed_test.active.current_limit_a+.000001f);
  return ok;
}
static void run_ms(unsigned ms) {
  for(unsigned i=0;i<ms*CONTROL_SAMPLES_PER_MS;i++) assert(tick());
}
static void test_deferred_outer(void) {
 float expected[300], actual[300];
 reset(7); arm();
 for(unsigned i=0;i<300;i++) {
   assert(tick());
   expected[i]=last_input.q_reference;
 }
 reset(7); arm();
 float held=0;
 uint32_t carrier=DWT->CYCCNT;
 for(unsigned i=0;i<300;i++) {
   foc_input_t in={0};
   carrier+=CONTROL_PERIOD_TICKS;
   DWT->CYCCNT=carrier;
   encoder.sample_cycle=DWT->CYCCNT;
   encoder.multi_turn_count+=100;
   encoder.electrical_angle_rad=(float)i*.001f;
   assert(SpeedCommission_FastReference(&in));
   if(g_speed_test.state==2) {
     assert(in.electrical_angle_rad==encoder.electrical_angle_rad);
     assert(in.q_reference==held);
   }
   DWT->CYCCNT+=SystemCoreClock/40000U; /* 25 us real dispatch latency. */
   SpeedCommission_OuterService();
   held=g_speed_test.iq_reference;
   actual[i]=held;
   assert(actual[i]==expected[i]);
 }
 g_speed_run.enable=0;
 foc_input_t in={0};
 assert(SpeedCommission_FastReference(&in));
 assert(in.q_reference==0 && g_speed_test.state==3);
 SpeedCommission_OuterService();
 assert(g_speed_test.state==3);
 reset(7); arm(); run_ms(10);
 encoder_valid=0;
 assert(!SpeedCommission_FastReference(&in) && g_speed_test.fault==4);
 reset(7); arm(); run_ms(10);
 /* Keep ADC fresh while deliberately starving PendSV beyond 2 ms. */
 for(unsigned i=0;i<2U*CONTROL_SAMPLES_PER_MS+1U;i++) {
   DWT->CYCCNT+=CONTROL_PERIOD_TICKS;
   encoder.sample_cycle=DWT->CYCCNT;
   uint8_t ok=SpeedCommission_FastReference(&in);
   if(!ok) break;
 }
 assert(g_speed_test.state==4 && g_speed_test.fault==6);
 SpeedCommission_OuterService();
 assert(g_speed_test.state==4 && g_speed_test.fault==6);
 puts("PASS deferred outer PI bit-exact against legacy schedule, fresh per-ADC angle, held Iq and stop/fault");
}
static void test_user_speed_ramp(void) {
  reset(7);
  g_user_control.status.state=USER_RUNNING;
  g_user_control.status.active_mode=USER_MODE_SPEED;
  g_user_control.speed.target_rpm=500;
  g_user_control.speed.acceleration_rpm_s=100;
  g_user_control.speed.deceleration_rpm_s=200;
  g_user_control.speed.forward_limit_rpm=400;
  g_user_control.speed.reverse_limit_rpm=300;
  arm(); run_ms(105);
  assert(g_speed_run.applied_rpm>9.8f && g_speed_run.applied_rpm<10.2f);
  g_speed_run.target_rpm=0; run_ms(100);
  assert(g_speed_run.applied_rpm==0);
  g_speed_run.target_rpm=-1000; run_ms(4000);
  assert(fabsf(g_speed_run.applied_rpm+300)<.01f);
  g_user_control.speed.acceleration_rpm_s=NAN;
  for(unsigned i=0;i<CONTROL_SAMPLES_PER_MS && g_speed_test.state!=4;i++) (void)tick();
  assert(g_speed_test.state==4 && g_speed_test.fault==9);
  g_user_control.status.state=USER_IDLE;
  puts("PASS real speed executor uses user acceleration/deceleration/reverse limit and rejects invalid live tuning");
}
static void reset_encoder_debug(void) {
  reset(ENCODER_DEBUG_OPEN_LOOP_KIND);
  g_speed_run.enable=0;
  g_speed_test.calibration_ok=0; /* Synthetic angle needs no encoder profile. */
  g_speed_test.requested=(speed_test_config_t){
    .kind=ENCODER_DEBUG_OPEN_LOOP_KIND,.duration_s=1.0f,.amplitude=.1f,
    .frequency_hz=.5f,.kp=0,.ki=0,.current_limit_a=.3f,
    .speed_limit_rad_s=20,.record_divider=8,.prbs_bit_ms=16};
}
static void test_encoder_debug(void) {
  reset_encoder_debug();
  encoder_valid=0;
  encoder.angle_valid=encoder.speed_valid=encoder.electrical_angle_valid=0;
  encoder.mechanical_speed_rad_s=encoder.electrical_angle_rad=NAN;
  arm();
  for(uint32_t faults=1;faults<64;faults++) {
    encoder.fault_flags=faults;
    run_ms(10);
    assert(g_speed_test.feedback_fault==faults && g_speed_test.state==2);
    assert(last_input.angle_valid && last_input.reference_valid);
    assert(isfinite(last_input.electrical_angle_rad) &&
           last_input.electrical_angle_rad>=0 && last_input.electrical_angle_rad<6.283186f);
    assert(last_input.d_reference>=0 && last_input.d_reference<=.100001f);
    assert(last_input.q_reference==0 && g_speed_test.iq_reference==0);
  }
  assert(g_speed_test.angle>0 && g_speed_test.reference>0);
  assert(g_speed_test.reference<=g_speed_test.active.frequency_hz);
  for(unsigned i=0;i<8000 && g_speed_test.state==2;i++) assert(tick());
  assert(g_speed_test.state==3 && g_speed_test.fault==0);
  assert(last_input.d_reference==0 && last_input.q_reference==0);
  puts("PASS bounded synthetic-angle debug ignores all encoder faults, never consumes NaN angle/speed");

  reset_encoder_debug(); arm(); run_ms(100);
  g_speed_test.command=2; SpeedCommission_Service(0);
  assert(g_speed_test.state==4 && g_speed_test.fault==1);
  reset_encoder_debug(); arm(); run_ms(100);
  DWT->CYCCNT+=SystemCoreClock/1000;
  assert(!tick() && g_speed_test.fault==6);
  reset_encoder_debug(); arm(); run_ms(100);
  DWT->CYCCNT+=SystemCoreClock/1000;
  SpeedCommission_Service(0); /* ADC stopped: background must still fault. */
  assert(g_speed_test.state==4 && g_speed_test.fault==6);
  reset_encoder_debug(); arm();
  DWT->CYCCNT+=SystemCoreClock/5U+1U;
  SpeedCommission_Service(0); /* No first ADC callback after arming. */
  assert(g_speed_test.state==4 && g_speed_test.fault==8);
  reset_encoder_debug(); arm();
  for(unsigned i=0;i<600000;i++) {
    DWT->CYCCNT+=1; /* Many callbacks must not advance the fixed 250 ms dwell. */
    assert(SpeedCommission_Reference(&last_input));
  }
  assert(g_speed_test.angle==0 && g_speed_test.reference==0);
  puts("PASS debug abort, ISR/background missed deadlines, missing first ADC and wall-clock alignment dwell");

  for(unsigned which=0;which<9;which++) {
    reset_encoder_debug();
    switch(which) {
      case 0: g_speed_test.requested.duration_s=ENCODER_DEBUG_MAX_DURATION_S+.01f; break;
      case 1: g_speed_test.requested.frequency_hz=ENCODER_DEBUG_MAX_ELECTRICAL_HZ+.01f; break;
      case 2: g_speed_test.requested.current_limit_a=ENCODER_DEBUG_MAX_CURRENT_A+.01f; break;
      case 3: g_speed_test.requested.amplitude=.31f; break;
      case 4: g_speed_test.requested.amplitude=-.1f; break;
      case 5: g_speed_test.requested.kp=.01f; break;
      case 6: g_speed_test.requested.ki=.01f; break;
      case 7: g_speed_test.requested.frequency_hz=NAN; break;
      default: g_current_loop_test.state=CURRENT_TEST_ARMED; break;
    }
    g_speed_test.command=1; SpeedCommission_Service(1);
    assert(g_speed_test.state==4 && g_speed_test.fault==3);
  }
  reset_encoder_debug(); g_speed_test.command=1; SpeedCommission_Service(0);
  assert(g_speed_test.fault==2);
  for(uint32_t state=1;state<=2;state++) {
    g_speed_test.active.kind=ENCODER_DEBUG_OPEN_LOOP_KIND;
    g_speed_test.state=state;
    assert(SpeedCommission_PhaseCurrentFault(.81f,0,0,ENCODER_DEBUG_PHASE_TRIP_A));
    assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,ENCODER_DEBUG_PHASE_TRIP_A));
    assert(!SpeedCommission_PhaseCurrentFault(.1f,-.1f,0,ENCODER_DEBUG_PHASE_TRIP_A));
  }
  puts("PASS debug current/frequency/time caps, mutual exclusion, output-off interlock and current trips");

  reset(3); g_speed_run.enable=0; g_speed_test.calibration_ok=0;
  g_speed_test.command=ENCODER_DEBUG_START_COMMAND; SpeedCommission_Service(1);
  assert(g_speed_test.state==1 && g_speed_test.active.kind==ENCODER_DEBUG_OPEN_LOOP_KIND);
  assert(g_speed_test.active.duration_s==ENCODER_DEBUG_MAX_DURATION_S &&
         g_speed_test.active.amplitude==.1f && g_speed_test.active.frequency_hz==.5f);
  encoder_valid=0;
  encoder.fault_flags=ENCODER_FAULT_CRC|ENCODER_FAULT_STALE;
  for(unsigned i=0;i<102000 && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && g_speed_test.fault==0 &&
         g_speed_test.elapsed>=5 && g_speed_test.elapsed<5.001f);
  assert(g_speed_test.reference<=.5f && isfinite(g_speed_test.angle));
  reset(7); arm(); run_ms(10); encoder_valid=0;
  assert(!tick() && g_speed_test.fault==4);
  reset(7); g_speed_test.calibration_ok=0;
  g_speed_test.command=1; SpeedCommission_Service(1);
  assert(g_speed_test.fault==3);
  puts("PASS one-command preset stops at 5 s across DWT wrap; normal speed feedback/calibration gates unchanged");
}
static void test_host_guard(void) {
  reset(7); g_speed_run_guard.timeout_ms=100; arm();
  assert(!SpeedCommission_PhaseCurrentFault(6.0f,-6.0f,0,15.0f));
  assert(!SpeedCommission_PhaseCurrentFault(40.0f,-20.0f,-20.0f,15.0f));
  assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,15.0f));
  assert(!SpeedCommission_PhaseCurrentFault(6.01f,0,0,15.0f));
  assert(!SpeedCommission_PhaseCurrentFault(0,-6.01f,0,15.0f));
  assert(!SpeedCommission_PhaseCurrentFault(0,0,6.01f,15.0f));
  run_ms(50); g_speed_run_guard.heartbeat++; run_ms(50);
  assert(g_speed_test.state==2 && !g_speed_run_guard.expired);
  /* Config writes cannot disable or extend an armed watchdog. */
  g_speed_run_guard.timeout_ms=0;
  assert(!SpeedCommission_PhaseCurrentFault(6.01f,0,0,15.0f));
  for(unsigned i=0;i<2100 && g_speed_test.state==2;i++) (void)tick();
  assert(g_speed_test.state==4 && g_speed_test.fault==13 &&
         !g_speed_run.enable && g_speed_run_guard.expired);
  g_speed_run_guard.heartbeat++; SpeedCommission_Service(0);
  assert(g_speed_test.state==4 && !g_speed_run.enable);
  reset(7); g_speed_run_guard.timeout_ms=100; arm(); run_ms(10);
  DWT->CYCCNT+=SystemCoreClock/10;
  g_speed_run_guard.heartbeat++; /* Too late, including across DWT wrap. */
  SpeedCommission_Service(0);
  assert(g_speed_test.fault==13 && g_speed_run_guard.expired);
  reset(7); g_speed_run_guard.timeout_ms=100; arm();
  DWT->CYCCNT+=SystemCoreClock/10; SpeedCommission_Service(0);
  assert(g_speed_test.fault==13); /* Armed, no ADC ever delivered. */
  for(unsigned i=0;i<2;i++) {
    reset(7); g_speed_run_guard.timeout_ms=i ? 10001U : 99U;
    g_speed_test.command=1; SpeedCommission_Service(1);
    assert(g_speed_test.state==4 && g_speed_test.fault==3);
  }
  puts("PASS host watchdog expiry/heartbeat/late write/latched timeout/background/armed/DWT wrap");
}
static void test_speed_command_boundary(void) {
  for(int sign=-1;sign<=1;sign+=2) {
    reset(7); g_speed_run_guard.timeout_ms=8000;
    g_speed_run.target_rpm=(float)sign*SPEED_RUN_MAX_RPM;
    arm(); run_ms(10);
    assert(g_speed_test.state==2);
    assert(!SpeedCommission_PhaseCurrentFault(6.01f,0,0,15.0f));
    g_speed_run.target_rpm=(float)sign*(SPEED_RUN_MAX_RPM+1.0f);
    for(unsigned i=0;i<30 && g_speed_test.state==2;i++) (void)tick();
    assert(g_speed_test.state==4 && g_speed_test.fault==9);
    reset(7); g_speed_run.target_rpm=(float)sign*(SPEED_RUN_MAX_RPM+1.0f);
    g_speed_test.command=1; SpeedCommission_Service(1);
    assert(g_speed_test.state==4 && g_speed_test.fault==3);
  }
  puts("PASS signed command ceilings at arm/live update; host watchdog does not impose a phase-amplitude ceiling");
}
static void test_overload_guard(void) {
  speed_overload_guard_state_t s={0};
  assert(!SpeedOverload_Arm(&s,1,0,168000000U));
  assert(!SpeedOverload_Arm(&s,2,100,168000000U));
  assert(!SpeedOverload_Arm(&s,1,100,UINT32_MAX));
  assert(SpeedOverload_Arm(&s,1,8000,168000000U));
  const float currents[5]={6,12,15,18,24};
  for(unsigned trial=0;trial<5;trial++) {
    s=(speed_overload_guard_state_t){0};
    assert(SpeedOverload_Arm(&s,1,8000,SystemCoreClock));
    uint32_t now=UINT32_MAX-1000U, samples=0;
    assert(SpeedOverload_Observe(&s,now,currents[trial]*currents[trial]));
    while(samples<400000U && !s.expired) {
      now+=CONTROL_PERIOD_TICKS; samples++;
      (void)SpeedOverload_Observe(&s,now,currents[trial]*currents[trial]);
    }
    if(trial==0) assert(!s.expired && !s.started && s.excess_i2_cycles_q16==0);
    else {
      float expected=1440.0f/(currents[trial]*currents[trial]-36.0f);
      float actual=(float)samples/(float)CONTROL_FREQUENCY_HZ;
      assert(s.expired && !s.invalid && fabsf(actual-expected)<.0011f);
      assert(!SpeedOverload_Check(&s,now));
      printf("PASS RMS %.0f A I2t trip %.5f s, analytic %.5f s\n",currents[trial],actual,expected);
    }
  }
  assert(!SpeedOverload_Arm(&s,1,8000,168000000U));
  assert(SpeedOverload_Arm(&s,0,8000,168000000U));
  assert(!SpeedOverload_Arm(&s,1,8000,168000000U));
  s=(speed_overload_guard_state_t){0};
  assert(SpeedOverload_Arm(&s,1,8000,SystemCoreClock));
  uint32_t now=0;
  assert(SpeedOverload_Observe(&s,now,144));
  for(unsigned i=0;i<2U*CONTROL_FREQUENCY_HZ;i++) assert(SpeedOverload_Observe(&s,now+=CONTROL_PERIOD_TICKS,144));
  uint64_t debt=s.excess_i2_cycles_q16;
  assert(debt==(uint64_t)216U*SystemCoreClock*65536U);
  for(unsigned i=0;i<2U*CONTROL_FREQUENCY_HZ;i++) assert(SpeedOverload_Observe(&s,now+=CONTROL_PERIOD_TICKS,36));
  assert(s.excess_i2_cycles_q16==debt); /* No invented cooling/reset below rated. */
  assert(!SpeedOverload_Check(&s,now+SystemCoreClock/2000U+1U) && s.invalid);
  s=(speed_overload_guard_state_t){0};
  assert(SpeedOverload_Arm(&s,1,8000,SystemCoreClock));
  assert(!SpeedOverload_Observe(&s,0,NAN) && s.invalid);

  /* Preserve small near-rated excess, with no float accumulator cancellation.
     Also cover nonconstant loading within the 1 ms square-average block. */
  s=(speed_overload_guard_state_t){0};
  assert(SpeedOverload_Arm(&s,1,8000,SystemCoreClock));
  now=0;
  const float near_rated_square=6.01f*6.01f;
  assert(SpeedOverload_Observe(&s,now,near_rated_square));
  for(unsigned i=0;i<10U*CONTROL_FREQUENCY_HZ;i++)
    assert(SpeedOverload_Observe(&s,now+=CONTROL_PERIOD_TICKS,near_rated_square));
  float energy=(float)s.excess_i2_cycles_q16/((float)SystemCoreClock*65536.0f);
  assert(fabsf(energy-(near_rated_square-36.0f)*10.0f)<.001f);
  s=(speed_overload_guard_state_t){0};
  assert(SpeedOverload_Arm(&s,1,8000,SystemCoreClock));
  now=0;
  assert(SpeedOverload_Observe(&s,now,0));
  for(unsigned i=0;i<CONTROL_FREQUENCY_HZ;i++)
    assert(SpeedOverload_Observe(&s,now+=CONTROL_PERIOD_TICKS,(i%CONTROL_SAMPLES_PER_MS)<(CONTROL_SAMPLES_PER_MS/2U) ? 144.0f : 0.0f));
  assert(s.excess_i2_cycles_q16==(uint64_t)36U*SystemCoreClock*65536U);
  assert(fabsf(SpeedOverload_MeanSquare(&s)-72.0f)<.0001f);
  assert(!SpeedOverload_Observe(&s,now,144) && s.invalid); /* Duplicate time. */

  s=(speed_overload_guard_state_t){0};
  assert(!SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,NAN));
  assert(!SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,-1));
  assert(!SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,1440));
  assert(!SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,1439.9999f));
  assert(SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,82.563f));
  energy=(float)s.excess_i2_cycles_q16/((float)SystemCoreClock*65536.0f);
  assert(energy>=82.563f && energy<82.564f);
  now=UINT32_MAX-1000U;
  assert(SpeedOverload_Observe(&s,now,324));
  unsigned seeded_samples=0;
  while(!s.expired && seeded_samples<100000U) {
    seeded_samples++; now+=CONTROL_PERIOD_TICKS; (void)SpeedOverload_Observe(&s,now,324);
  }
  assert(s.expired && fabsf((float)seeded_samples/(float)CONTROL_FREQUENCY_HZ-(1440.0f-energy)/288.0f)<.0011f);
  debt=s.excess_i2_cycles_q16;
  assert(SpeedOverload_Arm(&s,0,8000,SystemCoreClock) && s.excess_i2_cycles_q16==debt);
  assert(!SpeedOverload_ArmWithDebt(&s,1,8000,SystemCoreClock,0));
  puts("PASS warm-start heat rounded up, earlier trip and retained debt across non-I2t arm");

  reset(7); g_speed_run_overload.request=1;
  g_speed_test.command=1; SpeedCommission_Service(1);
  assert(g_speed_test.fault==3); /* Cannot relax phase guard without deadman. */
  reset(7); g_speed_run_guard.timeout_ms=8000; g_speed_run_overload.request=1;
  g_speed_test.requested.current_limit_a=SPEED_RUN_OVERLOAD_IQ_LIMIT_A;
  g_speed_test.requested.kp=.2f; g_speed_run.target_rpm=8500;
  arm(); run_ms(1000);
  assert(g_speed_test.iq_reference>6.0f && g_speed_test.iq_reference<=SPEED_RUN_OVERLOAD_IQ_LIMIT_A);
  assert(!SpeedCommission_PhaseCurrentFault(30.01f,0,0,15));
  assert(tick()); /* I2t observations require distinct ADC sample times. */
  assert(!SpeedCommission_PhaseCurrentFault(0,-30.01f,0,15));
  assert(tick());
  assert(!SpeedCommission_PhaseCurrentFault(0,0,30.01f,15));
  assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,15));
  g_speed_run_overload.request=g_speed_run_overload.started=0;
  g_speed_run_guard.timeout_ms=0; /* Neither public write disables latched guards. */
  unsigned steps=0;
  for(;steps<5U*CONTROL_FREQUENCY_HZ+100U;steps++) {
    assert(tick());
    if(! (steps%CONTROL_FREQUENCY_HZ)) g_speed_run_guard.heartbeat++;
    if(SpeedCommission_PhaseCurrentFault(25.4558441f,-12.7279221f,-12.7279221f,15)) break;
  }
  SpeedCommission_Service(0);
  assert(steps>=5U*CONTROL_FREQUENCY_HZ-CONTROL_SAMPLES_PER_MS && steps<5U*CONTROL_FREQUENCY_HZ+100U);
  assert(g_speed_test.fault==14 && g_speed_test.state==4 && !g_speed_run.enable);
  assert(g_speed_run_overload.expired && g_speed_run_overload.energy_a2s>=1440);
  reset(7); g_speed_run_guard.timeout_ms=8000; g_speed_run_overload.request=1;
  g_speed_test.command=1; SpeedCommission_Service(1);
  assert(g_speed_test.fault==3); /* Resetting public RAM cannot rearm one-shot. */
  puts("PASS RMS I2t integration, retained debt, missing/nonfinite feedback, DWT wrap and no RAM rearm");
}
static void test_start_direction_guard(void) {
  for(int sign=-1;sign<=1;sign+=2) {
    reset(7); g_speed_run_guard.timeout_ms=8000;
    g_speed_run.target_rpm=(float)sign*500;
    arm(); run_ms(200);
    encoder.mechanical_speed_rad_s=(float)-sign*20;
    for(unsigned i=0;i<30 && g_speed_test.state==2;i++) (void)tick();
    assert(g_speed_test.fault==16 && g_speed_test.state==4 && !g_speed_run.enable);
    reset(7); g_speed_run_guard.timeout_ms=8000;
    g_speed_run.target_rpm=(float)sign*500;
    arm(); run_ms(200);
    encoder.mechanical_speed_rad_s=(float)sign*20; run_ms(2);
    assert(g_speed_test.state==2 && !g_speed_test.fault);
  }
  reset(7); g_speed_run_guard.timeout_ms=8000;
  arm(); run_ms(2200);
  encoder.mechanical_speed_rad_s=-20; run_ms(2);
  assert(g_speed_test.state==2); /* Established reversal is not a startup fault. */
  puts("PASS guarded startup wrong-direction trip, both signs and established reversal");
}
static void reset_alignment_sweep(void) {
  reset(SPEED_ALIGNMENT_SWEEP_KIND);
  g_speed_run.enable=0;
  g_speed_test.calibration_ok=0;
  encoder.electrical_angle_valid=0;
  encoder.electrical_angle_rad=NAN;
  g_speed_test.requested=(speed_test_config_t){
    .kind=SPEED_ALIGNMENT_SWEEP_KIND,.duration_s=12,.amplitude=3,
    .angle_end_rad=SPEED_ALIGNMENT_SWEEP_DISTANCE_RAD,
    .current_limit_a=3,.speed_limit_rad_s=20,.record_divider=100,.prbs_bit_ms=16};
}
static void test_alignment_sweep(void) {
  reset_alignment_sweep(); arm();
  float previous=0, maximum=0;
  for(unsigned i=0;i<241000 && g_speed_test.state!=3;i++) {
    assert(tick());
    assert(last_input.angle_valid && isfinite(last_input.electrical_angle_rad));
    assert(fabsf(last_input.electrical_angle_rad-previous)<.0005f);
    assert(last_input.electrical_angle_rad>=0 &&
           last_input.electrical_angle_rad<=SPEED_ALIGNMENT_SWEEP_DISTANCE_RAD);
    assert(last_input.d_reference>=0 && last_input.d_reference<=3);
    assert(last_input.q_reference==0);
    if(g_speed_test.elapsed<3 || g_speed_test.elapsed>=9)
      assert(last_input.electrical_angle_rad==0);
    if(last_input.electrical_angle_rad>maximum) maximum=last_input.electrical_angle_rad;
    previous=last_input.electrical_angle_rad;
  }
  assert(g_speed_test.state==3 && g_speed_test.fault==0);
  assert(maximum>12.56f && g_speed_test.count>=119 && g_speed_test.count<=128);
  assert(last_input.d_reference==0 && last_input.q_reference==0);
  assert(g_speed_test.records[40].reference>0 && g_speed_test.records[70].reference>0);
  for(unsigned which=0;which<7;which++) {
    reset_alignment_sweep();
    switch(which) {
      case 0: g_speed_test.requested.duration_s=11; break;
      case 1: g_speed_test.requested.angle_end_rad=-12.5663706f; break;
      case 2: g_speed_test.requested.amplitude=3.01f; break;
      case 3: g_speed_test.requested.record_divider=99; break;
      case 4: g_speed_test.requested.kp=.01f; break;
      case 5: g_speed_test.requested.bias=.1f; break;
      default: g_speed_test.requested.amplitude=0; break;
    }
    g_speed_test.command=1; SpeedCommission_Service(1);
    assert(g_speed_test.state==4 && g_speed_test.fault==3);
  }
  reset_alignment_sweep(); arm(); run_ms(100); encoder_valid=0;
  assert(!tick() && g_speed_test.fault==4);
  reset_alignment_sweep(); arm(); run_ms(100); encoder.mechanical_speed_rad_s=20.01f;
  assert(!tick() && g_speed_test.fault==5);
  reset_alignment_sweep(); arm(); run_ms(100); DWT->CYCCNT+=SystemCoreClock/1000;
  SpeedCommission_Service(0);
  assert(g_speed_test.state==4 && g_speed_test.fault==6);
  reset_alignment_sweep(); arm(); DWT->CYCCNT+=SystemCoreClock/5U+1U;
  SpeedCommission_Service(0);
  assert(g_speed_test.state==1); /* No ADC/no excitation: OFF reservation. */
  assert(tick()); /* Starts the neutral readiness window, not the 3 A ramp. */
  assert(last_input.d_reference==0 && g_speed_test.ready_count==1);
  DWT->CYCCNT+=SystemCoreClock/5U+1U;
  SpeedCommission_Service(0);
  assert(g_speed_test.state==4 && g_speed_test.fault==8);
  reset_alignment_sweep(); g_current_loop_test.state=CURRENT_TEST_ARMED;
  g_speed_test.command=1; SpeedCommission_Service(1);
  assert(g_speed_test.state==4 && g_speed_test.fault==3);
  assert(SpeedCommission_PhaseCurrentFault(15.01f,0,0,15));
  puts("PASS 12 s bidirectional full-mechanical alignment, recorded field, ramp/time/current/feedback/speed gates");
}
static void test_alignment_near_rotor_start(void) {
  for(int sign=-1;sign<=1;sign++) {
    reset(1); g_speed_run.enable=0; g_speed_test.calibration_ok=0;
    g_speed_test.requested=(speed_test_config_t){
      .kind=1,.duration_s=5,.amplitude=3,.bias=(float)sign*3.0f,
      .angle_end_rad=-4.712389f,.current_limit_a=3,.speed_limit_rad_s=20,
      .record_divider=42,.prbs_bit_ms=16};
    arm();
    for(unsigned i=0;i<101000 && g_speed_test.state!=3;i++) {
      assert(tick());
      if(g_speed_test.first_cycle && g_speed_test.state==2) {
        float p=fmaxf(0,fminf(1,(g_speed_test.elapsed/5.0f-.25f)*2.0f));
        float expected=(float)sign*3.0f+p*(-4.712389f-(float)sign*3.0f);
        assert(fabsf(last_input.electrical_angle_rad-expected)<.00001f);
        assert(fabsf(g_speed_test.reference-expected)<.00001f);
        assert(last_input.d_reference>=0 && last_input.d_reference<=3);
        assert(last_input.q_reference==0);
      }
    }
    assert(g_speed_test.state==3 && g_speed_test.fault==0);
    assert(!g_speed_test.calibration_ok); /* Rough start never certifies zero. */
    assert(last_input.d_reference==0 && last_input.q_reference==0);
  }
  reset(1); g_speed_test.requested.amplitude=3;
  g_speed_test.requested.current_limit_a=3;
  g_speed_test.requested.bias=3.2f;
  g_speed_test.command=1; SpeedCommission_Service(1);
  assert(g_speed_test.state==4 && g_speed_test.fault==3);
  reset(1); g_speed_test.requested.amplitude=3;
  g_speed_test.requested.current_limit_a=3;
  g_speed_test.requested.bias=3.0f;
  arm(); run_ms(100); encoder.mechanical_speed_rad_s=20.01f;
  assert(!tick() && g_speed_test.fault==5);
  puts("PASS near-rotor static alignment, legacy zero bias, recorded field, finite bounds and unchanged overspeed trip");
}
static void reset_voltage_alignment(void) {
  reset(SPEED_VOLTAGE_ALIGNMENT_KIND);
  g_speed_test.direction=SPEED_BOARD_ENCODER_DIRECTION;
  g_speed_test.pole_pairs=SPEED_BOARD_ENCODER_POLE_PAIRS;
  encoder.raw_angle=100000U;
  g_speed_test.calibration_ok=0;
  encoder.electrical_angle_valid=0;
  encoder.electrical_angle_rad=NAN; /* Must use the synthetic field instead. */
  g_speed_voltage_alignment.dc_bus_voltage_v=24;
  g_speed_test.requested=(speed_test_config_t){
    .kind=SPEED_VOLTAGE_ALIGNMENT_KIND,.duration_s=5,.amplitude=1,
    .bias=.4f,.angle_end_rad=1.570796327f,.current_limit_a=15,
    .speed_limit_rad_s=20,.record_divider=40,.prbs_bit_ms=16};
}
static void test_voltage_alignment(void) {
  reset_voltage_alignment(); arm(); run_ms(100);
  assert(last_input.d_reference==0 && last_input.q_reference==0);
  assert(fabsf(last_input.electrical_angle_rad)<1e-6f);
  assert(g_speed_voltage_alignment.d_voltage_v==1.5f);
  assert(g_speed_voltage_alignment.q_voltage_v==0);
  run_ms(2000);
  assert(g_speed_voltage_alignment.d_voltage_v==1.5f);
  g_foc_controller.algorithm.output.d_voltage=SPEED_VOLTAGE_ALIGNMENT_MAX_V/24.0f;
  g_foc_controller.algorithm.output.q_voltage=0;
  for(unsigned i=0;i<70000 && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && !g_speed_test.fault && g_speed_test.count==75);
  assert(g_speed_voltage_alignment.d_voltage_v==0 && last_input.d_reference==0);
  assert(fabsf(g_speed_test.records[74].iq_reference-1.5f)<1e-6f);
  assert(g_speed_test.records[74].voltage_q==0);
  assert(fabsf(g_speed_test.records[74].reference)<1e-6f);
  assert(!g_speed_test.calibration_ok);
  SpeedCommission_Service(0); /* Never apply while energized. */
  assert(!g_speed_test.calibration_ok);
  SpeedCommission_Service(1);
  assert(g_speed_test.calibration_ok);
  assert(isfinite(g_speed_test.electrical_offset_rad));
  assert(fabsf(g_speed_test.electrical_offset_rad-
      200000.0f*(6.28318530718f/2097152.0f))<1e-5f);
  reset_voltage_alignment(); arm(); run_ms(10); encoder_valid=0;
  assert(!tick() && g_speed_test.fault==4);
  reset_voltage_alignment(); arm(); run_ms(10); encoder.mechanical_speed_rad_s=21;
  assert(tick() && g_speed_test.fault==0);
  encoder.mechanical_speed_rad_s=-100;
  for(unsigned i=0;i<4U*CONTROL_FREQUENCY_HZ && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && !g_speed_test.fault);
  SpeedCommission_Service(1);
  assert(g_speed_test.calibration_ok);
  reset_voltage_alignment(); arm(); run_ms(10); encoder.mechanical_speed_rad_s=NAN;
  assert(!tick() && g_speed_test.fault==4);
  reset_voltage_alignment(); arm(); run_ms(10); DWT->CYCCNT+=SystemCoreClock/1000;
  SpeedCommission_Service(0); assert(g_speed_test.fault==6);
  reset_voltage_alignment(); arm(); run_ms(10);
  g_speed_voltage_alignment.dc_bus_voltage_v=17;
  assert(!tick() && g_speed_test.fault==3);
  reset_voltage_alignment(); arm(); run_ms(10);
  assert(!SpeedCommission_PhaseCurrentFault(9,-4.5f,-4.5f,15));
  assert(!SpeedCommission_PhaseCurrentFault(9.73827076f,-3.30270815f,-6.43556261f,15));
  assert(!SpeedCommission_PhaseCurrentFault(16,-8,-8,15));
  assert(!SpeedCommission_PhaseCurrentFault(7,-3.5f,-3.5f,15));
  assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,15));
  assert(SpeedCommission_PhaseCurrentFault(INFINITY,0,0,15));
  g_speed_test.state=3U;
  assert(SpeedCommission_PhaseCurrentFault(16,-8,-8,15));
  puts("PASS voltage alignment: fixed 1.5 V, 3 s cutoff, OFF-only zero commit, active amplitude bypass, invalid sample/encoder/deadline gates");
  reset_voltage_alignment();
  g_encoder_calibration.requested=(encoder_calibration_config_t){1.0f,.1f,0};
  arm(); run_ms(50);
  assert(g_speed_voltage_alignment.d_voltage_v==1.0f);
  assert(g_speed_test.active.duration_s==.1f);
  g_encoder_calibration.requested=(encoder_calibration_config_t){2.0f,8.0f,0};
  for(unsigned i=0;i<2000 && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && !g_speed_test.calibration_ok);
  SpeedCommission_Service(1);
  assert(g_speed_test.calibration_ok && g_speed_test.state==0U &&
         g_encoder_calibration.state==ENCODER_CAL_DONE);
  puts("PASS user-selected 1.0 V/0.1 s alignment and immediate calibrated feedback handoff");
  reset_voltage_alignment();
  g_encoder_calibration.requested=(encoder_calibration_config_t){1.0f,.1f,1.0f};
  arm(); run_ms(50);
  assert(g_speed_voltage_alignment.d_voltage_v==1.0f &&
         g_speed_voltage_alignment.q_voltage_v==1.0f);
  assert(fabsf(g_speed_test.active.amplitude-sqrtf(2.0f))<1e-6f);
  for(unsigned i=0;i<2000 && g_speed_test.state!=3;i++) assert(tick());
  assert(g_encoder_calibration.state==ENCODER_CAL_CAPTURED);
  SpeedCommission_Service(1);
  assert(g_encoder_calibration.state==ENCODER_CAL_DONE && g_speed_test.calibration_ok);
  puts("PASS D/Q voltage calibration executes both axes and commits the vector-direction compensated zero");
}

int main(void) {
  assert(g_speed_run.enable==SPEED_RUN_BOOT_AUTO_START);
  assert(g_speed_run.encoder_tolerance_cycles==10U);
  /* Match main: queue the default ON request before the OFF-only loader. */
  g_speed_run.enable=0U;
  assert(sizeof(speed_run_t)==64U); /* Existing 60-byte prefix stays intact. */
  assert(CONTROL_PERIOD_TICKS==8400U && CONTROL_READY_SAMPLES==100U);
  assert(CONTROL_SAMPLES_PER_MS==20U && fabsf(CONTROL_PERIOD_S-0.00005f)<1e-10f);
  uint32_t step_rate = 640U; /* ceil(2^21 * 200 rev/s * 256 / 168 MHz) */
  assert(Encoder_RawStepPlausible(100000U,114855U,8400U,step_rate));
  assert(Encoder_RawStepPlausible(114855U,100000U,8400U,step_rate));
  assert(Encoder_RawStepPlausible(2097000U,14703U,8400U,step_rate));
  assert(Encoder_RawStepPlausible(14703U,2097000U,8400U,step_rate));
  assert(!Encoder_RawStepPlausible(115328U,1927810U,35376U,step_rate));
  assert(!Encoder_RawStepPlausible(0U,1048576U,25200U,step_rate));
  assert(Encoder_RawStepPlausible(0U,1048576U,1680000U,step_rate));
  assert(Encoder_RawStepPlausible(1000U,1256U,0U,step_rate));
  assert(!Encoder_RawStepPlausible(1000U,1257U,0U,step_rate));
  assert(!Encoder_RawStepPlausible(2097152U,0U,8400U,step_rate));
  puts("PASS raw angle plausibility, wrap/reverse, 8500 rpm, impossible jump, long-gap cases");
  assert(SpeedCommission_LoadBoardCalibration());
  assert(g_speed_test.calibration_ok==0 && g_speed_test.direction==-1 &&
         g_speed_test.pole_pairs==2 &&
         g_speed_test.electrical_offset_rad==0.0f);
  assert(!g_speed_run.enable && !g_speed_test.command && !g_speed_test.state);
  encoder_configure_ok=0;
  assert(!SpeedCommission_LoadBoardCalibration());
  assert(!g_speed_test.calibration_ok && !g_speed_run.enable);
  encoder_configure_ok=1;
  assert(SpeedCommission_LoadBoardCalibration());
  g_speed_run.enable=1;
  assert(!SpeedCommission_LoadBoardCalibration());
  assert(g_speed_run.enable==1 && g_speed_test.calibration_ok==0);
  g_speed_run.enable=0;
  g_speed_test.state=4;
  g_speed_test.fault=11;
  assert(!SpeedCommission_LoadBoardCalibration());
  assert(g_speed_test.state==4 && g_speed_test.fault==11);
  g_speed_test.state=0;
  g_speed_test.fault=0;
  puts("PASS zero boot seed remains uncalibrated, configuration failure, no fault clearing");
  assert(SPEED_RUN_PHASE_TRIP_ENABLED==0U);
  g_speed_test.active.kind=SPEED_CONTINUOUS_KIND;
  for(uint32_t state=1;state<=2;state++) {
    g_speed_test.state=state;
    assert(!SpeedCommission_PhaseCurrentFault(-13.9386082f,17.3380394f,
                                               -3.39943123f,15.0f));
    assert(!SpeedCommission_PhaseCurrentFault(100.0f,-100.0f,0.0f,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(NAN,0,0,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(0,NAN,0,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(0,0,NAN,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(INFINITY,0,0,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(0,-INFINITY,0,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(0,0,INFINITY,15.0f));
  }
  g_speed_test.state=0;
  assert(SpeedCommission_PhaseCurrentFault(17.34f,0,0,15.0f));
  g_speed_test.state=2;
  for(uint32_t kind=1;kind<SPEED_CONTINUOUS_KIND;kind++) {
    g_speed_test.active.kind=kind;
    assert(SpeedCommission_PhaseCurrentFault(17.34f,0,0,15.0f));
    assert(SpeedCommission_PhaseCurrentFault(0,0.81f,0,0.8f));
    assert(!SpeedCommission_PhaseCurrentFault(0.1f,-0.1f,0,0.25f));
  }
  puts("PASS continuous phase amplitude trip disabled; NaN/Inf and bounded-test trips retained");
  for(uint32_t data=0;data<0x1000000U;data++) {
    uint8_t crc=0;
    for(unsigned bit=0;bit<24;bit++) {
      uint8_t feedback=(uint8_t)(((crc>>7)^(data>>(23-bit)))&1U);
      crc=(uint8_t)(crc<<1);
      if(feedback) crc^=0x07;
    }
    assert(crc==MT6835_Crc24Fast(data));
  }
  puts("PASS all 16777216 CRC24 inputs match original bitwise algorithm");
  reset(7); arm(); run_ms(100);
  assert(g_speed_run.applied_rpm>27 && g_speed_run.applied_rpm<30);
  run_ms(70000); /* Multiple DWT wraps, far beyond bounded-test duration. */
  assert(g_speed_test.state==2 && g_speed_test.count==0);
  assert(g_speed_run.elapsed_s>70 && g_speed_run.elapsed_s<70.2f);
  assert(fabsf(g_speed_run.applied_rpm-500)<.01f);
  /* The stub advances 100 counts per callback at the configured control rate. */
  assert(fabsf(g_speed_run.turns - g_speed_run.elapsed_s *
      (float)CONTROL_FREQUENCY_HZ * 100.0f / 2097152.0f) < 0.01f);
  g_speed_run.target_rpm=-500; run_ms(4000);
  assert(fabsf(g_speed_run.applied_rpm+500)<.01f);
  g_speed_run.target_rpm=0; run_ms(2000);
  assert(fabsf(g_speed_run.applied_rpm)<.01f && g_speed_test.state==2);
  g_speed_run.enable=0; assert(tick());
  assert(g_speed_test.state==3 && g_speed_test.fault==0);
  puts("PASS continuous >70 s, DWT wrap, live ramp/reversal/zero, explicit stop");

  reset(7); g_speed_run.target_rpm=NAN;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  reset(7); arm(); run_ms(10); g_speed_run.target_rpm=SPEED_RUN_MAX_RPM+1;
  for(unsigned i=0;i<30 && g_speed_test.state==2;i++) (void)tick();
  assert(g_speed_test.fault==9);
  reset(7); arm(); run_ms(10); g_speed_run.target_rpm=NAN;
  for(unsigned i=0;i<30 && g_speed_test.state==2;i++) (void)tick();
  assert(g_speed_test.fault==9);
  reset(7); arm(); run_ms(10); encoder.mechanical_speed_rad_s=951;
  assert(!tick() && g_speed_test.fault==5);
  reset(7); arm(); run_ms(10); encoder_valid=0;
  assert(!tick() && g_speed_test.fault==4);
  reset(7); arm(); run_ms(10); DWT->CYCCNT+=SystemCoreClock/1000;
  assert(!tick() && g_speed_test.fault==6);
  reset(7); g_speed_test.calibration_ok=0;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  reset(7); g_speed_test.command=1; SpeedCommission_Service(0);
  assert(g_speed_test.fault==2);
  puts("PASS invalid command, calibration/output interlocks, overspeed, feedback, deadline faults");
  reset(7); g_speed_test.requested.current_limit_a=SPEED_RUN_IQ_LIMIT_A;
  arm(); run_ms(2000);
  assert(isfinite(g_speed_test.iq_reference) && g_speed_test.iq_reference>25.0f);
  reset(3); g_speed_test.requested.current_limit_a=SPEED_RUN_IQ_LIMIT_A;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  puts("PASS continuous Iq exceeds 25 A without software amplitude clamp; bounded-test cap retained");

  reset(1); g_speed_test.calibration_ok=0;
  g_speed_test.requested.duration_s=3;
  g_speed_test.requested.current_limit_a=SPEED_ALIGNMENT_CURRENT_LIMIT_A;
  g_speed_test.requested.amplitude=SPEED_ALIGNMENT_CURRENT_LIMIT_A;
  arm(); run_ms(100);
  assert(g_speed_test.state==2);
  assert(last_input.d_reference>0 && last_input.d_reference<.36f);
  run_ms(2100);
  assert(g_speed_test.state==2);
  assert(fabsf(last_input.d_reference-SPEED_ALIGNMENT_CURRENT_LIMIT_A)<.0001f);
  assert(fabsf(last_input.d_reference/sqrtf(2)-5.0f)<.0001f);
  for(unsigned i=0;i<20000U && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && !g_speed_test.fault);
  assert(last_input.d_reference==0 && last_input.q_reference==0);
  reset(1); g_speed_test.requested.current_limit_a=SPEED_ALIGNMENT_CURRENT_LIMIT_A+.01f;
  g_speed_test.requested.amplitude=1;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  reset(1); g_speed_test.requested.duration_s=5.01f;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  reset_alignment_sweep();
  g_speed_test.requested.current_limit_a=SPEED_ALIGNMENT_CURRENT_LIMIT_A;
  g_speed_test.requested.amplitude=SPEED_ALIGNMENT_CURRENT_LIMIT_A;
  g_speed_test.command=1; SpeedCommission_Service(1); assert(g_speed_test.fault==3);
  puts("PASS 5 A RMS = 7.071 A Id contract, two-second ramp, five-second deadline, legacy sweep cap retained");

  reset(3); arm();
  for(unsigned i=0;i<41000 && g_speed_test.state!=3;i++) assert(tick());
  assert(g_speed_test.state==3 && g_speed_test.count==128);
  reset(5); g_speed_test.requested.amplitude=.1f;
  for(unsigned i=0;i<127;i++) g_speed_test.prbs_sequence[i]=(uint8_t)(i&1U);
  arm(); run_ms(400);
  assert(g_speed_test.state==2 && g_speed_test.count>40);
  assert(fabsf(g_speed_test.iq_reference)==.1f);
  puts("PASS legacy bounded timeout/recorder and PRBS regression");
  test_encoder_debug();
  test_host_guard();
  test_speed_command_boundary();
  test_start_direction_guard();
  test_alignment_sweep();
  test_alignment_near_rotor_start();
  test_voltage_alignment();
  test_overload_guard();
  reset(3);
  g_speed_test.command=1;
  /* A debugger command arriving after the application's startup snapshot
     must remain pending, not be armed without the matching output start. */
  SpeedCommission_ServiceCommand(1, 0);
  assert(g_speed_test.command==1 && g_speed_test.state==0);
  SpeedCommission_ServiceCommand(1, 1);
  assert(g_speed_test.command==0 && g_speed_test.state==1);
  puts("PASS deferred debugger command consumption");
  test_deferred_outer();
  test_user_speed_ramp();
  reset(7);
  g_user_control.enable=0U;
  g_user_control.mode=USER_MODE_SPEED;
  g_user_control.status.active_mode=USER_MODE_SPEED;
  g_user_control.status.state=USER_RUNNING;
  g_user_control.speed.acceleration_rpm_s=100000.0f;
  g_user_control.speed.deceleration_rpm_s=100000.0f;
  g_speed_response.requested=(speed_response_config_t){0,2,256,300,50,0,.1f};
  g_speed_response.command=1;
  SpeedResponse_Service(1);
  assert(g_speed_response.state==1U);
  g_speed_response.requested.amplitude_rpm=9999.0f; /* Latched recipe must win. */
  g_user_control.enable=1U;
  arm();
  for(unsigned i=0;i<15000U && g_speed_response.state!=3U;i++) {
    foc_input_t input={0};
    DWT->CYCCNT+=CONTROL_PERIOD_TICKS;
    encoder.sample_cycle=DWT->CYCCNT;
    encoder.sequence++;
    assert(SpeedCommission_FastReference(&input));
    if(SpeedCommission_OuterPending()) SpeedCommission_OuterService();
  }
  assert(g_speed_response.state==3U && g_speed_response.count==256U);
  assert(!g_user_control.enable && !g_speed_run.enable);
  assert(g_current_loop_test.samples[0].d_reference_a==300.0f);
  assert(g_current_loop_test.samples[255].d_reference_a==350.0f);
  assert(g_current_loop_test.samples[255].time_s>.50f);
  assert(g_speed_response.active.amplitude_rpm==50.0f);
  reset(7); g_user_control.enable=0U;
  g_speed_response.requested=(speed_response_config_t){0,2,256,300,201,0,.1f};
  g_speed_response.command=1; SpeedResponse_Service(1);
  assert(g_speed_response.state==4U && g_speed_response.fault==3U);
  reset(7); g_user_control.enable=0U;
  g_speed_response.requested=(speed_response_config_t){0,2,256,300,50,0,.8f};
  g_speed_response.command=1; SpeedResponse_Service(1);
  g_user_control.enable=1U; arm();
  for(unsigned i=0;i<33000U && g_speed_response.state!=3U;i++) {
    foc_input_t input={0}; DWT->CYCCNT+=CONTROL_PERIOD_TICKS;
    encoder.sample_cycle=DWT->CYCCNT; encoder.sequence++;
    assert(SpeedCommission_FastReference(&input));
    if(SpeedCommission_OuterPending()) SpeedCommission_OuterService();
  }
  assert(g_speed_response.state==3U && g_speed_response.count==256U);
  assert(g_current_loop_test.samples[0].time_s>=.699f);
  assert(g_current_loop_test.samples[0].d_reference_a==300.0f);
  assert(g_current_loop_test.samples[255].d_reference_a==350.0f);
  assert(!g_user_control.enable && !g_speed_run.enable);
  reset(7); g_user_control.enable=0U;
  g_speed_response.requested=(speed_response_config_t){1,1,512,300,50,160,.1f};
  g_speed_response.command=1; SpeedResponse_Service(1);
  assert(g_speed_response.state==1U && g_speed_response.active.frequency_hz==160.0f);
  reset(7); g_user_control.enable=0U;
  g_speed_response.requested=(speed_response_config_t){1,1,512,300,50,161,.1f};
  g_speed_response.command=1; SpeedResponse_Service(1);
  assert(g_speed_response.state==4U && g_speed_response.fault==3U);
  puts("PASS bounded speed-response recorder: normal PendSV PI, immutable recipe, DWT wrap, automatic OFF, amplitude and 160 Hz gates");
  return 0;
}
