#include "speed_commission.h"

#include "speed_overload_guard.h"
#include "encoder.h"
#include "current_loop_test.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <string.h>
/* Host checks this read-only marker before using kind-1 bias as start angle. */
volatile const uint32_t g_speed_alignment_profile_version = SPEED_ALIGNMENT_PROFILE_VERSION;
volatile const uint32_t g_speed_voltage_alignment_profile_version = SPEED_VOLTAGE_ALIGNMENT_PROFILE_VERSION;
foc_voltage_command_t g_speed_voltage_alignment;
extern current_loop_test_t g_current_loop_test;
/* User-fixed boot default. main temporarily queues this request while loading
   calibration, then submits it once after feedback is ready. Do not change
   the default without explicit user approval; runtime fault/OFF still wins. */
speed_run_t g_speed_run = {.enable=0U, .target_rpm=500.0f,
 .encoder_tolerance_cycles=ENCODER_MAX_SAMPLE_AGE_CYCLES};
speed_run_guard_t g_speed_run_guard;
speed_run_overload_t g_speed_run_overload;
static volatile speed_overload_guard_state_t overload;
static uint32_t guard_timeout_cycles, guard_sequence, guard_cycle;
volatile uint32_t g_encoder_debug_start_block_mask;
/* Accumulate only unsigned cycle differences: DWT wraps every ~25.6 s.
   Convert to seconds at the 1 kHz outer tick, not in every ADC ISR. */
static volatile uint64_t run_cycles;
static int64_t run_start_count;
static volatile int64_t run_delta_count;
static volatile uint8_t voltage_zero_pending;
static float voltage_zero_offset;

void SpeedCommission_PublishRunTelemetry(void) {
 if(g_speed_test.active.kind!=SPEED_CONTINUOUS_KIND || g_speed_test.state!=2U) return;
 /* Expensive int64-to-float conversions belong in the background, not the
    20 kHz ADC interrupt. Copy the two counters with a short interrupt mask. */
 uint32_t mask=__get_PRIMASK();
 __disable_irq();
 uint64_t cycles=run_cycles;
 int64_t count=run_delta_count;
 if(!mask) __enable_irq();
 g_speed_run.elapsed_s=(float)cycles/(float)SystemCoreClock;
 g_speed_run.turns=(float)count/2097152.0f;
 g_speed_test.elapsed=g_speed_run.elapsed_s;
}

speed_commission_t g_speed_test = {
 .direction=1, .pole_pairs=2,
 /* Legacy bounded test defaults. Continuous operation uses g_speed_run. */
 .requested={.kind=3,.duration_s=2.0f,.amplitude=10.0f,
 .angle_end_rad=1.570796327f,.kp=0.032f,.ki=1.0f,
 .current_limit_a=0.30f,.speed_limit_rad_s=20.0f,.record_divider=8,.prbs_bit_ms=16}
};
static float clip(float x,float lim) { return x>lim ? lim : (x < -lim ? -lim : x); }
static uint8_t fail(uint32_t reason) {
 g_speed_test.fault=reason; g_speed_test.state=4; return 0;
}
static uint8_t SpeedRun_HostGuardCheck(void) {
 if(g_speed_test.active.kind!=SPEED_CONTINUOUS_KIND || !guard_timeout_cycles ||
    (g_speed_test.state!=1U && g_speed_test.state!=2U)) return 1U;
 uint32_t mask=__get_PRIMASK();
 __disable_irq();
 if(g_speed_test.state!=1U && g_speed_test.state!=2U) {
   if(!mask) __enable_irq();
   return 1U; /* An ADC fault may have preempted the initial check. */
 }
 uint32_t now=DWT->CYCCNT;
 uint8_t ok=1U;
 /* Check expiry BEFORE accepting a changed heartbeat: a late host write
    cannot revive an expired run. Unsigned subtraction handles DWT wrap. */
 if((now-guard_cycle)>=guard_timeout_cycles) {
   g_speed_run_guard.expired=1U;
   g_speed_run.enable=0U;
   ok=fail(13U);
 } else if(g_speed_run_guard.heartbeat!=guard_sequence) {
   guard_sequence=g_speed_run_guard.heartbeat;
   guard_cycle=now;
 }
 if(ok && !SpeedOverload_Check(&overload,now)) {
   g_speed_run_overload.expired=overload.expired;
   g_speed_run_overload.invalid=overload.invalid;
   g_speed_run.enable=0U;
   ok=fail(overload.invalid ? 15U : 14U);
 }
 if(!mask) __enable_irq();
 return ok;
}
uint8_t SpeedCommission_LoadBoardCalibration(void) {
 uint32_t mask=__get_PRIMASK();
 __disable_irq();
 /* Boot-only loader: never replace a running, completed or faulted test's
    calibration. No output request, first-fault record or heat debt is cleared. */
 if(g_speed_run.enable || g_speed_test.state || g_speed_test.command ||
    g_speed_test.calibration_command) {
   if(!mask) __enable_irq();
   return 0U;
 }
 g_speed_test.calibration_ok=0U;
 if(!Encoder_ConfigureMotor(SPEED_BOARD_ENCODER_DIRECTION,
                            SPEED_BOARD_ENCODER_POLE_PAIRS,
                            SPEED_BOARD_ENCODER_OFFSET_RAD)) {
   if(!mask) __enable_irq();
   return 0U;
 }
 g_speed_test.direction=SPEED_BOARD_ENCODER_DIRECTION;
 g_speed_test.pole_pairs=SPEED_BOARD_ENCODER_POLE_PAIRS;
 g_speed_test.electrical_offset_rad=SPEED_BOARD_ENCODER_OFFSET_RAD;
 /* Direction/pole pairs are configured, but zeroing has not completed. */
 g_speed_test.calibration_ok=0U;
 if(!mask) __enable_irq();
 return 1U;
}
uint8_t SpeedCommission_PhaseCurrentFault(float u_a,float v_a,float w_a,
                                         float threshold_a) {
 /* An invalid sample must never propagate into the PI, even when the user
    explicitly bypasses the continuous-mode phase-amplitude software trip. */
 if(!isfinite(u_a) || !isfinite(v_a) || !isfinite(w_a) ||
    !isfinite(threshold_a) || threshold_a<=0.0f) return 1U;
 /* User-selected hardware-limited voltage alignment: no software RMS or
    phase-amplitude trip while armed/running. Keep nonfinite rejection above,
    telemetry, the fixed duration and all other modes' checks unchanged. */
 if(g_speed_test.active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND &&
    (g_speed_test.state==1U || g_speed_test.state==2U))
   return 0U;
 /* Host watchdog does not re-enable a phase-amplitude trip. The optional
    explicitly armed I2t model remains independent of instantaneous peaks. */
 if(guard_timeout_cycles && g_speed_test.active.kind==SPEED_CONTINUOUS_KIND &&
    (g_speed_test.state==1U || g_speed_test.state==2U)) {
   if(overload.armed && !SpeedOverload_Observe(&overload,DWT->CYCCNT,
        (u_a*u_a+v_a*v_a+w_a*w_a)*(1.0f/3.0f))) {
     g_speed_run.enable=0U;
     (void)fail(overload.invalid ? 15U : 14U);
     return 1U;
   }
   g_speed_run_overload.started=overload.started;
   return 0U;
 }
 if(!SPEED_RUN_PHASE_TRIP_ENABLED &&
    g_speed_test.active.kind==SPEED_CONTINUOUS_KIND &&
    (g_speed_test.state==1U || g_speed_test.state==2U)) return 0U;
 return (fabsf(u_a)>threshold_a || fabsf(v_a)>threshold_a ||
         fabsf(w_a)>threshold_a) ? 1U : 0U;
}
void SpeedCommission_Service(uint8_t off) {
 SpeedCommission_ServiceCommand(off, g_speed_test.command);
}
void SpeedCommission_ServiceCommand(uint8_t off, uint32_t accepted_command) {
 speed_commission_t *s=&g_speed_test;
 uint8_t guard_ok=SpeedRun_HostGuardCheck();
 /* A Cortex-M4 cannot atomically read a 64-bit ISR accumulator. Snapshot
    under a short mask; conversions/division/sqrt stay outside that mask. */
 uint32_t overload_mask=__get_PRIMASK();
 __disable_irq();
 speed_overload_guard_state_t thermal=overload;
 if(!overload_mask) __enable_irq();
 g_speed_run_overload.elapsed_ms=(uint32_t)(thermal.elapsed_cycles/(SystemCoreClock/1000U));
 g_speed_run_overload.armed=thermal.armed;
 g_speed_run_overload.started=thermal.started;
 g_speed_run_overload.used=thermal.used;
 g_speed_run_overload.expired=thermal.expired;
 g_speed_run_overload.invalid=thermal.invalid;
 g_speed_run_overload.energy_a2s=(float)thermal.excess_i2_cycles_q16/((float)SystemCoreClock*65536.0f);
 g_speed_run_overload.budget_a2s=(float)SPEED_OVERLOAD_BUDGET_A2S;
 g_speed_run_overload.current_rms_a=sqrtf(SpeedOverload_MeanSquare(&thermal));
 if(!guard_ok) return;
 /* Commit the angle captured while energized, never a relaxed OFF angle.
    The ISR only stages it; reconfiguration is allowed after PWM is OFF. */
 if(voltage_zero_pending && off) {
   voltage_zero_pending=0U;
   if(s->state==3U && !s->fault && s->command!=2U &&
      Encoder_ConfigureMotor((int8_t)s->direction,(uint16_t)s->pole_pairs,
                             voltage_zero_offset)) {
     s->electrical_offset_rad=voltage_zero_offset;
     s->calibration_ok=1U;
   } else {
     s->calibration_ok=0U;
     (void)fail(7U);
   }
 }
 if(s->active.kind==ENCODER_DEBUG_OPEN_LOOP_KIND ||
    s->active.kind==SPEED_ALIGNMENT_SWEEP_KIND ||
    s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
   /* Independent background observer: an absent ADC callback must not also
      disable the diagnostic's time guard. Snapshot time/state with IRQs
      masked so a newer ISR timestamp cannot appear later than `now`. */
   uint32_t mask=__get_PRIMASK();
   __disable_irq();
   uint32_t now=DWT->CYCCNT;
   /* Kind 9 is explicitly armed/inspected while OFF by the host. Its
      excitation starts only after 100 ADC/encoder-ready callbacks; do not
      expire that harmless reservation while SWD snapshots are in progress.
      Once callbacks begin, ready_cycle and the 200 ms gate are enforced. */
   if(s->state==1U &&
      (s->active.kind==ENCODER_DEBUG_OPEN_LOOP_KIND || s->ready_count) &&
      (now-s->ready_cycle)>SystemCoreClock/5U)
     (void)fail(8);
   else if(s->state==2U && (now-s->last_cycle)>SystemCoreClock/2000U)
     (void)fail(6);
   if(!mask) __enable_irq();
 }
 if(s->calibration_command && off && s->state!=1 && s->state!=2) {
   s->calibration_command=0;
   s->calibration_ok=(s->pole_pairs==2 && fabsf(s->electrical_offset_rad)<7.0f) ?
     Encoder_ConfigureMotor((int8_t)s->direction,2,s->electrical_offset_rad):0;
 }
 if(!accepted_command) return;
 uint32_t cmd=accepted_command; s->command=0;
 if(cmd==2) { (void)fail(1); return; }
 if(!off || s->state==2) { (void)fail(2); return; }
 if(cmd==3) { s->state=0; return; }
 speed_test_config_t c=s->requested;
 if(cmd==1U && c.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
   /* Fixed zeroing recipe; no field sweep, voltage ramp or current PI. */
   c=(speed_test_config_t){.kind=SPEED_VOLTAGE_ALIGNMENT_KIND,
     .duration_s=SPEED_VOLTAGE_ALIGNMENT_DURATION_S,
     .amplitude=SPEED_VOLTAGE_ALIGNMENT_MAX_V,
     .current_limit_a=SPEED_RUN_PHASE_TRIP_A,.speed_limit_rad_s=20.0f,
     .record_divider=40U,.prbs_bit_ms=16U};
   s->requested=c;
   if((s->direction!=1 && s->direction!=-1) || s->pole_pairs!=2U) {
     (void)fail(3U); return;
   }
 }
 if(cmd==ENCODER_DEBUG_START_COMMAND) {
   c=(speed_test_config_t){.kind=ENCODER_DEBUG_OPEN_LOOP_KIND,
     .duration_s=ENCODER_DEBUG_MAX_DURATION_S,.amplitude=.10f,.frequency_hz=.50f,
     .current_limit_a=ENCODER_DEBUG_MAX_CURRENT_A,.speed_limit_rad_s=20.0f,
     .record_divider=8,.prbs_bit_ms=16};
   s->requested=c;
   cmd=1;
 }
 if(c.kind==5 || c.kind==6) {
   if(g_current_loop_test.state!=CURRENT_TEST_IDLE) { (void)fail(3); return; }
   uint32_t ones=0;
   for(uint32_t i=0;i<127;i++) {
     if(s->prbs_sequence[i]>1U) { (void)fail(3); return; }
     ones+=s->prbs_sequence[i];
   }
   if(ones<20U || ones>107U) { (void)fail(3); return; }
 }
 if(cmd!=1 || c.kind<1 || c.kind>SPEED_VOLTAGE_ALIGNMENT_KIND ||
   !isfinite(c.duration_s) || c.duration_s<0.05f || c.duration_s>12.0f ||
   !isfinite(c.amplitude) || !isfinite(c.bias) ||
   ((c.kind==1U || c.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) &&
                  (c.duration_s>5.0f || fabsf(c.bias)>3.141593f ||
                   g_speed_alignment_profile_version!=SPEED_ALIGNMENT_PROFILE_VERSION)) ||
   !isfinite(c.frequency_hz) || c.frequency_hz<0 || c.frequency_hz>30 ||
   !isfinite(c.angle_end_rad) || fabsf(c.angle_end_rad)>
       (c.kind==SPEED_ALIGNMENT_SWEEP_KIND ? SPEED_ALIGNMENT_SWEEP_DISTANCE_RAD : 6.3f) ||
   !isfinite(c.kp) || c.kp<0 || c.kp>0.2f ||
   !isfinite(c.ki) || c.ki<0 || c.ki>2.0f ||
   !isfinite(c.current_limit_a) || c.current_limit_a<=0 ||
     c.current_limit_a>(c.kind==SPEED_CONTINUOUS_KIND ?
                        (g_speed_run_overload.request==1U && g_speed_run_guard.timeout_ms ?
                         SPEED_RUN_OVERLOAD_IQ_LIMIT_A : SPEED_RUN_IQ_LIMIT_A) :
                        (c.kind==SPEED_VOLTAGE_ALIGNMENT_KIND ? SPEED_RUN_PHASE_TRIP_A :
                        (c.kind==1U ? SPEED_ALIGNMENT_CURRENT_LIMIT_A :
                         (c.kind==SPEED_ALIGNMENT_SWEEP_KIND ?
                          SPEED_ALIGNMENT_LEGACY_CURRENT_LIMIT_A : 0.5f)))) ||
   !isfinite(c.speed_limit_rad_s) || c.speed_limit_rad_s<=0 ||
     c.speed_limit_rad_s>(c.kind==SPEED_CONTINUOUS_KIND ? 950.0f : 20.0f) ||
   c.record_divider<1 || c.record_divider>100 ||
   ((c.kind<=2 || c.kind==SPEED_ALIGNMENT_SWEEP_KIND) && fabsf(c.amplitude)>c.current_limit_a) ||
   (c.kind==5 && fabsf(c.amplitude)+fabsf(c.bias)>c.current_limit_a) ||
   ((c.kind==5 || c.kind==6) && (c.duration_s>10.8f || c.record_divider!=8 ||
                  (c.prbs_bit_ms!=16 && c.prbs_bit_ms!=32 && c.prbs_bit_ms!=64))) ||
   ((c.kind==3 || c.kind==4 || c.kind==6) && fabsf(c.amplitude)+fabsf(c.bias)>c.speed_limit_rad_s*0.5f) ||
   (c.kind!=1 && c.kind!=SPEED_ALIGNMENT_SWEEP_KIND && c.kind!=SPEED_VOLTAGE_ALIGNMENT_KIND &&
    c.kind!=ENCODER_DEBUG_OPEN_LOOP_KIND && !s->calibration_ok) ||
   (c.kind==SPEED_VOLTAGE_ALIGNMENT_KIND &&
    (c.duration_s!=SPEED_VOLTAGE_ALIGNMENT_DURATION_S ||
     c.amplitude<=0 || c.amplitude>SPEED_VOLTAGE_ALIGNMENT_MAX_V ||
     c.current_limit_a!=SPEED_RUN_PHASE_TRIP_A || c.frequency_hz!=0 ||
     c.kp!=0 || c.ki!=0 || c.record_divider!=40U ||
     g_speed_voltage_alignment_profile_version!=SPEED_VOLTAGE_ALIGNMENT_PROFILE_VERSION ||
     g_current_loop_test.state!=CURRENT_TEST_IDLE)) ||
   (c.kind==SPEED_ALIGNMENT_SWEEP_KIND &&
    (c.duration_s!=SPEED_ALIGNMENT_SWEEP_DURATION_S ||
     c.angle_end_rad!=SPEED_ALIGNMENT_SWEEP_DISTANCE_RAD ||
     c.amplitude<=0 || c.bias!=0 || c.frequency_hz!=0 ||
     c.kp!=0 || c.ki!=0 || c.record_divider!=100U ||
     g_current_loop_test.state!=CURRENT_TEST_IDLE)) ||
   (c.kind==ENCODER_DEBUG_OPEN_LOOP_KIND &&
     (c.duration_s>ENCODER_DEBUG_MAX_DURATION_S ||
      c.frequency_hz>ENCODER_DEBUG_MAX_ELECTRICAL_HZ ||
      c.current_limit_a>ENCODER_DEBUG_MAX_CURRENT_A ||
      c.amplitude<0 || c.amplitude>c.current_limit_a || c.bias!=0 ||
      c.kp!=0 || c.ki!=0 || g_current_loop_test.state!=CURRENT_TEST_IDLE)) ||
   (c.kind==SPEED_CONTINUOUS_KIND &&
     (g_speed_run.enable!=1U ||
      (g_speed_run_guard.timeout_ms &&
       (g_speed_run_guard.timeout_ms<100U || g_speed_run_guard.timeout_ms>10000U)) ||
      !isfinite(g_speed_run.target_rpm) ||
      fabsf(g_speed_run.target_rpm)>SPEED_RUN_MAX_RPM ||
      fabsf(g_speed_run.target_rpm)*SPEED_RPM_TO_RAD_S>=c.speed_limit_rad_s ||
      g_current_loop_test.state!=CURRENT_TEST_IDLE))) { (void)fail(3); return; }
 if(c.kind==SPEED_CONTINUOUS_KIND &&
    !SpeedOverload_ArmWithDebt(&overload,g_speed_run_overload.request,
        g_speed_run_guard.timeout_ms,SystemCoreClock,
        g_speed_run_overload.initial_energy_a2s)) { (void)fail(3); return; }
 s->active=c; s->fault=0; s->count=0; s->ticks=0; s->record_ticks=0;
 voltage_zero_pending=0U;
 if(c.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) s->calibration_ok=0U;
 if(c.kind>=5) memset(s->records,0,sizeof(s->records));
 s->elapsed=0; s->integral=0; s->iq_reference=0; s->reference=0;
 g_speed_voltage_alignment.d_voltage_v=g_speed_voltage_alignment.q_voltage_v=0;
 s->max_interval_cycles=0; s->ready_cycle=s->ready_count=0;
 if(c.kind==ENCODER_DEBUG_OPEN_LOOP_KIND) {
   s->angle=0;
   s->ready_cycle=DWT->CYCCNT;
 }
 run_cycles=0; run_delta_count=0;
 if(c.kind==SPEED_CONTINUOUS_KIND) {
   g_speed_run_guard.active_timeout_ms=g_speed_run_guard.timeout_ms;
   g_speed_run_guard.expired=0U;
   guard_timeout_cycles=g_speed_run_guard.active_timeout_ms*(SystemCoreClock/1000U);
   guard_sequence=g_speed_run_guard.heartbeat;
   guard_cycle=DWT->CYCCNT;
   g_speed_run.applied_rpm=g_speed_run.actual_rpm=0;
   g_speed_run.elapsed_s=g_speed_run.turns=0;
   g_speed_run.iq_reference_a=g_speed_run.iq_a=g_speed_run.id_a=0;
   g_speed_run.uq_pu=0; g_speed_run.feedback_status_raw=0;
 }
 s->state=1; /* Publish only after watchdog and telemetry initialization. */
}
static uint8_t EncoderDebug_Reference(foc_input_t *in) {
 speed_commission_t *s=&g_speed_test;
 uint32_t now=DWT->CYCCNT;
 if(s->state==1U) {
   s->first_cycle=s->last_cycle=s->tick_cycle=now;
   s->state=2U;
 }
 uint32_t gap=now-s->last_cycle;
 s->last_cycle=now;
 if(gap>s->max_interval_cycles) s->max_interval_cycles=gap;
 if(gap>SystemCoreClock/2000U) return fail(6);
 float dt=(float)gap/(float)SystemCoreClock;
 s->elapsed=(float)(now-s->first_cycle)/(float)SystemCoreClock;
 in->d_reference=in->q_reference=0;
 if(s->elapsed>=s->active.duration_s) s->state=3U;
 else {
   /* 250 ms fixed-angle dwell, then <=1 Hz/s ELECTRICAL frequency slew.
      Real rotor position/speed need not track this synthetic field. */
   float previous_elapsed=s->elapsed-dt;
   float rotation_dt=fmaxf(0.0f,s->elapsed-fmaxf(.25f,previous_elapsed));
   s->reference=fminf(s->active.frequency_hz,s->reference+rotation_dt);
   s->angle+=6.283185307f*s->reference*rotation_dt;
   if(s->angle>=6.283185307f) s->angle-=6.283185307f;
   in->d_reference=fminf(s->active.amplitude,.20f*s->elapsed);
   if(gap) {
     extern foc_controller_t g_foc_controller;
     g_foc_controller.algorithm.parameters.sample_period_s=dt;
   }
 }
 /* These validity bits describe a finite, locally generated angle/reference,
    NOT a CRC-bypassed encoder sample. current_valid is still decided by ADC. */
 in->electrical_angle_rad=s->angle;
 in->angle_valid=in->reference_valid=1U;
 return 1U;
}
uint8_t SpeedCommission_Reference(foc_input_t *in) {
 speed_commission_t *s=&g_speed_test;
 encoder_snapshot_t e={0};
 g_speed_voltage_alignment.d_voltage_v=g_speed_voltage_alignment.q_voltage_v=0;
 if(s->state!=1 && s->state!=2) return 0;
 if(!SpeedRun_HostGuardCheck()) return 0;
 if(s->active.kind==SPEED_CONTINUOUS_KIND && g_speed_run.enable!=1U) {
   /* Operator stop is not an invalid-feedback fault. ADC caller removes PWM. */
   s->state=3;
   in->d_reference=in->q_reference=0;
   in->electrical_angle_rad=s->angle;
   in->angle_valid=in->reference_valid=1;
   return 1;
 }
 uint8_t valid=Encoder_ReadSnapshot(&e);
 if(s->active.kind==SPEED_CONTINUOUS_KIND) g_speed_run.feedback_status_raw=e.status_raw;
 s->feedback_fault=e.fault_flags;
 s->feedback_age_cycles=DWT->CYCCNT-e.sample_cycle;
 if(s->active.kind==ENCODER_DEBUG_OPEN_LOOP_KIND)
   return EncoderDebug_Reference(in);
 if(s->state==1) {
   uint32_t now=DWT->CYCCNT;
   if(!s->ready_cycle) s->ready_cycle=now;
   if((now-s->ready_cycle)>SystemCoreClock/5U) return fail(8);
   if(valid && e.angle_valid && e.speed_valid && isfinite(e.mechanical_speed_rad_s))
     s->ready_count++;
   else s->ready_count=0;
   if(s->ready_count<CONTROL_READY_SAMPLES) {
     in->d_reference=in->q_reference=0; in->electrical_angle_rad=0;
     in->angle_valid=in->reference_valid=1; return 1;
   }
 }
 if(!valid || !e.angle_valid || !e.speed_valid ||
    !isfinite(e.mechanical_speed_rad_s)) return fail(4);
 /* Voltage alignment has no speed-amplitude gate by user configuration. */
 if(s->active.kind!=SPEED_VOLTAGE_ALIGNMENT_KIND &&
    fabsf(e.mechanical_speed_rad_s)>s->active.speed_limit_rad_s) return fail(5);
 uint32_t now=DWT->CYCCNT;
 if(s->state==1) {
   s->first_cycle=s->tick_cycle=now; s->last_cycle=now; s->state=2;
   run_start_count=e.multi_turn_count;
 }
 uint32_t gap=now-s->last_cycle; s->last_cycle=now;
 if(gap>s->max_interval_cycles) s->max_interval_cycles=gap;
 if(gap>SystemCoreClock/2000U) return fail(6);
 /* The current PI must integrate elapsed time, including any delayed trigger. */
 if(gap) {
   extern foc_controller_t g_foc_controller;
   g_foc_controller.algorithm.parameters.sample_period_s=(float)gap/(float)SystemCoreClock;
 }
 uint32_t elapsed_ms=0;
 if(s->active.kind==SPEED_CONTINUOUS_KIND) run_cycles+=gap;
 else {
   s->elapsed=(float)(now-s->first_cycle)/(float)SystemCoreClock;
   elapsed_ms=(now-s->first_cycle)/(SystemCoreClock/1000U);
 }
 if(s->active.kind!=SPEED_CONTINUOUS_KIND && s->elapsed>=s->active.duration_s) {
   if(s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
     /* raw_angle is independent of the old MCU offset and direction config. */
     /* Valid/finite feedback was checked above; no end-of-alignment speed gate. */
     float zero=-(float)s->direction*(float)s->pole_pairs*
         (float)e.raw_angle*(6.28318530718f/2097152.0f);
     while(zero<0.0f) zero+=6.28318530718f;
     while(zero>=6.28318530718f) zero-=6.28318530718f;
     voltage_zero_offset=zero;
     voltage_zero_pending=1U;
   }
   s->state=3; in->d_reference=in->q_reference=0;
   in->electrical_angle_rad=s->angle;
   in->angle_valid=in->reference_valid=1; return 1;
 }
 if(s->active.kind!=1 && s->active.kind!=SPEED_ALIGNMENT_SWEEP_KIND &&
    s->active.kind!=SPEED_VOLTAGE_ALIGNMENT_KIND &&
    !e.electrical_angle_valid) return fail(7);
 s->angle=e.electrical_angle_rad;
 if(s->active.kind==1 || s->active.kind==SPEED_ALIGNMENT_SWEEP_KIND ||
    s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
   if(s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
     s->angle=s->reference=0.0f;
   } else if(s->active.kind==SPEED_ALIGNMENT_SWEEP_KIND) {
     /* 0..3 s ramp/hold, 3..6 forward full mechanical turn,
        6..9 reverse, 9..12 final hold. Same 3 A/current/speed interlocks.
        Record the REAL commanded unwrapped field, not a derived timestamp. */
     float p=s->elapsed<6.0f ? (s->elapsed-3.0f)/3.0f : (9.0f-s->elapsed)/3.0f;
     p=fmaxf(0,fminf(1,p));
     s->angle=p*s->active.angle_end_rad;
     s->reference=s->angle;
   } else {
     float p=(s->elapsed/s->active.duration_s-0.25f)*2.0f;
     p=fmaxf(0,fminf(1,p));
     /* A fixed zero field can pull an antipodal resting rotor through half
        an electrical turn before the sweep even begins. The host may seed
        a nearby field from a rough offset; it is NOT motor calibration.
        Zero bias retains the legacy trajectory and all existing trips. */
     s->angle=s->active.bias+p*(s->active.angle_end_rad-s->active.bias);
     s->reference=s->angle;
   }
   float ramp=(s->active.kind==1U &&
               fabsf(s->active.amplitude)>SPEED_ALIGNMENT_LEGACY_CURRENT_LIMIT_A) ?
               SPEED_ALIGNMENT_CURRENT_RAMP_A_S : SPEED_ALIGNMENT_LEGACY_RAMP_A_S;
   if(s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND) {
     /* No current feedback in the voltage command. Vdc is a configured,
        nominal value (not a bus measurement); reject outside this profile. */
     if(!isfinite(g_speed_voltage_alignment.dc_bus_voltage_v) ||
        g_speed_voltage_alignment.dc_bus_voltage_v<18.0f ||
        g_speed_voltage_alignment.dc_bus_voltage_v>28.0f ||
        s->active.amplitude>SPEED_VOLTAGE_ALIGNMENT_LIMIT_PU*g_speed_voltage_alignment.dc_bus_voltage_v) return fail(3);
     g_speed_voltage_alignment.d_voltage_v=
       SPEED_VOLTAGE_ALIGNMENT_MAX_V;
     in->d_reference=0;
   } else in->d_reference=clip(s->active.amplitude,ramp*s->elapsed);
   in->q_reference=0;
 } else {
   /* PRBS edges and recorder share this ADC-cycle millisecond clock.
      Previously the free-running 1 ms PI clock could update an edge after
      its 8 ms record, producing an apparent extra sample of input delay. */
   float prbs_ref=0;
   if((s->active.kind==5 || s->active.kind==6) && elapsed_ms>=200U) {
     uint32_t bit=((elapsed_ms-200U)/s->active.prbs_bit_ms)%127U;
     prbs_ref=s->active.bias+s->active.amplitude*(s->prbs_sequence[bit] ? 1.0f : -1.0f);
   }
   if(s->active.kind==5) {
     s->reference=prbs_ref;
     s->iq_reference=clip(prbs_ref,s->active.current_limit_a);
   }
   if((now-s->tick_cycle)>=SystemCoreClock/1000U) {
     float dt=(float)(now-s->tick_cycle)/(float)SystemCoreClock;
     s->tick_cycle=now; s->ticks++;
     float ref=s->elapsed<0.2f ? 0 : s->active.amplitude;
     if(s->active.kind==4) ref=s->active.bias+s->active.amplitude*
         sinf(6.283185307f*s->active.frequency_hz*s->elapsed);
     if(s->active.kind==5 || s->active.kind==6) ref=prbs_ref;
     if(s->active.kind==SPEED_CONTINUOUS_KIND) {
       float target=g_speed_run.target_rpm;
       if(!isfinite(target) || fabsf(target)>SPEED_RUN_MAX_RPM ||
          fabsf(target)*SPEED_RPM_TO_RAD_S>=s->active.speed_limit_rad_s)
         return fail(9);
       ref=s->reference+clip(target*SPEED_RPM_TO_RAD_S-s->reference,
                            SPEED_RUN_RAMP_RPM_S*SPEED_RPM_TO_RAD_S*dt);
     }
     s->reference=ref;
     /* Guarded tests start stationary. A substantial opposite speed in the
        first two seconds indicates phase/polarity loss, not an Iq demand to
        keep integrating. Established live reversals retain their behavior. */
     if(s->active.kind==SPEED_CONTINUOUS_KIND && guard_timeout_cycles &&
        run_cycles<(uint64_t)SPEED_RUN_START_DIRECTION_CHECK_MS*(SystemCoreClock/1000U) &&
        fabsf(ref)>SPEED_RUN_START_DIRECTION_COMMAND_RPM*SPEED_RPM_TO_RAD_S &&
        fabsf(e.mechanical_speed_rad_s)>SPEED_RUN_START_DIRECTION_FEEDBACK_RPM*SPEED_RPM_TO_RAD_S &&
        ref*e.mechanical_speed_rad_s<0.0f) {
       g_speed_run.applied_rpm=ref/SPEED_RPM_TO_RAD_S;
       g_speed_run.actual_rpm=e.mechanical_speed_rad_s/SPEED_RPM_TO_RAD_S;
       g_speed_run.enable=0U;
       return fail(16U);
     }
     if(s->active.kind==2 || s->active.kind==5) s->iq_reference=clip(ref,s->active.current_limit_a);
     else {
       float err=ref-e.mechanical_speed_rad_s;
       if(s->active.kind==SPEED_CONTINUOUS_KIND &&
          s->active.current_limit_a==FLT_MAX) {
         extern foc_controller_t g_foc_controller;
         const foc_output_t *o=&g_foc_controller.algorithm.output;
         /* No current-amplitude clamp. Avoid integrating further only when
            voltage saturation prevents the requested current from tracking. */
         if(!(o->saturated && o->q_current_error*err>0.0f))
           s->integral+=s->active.ki*err*dt;
         s->iq_reference=s->active.kp*err+s->integral;
         if(!isfinite(s->iq_reference)) return fail(3U);
       } else {
       float proposed=clip(s->integral+s->active.ki*err*dt,s->active.current_limit_a);
       float u=s->active.kp*err+proposed;
       if((u> s->active.current_limit_a && err>0) ||
          (u<-s->active.current_limit_a && err<0)) u=s->active.kp*err+s->integral;
       else s->integral=proposed;
       s->iq_reference=clip(u,s->active.current_limit_a);
       }
     }
     if(s->active.kind==SPEED_CONTINUOUS_KIND) {
       extern foc_controller_t g_foc_controller;
       const foc_output_t *o=&g_foc_controller.algorithm.output;
       g_speed_run.applied_rpm=ref/SPEED_RPM_TO_RAD_S;
       g_speed_run.actual_rpm=e.mechanical_speed_rad_s/SPEED_RPM_TO_RAD_S;
       run_delta_count=e.multi_turn_count-run_start_count;
       g_speed_run.iq_reference_a=s->iq_reference;
       g_speed_run.iq_a=o->q_current; g_speed_run.id_a=o->d_current;
       g_speed_run.uq_pu=o->q_voltage;
     }
   }
   in->d_reference=0; in->q_reference=s->iq_reference;
 }
 /* Continuous mode never terminates because a recorder is full or time expires. */
 if(s->active.kind==SPEED_CONTINUOUS_KIND) {
   in->electrical_angle_rad=s->angle;
   in->angle_valid=in->reference_valid=1;
   return 1;
 }
 /* Recorder runs independently of PI cadence. Values are not display-filtered. */
 uint32_t record=elapsed_ms/s->active.record_divider;
 if(s->active.kind>=5 && s->active.kind!=SPEED_ALIGNMENT_SWEEP_KIND &&
    s->active.kind!=SPEED_VOLTAGE_ALIGNMENT_KIND) {
   if(g_current_loop_test.state!=CURRENT_TEST_IDLE) return fail(3);
   if(s->count<1365 && (s->count==0 || record>s->record_ticks)) {
     s->record_ticks=record;
     g_current_loop_test.speed_prbs_samples[s->count].time_s=s->elapsed;
     /* Kind 6 records speed reference (rad/s), kind 5 records Iq command (A). */
     g_current_loop_test.speed_prbs_samples[s->count].iq_command_a=
       s->active.kind==6 ? s->reference : in->q_reference;
     g_current_loop_test.speed_prbs_samples[s->count].speed_rad_s=e.mechanical_speed_rad_s;
     s->count++;
     /* Independent sparse inner-loop diagnostics, using existing idle RAM.
        Only valid rows have time_s > 0. Current values precede this command
        by one current-loop cycle; this is NOT a high-rate current FRF. */
     uint32_t di=elapsed_ms/100U;
     if(di<128U && elapsed_ms%100U<s->active.record_divider) {
       extern foc_controller_t g_foc_controller;
       foc_output_t *o=&g_foc_controller.algorithm.output;
       speed_test_record_t r={s->elapsed,s->reference,e.mechanical_speed_rad_s,
         o->d_current,o->q_current,in->q_reference,e.mechanical_angle_rad,o->q_voltage};
       s->records[di]=r;
     }
   }
 } else
 if(s->count<128 && (s->count==0 || record>s->record_ticks)) {
   s->record_ticks=record;
   extern foc_controller_t g_foc_controller;
   foc_output_t *o=&g_foc_controller.algorithm.output;
   speed_test_record_t r={s->elapsed,s->reference,e.mechanical_speed_rad_s,
     o->d_current,o->q_current,
     s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND ?
       o->d_voltage*g_speed_voltage_alignment.dc_bus_voltage_v : in->q_reference,
     e.mechanical_angle_rad,
     s->active.kind==SPEED_VOLTAGE_ALIGNMENT_KIND ?
       o->q_voltage*g_speed_voltage_alignment.dc_bus_voltage_v : o->q_voltage};
   s->records[s->count++]=r;
 }
 in->electrical_angle_rad=s->angle;
 in->angle_valid=in->reference_valid=1;
 return 1;
}
