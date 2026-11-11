/* Compile the very same application dispatcher included by main.c. */
#include "user_control.h"
#include "encoder_calibration.h"
#include "encoder.h"
#include "foc_control_manager.h"
#include "speed_commission.h"
#include "output_position.h"
#include "stm32g4xx_hal.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

foc_control_manager_t g_global_mode_control;
foc_controller_t g_foc_controller;
speed_run_t g_speed_run;
speed_commission_t g_speed_test;
encoder_calibration_t g_encoder_calibration;
volatile output_position_t g_output_position;
volatile uint8_t g_encoder_feedback_required;
static uint32_t tick, stops;
static uint8_t moe, power_safe=1, sampling_done=1;
#define CURRENT_SAMPLING_CAL_DONE 3
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t CurrentSampling_GetCalibrationState(void) { return sampling_done?3:0; }
static uint8_t Tim1Pwm_IsOutputEnabled(void) { return moe; }
static void Tim1Pwm_EmergencyStop(void) { moe=0; }
uint8_t Encoder_ReadSnapshot(encoder_snapshot_t *s) { (void)s; return 0; }
void FOC_ControllerStop(foc_controller_t *c) { c->running=0; moe=0; stops++; }
uint8_t EncoderCalibration_Busy(void) {
  return g_encoder_calibration.state>=ENCODER_CAL_QUEUED &&
         g_encoder_calibration.state<=ENCODER_CAL_CAPTURED;
}
static uint8_t MainFoc_IsPowerStageSafe(void *p) { (void)p; return power_safe; }
static void MainFoc_ConfigureCurrentStageParameters(foc_current_stage_t stage) {
  assert(stage==FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP);
}
#include "../../Servo/Src/user_control_dispatch.inc"

static volatile uint8_t boot;
static void reset(unsigned mode) {
  memset(&g_global_mode_control,0,sizeof(g_global_mode_control));
  memset(&g_foc_controller,0,sizeof(g_foc_controller));
  memset(&g_speed_test,0,sizeof(g_speed_test));
  memset(&g_speed_run,0,sizeof(g_speed_run));
  memset(&g_encoder_calibration,0,sizeof(g_encoder_calibration));
  memset((void *)&g_output_position,0,sizeof(g_output_position));
  g_user_control.status.state=USER_IDLE;
  g_user_control.status.fault=0;
  g_user_control.status.active_mode=USER_MODE_COUNT;
  g_user_control.enable=1; g_user_control.mode=mode;
  g_user_control.speed.target_rpm=1000;
  g_user_control.speed.forward_limit_rpm=500;
  g_user_control.speed.reverse_limit_rpm=300;
  g_user_control.speed.acceleration_rpm_s=100;
  g_user_control.speed.deceleration_rpm_s=200;
  g_user_control.position.target_deg=720;
  g_user_control.calibration.ud_v=1;
  g_user_control.calibration.uq_v=1;
  g_user_control.calibration.duration_s=.1f;
  g_global_mode_control.active_mode=FOC_CONTROL_MODE_CURRENT_OPEN_LOOP;
  g_speed_test.calibration_ok=1;
  boot=moe=0; tick=100; power_safe=sampling_done=1;
}
static void backend_running(void) {
  if(g_user_control.mode==USER_MODE_POSITION) {
    assert(g_output_position.command==1);
    g_output_position.command=0;
    g_output_position.state=1;
    g_output_position.actual_deg=123;
    g_speed_run.enable=1;
  }
  g_global_mode_control.runtime.output_request=1;
  g_global_mode_control.runtime.output_armed=1;
  g_foc_controller.running=moe=1;
}
int main(void) {
  reset(USER_MODE_POSITION); boot=3;
  MainUser_Service(&boot); assert(boot==3 && g_user_control.status.state==USER_STARTING);
  g_user_control.enable=0; MainUser_Service(&boot);
  assert(!boot && !moe && !g_speed_run.enable);
  for(unsigned mode=0;mode<USER_MODE_COUNT;mode++) {
    reset(mode); MainUser_Service(&boot);
    assert(g_user_control.status.state==USER_STARTING &&
           g_user_control.status.active_mode==mode);
    if(mode==USER_MODE_ENCODER_CALIBRATION) {
      assert(g_encoder_calibration.command==1 &&
             g_encoder_calibration.requested.q_voltage_v==1);
      g_encoder_calibration.state=ENCODER_CAL_RUNNING;
      MainUser_Service(&boot); assert(g_user_control.status.state==USER_RUNNING);
      g_user_control.calibration.ud_v=NAN;
      MainUser_Service(&boot); assert(g_user_control.status.state==USER_RUNNING);
      g_encoder_calibration.state=ENCODER_CAL_DONE;
      MainUser_Service(&boot); assert(g_user_control.status.state==USER_DONE);
      MainUser_Service(&boot); assert(g_encoder_calibration.command==1);
    } else {
      backend_running(); MainUser_Service(&boot);
      assert(g_user_control.status.state==USER_RUNNING);
      if(mode==USER_MODE_POSITION) assert(g_output_position.target_deg==843);
      if(mode==USER_MODE_SPEED) assert(g_speed_run.target_rpm==500);
      if(mode==USER_MODE_OPEN_CURRENT || mode==USER_MODE_VOLTAGE)
        assert(!g_encoder_feedback_required);
    }
    g_user_control.enable=0; MainUser_Service(&boot);
    assert(g_user_control.status.state==USER_IDLE && !moe &&
           !g_global_mode_control.runtime.output_request && !g_speed_run.enable);
  }
  reset(USER_MODE_TORQUE); g_speed_test.calibration_ok=0;
  MainUser_Service(&boot); assert(g_user_control.status.fault==USER_NOT_READY && !moe);
  reset(USER_MODE_OPEN_CURRENT); g_speed_test.calibration_ok=0;
  MainUser_Service(&boot); assert(g_user_control.status.state==USER_STARTING);
  reset(USER_MODE_SPEED); MainUser_Service(&boot);
  tick+=1001; MainUser_Service(&boot); assert(g_user_control.status.state==USER_FAULT);
  reset(USER_MODE_TORQUE); MainUser_Service(&boot); backend_running(); MainUser_Service(&boot);
  g_user_control.mode=USER_MODE_OPEN_CURRENT; MainUser_Service(&boot);
  assert(!moe && g_user_control.status.state==USER_IDLE);
  MainUser_Service(&boot); assert(g_user_control.status.state==USER_STARTING);
  reset(USER_MODE_OPEN_CURRENT); g_global_mode_control.runtime.fault_flags=4;
  MainUser_Service(&boot); assert(g_user_control.status.state==USER_FAULT);
  g_user_control.enable=0; MainUser_Service(&boot);
  assert(g_global_mode_control.runtime.fault_flags==4);
  g_user_control.enable=1; MainUser_Service(&boot);
  assert(g_user_control.status.state==USER_FAULT && !moe);
  assert(stops>0);
  puts("PASS real application dispatcher: six modes, OFF/ON, boot cancellation, mode-switch OFF boundary, position origin, immutable calibration, startup timeout, retained protection");
}
