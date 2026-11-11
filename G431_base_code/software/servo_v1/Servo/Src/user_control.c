#include "user_control.h"
#include <math.h>

volatile user_control_t g_user_control = {
  .enable = 1U, .mode = USER_MODE_POSITION,
  .calibration = {1.5f, 0.0f, 3.0f},
  .speed = {0.0f, 9000.0f, 9000.0f, 9000.0f, 9000.0f},
  .status = {.active_mode = USER_MODE_COUNT}
};
static float angle;
void UserControl_ResetAngle(void) { angle = 0.0f; }
uint8_t UserControl_Validate(uint32_t mode)
{
  const volatile user_control_t *u = &g_user_control;
  switch (mode) {
  case USER_MODE_VOLTAGE:
    return isfinite(u->voltage.ud_v) && isfinite(u->voltage.uq_v) &&
      isfinite(u->voltage.electrical_hz) && fabsf(u->voltage.electrical_hz)<=300.0f;
  case USER_MODE_ENCODER_CALIBRATION: {
    float v=hypotf(u->calibration.ud_v,u->calibration.uq_v);
    return isfinite(v) && v>0.0f && v<=2.8f &&
      isfinite(u->calibration.duration_s) && u->calibration.duration_s>=0.05f &&
      u->calibration.duration_s<=10.0f;
  }
  case USER_MODE_OPEN_CURRENT:
    return isfinite(u->open_current.id_a) && isfinite(u->open_current.iq_a) &&
      isfinite(u->open_current.electrical_hz) && fabsf(u->open_current.electrical_hz)<=300.0f;
  case USER_MODE_TORQUE:
    return isfinite(u->torque.id_a) && isfinite(u->torque.iq_a);
  case USER_MODE_SPEED:
    return isfinite(u->speed.target_rpm) &&
      isfinite(u->speed.acceleration_rpm_s) && u->speed.acceleration_rpm_s>0 &&
      isfinite(u->speed.deceleration_rpm_s) && u->speed.deceleration_rpm_s>0 &&
      isfinite(u->speed.forward_limit_rpm) && u->speed.forward_limit_rpm>0 &&
      u->speed.forward_limit_rpm<=9000.0f &&
      isfinite(u->speed.reverse_limit_rpm) && u->speed.reverse_limit_rpm>0 &&
      u->speed.reverse_limit_rpm<=9000.0f;
  case USER_MODE_POSITION: return isfinite(u->position.target_deg);
  default: return 0;
  }
}
float UserControl_SpeedRamp(float applied, float target, float dt)
{
  const volatile user_control_t *u=&g_user_control;
  if(target>u->speed.forward_limit_rpm) target=u->speed.forward_limit_rpm;
  if(target<-u->speed.reverse_limit_rpm) target=-u->speed.reverse_limit_rpm;
  /* Reversal first brakes to zero, then accelerates with remaining time. */
  if(applied*target<0.0f) {
    float stop_time=fabsf(applied)/u->speed.deceleration_rpm_s;
    if(dt>=stop_time) { dt-=stop_time; applied=0.0f; }
    else return applied-copysignf(u->speed.deceleration_rpm_s*dt,applied);
  }
  float rate=fabsf(target)>fabsf(applied) ?
      u->speed.acceleration_rpm_s : u->speed.deceleration_rpm_s;
  float delta=target-applied, step=rate*dt;
  return fabsf(delta)<=step ? target : applied+copysignf(step,delta);
}
uint8_t UserControl_FastReference(foc_input_t *in,float dt,float enc,uint8_t valid)
{
  uint32_t mode=g_user_control.status.active_mode;
  if(g_user_control.enable!=1U || g_user_control.mode!=mode ||
     !UserControl_Validate(mode) || !isfinite(dt) || dt<=0) return 0;
  if(mode==USER_MODE_TORQUE) {
    if(!valid || !isfinite(enc)) return 0;
    in->electrical_angle_rad=enc;
    in->d_reference=g_user_control.torque.id_a;
    in->q_reference=g_user_control.torque.iq_a;
  } else {
    float hz=mode==USER_MODE_VOLTAGE ? g_user_control.voltage.electrical_hz :
        g_user_control.open_current.electrical_hz;
    angle+=6.28318530718f*hz*dt;
    if(angle>=6.28318530718f) angle-=6.28318530718f;
    if(angle<0) angle+=6.28318530718f;
    in->electrical_angle_rad=angle;
    in->d_reference=mode==USER_MODE_OPEN_CURRENT ? g_user_control.open_current.id_a : 0;
    in->q_reference=mode==USER_MODE_OPEN_CURRENT ? g_user_control.open_current.iq_a : 0;
  }
  in->angle_valid=in->reference_valid=1U;
  return 1;
}
