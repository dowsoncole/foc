#include "user_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
  assert(g_user_control.enable==1 && g_user_control.mode==USER_MODE_POSITION);
  for(unsigned i=0;i<USER_MODE_COUNT;i++) assert(UserControl_Validate(i));
  assert(!UserControl_Validate(USER_MODE_COUNT));
  g_user_control.speed.acceleration_rpm_s=100;
  g_user_control.speed.deceleration_rpm_s=200;
  g_user_control.speed.forward_limit_rpm=500;
  g_user_control.speed.reverse_limit_rpm=300;
  assert(UserControl_SpeedRamp(0,1000,1)==100);
  assert(UserControl_SpeedRamp(450,1000,1)==500);
  assert(UserControl_SpeedRamp(500,0,1)==300);
  assert(UserControl_SpeedRamp(100,-300,.25f)==50);
  assert(UserControl_SpeedRamp(100,-1000,1)==-50);
  assert(UserControl_SpeedRamp(-250,-1000,1)==-300);
  assert(UserControl_SpeedRamp(-100,1000,1)==50);
  g_user_control.speed.acceleration_rpm_s=NAN;
  assert(!UserControl_Validate(USER_MODE_SPEED));

  foc_input_t in={0};
  g_user_control.mode=g_user_control.status.active_mode=USER_MODE_OPEN_CURRENT;
  g_user_control.open_current.id_a=1.25f;
  g_user_control.open_current.iq_a=-.5f;
  g_user_control.open_current.electrical_hz=100;
  UserControl_ResetAngle();
  for(unsigned i=0;i<20000;i++) {
    assert(UserControl_FastReference(&in,.00005f,NAN,0));
    assert(in.electrical_angle_rad>=0 && in.electrical_angle_rad<6.283186f);
    assert(in.d_reference==1.25f && in.q_reference==-.5f);
  }
  assert(fabsf(remainderf(in.electrical_angle_rad,6.28318530718f))<.01f);
  g_user_control.open_current.electrical_hz=-100;
  UserControl_ResetAngle();
  assert(UserControl_FastReference(&in,.00005f,NAN,0));
  assert(in.electrical_angle_rad>6.2f);
  g_user_control.enable=0;
  assert(!UserControl_FastReference(&in,.00005f,0,1));
  g_user_control.enable=1;
  g_user_control.mode=USER_MODE_TORQUE;
  assert(!UserControl_FastReference(&in,.00005f,1,1)); /* Switch boundary. */
  g_user_control.status.active_mode=USER_MODE_TORQUE;
  assert(!UserControl_FastReference(&in,.00005f,NAN,0));
  g_user_control.torque.id_a=-1; g_user_control.torque.iq_a=2;
  assert(UserControl_FastReference(&in,.00005f,1.2f,1));
  assert(in.electrical_angle_rad==1.2f && in.d_reference==-1 && in.q_reference==2);
  g_user_control.position.target_deg=720;
  assert(UserControl_Validate(USER_MODE_POSITION));
  g_user_control.position.target_deg=-1080;
  assert(UserControl_Validate(USER_MODE_POSITION));
  g_user_control.position.target_deg=INFINITY;
  assert(!UserControl_Validate(USER_MODE_POSITION));
  g_user_control.calibration.ud_v=0; g_user_control.calibration.uq_v=-1.5f;
  assert(UserControl_Validate(USER_MODE_ENCODER_CALIBRATION));
  g_user_control.calibration.ud_v=2.8f;
  assert(!UserControl_Validate(USER_MODE_ENCODER_CALIBRATION));
  puts("PASS user modes/units, speed limits/accel/decel/reversal, encoder-free rotating current, torque feedback gates, multi-turn targets");
}
