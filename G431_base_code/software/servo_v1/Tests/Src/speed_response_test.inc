/* Included by speed_commission.c: deliberately shares its private outer-loop
   sample, preserving the production PendSV PI and one-publish Iq semantics. */
volatile speed_response_test_t g_speed_response={.version=2U};
extern foc_controller_t g_foc_controller;
static void SpeedResponse_Stop(uint32_t fault)
{
  g_speed_response.fault=fault;
  g_speed_response.state=fault ? 4U : 3U;
  g_user_control.enable=0U;
  g_speed_run.enable=0U;
}
void SpeedResponse_Service(uint8_t off)
{
  uint32_t command=g_speed_response.command;
  if(command) {
    g_speed_response.command=0U;
    if(command==2U) { SpeedResponse_Stop(1U); return; }
    if(command!=1U || !off || g_user_control.enable ||
       g_current_loop_test.state!=CURRENT_TEST_IDLE ||
       g_speed_test.state==1U || g_speed_test.state==2U || g_speed_test.fault) {
      SpeedResponse_Stop(2U); return;
    }
    speed_response_config_t c=g_speed_response.requested;
    if(c.kind>1U || c.divider<1U || c.divider>6U || c.samples<128U || c.samples>512U ||
       !isfinite(c.bias_rpm) || !isfinite(c.amplitude_rpm) ||
       !isfinite(c.frequency_hz) || !isfinite(c.delay_s) ||
       fabsf(c.bias_rpm)>1000.0f || fabsf(c.amplitude_rpm)>200.0f ||
       fabsf(c.amplitude_rpm)<1.0f || c.frequency_hz<0.0f || c.frequency_hz>160.0f ||
       (c.kind==1U && c.frequency_hz<2.0f) ||
       c.delay_s<0.05f || c.delay_s>2.8f ||
       fmaxf(0.0f,c.delay_s-.1f)+(float)(c.samples*c.divider)*.001f>4.4f) {
      SpeedResponse_Stop(3U); return;
    }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    g_speed_response.active=c;
    g_speed_response.count=g_speed_response.fault=g_speed_response.first_cycle=0U;
    g_speed_response.ticks=g_speed_response.saturated_rows=g_speed_response.last_cycle=0U;
    g_speed_response.state=1U;
    if(!mask) __enable_irq();
  }
  if(g_speed_response.state==1U || g_speed_response.state==2U) {
    if(g_speed_test.fault || g_current_loop_test.state!=CURRENT_TEST_IDLE ||
       (g_user_control.enable && g_user_control.mode!=USER_MODE_SPEED))
      SpeedResponse_Stop(4U);
    else if(g_speed_response.state==2U && !g_user_control.enable)
      SpeedResponse_Stop(1U);
  }
}
static float SpeedResponse_Target(float target, uint32_t now)
{
  if(g_speed_response.state==1U && g_user_control.enable &&
     g_user_control.status.active_mode==USER_MODE_SPEED) {
    g_speed_response.first_cycle=now;
    g_speed_response.state=2U;
  }
  if(g_speed_response.state!=2U) return target;
  float t=(float)(now-g_speed_response.first_cycle)/(float)SystemCoreClock;
  float elapsed=t-g_speed_response.active.delay_s;
  return g_speed_response.active.bias_rpm+(elapsed<0.0f ? 0.0f :
    g_speed_response.active.amplitude_rpm*(g_speed_response.active.kind==0U ? 1.0f :
    sinf(6.283185307f*g_speed_response.active.frequency_hz*elapsed)));
}
static void SpeedResponse_Observe(uint32_t now)
{
  if(g_speed_response.state!=2U) return;
  float elapsed=(float)(now-g_speed_response.first_cycle)/(float)SystemCoreClock;
  if(elapsed<g_speed_response.active.delay_s-.1f) return;
  uint32_t tick=g_speed_response.ticks++;
  if(tick%g_speed_response.active.divider) return;
  uint32_t count=g_speed_response.count;
  if(count>=g_speed_response.active.samples) { SpeedResponse_Stop(0U); return; }
  current_test_sample_t row={0};
  row.source_sequence=outer_feedback.sequence;
  row.time_s=(float)(now-g_speed_response.first_cycle)/(float)SystemCoreClock;
  row.d_reference_a=SpeedResponse_Target(g_speed_run.target_rpm,now); /* requested rpm */
  row.q_reference_a=g_speed_test.reference/SPEED_RPM_TO_RAD_S; /* applied rpm */
  row.d_current_a=outer_feedback.mechanical_speed_rad_s/SPEED_RPM_TO_RAD_S;
  row.q_current_a=g_foc_controller.algorithm.output.q_current;
  row.d_voltage_pu=(float)outer_feedback.raw_angle; /* exact 21-bit count */
  row.q_voltage_pu=g_speed_test.iq_reference; /* Iq command, A, for plant FRF. */
  if(g_foc_controller.algorithm.output.saturated) g_speed_response.saturated_rows++;
  g_current_loop_test.samples[count]=row;
  g_speed_response.last_cycle=now;
  g_speed_response.count=count+1U;
  if(count+1U>=g_speed_response.active.samples) SpeedResponse_Stop(0U);
}
