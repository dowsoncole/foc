/* User commands are translated here, at the application boundary. */
static void MainUser_Stop(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  r->output_request = r->output_armed = 0U;
  g_speed_run.enable = 0U;
  g_speed_test.command = 0U;
  if (g_speed_test.state != 4U) g_speed_test.state = 0U;
  if (EncoderCalibration_Busy()) g_encoder_calibration.command = 2U;
  g_output_position.command = 2U;
  FOC_ControllerStop(&g_foc_controller);
  Tim1Pwm_EmergencyStop();
  r->running = r->output_enabled = 0U;
  if (!mask) __enable_irq();
}
static void MainUser_Fail(uint32_t reason)
{
  MainUser_Stop();
  g_user_control.status.fault = reason;
  g_user_control.status.state = USER_FAULT;
}
static void MainUser_Service(volatile uint8_t *boot_pending)
{
  static uint32_t start_ms;
  static float position_origin;
  static uint8_t position_origin_pending;
  volatile user_control_t *u = &g_user_control;
  foc_control_runtime_t *r = &g_global_mode_control.runtime;
  encoder_snapshot_t feedback = {0};
  u->status.actual_rpm = Encoder_ReadSnapshot(&feedback) && feedback.speed_valid ?
      feedback.mechanical_speed_rad_s / SPEED_RPM_TO_RAD_S : 0.0f;
  u->status.actual_position_deg = g_output_position.actual_deg - position_origin;
  u->status.id_a = g_foc_controller.algorithm.output.d_current;
  u->status.iq_a = g_foc_controller.algorithm.output.q_current;
  u->status.zero_electrical_rad = g_encoder_calibration.offset_rad;
  u->status.calibration_valid = g_encoder_calibration.valid;

  /* Preserve the existing boot alignment/hold sequence. An explicit user
     disable or different mode takes ownership before it can auto-start. */
  if (*boot_pending) {
    if (u->enable == 1U && u->mode == USER_MODE_POSITION) {
      u->status.state = USER_STARTING;
      u->status.active_mode = USER_MODE_COUNT;
      return;
    }
    *boot_pending = 0U;
    MainUser_Stop();
    u->status.state = USER_IDLE;
    return;
  }
  if (u->enable == 0U) {
    if (u->status.state != USER_IDLE || r->output_request || g_speed_run.enable)
      MainUser_Stop();
    u->status.state = USER_IDLE;
    u->status.fault = USER_OK;
    u->status.active_mode = USER_MODE_COUNT;
    return;
  }
  if (u->status.state == USER_STARTING && u->status.active_mode == USER_MODE_COUNT)
    u->status.state = USER_IDLE; /* Adopt the completed boot hold below. */
  if (u->status.state == USER_FAULT) return;
  if (u->status.state == USER_DONE && u->status.active_mode == u->mode) return;
  if (u->enable != 1U ||
      (!(u->mode == USER_MODE_ENCODER_CALIBRATION && u->status.state != USER_IDLE) &&
       !UserControl_Validate(u->mode))) {
    MainUser_Fail(USER_BAD_PARAMETER); return;
  }
  if (r->fault_flags || g_foc_controller.fault_flags || g_speed_test.fault ||
      g_speed_run.fault || g_output_position.fault) {
    MainUser_Fail(USER_BACKEND_FAULT); return;
  }
  if (u->status.active_mode != u->mode && u->status.state != USER_IDLE) {
    MainUser_Stop();
    u->status.state = USER_IDLE;
    u->status.active_mode = USER_MODE_COUNT;
    return; /* One full OFF service iteration before a new start. */
  }
  if (u->status.state == USER_IDLE) {
    /* Adopt the legacy boot hold without overwriting the requested target. */
    if (u->mode == USER_MODE_POSITION && g_output_position.state == 1U &&
        g_speed_run.enable) {
      u->status.active_mode = u->mode;
      u->status.state = USER_RUNNING;
      position_origin = g_output_position.actual_deg;
      return;
    }
    if (r->output_request || r->output_armed || r->precharge_in_progress ||
        g_foc_controller.running || Tim1Pwm_IsOutputEnabled()) {
      MainUser_Stop(); return;
    }
    if (g_global_mode_control.active_mode != FOC_CONTROL_MODE_CURRENT_OPEN_LOOP) {
      g_global_mode_control.requested_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP;
      return;
    }
    if (CurrentSampling_GetCalibrationState() != CURRENT_SAMPLING_CAL_DONE ||
        !MainFoc_IsPowerStageSafe(NULL)) {
      MainUser_Fail(USER_NOT_READY); return;
    }
    if ((u->mode == USER_MODE_TORQUE || u->mode == USER_MODE_SPEED ||
         u->mode == USER_MODE_POSITION) && !g_speed_test.calibration_ok) {
      MainUser_Fail(USER_NOT_READY); return;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    g_speed_test.state = 0U;
    g_speed_test.active.kind = 0U;
    g_global_mode_control.current_open_loop.requested_stage =
      g_global_mode_control.current_open_loop.active_stage = FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP;
    MainFoc_ConfigureCurrentStageParameters(FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP);
    foc_parameters_t *p = &g_foc_controller.algorithm.parameters;
    p->voltage_limit = SPEED_RUN_VOLTAGE_LIMIT_PU;
    p->current_limit = SPEED_RUN_IQ_LIMIT_A;
    p->d_controller.integral_min = p->q_controller.integral_min = -p->voltage_limit;
    p->d_controller.integral_max = p->q_controller.integral_max = p->voltage_limit;
    g_encoder_feedback_required = u->mode != USER_MODE_VOLTAGE &&
        u->mode != USER_MODE_OPEN_CURRENT;
    UserControl_ResetAngle();
    u->status.active_mode = u->mode;
    u->status.state = USER_STARTING;
    start_ms = HAL_GetTick();
    if (u->mode == USER_MODE_ENCODER_CALIBRATION) {
      g_encoder_calibration.requested = (encoder_calibration_config_t){
        u->calibration.ud_v, u->calibration.duration_s, u->calibration.uq_v};
      g_encoder_calibration.command = 1U;
    } else if (u->mode == USER_MODE_POSITION) {
      position_origin_pending = 1U;
      g_output_position.command = 1U;
    } else if (u->mode == USER_MODE_SPEED) {
      float target = u->speed.target_rpm;
      if (target > u->speed.forward_limit_rpm) target = u->speed.forward_limit_rpm;
      if (target < -u->speed.reverse_limit_rpm) target = -u->speed.reverse_limit_rpm;
      g_speed_run.target_rpm = target;
      g_speed_run.enable = 1U;
    } else r->output_request = 1U;
    if (!mask) __enable_irq();
    return;
  }
  if (u->mode == USER_MODE_ENCODER_CALIBRATION) {
    if (g_encoder_calibration.state == ENCODER_CAL_FAILED)
      MainUser_Fail(USER_BACKEND_FAULT);
    else if (g_encoder_calibration.state == ENCODER_CAL_DONE) u->status.state = USER_DONE;
    else if (g_encoder_calibration.state == ENCODER_CAL_RUNNING) u->status.state = USER_RUNNING;
    return;
  }
  if (u->mode == USER_MODE_SPEED) {
    float target = u->speed.target_rpm;
    if (target > u->speed.forward_limit_rpm) target = u->speed.forward_limit_rpm;
    if (target < -u->speed.reverse_limit_rpm) target = -u->speed.reverse_limit_rpm;
    g_speed_run.target_rpm = target;
  }
  if (u->mode == USER_MODE_POSITION && g_output_position.state == 1U) {
    if (position_origin_pending) {
      position_origin = g_output_position.actual_deg;
      position_origin_pending = 0U;
    }
    g_output_position.target_deg = position_origin + u->position.target_deg;
  }
  if (r->output_armed && Tim1Pwm_IsOutputEnabled()) u->status.state = USER_RUNNING;
  else if (u->status.state == USER_RUNNING ||
           (uint32_t)(HAL_GetTick()-start_ms) > 1000U)
    MainUser_Fail(USER_BACKEND_FAULT);
}
