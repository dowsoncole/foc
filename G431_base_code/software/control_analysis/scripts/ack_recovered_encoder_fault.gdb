# Deliberate one-time acknowledgement, never an automatic retry.
# Run ONLY after non-halting observation proves TIM1 MOE=0, using matching AXF.
source software/control_analysis/scripts/require_output_stopped.gdb
if ((*(unsigned int *)0x40012c44) & 0x8000) || g_global_mode_control.runtime.precharge_in_progress || !g_global_mode_control.runtime.power_stage_safe
  echo REFUSED: PWM/precharge/power-stage condition\n
  detach
  quit 1
end
if g_foc_controller.fault_flags != 6 || g_global_mode_control.runtime.fault_flags != 6 || g_speed_test.fault != 4 || g_speed_test.feedback_fault != 4 || g_current_trip.count != 0
  echo REFUSED: not the reviewed encoder-only fault\n
  detach
  quit 1
end
if !g_encoder_snapshot.angle_valid || !g_encoder_snapshot.speed_valid || !g_encoder_snapshot.electrical_angle_valid || g_encoder_snapshot.fault_flags || g_encoder_snapshot.status_raw || !g_speed_test.calibration_ok
  echo REFUSED: feedback/calibration not recovered\n
  detach
  quit 1
end
if g_encoder_snapshot.mechanical_speed_rad_s > 1 || g_encoder_snapshot.mechanical_speed_rad_s < -1
  echo REFUSED: rotor not stationary\n
  detach
  quit 1
end
set var g_speed_run.enable=0
set var g_foc_controller.fault_flags=0
set var g_global_mode_control.runtime.fault_flags=0
set var g_speed_run.target_rpm=1500.0
set var g_speed_test.command=3
echo Acknowledged reviewed recovered encoder fault; output remains OFF.\n
