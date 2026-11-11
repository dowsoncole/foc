source software/control_analysis/scripts/require_output_stopped.gdb
if g_encoder_diagnostics.angle_step_error_count != 1 || g_encoder_diagnostics.crc_error_count || g_encoder_diagnostics.alarm_count || g_encoder_diagnostics.spi_resync_count != 1 || g_encoder_diagnostics.spi_resync_refused_count || !g_encoder_snapshot.angle_valid || !g_encoder_snapshot.electrical_angle_valid || g_encoder_snapshot.fault_flags
  echo FAIL: injected CRC-valid impossible step was not rejected/recovered as expected\n
  detach
  quit 1
end
p g_encoder_diagnostics
p g_encoder_fault_capture.first
p g_encoder_snapshot
echo PASS: impossible angle rejected; fresh validated feedback recovered, motor OFF.\n
# Reset ONLY diagnostic event counters, not controller faults or calibration.
# Later real errors replace .first because count is zero again.
set var g_encoder_fault_capture.count=0
set var g_encoder_diagnostics.angle_step_error_count=0
set var g_encoder_diagnostics.spi_resync_count=0
echo Diagnostic event-count baseline reset after the OFF-only injection.\n
detach
