# One reviewed, manual commissioning retry; invoke only with matching AXF.
# Before attaching, independently confirm TIM1 MOE=0 using non-halting read.
source software/control_analysis/scripts/require_output_stopped.gdb
if ((*(unsigned int *)0x40012c44) & 0x8000) || !g_global_mode_control.runtime.power_stage_safe || g_current_trip.count || !g_speed_test.calibration_ok
  echo REFUSED: power/current/calibration condition\n
  detach
  quit 1
end
if s_encoder.initialized || s_encoder.transfer_busy || s_async_busy || hspi1.State != 1
  echo REFUSED: pause acquisition and wait for the SPI/DMA transaction to finish\n
  detach
  quit 1
end
if g_foc_controller.fault_flags != 6 || g_global_mode_control.runtime.fault_flags != 6 || g_speed_test.feedback_fault != 4
  echo REFUSED: not the reviewed encoder-only fault\n
  detach
  quit 1
end
if !g_encoder_snapshot.angle_valid || !g_encoder_snapshot.speed_valid || !g_encoder_snapshot.electrical_angle_valid || g_encoder_snapshot.fault_flags || g_encoder_snapshot.status_raw || g_encoder_snapshot.mechanical_speed_rad_s > 1 || g_encoder_snapshot.mechanical_speed_rad_s < -1
  echo REFUSED: encoder has not recovered while stationary\n
  detach
  quit 1
end
# 168 MHz/64 = 2.625 MHz. Disable SPE before changing BR, keep other CR1 bits.
set $spi_cr1=*(unsigned int*)0x40013000
set *(unsigned int*)0x40013000=$spi_cr1 & ~0x40
set var hspi1.Init.BaudRatePrescaler=40
set *(unsigned int*)0x40013000=($spi_cr1 & ~0x38) | 0x28
set var g_speed_run.enable=0
set var g_speed_run.target_rpm=8500.0
set var g_speed_test.command=3
set var g_foc_controller.fault_flags=0
set var g_global_mode_control.runtime.fault_flags=0
set var s_encoder.initialized=1
echo Reviewed fault acknowledged; clock64 selected, output remains OFF.\n
