target remote localhost:3333
source software/control_analysis/scripts/require_current_off.gdb
if g_speed_test.state != 1 || g_current_loop_test.state != 0 || g_global_mode_control.active_mode != 1 || g_global_mode_control.current_open_loop.active_stage != 2
  echo Refused: incompatible state\n
  detach
  quit 1
end
if g_global_mode_control.runtime.power_stage_safe == 0
  detach
  quit 1
end
set g_global_mode_control.runtime.output_request=1
echo Bounded speed test requested\n
detach
