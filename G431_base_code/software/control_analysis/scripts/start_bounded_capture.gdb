set pagination off
target remote localhost:3333
if g_current_loop_test.state == 1 && g_global_mode_control.current_open_loop.active_stage == 2 && g_global_mode_control.runtime.fault_flags == 0 && g_global_mode_control.runtime.power_stage_safe == 1
  set g_global_mode_control.runtime.output_request = 1
  printf "Bounded capture requested; firmware stops at capture end or fault.\n"
else
  printf "REFUSED: capture/stage/safety not ready.\n"
end
detach
quit
