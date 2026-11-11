if g_global_mode_control.runtime.output_request || g_global_mode_control.runtime.output_armed || g_foc_controller.running || g_global_mode_control.runtime.fault_flags
  printf "REFUSED: output active or fault latched.\n"
  detach
  quit 1
end
