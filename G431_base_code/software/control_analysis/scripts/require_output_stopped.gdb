# Allow fault inspection only after the output/controller has stopped.
if g_global_mode_control.runtime.output_request || g_global_mode_control.runtime.output_armed || g_foc_controller.running || g_global_mode_control.runtime.output_enabled
  echo REFUSED: output not stopped\n
  detach
  quit 1
end
