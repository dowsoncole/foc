set pagination off
set confirm off
set mem inaccessible-by-default off
target remote localhost:3334
if (*(unsigned int *)0x40012c44 & 0x8000)
  echo REFUSED: PWM output is active; no reset or RAM writes permitted.\n
  detach
  quit 1
end
monitor reset halt
hbreak *main
continue
if $pc != ((unsigned int)&main & 0xfffffffe)
  echo REFUSED: did not reach main.\n
  quit 1
end
set var g_user_control.enable = 0
set var g_speed_run.enable = 0
delete breakpoints
echo Reset at main; both user and legacy enables OFF before initialization.\n
detach
quit
