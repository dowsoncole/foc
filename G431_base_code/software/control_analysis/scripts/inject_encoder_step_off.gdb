# Software receive-buffer fault injection, NEVER a motor-running test.
# Use the matching AXF, after an independent non-halting MOE=0 check.
source software/control_analysis/scripts/require_output_stopped.gdb
if ((*(unsigned int*)0x40012c44) & 0x8000) || g_speed_run.enable
  echo REFUSED: motor active\n
  detach
  quit 1
end
break Encoder_TransferComplete
continue
if status != 0 || !s_encoder.have_previous_angle
  echo REFUSED: no completed valid baseline\n
  delete breakpoints
  detach
  quit 1
end
set $raw=((s_encoder.previous_raw_angle+524288)&2097151)
set $payload=$raw<<3
set $crc=0
set $i=0
while $i<24
  set $feedback=(($crc>>7)^($payload>>(23-$i)))&1
  set $crc=($crc<<1)&255
  if $feedback
    set $crc=$crc^7
  end
  set $i=$i+1
end
set var s_encoder.rx[2]=($payload>>16)&255
set var s_encoder.rx[3]=($payload>>8)&255
set var s_encoder.rx[4]=$payload&255
set var s_encoder.rx[5]=$crc
echo Injected CRC-correct 90-degree raw step into software RX buffer, output OFF.\n
p s_encoder.previous_raw_angle
p $raw
p s_encoder.rx
delete breakpoints
detach
