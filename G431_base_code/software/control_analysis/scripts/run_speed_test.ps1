param(
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-z0-9_]+$')][string]$Tag,
 [ValidateSet(1,2,3,4,5,6)][int]$Kind=1,
 [double]$Amplitude=.1,[double]$Duration=4,[double]$AngleEnd=1.570796327,
 [double]$Kp=.01,[double]$Ki=.02,[double]$Frequency=1,[double]$Bias=0,[double]$CurrentLimit=.1,
 [ValidateRange(1,100)][int]$Divider=32,
 [string]$PrbsFile='', [ValidateSet(16,32,64)][int]$PrbsBitMs=16
)
$ErrorActionPreference='Stop'
Set-Location (Resolve-Path "$PSScriptRoot/../../..")
$gdb='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.8.0-win32-x64/tools/gdb/bin/arm-none-eabi-gdb.exe'
$elf='software/control_analysis/build_speed/out/servo_v1/servo_v1/servo_v1.axf'
$prefix="software/control_analysis/results/speed_$Tag"
if(Test-Path "$prefix.read.log") {throw 'Choose a new tag; results already exist'}
function Stage($name,$commands) {
 $guard=if($name -eq 'read') {'software/control_analysis/scripts/require_output_stopped.gdb'} else {'software/control_analysis/scripts/require_current_off.gdb'}
 $args=@('-batch',$elf,'-ex','target remote localhost:3333','-x',$guard)
 foreach($c in $commands) {$args+=@('-ex',$c)}
 $args+=@('-ex','detach')
 & $gdb @args *> "$prefix.$name.log"
 if($LASTEXITCODE -or (Select-String -Path "$prefix.$name.log" -Pattern 'Failed to open|Cannot access|No symbol|Invalid cast|Error in sourced|not defined')) {throw "Stage $name failed"}
}
$fields=@{kind=$Kind;amplitude=$Amplitude;duration_s=$Duration;angle_end_rad=$AngleEnd;kp=$Kp;ki=$Ki;frequency_hz=$Frequency;bias=$Bias;record_divider=$Divider;current_limit_a=$CurrentLimit}
$fields.prbs_bit_ms=$PrbsBitMs
$cmds=@('set g_current_loop_test.command=3','set g_speed_test.command=3')
foreach($k in $fields.Keys) { $v=([double]$fields[$k]).ToString([Globalization.CultureInfo]::InvariantCulture); $cmds+="set g_speed_test.requested.$k=$v" }
if($Kind -ge 5) {
 if(!(Test-Path $PrbsFile) -or (Get-Item $PrbsFile).Length -ne 127) {throw 'PRBS file must contain exactly 127 binary bytes'}
 $prbsPath=(Resolve-Path $PrbsFile).Path.Replace('\','/')
 if($prbsPath -match '\s') {throw 'GDB restore path must not contain whitespace'}
 $cmds+="restore $prbsPath binary &g_speed_test.prbs_sequence[0]"
 $cmds+="dump binary memory $prefix.sequence_readback.bin &g_speed_test.prbs_sequence[0] &g_speed_test.prbs_sequence[127]"
}
Stage configure $cmds
if($Kind -ge 5) {
 if([Convert]::ToBase64String([IO.File]::ReadAllBytes($prbsPath)) -ne [Convert]::ToBase64String([IO.File]::ReadAllBytes((Join-Path (Get-Location) "$prefix.sequence_readback.bin")))) {throw 'PRBS readback mismatch; will not arm'}
}
Stage arm @('set g_speed_test.command=1')
& $gdb -batch $elf -x software/control_analysis/scripts/start_speed_test.gdb *> "$prefix.start.log"
if($LASTEXITCODE -or !(Select-String -Path "$prefix.start.log" -SimpleMatch 'Bounded speed test requested')) {throw 'Start refused'}
# Firmware terminates excitation on its own wall-clock deadline. Do not halt while active.
Start-Sleep -Milliseconds ([int](1000*$Duration+1000))
$dump=if($Kind -ge 5) {"dump binary memory $prefix.bin &g_current_loop_test.speed_prbs_samples[0] &g_current_loop_test.speed_prbs_samples[1365]"} else {"dump binary memory $prefix.bin &g_speed_test.records[0] &g_speed_test.records[128]"}
Stage read @('p g_speed_test.state','p g_speed_test.fault','p g_speed_test.count',
 'p g_speed_test.active','p g_speed_test.elapsed','p g_speed_test.max_interval_cycles',
 'p g_encoder_snapshot','p g_encoder_diagnostics','p g_current_trip','p g_global_mode_control.runtime',
 'p g_speed_test.feedback_fault','p g_speed_test.feedback_age_cycles',
 $dump)
if($Kind -ge 5) {
 Stage diagnostics @("dump binary memory $prefix.inner.bin &g_speed_test.records[0] &g_speed_test.records[128]")
}
Get-Content "$prefix.read.log" | Select-String '^\$'
if(!(Select-String -Path "$prefix.read.log" -Pattern '^\$1 = 3$')) {throw 'Test did not complete; investigate before retry'}
if($Kind -lt 5) { py -3 software/control_analysis/scripts/analyze_speed_test.py "$prefix.bin" }
