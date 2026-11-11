param(
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-z0-9_]+$')][string]$Tag,
 [double]$Kp=0.005,[double]$Ki=0.2,
 [ValidateSet(0,1)][int]$Axis=0,
 [ValidateRange(-0.1,0.1)][double]$Amplitude=0.05,
 [ValidateSet(1,4,8)][int]$Average=4
)
$ErrorActionPreference='Stop'
Set-Location (Resolve-Path "$PSScriptRoot\..\..\..")
$gdb='C:\Users\penghao\.vscode\extensions\arm.vscode-cmsis-debugger-1.8.0-win32-x64\tools\gdb\bin\arm-none-eabi-gdb.exe'
$elf='software/servo_v1/MDK-ARM/out/servo_v1/servo_v1/servo_v1.axf'
$result="software/control_analysis/results/pi_$Tag"
function Invoke-Stage([string]$Name,[string[]]$Commands,[switch]$CheckOff) {
  $arguments=@('-batch',$elf,'-ex','target remote localhost:3333')
  if($CheckOff) {$arguments+=@('-x','software/control_analysis/scripts/require_current_off.gdb')}
  foreach($command in $Commands) {$arguments+=@('-ex',$command)}
  $arguments+=@('-ex','detach')
  & $gdb @arguments *> "$result.$Name.log"
  if($LASTEXITCODE) {throw "GDB stage $Name failed; inspect $result.$Name.log"}
}
$kpText=$Kp.ToString([Globalization.CultureInfo]::InvariantCulture)
$kiText=$Ki.ToString([Globalization.CultureInfo]::InvariantCulture)
$ampText=$Amplitude.ToString([Globalization.CultureInfo]::InvariantCulture)
Invoke-Stage configure @(
 "set g_current_tuning.requested_d.kp=$kpText",
 "set g_current_tuning.requested_d.ki=$kiText",
 'set g_current_tuning.requested_d.integral=0',
 'set g_current_tuning.requested_d.integral_min=-0.01',
 'set g_current_tuning.requested_d.integral_max=0.01',
 'set g_current_tuning.requested_q=g_current_tuning.requested_d',
 'set g_current_tuning.apply_request=2',
 'set g_current_loop_test.command=3',
 "set g_current_loop_test.requested.axis=$Axis",
 "set g_current_loop_test.requested.amplitude_a=$ampText",
 'set g_current_loop_test.requested.kind=0',
 'set g_current_loop_test.requested.settle_s=0.005',
 'set g_current_loop_test.requested.step_delay_s=0.005',
 "set g_current_loop_test.requested.average_count=$Average"
) -CheckOff
Invoke-Stage arm @('p g_current_tuning.status','p g_foc_controller.algorithm.parameters','p g_encoder_snapshot','set g_current_loop_test.command=1') -CheckOff
& $gdb -batch $elf -x software/control_analysis/scripts/start_bounded_capture.gdb *> "$result.start.log"
if($LASTEXITCODE -or !(Select-String -Path "$result.start.log" -SimpleMatch 'Bounded capture requested')) {throw 'Capture refused'}
Start-Sleep -Milliseconds 600
Invoke-Stage read @(
 'p g_current_loop_test.state','p g_current_loop_test.abort_reason',
 'p g_current_loop_test.sample_count','p g_current_loop_test.saturation_count',
 'p g_current_trip','p g_global_mode_control.runtime','p g_tim1_pwm_sampling',
 'p g_waveform_uart_completed','p g_waveform_uart_dma_errors',
 'p g_encoder_snapshot','p g_encoder_diagnostics',
 'p g_current_test_timing',
 "dump binary memory $result.bin &g_current_loop_test.samples[0] &g_current_loop_test.samples[512]"
)
Get-Content "$result.read.log" | Select-String '^\$'
if (!(Select-String -Path "$result.read.log" -SimpleMatch 'CURRENT_TEST_DONE')) {throw 'Capture did not complete; do not retry without investigating'}
py -3 software/control_analysis/scripts/score_pi_capture.py "$result.bin" --axis $Axis --kp $kpText --ki $kiText --average $Average
