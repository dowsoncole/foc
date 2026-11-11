param(
 [Parameter(Mandatory=$true)][string]$Output,
 [Parameter(Mandatory=$true)][string]$Axf,
 [string]$Probe='150710982'
)
# Explicit motion recipe: run only with fixed motor, unloaded shaft and stop access.
# Existing output subdirectories are never overwritten; any error stops the batch.
$ErrorActionPreference='Stop'
$python='C:/Users/penghao/AppData/Local/Programs/Python/Python312/python.exe'
$measure=Join-Path $PSScriptRoot 'measure_speed_response.py'
$common=@('--axf',$Axf,'--probe',$Probe,'--kp','0.030637519','--ki','0.13476460',
 '--estimator-tau-ms','1','--use-default','--run')
$steps=@(
 @('balanced_default_step',300,100,100000,1),
 @('balanced_default_ramp',300,100,9000,1),
 @('balanced_default_1000',800,200,100000,4)
)
foreach($step in $steps) {
 & $python -B $measure (Join-Path $Output $step[0]) @common --bias $step[1] `
   --amplitude $step[2] --ramp $step[3] --step-delay 2 --divider $step[4]
 if($LASTEXITCODE) {throw ('Failed step: '+$step[0])}
}
foreach($frequency in @(2,5,10,20,30,40,60,80,100,120,160)) {
 & $python -B $measure (Join-Path $Output ('balanced_f'+$frequency)) @common `
   --bias 300 --amplitude 50 --frequency $frequency
 if($LASTEXITCODE) {throw ('Failed frequency: '+$frequency)}
}
