param([Parameter(Mandatory=$true)][ValidatePattern('^[a-z0-9_]+$')][string]$Tag)
$ErrorActionPreference='Stop'
Set-Location (Resolve-Path "$PSScriptRoot\..\..\..")
$toolRoot='C:\Users\penghao\.vscode\extensions\arm.vscode-cmsis-debugger-1.8.0-win32-x64\tools'
$probe="$toolRoot\pyocd\pyocd.exe"
$gdb="$toolRoot\gdb\bin\arm-none-eabi-gdb.exe"
$pack='C:\Users\penghao\AppData\Local\Arm\Packs\Keil\STM32G4xx_DFP\2.2.0'
$elf='software/servo_v1/MDK-ARM/out/servo_v1/servo_v1/servo_v1.axf'
$result="software/control_analysis/results/adc_$Tag"
$connection=@('--no-config','--pack',$pack,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-f','1000000')
$check = & $probe commander @connection -M attach -c 'read32 0x40012c44 4'
if ($LASTEXITCODE -ne 0 -or "$check" -notmatch '40012c44:\s+([0-9a-fA-F]{8})') { throw 'Cannot verify MOE' }
if (([Convert]::ToUInt32($Matches[1],16) -band 0x8000) -ne 0) { throw 'PWM active: refusing flash or halt' }
$env:PATH='C:\Users\penghao\.vcpkg\artifacts\2139c4c6\tools.kitware.cmake\3.31.12\bin;C:\Users\penghao\.vcpkg\artifacts\2139c4c6\tools.ninja.build.ninja\1.13.2;'+$env:PATH
$env:AC6_TOOLCHAIN_6_24_0='C:\Users\penghao\AppData\Local\Keil_v5\ARM\ARMCLANG\Bin'
& 'C:\Users\penghao\.vcpkg\artifacts\2139c4c6\tools.open.cmsis.pack.cmsis.toolbox\2.15.0\bin\cbuild.exe' software/servo_v1/MDK-ARM/servo_v1.csolution.yml
if ($LASTEXITCODE) { throw 'Build failed' }
& $probe load @connection --format elf $elf
if ($LASTEXITCODE) { throw 'Flash failed' }
$server = Start-Process $probe -ArgumentList (@('gdbserver')+$connection+@('-M','attach','-p','3333','-O','persist=true')) -WindowStyle Hidden -PassThru -RedirectStandardOutput "$result.server.log" -RedirectStandardError "$result.server_error.log"
try {
  Start-Sleep -Seconds 2
  & $gdb -batch $elf -ex 'target remote localhost:3333' -ex 'set g_adc_noise.command = 1' -ex detach *> "$result.arm.log"
  if ($LASTEXITCODE) { throw 'Capture arm failed' }
  Start-Sleep -Milliseconds 150
  & $gdb -batch $elf -ex 'target remote localhost:3333' -ex 'p g_adc_noise.command' -ex 'p g_adc_noise.count' -ex 'p g_global_mode_control.runtime' -ex 'p g_tim1_pwm_sampling' -ex 'p g_foc_controller.algorithm.parameters' -ex "dump binary memory $result.bin &g_adc_noise ((char*)&g_adc_noise)+sizeof(g_adc_noise)" -ex detach *> "$result.read.log"
  if ($LASTEXITCODE) { throw 'Capture read failed' }
} finally {
  if (!$server.HasExited) { Stop-Process -Id $server.Id }
}
& $probe commander @connection -M attach -c 'read32 0x40012c44 4; read32 0x5000000c 16; status' | Tee-Object "$result.registers.log"
py -3 software/control_analysis/scripts/summarize_adc_noise.py "$result.bin"
