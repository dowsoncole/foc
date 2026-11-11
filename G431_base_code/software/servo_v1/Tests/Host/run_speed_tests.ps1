param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$firmwareRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild=Join-Path ([IO.Path]::GetTempPath()) ('speed-host-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
$exe=Join-Path $testBuild 'test_speed.exe'
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 -DOUTPUT_AXIS_ENABLE=0 -DCURRENT_LIMIT_HARDWARE_ONLY=0 `
  ('-I'+(Join-Path $PSScriptRoot 'speed_stubs')) `
  ('-I'+(Join-Path $firmwareRoot 'Core/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'FOC/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Servo/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Tests/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Encoder/Inc')) `
  (Join-Path $firmwareRoot 'Servo/Src/speed_commission.c') `
  (Join-Path $firmwareRoot 'Servo/Src/encoder_calibration.c') `
  (Join-Path $firmwareRoot 'Servo/Src/user_control.c') `
  (Join-Path $PSScriptRoot 'test_speed_commission.c') -o $exe -lm
if($LASTEXITCODE -ne 0) { throw 'Speed host test compile failed' }
& $exe
if($LASTEXITCODE -ne 0) { throw 'Speed host tests failed' }

# Above preserves legacy numerical/state regressions; also exercise the actual
# hardware-only current policy used by the board build.
& (Join-Path $PSScriptRoot 'run_hardware_current_policy_tests.ps1') -Compiler $Compiler
