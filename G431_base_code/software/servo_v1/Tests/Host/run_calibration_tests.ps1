param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$firmwareRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild=Join-Path ([IO.Path]::GetTempPath()) ('calibration-host-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $firmwareRoot 'Servo/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Encoder/Inc')) `
  (Join-Path $firmwareRoot 'Servo/Src/encoder_calibration.c') `
  (Join-Path $firmwareRoot 'Modules/Encoder/Src/encoder_calibration_record.c') `
  (Join-Path $PSScriptRoot 'test_encoder_calibration.c') -o (Join-Path $testBuild 'encoder.exe') -lm
if($LASTEXITCODE -ne 0) { throw 'Encoder calibration compile failed' }
& (Join-Path $testBuild 'encoder.exe')
if($LASTEXITCODE -ne 0) { throw 'Encoder calibration tests failed' }
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $PSScriptRoot 'current_sampling_stubs')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Current_Sampling/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Core/Inc')) `
  (Join-Path $firmwareRoot 'Modules/Current_Sampling/Src/current_sampling.c') `
  (Join-Path $PSScriptRoot 'test_boot_current_calibration.c') -o (Join-Path $testBuild 'current.exe')
if($LASTEXITCODE -ne 0) { throw 'Current calibration compile failed' }
& (Join-Path $testBuild 'current.exe')
if($LASTEXITCODE -ne 0) { throw 'Current calibration tests failed' }
