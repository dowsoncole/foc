param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$firmwareRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild=Join-Path ([IO.Path]::GetTempPath()) ('user-control-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $firmwareRoot 'FOC/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Servo/Inc')) `
  (Join-Path $firmwareRoot 'Servo/Src/user_control.c') `
  (Join-Path $PSScriptRoot 'test_user_control.c') -o (Join-Path $testBuild 'user.exe') -lm
if($LASTEXITCODE -ne 0) { throw 'User control compile failed' }
& (Join-Path $testBuild 'user.exe')
if($LASTEXITCODE -ne 0) { throw 'User control tests failed' }
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $PSScriptRoot 'speed_stubs')) `
  ('-I'+(Join-Path $firmwareRoot 'Core/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'FOC/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Servo/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Encoder/Inc')) `
  (Join-Path $firmwareRoot 'Servo/Src/user_control.c') `
  (Join-Path $PSScriptRoot 'test_user_dispatch.c') -o (Join-Path $testBuild 'dispatcher.exe') -lm
if($LASTEXITCODE -ne 0) { throw 'User dispatcher compile failed' }
& (Join-Path $testBuild 'dispatcher.exe')
if($LASTEXITCODE -ne 0) { throw 'User dispatcher tests failed' }
