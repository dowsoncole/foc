param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$fw=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out=Join-Path $env:TEMP ('output-position-'+[guid]::NewGuid().ToString('N')+'.exe')
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $PSScriptRoot 'output_position_stubs')) `
  ('-I'+$fw+'/Modules/Encoder/Inc') ('-I'+$fw+'/Servo/Inc') `
  ($fw+'/Modules/Encoder/Src/mpt_t485_protocol.c') `
  ($fw+'/Servo/Src/output_position_math.c') ($fw+'/Servo/Src/output_position.c') `
  (Join-Path $PSScriptRoot 'test_output_position.c') -o $out
if($LASTEXITCODE) { throw 'Host compilation failed' }
& $out
if($LASTEXITCODE) { throw 'Host tests failed' }
