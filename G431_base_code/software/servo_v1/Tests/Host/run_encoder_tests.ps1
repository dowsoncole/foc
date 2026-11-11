param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$firmwareRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild=Join-Path ([IO.Path]::GetTempPath()) ('encoder-host-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
foreach ($diagnosticMode in @(0, 1, 2)) {
$exe=Join-Path $testBuild ('test_encoder_'+$diagnosticMode+'.exe')
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  '-DMT6701_SSI_ENABLE=0' `
  ('-DENCODER_FAULT_TRIGGER_DIAGNOSTIC='+$diagnosticMode) `
  ('-I'+(Join-Path $PSScriptRoot 'encoder_stubs')) `
  ('-I'+(Join-Path $firmwareRoot 'Core/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Encoder/Inc')) `
  (Join-Path $firmwareRoot 'Modules/Encoder/Src/spi1_encoder.c') `
  (Join-Path $firmwareRoot 'Modules/Encoder/Src/mt6835.c') `
  (Join-Path $firmwareRoot 'Modules/Encoder/Src/encoder.c') `
  (Join-Path $PSScriptRoot 'test_encoder_transport.c') -o $exe -lm
if($LASTEXITCODE -ne 0) { throw 'Encoder host test compile failed' }
& $exe
if($LASTEXITCODE -ne 0) { throw 'Encoder host tests failed' }
}
