param([string]$Compiler='C:/Program Files/JetBrains/CLion 2025.2.5/bin/mingw/bin/gcc.exe')
$ErrorActionPreference='Stop'
$env:PATH=(Split-Path -Parent $Compiler)+';'+$env:PATH
$firmwareRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild=Join-Path ([IO.Path]::GetTempPath()) ('mt6701-spi3-host-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
$exe=Join-Path $testBuild 'test_mt6701_spi3.exe'
& $Compiler -std=c11 -Wall -Wextra -Werror -O2 `
  ('-I'+(Join-Path $PSScriptRoot 'spi3_stubs')) `
  ('-I'+(Join-Path $firmwareRoot 'Modules/Encoder/Inc')) `
  ('-I'+(Join-Path $firmwareRoot 'Tests/Inc')) `
  (Join-Path $firmwareRoot 'Modules/Encoder/Src/spi3_encoder.c') `
  (Join-Path $firmwareRoot 'Tests/Src/mt6701_spi3_test.c') `
  (Join-Path $PSScriptRoot 'test_mt6701_spi3.c') -o $exe
if($LASTEXITCODE -ne 0) { throw 'MT6701 SPI3 host test compile failed' }
& $exe
if($LASTEXITCODE -ne 0) { throw 'MT6701 SPI3 host tests failed' }
