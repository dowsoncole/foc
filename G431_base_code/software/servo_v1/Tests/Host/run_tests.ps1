param([string]$VcVarsPath = '', [string]$ZigPath = '')
$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testBuild = Join-Path ([System.IO.Path]::GetTempPath()) ('foc-ovm-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
$testSources = @(
    (Join-Path $firmwareRoot 'FOC/Src/foc_modulation.c'),
    (Join-Path $firmwareRoot 'FOC/Src/foc_algorithm.c'),
    (Join-Path $firmwareRoot 'FOC/Src/foc_interface.c'),
    (Join-Path $firmwareRoot 'Servo/Src/servo_manager.c'),
    (Join-Path $firmwareRoot 'Modules/Three_Phase_Driver/Src/pwm_sampling.c'),
    (Join-Path $firmwareRoot 'Tests/Src/waveform_generator.c'),
    (Join-Path $PSScriptRoot 'test_pwm_sampling.c'),
    (Join-Path $PSScriptRoot 'test_waveform_generator.c'),
    (Join-Path $PSScriptRoot 'test_foc_modulation.c')
)
$compilerArgs = @('/nologo', '/std:c11', '/W4', '/WX', '/O2', '/utf-8',
    ('/I' + (Join-Path $firmwareRoot 'FOC/Inc')),
    ('/I' + (Join-Path $firmwareRoot 'Servo/Inc')),
    ('/I' + (Join-Path $firmwareRoot 'Modules/Three_Phase_Driver/Inc')),
    ('/I' + (Join-Path $firmwareRoot 'Tests/Inc')))
Push-Location $testBuild
try {
    if ($VcVarsPath) {
        if (-not (Test-Path -LiteralPath $VcVarsPath)) { throw 'vcvars64.bat not found' }
        $compilerEnvironment = & cmd.exe /d /c ('call "' + $VcVarsPath + '" >nul && set')
        if ($LASTEXITCODE -ne 0) { throw 'MSVC environment setup failed' }
        foreach ($entry in $compilerEnvironment) {
            if ($entry -match '^([^=]+)=(.*)$') {
                [Environment]::SetEnvironmentVariable($matches[1], $matches[2], 'Process')
            }
        }
    }
    if ($ZigPath) {
        & $ZigPath cc -std=c99 -Wall -Wextra -Werror -O2 `
            ('-I' + (Join-Path $firmwareRoot 'FOC/Inc')) `
            ('-I' + (Join-Path $firmwareRoot 'Servo/Inc')) `
            ('-I' + (Join-Path $firmwareRoot 'Modules/Three_Phase_Driver/Inc')) `
            ('-I' + (Join-Path $firmwareRoot 'Tests/Inc')) `
            @testSources -o test_foc.exe -lm
    } else {
        & cl.exe @compilerArgs @testSources /Fe:test_foc.exe
    }
    if ($LASTEXITCODE -ne 0) { throw 'Host C compilation failed' }
    & (Join-Path $testBuild 'test_foc.exe')
    if ($LASTEXITCODE -ne 0) { throw 'FOC host tests failed' }
} finally {
    Pop-Location
}
