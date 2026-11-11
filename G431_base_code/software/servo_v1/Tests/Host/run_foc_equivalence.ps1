param(
    [Parameter(Mandatory=$true)][string]$BaselinePath,
    [Parameter(Mandatory=$true)][string]$GccPath
)
$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$baseline = (Resolve-Path -LiteralPath $BaselinePath).Path
$testBuild = Join-Path ([IO.Path]::GetTempPath()) ('foc-equivalence-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuild | Out-Null
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$options = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', '-ffp-contract=off',
    ('-I' + (Join-Path $firmwareRoot 'FOC/Inc')))
$symbols = @('FOC_ModulationCompute', 'FOC_AlgorithmInit', 'FOC_AlgorithmReset',
    'FOC_AlgorithmStep', 'FOC_InputIsValid', 'FOC_TransformCurrentDQ',
    'FOC_AlgorithmVoltageStep', 'FOC_AlgorithmVoltageCommandStep',
    'FOC_AlgorithmVoltageFeedbackStep', 'FOC_PI_IsValid', 'FOC_PI_TryStep',
    'FOC_PI_Step', 'FOC_PI_Reset')
$rename = @($symbols | ForEach-Object { '-D' + $_ + '=Before_' + $_ })
foreach ($name in @('foc_algorithm', 'foc_modulation', 'foc_pi')) {
    & $GccPath @options @rename -c (Join-Path $baseline ($name + '.c')) -o (Join-Path $testBuild ($name + '.o'))
    if ($LASTEXITCODE -ne 0) { throw 'Baseline compilation failed' }
}
$sources = @('foc_algorithm.c', 'foc_modulation.c', 'foc_pi.c') |
    ForEach-Object { Join-Path $firmwareRoot ('FOC/Src/' + $_) }
& $GccPath @options @sources (Join-Path $PSScriptRoot 'test_foc_equivalence.c') `
    (Join-Path $testBuild 'foc_algorithm.o') (Join-Path $testBuild 'foc_modulation.o') `
    (Join-Path $testBuild 'foc_pi.o') `
    -o (Join-Path $testBuild 'test_equivalence.exe') -lm
if ($LASTEXITCODE -ne 0) { throw 'Equivalence test compilation failed' }
& (Join-Path $testBuild 'test_equivalence.exe')
if ($LASTEXITCODE -ne 0) { throw 'Baseline equivalence failed' }
