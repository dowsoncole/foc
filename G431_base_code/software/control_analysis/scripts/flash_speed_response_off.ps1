param(
 [Parameter(Mandatory=$true)][string]$Output,
 [string]$Probe='150710982'
)
$ErrorActionPreference='Stop'
$root=(Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
Set-Location $root
$destination=[IO.Path]::GetFullPath($Output)
if(Test-Path -LiteralPath $destination) {throw 'Use a new output directory'}
New-Item -ItemType Directory -Path $destination | Out-Null
$debug='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools'
$probeTool=Join-Path $debug 'pyocd/pyocd.exe'
$gdb=Join-Path $debug 'gdb/bin/arm-none-eabi-gdb.exe'
$python='C:/Users/penghao/AppData/Local/Programs/Python/Python312/python.exe'
$pack='C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0'
$boardArgs=@('--no-config','--pack',$pack,'-t','stm32g431vbtx','-u',$Probe,'-W','-f','100000')
$ErrorActionPreference='Continue' # Windows PowerShell 5 treats native INFO stderr as an ErrorRecord.
$before=& $probeTool commander @boardArgs -M attach -c 'read32 0x40012c44 4; status' 2>&1
$beforeExit=$LASTEXITCODE
$ErrorActionPreference='Stop'
$before | Out-File -LiteralPath (Join-Path $destination 'before.log')
if($beforeExit) {throw 'Cannot read output state'}
$match=[regex]::Match(($before -join "`n"),'40012c44:\s+([0-9a-fA-F]{8})')
if(!$match.Success -or ([Convert]::ToUInt32($match.Groups[1].Value,16)-band 0x8000)) {
 throw 'REFUSED: output must already be OFF'
}
$image=Join-Path $destination 'firmware.axf'
$hex=Join-Path $destination 'firmware.hex'
Copy-Item -LiteralPath 'software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf' -Destination $image
Copy-Item -LiteralPath 'software/servo_v1/MDK-ARM/servo_v1/servo_v1.hex' -Destination $hex
$ErrorActionPreference='Continue'
& $probeTool load @boardArgs -M halt --no-reset --erase sector --format hex $hex *> (Join-Path $destination 'flash.log')
$flashExit=$LASTEXITCODE
$ErrorActionPreference='Stop'
if($flashExit) {throw 'Flash failed; leave the board stopped'}
& $python -B software/control_analysis/scripts/measure_speed_response.py (Join-Path $destination 'verification') --probe $Probe --axf $image
if($LASTEXITCODE) {throw 'Flash verification failed; do not run'}
$serverArgs=@('gdbserver')+$boardArgs+@('-M','attach','-p','3334','-T','4445','-R','5556','--persist')
$server=Start-Process -FilePath $probeTool -ArgumentList $serverArgs -WindowStyle Hidden -PassThru `
 -RedirectStandardOutput (Join-Path $destination 'server.log') `
 -RedirectStandardError (Join-Path $destination 'server.err.log')
try {
 # The read-only verification has ended; the owned GDB server is now the
 # exclusive probe user. The script checks MOE, resets to main and disables
 # both command sources before allowing initialization to execute.
 Start-Sleep -Milliseconds 1800
 $ErrorActionPreference='Continue'
 & $gdb --batch $image -x software/control_analysis/scripts/speed_response_reset_off.gdb *> (Join-Path $destination 'start_off.log')
 $startExit=$LASTEXITCODE
 $ErrorActionPreference='Stop'
 if($startExit) {throw 'OFF startup failed'}
} finally {
 Stop-Process -Id $server.Id -ErrorAction SilentlyContinue
}
& $probeTool commander @boardArgs -M attach -c 'continue; read32 0xe0001004 4; sleep 200; read32 0x40012c44 4; status' *> (Join-Path $destination 'after.log')
if($LASTEXITCODE) {throw 'Final output-state read failed'}
Get-Content -LiteralPath (Join-Path $destination 'after.log')
if(!(Select-String -LiteralPath (Join-Path $destination 'after.log') -Pattern 'Running' -Quiet)) {
 throw 'CPU did not resume; do not actuate'
}
