$ErrorActionPreference='Stop'
$base='C:/Users/penghao/Desktop/Servo_9948/G431_base_code'
$out=Join-Path $base 'software/control_analysis/results/customer_position_live_20261029/flash_customer'
$image=Join-Path $out 'firmware.axf'
$verified=Get-Content -Raw -LiteralPath (Join-Path $out 'recheck/result.json') | ConvertFrom-Json
if((Get-FileHash -LiteralPath $image -Algorithm SHA256).Hash.ToLowerInvariant() -ne $verified.axf_sha256) {throw 'Image identity changed'}
$debug='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools'
$probeTool=Join-Path $debug 'pyocd/pyocd.exe'
$gdb=Join-Path $debug 'gdb/bin/arm-none-eabi-gdb.exe'
$board=@('--no-config','--pack','C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0','-t','stm32g431vbtx','-u','150710982','-W','-f','50000')
$serverArgs=@('gdbserver')+$board+@('-M','attach','-p','3334','-T','4445','-R','5556','--persist')
$server=Start-Process -FilePath $probeTool -ArgumentList $serverArgs -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $out 'start_server.log') -RedirectStandardError (Join-Path $out 'start_server.err.log')
try {
  Start-Sleep -Milliseconds 1800
  & $gdb --batch $image -x (Join-Path $base 'software/control_analysis/scripts/speed_response_reset_off.gdb') *> (Join-Path $out 'start_off.log')
  if($LASTEXITCODE) {throw 'GDB OFF initialization refused or failed'}
} finally {
  Stop-Process -Id $server.Id -ErrorAction SilentlyContinue
}
& $probeTool commander @board -M attach -c 'continue; read32 0xe0001004 4; sleep 200; read32 0x40012c44 4; status' *> (Join-Path $out 'after_start.log')
if($LASTEXITCODE) {throw 'Startup readback failed'}
Get-Content -LiteralPath (Join-Path $out 'start_off.log')
Get-Content -LiteralPath (Join-Path $out 'after_start.log')
