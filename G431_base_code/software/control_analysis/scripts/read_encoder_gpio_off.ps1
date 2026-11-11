param(
  [Parameter(Mandatory=$true)][string]$Tag,
  [Parameter(Mandatory=$true)][uint32]$PausedFlagAddress,
  [Parameter(Mandatory=$true)][uint32]$BusyFlagAddress,
  [uint32]$RunAddress=0x20000198,
  [int]$Register=-1,
  [string]$ProbePath=$env:SERVO_PYOCD
)
# OFF-ONLY diagnostic. Resolve addresses against the verified board AXF first.
# Caller must pause Encoder_OnPwmPeriodIsr via s_encoder.initialized=0 while OFF.
# No motor command, no reset, no sensor register/EEPROM write.
# Default reads A003; -Register reads ONE byte with the 0x3 read opcode.
# Restores MCU GPIO/SPI configuration, but deliberately leaves encoder paused.
$ErrorActionPreference='Stop'
if($Tag -notmatch '^[A-Za-z0-9_-]+$' -or $BusyFlagAddress -ne $PausedFlagAddress+1 -or $Register -lt -1 -or $Register -gt 4095) {throw 'Invalid arguments'}
$resultDir=Join-Path $PSScriptRoot '../results'
$log=Join-Path $resultDir ($Tag+'.log')
$json=Join-Path $resultDir ($Tag+'.json')
if((Test-Path $log) -or (Test-Path $json)) {throw 'Refusing to overwrite evidence'}
$probe=if($ProbePath){$ProbePath}else{'C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.8.0-win32-x64/tools/pyocd/pyocd.exe'}
if(-not (Test-Path -LiteralPath $probe -PathType Leaf)){throw 'pyOCD not found; set SERVO_PYOCD or -ProbePath to the installed tool'}
function Invoke-Target([string]$command) {
  $raw=@(& $probe commander --no-config --pack 'C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0' -t stm32g431vbtx -u E1007200D0D2139393740544 -M attach -W -f 500000 -c $command)
  $raw | Add-Content -LiteralPath $log
  if($LASTEXITCODE -ne 0 -or ($raw -join "`n") -match 'Error|No connected|No available') {throw 'Target access failed'}
  return $raw
}
function Word-At($raw,[uint32]$address) {
  $pattern=('^{0:x8}:\s+([0-9a-f]{{8}})' -f $address)
  $rows=@($raw | Where-Object {$_ -match $pattern})
  if($rows.Count -ne 1) {throw 'Incomplete guard read'}
  [void]($rows[0] -match $pattern)
  return [Convert]::ToUInt32($matches[1],16)
}
$guard=Invoke-Target ('read32 0x40012c44 4; read8 0x{0:x8} 2; read32 0x{1:x8} 4; read32 0x48000000 4; read32 0x40013000 4; status' -f $PausedFlagAddress,$RunAddress)
if(((Word-At $guard 0x40012c44) -band 0x8000) -or (Word-At $guard $RunAddress)) {throw 'Refused: motor output/enable active'}
$pausedPattern=('^{0:x8}:\s+00\s+00\s' -f $PausedFlagAddress)
if(-not ($guard | Where-Object {$_ -match $pausedPattern})) {throw 'Refused: encoder not paused and idle'}
$mode=Word-At $guard 0x48000000
$spiCr1=Word-At $guard 0x40013000
$gpioMode=($mode -band [uint32]4294902783) -bor 0x4400 # clear PA5/6/7 modes; output PA5/7, input PA6
$commands=[Collections.Generic.List[string]]::new()
$commands.Add(('write32 0x40013000 0x{0:x}; write32 0x48000018 0x30; write32 0x48000000 0x{1:x8}; write32 0x48000018 0x100000' -f ($spiCr1 -band 0xffbf),$gpioMode))
$txBytes=if($Register -lt 0){@(0xa0,0x03,0,0,0,0)}else{@((0x30 -bor ($Register -shr 8)),($Register -band 0xff),0)}
foreach($byte in $txBytes) {
  for($bit=7;$bit -ge 0;$bit--) {
    $fall=if($byte -band (1 -shl $bit)) {0x00200080} else {0x00a00000}
    $commands.Add(('write32 0x48000018 0x{0:x}; write32 0x48000018 0x20; read32 0x48000010 4' -f $fall))
  }
}
$commands.Add('write32 0x48000018 0x10; status')
try { $sample=Invoke-Target ($commands -join '; ') }
finally {
  $null=Invoke-Target ('write32 0x48000018 0x30; write32 0x48000000 0x{0:x8}; write32 0x40013000 0x{1:x}; read32 0x40012c44 4; status' -f $mode,$spiCr1)
}
$pinRows=@($sample | Where-Object {$_ -match '^48000010:\s+([0-9a-f]{8})'})
if($pinRows.Count -ne $txBytes.Count*8) {throw 'Incomplete bit capture'}
$rx=@()
for($b=0;$b -lt $txBytes.Count;$b++) {
  $value=0
  for($bit=0;$bit -lt 8;$bit++) {
    [void]($pinRows[$b*8+$bit] -match '^48000010:\s+([0-9a-f]{8})')
    $value=($value -shl 1) -bor [int](([Convert]::ToUInt32($matches[1],16) -band 0x40) -ne 0)
  }
  $rx+=$value
}
$row=[ordered]@{time=(Get-Date -Format o);tx_hex=(($txBytes|ForEach-Object {'{0:X2}' -f $_}) -join ' ');rx_hex=(($rx|ForEach-Object {'{0:X2}' -f $_}) -join ' ');note='GPIO via SWD, not real-time SPI; encoder left paused; motor stays OFF'}
if($Register -ge 0){
  $row.register=$Register
  $row.value=$rx[2]
  $row.note+='; single register reads have no CRC; repeat to check consistency'
}else{
  $crc=0
  foreach($b in $rx[2..4]) {
    $crc=$crc -bxor $b
    for($bit=0;$bit -lt 8;$bit++) {$crc=if($crc -band 0x80){(($crc -shl 1) -bxor 7) -band 0xff}else{($crc -shl 1) -band 0xff}}
  }
  $row.computed_crc_hex=('{0:X2}' -f $crc)
  $row.crc_ok=($crc -eq $rx[5])
  $row.status_raw=($rx[4] -band 7)
  $row.angle_raw=((($rx[2] -shl 16) -bor ($rx[3] -shl 8) -bor $rx[4]) -shr 3)
}
$row | ConvertTo-Json | Set-Content -LiteralPath $json
$row | Format-List
