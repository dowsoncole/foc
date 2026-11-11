param(
  [Parameter(Mandatory=$true)][string]$Tag,
  [Parameter(Mandatory=$true)][uint32]$PausedFlagAddress,
  [Parameter(Mandatory=$true)][uint32]$BusyFlagAddress,
  [Parameter(Mandatory=$true)][ValidateRange(0,4095)][int]$ExpectedObservedZero,
  [uint32]$RunAddress=0x20000198,
  [string]$ProbePath=$env:SERVO_PYOCD
)
# Narrow repair: restore ONLY volatile ZERO_POS to the previously confirmed 0.
# Never emits AUTO_ZERO (0x5) or PROGRAM_EEPROM (0xC). Caller must verify the
# complete image/ABI, inhibit calibration and pause acquisition while OFF.
# Evidence and exact old ZERO_POS are required. Leaves acquisition paused.
$ErrorActionPreference='Stop'
if($Tag -notmatch '^[A-Za-z0-9_-]+$' -or $BusyFlagAddress -ne $PausedFlagAddress+1){throw 'Invalid arguments'}
$resultRoot=Join-Path $PSScriptRoot '../results'
$log=Join-Path $resultRoot ($Tag+'_write.log')
if(Test-Path -LiteralPath $log){throw 'Refusing to overwrite evidence'}
$probe=if($ProbePath){$ProbePath}else{'C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.8.0-win32-x64/tools/pyocd/pyocd.exe'}
if(-not (Test-Path -LiteralPath $probe -PathType Leaf)){throw 'pyOCD not found; set SERVO_PYOCD or -ProbePath to the installed tool'}
$reader=Join-Path $PSScriptRoot 'read_encoder_gpio_off.ps1'
function Read-Byte([int]$reg,[string]$suffix){
  $readTag=$Tag+'_'+$suffix
  & $reader -Tag $readTag -PausedFlagAddress $PausedFlagAddress -BusyFlagAddress $BusyFlagAddress -RunAddress $RunAddress -Register $reg -ProbePath $probe | Out-Null
  $record=Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $resultRoot ($readTag+'.json')) | ConvertFrom-Json
  if($record.register -ne $reg){throw 'Wrong register evidence'}
  return [int]$record.value
}
function Invoke-Target([string]$command){
  $raw=@(& $probe commander --no-config --pack 'C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0' -t stm32g431vbtx -u E1007200D0D2139393740544 -M attach -W -f 500000 -c $command)
  $raw | Add-Content -LiteralPath $log
  if($LASTEXITCODE -ne 0 -or ($raw -join "`n") -match 'Error|No connected|No available'){throw 'Target access failed'}
  return $raw
}
function Word-At($raw,[uint32]$address){
  $pattern=('^{0:x8}:\s+([0-9a-f]{{8}})' -f $address)
  $rows=@($raw | Where-Object {$_ -match $pattern})
  if($rows.Count -ne 1){throw 'Incomplete guard read'}
  [void]($rows[0] -match $pattern)
  return [Convert]::ToUInt32($matches[1],16)
}
$high=Read-Byte 9 'before_h'
$low=Read-Byte 10 'before_l'
$observed=($high -shl 4) -bor ($low -shr 4)
if($observed -ne $ExpectedObservedZero){throw 'Sensor zero differs from reviewed evidence; no write'}
foreach($item in @(@(9,0),@(10,($low -band 15)))){
  $reg=[int]$item[0]; $value=[int]$item[1]
  if($reg -notin @(9,10) -or ($reg -eq 9 -and $value -ne 0) -or
     ($reg -eq 10 -and $value -ne ($low -band 15))){throw 'Refused: write outside exact ZERO_POS repair'}
  if(($reg -eq 9 -and $high -eq $value) -or ($reg -eq 10 -and $low -eq $value)){continue}
  $guard=Invoke-Target ('read32 0x40012c44 4; read32 0x{0:x8} 4; read8 0x{1:x8} 2; read32 0x48000000 4; read32 0x40013000 4; status' -f $RunAddress,$PausedFlagAddress)
  if(((Word-At $guard 0x40012c44) -band 0x8000) -or (Word-At $guard $RunAddress)){throw 'Refused: motor active'}
  $pattern=('^{0:x8}:\s+00\s+00\s' -f $PausedFlagAddress)
  if(-not ($guard | Where-Object {$_ -match $pattern})){throw 'Refused: acquisition not paused/idle'}
  $mode=Word-At $guard 0x48000000
  $spiCr1=Word-At $guard 0x40013000
  $gpioMode=($mode -band [uint32]4294902783) -bor 0x4400
  $commands=[Collections.Generic.List[string]]::new()
  $commands.Add(('write32 0x40013000 0x{0:x}; write32 0x48000018 0x30; write32 0x48000000 0x{1:x8}; write32 0x48000018 0x100000' -f ($spiCr1 -band 0xffbf),$gpioMode))
  # Exactly 0x6 write, address 0x009/0x00A, only ZERO_POS fields change.
  foreach($byte in @(0x60,$reg,$value)){
    for($bit=7;$bit -ge 0;$bit--){
      $fall=if($byte -band (1 -shl $bit)){0x00200080}else{0x00a00000}
      $commands.Add(('write32 0x48000018 0x{0:x}; write32 0x48000018 0x20; read32 0x48000010 4' -f $fall))
    }
  }
  $commands.Add('write32 0x48000018 0x10; status')
  try{$sample=Invoke-Target ($commands -join '; ')}
  finally{$null=Invoke-Target ('write32 0x48000018 0x30; write32 0x48000000 0x{0:x8}; write32 0x40013000 0x{1:x}; read32 0x40012c44 4; status' -f $mode,$spiCr1)}
  $pins=@($sample | Where-Object {$_ -match '^48000010:\s+([0-9a-f]{8})'})
  if($pins.Count -ne 24){throw 'Incomplete write evidence'}
  # Rev1.3 p22 Fig20: MISO stays Hi-Z for ordinary 0x6 writes. The 0x55
  # acknowledgement belongs ONLY to AUTO_ZERO / PROGRAM_EEPROM, neither used
  # here. Verify the actual register instead; no failed-write retry loop.
  $readback=Read-Byte $reg ('verify_'+$reg)
  if($readback -ne $value){throw 'Register readback failed; output remains OFF, no retry'}
}
$afterH=Read-Byte 9 'after_h'
$afterL=Read-Byte 10 'after_l'
if($afterH -ne 0 -or $afterL -ne ($low -band 15)){throw 'Volatile zero restoration readback failed'}
Write-Output ('PASS volatile ZERO_POS restored {0:X3} -> 000; other low-byte bits preserved; EEPROM untouched; output OFF; acquisition remains paused' -f $observed)
