param(
  [Parameter(Mandatory=$true)][string]$Tag,
  [uint32]$RunAddress=0x20000198,
  [int]$Samples=5
)
# READ ONLY; never attaches GDB or halts/resets the motor.
# Default address must be checked against the current continuous-speed AXF.
# For a different build resolve &g_speed_run while OFF and pass -RunAddress.
$ErrorActionPreference='Stop'
if($Tag -notmatch '^[A-Za-z0-9_-]+$' -or $Samples -lt 1 -or $Samples -gt 30) { throw 'Invalid arguments' }
$resultDir=Join-Path $PSScriptRoot '../results'
$log=Join-Path $resultDir ($Tag+'.log')
$json=Join-Path $resultDir ($Tag+'.json')
if((Test-Path $log) -or (Test-Path $json)) { throw 'Refusing to overwrite a capture' }
$probe='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.8.0-win32-x64/tools/pyocd/pyocd.exe'
$rows=@()
for($sample=0;$sample -lt $Samples;$sample++) {
  $command=('read32 0x{0:x8} 60; read32 0x40012c44 4; status' -f $RunAddress)
  $raw=& $probe commander --no-config --pack 'C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0' -t stm32g431vbtx -u E1007200D0D2139393740544 -M attach -W -f 500000 -c $command
  $raw | Add-Content -LiteralPath $log
  $words=[Collections.Generic.List[uint32]]::new()
  foreach($line in $raw) {
    if($line -match '^([0-9a-f]{8}):\s+([0-9a-f ]+)\s+\|') {
      $lineAddress=[Convert]::ToUInt32($matches[1],16)
      if($lineAddress -ge $RunAddress -and $lineAddress -lt $RunAddress+60) {
        foreach($word in ($matches[2].Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word,16)) }
      }
    }
  }
  if($words.Count -ne 15) { throw 'Incomplete telemetry; no valid measurement' }
  $floats=@($words | ForEach-Object { [BitConverter]::ToSingle([BitConverter]::GetBytes($_),0) })
  $row=[pscustomobject]@{time=(Get-Date -Format o); enable=$words[0];target_rpm=$floats[1];applied_rpm=$floats[2];actual_rpm=$floats[3];turns=$floats[4];elapsed_s=$floats[5];iq_ref_a=$floats[6];iq_a=$floats[7];id_a=$floats[8];uq_pu=$floats[9];state=$words[10];fault=$words[11];phase_peak_a=$floats[12];feedback_status_raw=$words[13];start_block_mask=$words[14]}
  $rows+=$row
  $row | Format-List
  # Sparse non-halting Watch telemetry, NOT synchronized high-rate test data.
}
$rows | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $json
