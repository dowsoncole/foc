param(
  [Parameter(Mandatory=$true)][string]$Path,
  [uint32]$CpuClockHz=168000000
)
# OFFLINE ONLY. Decode an exact sizeof(encoder_fault_capture_t)=312-byte dump.
# No probe connection, target write, controller reset or hardware operation.
$ErrorActionPreference='Stop'
$captureBytes=[IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Path).Path)
if($captureBytes.Length -ne 312 -or !$CpuClockHz) {throw 'Expected a 312-byte encoder capture and nonzero CPU clock'}
function Read-U32([int]$offset) { return [BitConverter]::ToUInt32($captureBytes,$offset) }
$captureCount=Read-U32 0
$frameNames=@('first','last','recent0','recent1','recent2','recent3','recent4','recent5','recent6','recent7','last_alarm')
$frames=@()
for($frameIndex=0;$frameIndex -lt $frameNames.Count;$frameIndex++) {
  $offset=4+28*$frameIndex
  $cycle=Read-U32 $offset
  $previousCycle=Read-U32 ($offset+16)
  $received=@($captureBytes[($offset+20)..($offset+25)])
  $data24=([uint32]$received[2] -shl 16) -bor ([uint32]$received[3] -shl 8) -bor [uint32]$received[4]
  $crc=0
  foreach($dataByte in $received[2..4]) {
    $crc=$crc -bxor $dataByte
    for($bit=0;$bit -lt 8;$bit++) {
      $crc=if($crc -band 0x80) { (($crc -shl 1) -bxor 7) -band 255 } else { ($crc -shl 1) -band 255 }
    }
  }
  $elapsed=([uint64]$cycle+[uint64]4294967296-[uint64]$previousCycle) % [uint64]4294967296
  # A test may reset only count. Old nonzero ring slots then remain in RAM;
  # never present those slots as errors from the new measurement window.
  $inCountWindow=$null
  if($frameIndex -lt 2) { $inCountWindow=($captureCount -gt 0) }
  elseif($frameIndex -lt 10) {
    $inCountWindow=($captureCount -ge 8 -or ($frameIndex-2) -lt $captureCount)
  }
  $frames+=[pscustomobject]@{
    slot=$frameNames[$frameIndex]
    populated=([bool]((Read-U32 ($offset+4)) -ne 0))
    in_count_window=$inCountWindow
    cycle=$cycle
    reason=(Read-U32 ($offset+4))
    previous_angle=(Read-U32 ($offset+8))
    spi_status=(Read-U32 ($offset+12))
    previous_cycle=$previousCycle
    capture_since_previous_start_us=([double]$elapsed*1000000/$CpuClockHz)
    rx_hex=(($received | ForEach-Object {'{0:X2}' -f $_}) -join ' ')
    angle_raw=($data24 -shr 3)
    status_raw=($received[4] -band 7)
    received_crc_hex=('{0:X2}' -f $received[5])
    computed_crc_hex=('{0:X2}' -f $crc)
    crc_xor_hex=('{0:X2}' -f ($received[5] -bxor $crc))
    crc_ok=($received[5] -eq $crc)
    firmware_computed_crc_hex=('{0:X2}' -f $captureBytes[$offset+26])
  }
}
[pscustomobject]@{
  source=(Resolve-Path -LiteralPath $Path).Path
  count=$captureCount
  cpu_clock_hz=$CpuClockHz
  note='Ring slots are not chronological. populated only means nonzero reason; use in_count_window to exclude stale slots after count-only reset (assuming no uint32 count wrap). last_alarm has an independent lifetime, so its window is unknown/null. CRC XOR is a mismatch signature, not proof of which wire bits were corrupted. RAM dump is not an electrical waveform.'
  frames=$frames
} | ConvertTo-Json -Depth 5
