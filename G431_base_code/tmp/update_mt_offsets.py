from pathlib import Path
p=Path('software/servo_v1/Modules/Current_Sampling/Inc/current_sampling.h');s=p.read_text(encoding='utf-8');a=s.index('/* Verified OFF offsets');b=s.index('\ntypedef enum',a);s=s[:a]+'''/* Replacement PCB, MT6701 SSI active, MOE=0, 10 kHz, 2026-10-26.
   Two independent 1024-sample windows: U=1892.974/1892.964,
   V=1891.796/1891.876 ADC counts. Evidence:
   control_analysis/results/mt6701_speed_scan_20261026/off_adc_noise*.bin.
   Analog-chain or PCB changes require a new OFF calibration. */
#define CURRENT_SAMPLING_BOARD_OFFSET_U 1893U
#define CURRENT_SAMPLING_BOARD_OFFSET_V 1892U
'''+s[b:];p.write_text(s,encoding='utf-8')
