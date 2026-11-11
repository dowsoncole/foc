"""Actuating MT6701 commissioning: explicit --run required.
Verifies Flash, checks direction at 1 V/10 Hz, then fixed 1.5 V/3 s zeroing.
Run from repository root, with the boot voltage output already OFF.
"""
import sys, time,json,struct,shutil,math
from pathlib import Path
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from mt6701_probe import Probe,base
if '--run' not in sys.argv: raise SystemExit('Use --run to perform the bounded direction/alignment test')
root=Path('software/control_analysis/results')/time.strftime('mt6701_closed_setup_%Y%m%d_%H%M%S');root.mkdir()
image=root/'firmware.axf';shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
p=Probe(image,root/'trial');r={'verification':p.verify_flash()};a=p.addresses
try:
 _,before=p.snapshot('off');r['off']=before
 assert not before['moe'] and before['encoder_flags']==0 and before['crc_errors']==0
 # Existing 1V / 10Hz open-loop recipe: measure signed feedback before closing loops.
 p.command([f'write8 0x{a["runtime"]+1:x} 1','sleep 3000'])
 _,running=p.snapshot('direction');r['direction']=running
 p.command([f'write8 0x{a["runtime"]+1:x} 0','sleep 1000'])
 assert running['moe'] and running['crc_errors']==0 and running['step_errors']==0
 # 10 electrical Hz / 2 pole pairs = +31.4 mechanical rad/s.
 assert 20 < running['mechanical_speed_rad_s'] < 40, 'Encoder direction/pole count differs from positive field'
 p.command([f'write8 0x{a["mode"]:x} 1','sleep 200'])
 _,mode=p.snapshot('mode');r['mode']=mode;base.require_ready(mode,calibration_required=False)
 # Save normal speed gains before fixed recipe overwrites requested settings.
 requested=(root/'trial/mode/test.bin').read_bytes()[16:64]
 p.command([f'write32 0x{a["test"]+16:x} 10',f'write32 0x{a["test"]:x} 1','sleep 4000'])
 _,aligned=p.snapshot('aligned',full=True);r['aligned']=aligned
 assert aligned['speed_test_state']==3 and aligned['calibration_ok']==1 and not aligned['moe'] and not aligned['speed_test_fault'],'Alignment failed'
 assert aligned['crc_errors']==0 and aligned['step_errors']==0 and aligned['deadline_overruns']==0
 p.command([f'write32 0x{a["test"]+16+i:x} {base.u32(requested,i)}' for i in range(0,48,4)]+[f'write32 0x{a["test"]:x} 3'])
 print(json.dumps({k:{n:v[n] for n in ['mechanical_speed_rad_s','crc_errors','step_errors','calibration_ok','calibration_offset','speed_test_fault','moe']} for k,v in r.items() if k!='verification'},indent=2),flush=True)
except BaseException as e:r['error']=repr(e);print('ERROR',e,flush=True)
finally:
 try:
  p.command([f'write32 0x{a["run"]:x} 0',f'write8 0x{a["runtime"]+1:x} 0','sleep 100','write32 0x40012c44 0x02002054'])
  _,r['stopped']=p.snapshot('stopped',full=True)
  assert not r['stopped']['moe'] and not r['stopped']['enable']
 except BaseException as cleanup:r['cleanup_error']=repr(cleanup)
 (root/'summary.json').write_text(json.dumps(r,indent=2));print(root,flush=True)
if 'error' in r or 'cleanup_error' in r: raise SystemExit(1)
