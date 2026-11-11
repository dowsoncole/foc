"""Current task: read exact flash identity, then ordinary verified-RAM stop."""
from pathlib import Path
import hashlib,json,sys
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from measure_position_response import PositionTarget
from verify_encoder_artifacts import load_image

root=Path('software/control_analysis/results/customer_position_live_20261029')
root.mkdir(exist_ok=False)
candidates=[Path('software/control_analysis/results/position_tuning_20261029/flash_r17/firmware.axf'),
            Path('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf')]
expected=[]
for p in candidates:
    image=load_image(p)
    expected.append(bytes(image[a] for a in range(min(image),max(image)+1)))
t=PositionTarget(candidates[-1].resolve(),root,'150710982')
maximum=max(map(len,expected));parts=[]
for i in range(0,maximum,4096):
    path=root/f'flash_{i}.bin'
    t.run([t.dump(0x08000000+i,min(4096,maximum-i),path)],f'identity_{i}')
    parts.append(path.read_bytes())
data=b''.join(parts);(root/'initial_flash.bin').write_bytes(data)
matches=[i for i,e in enumerate(expected) if data[:len(e)]==e]
if not matches:raise RuntimeError('No exact firmware match; no RAM writes issued')
image=candidates[matches[0]]
identity=dict(image=str(image),verified_bytes=len(expected[matches[0]]),
              axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
(root/'identity.json').write_text(json.dumps(identity,indent=2))
print('MATCH',identity,flush=True)
t=PositionTarget(image.resolve(),root,'150710982')
_,before=t.snapshot('before_stop')
print('BEFORE',json.dumps(before),flush=True)
t.run([t.w(t.address['user'],0),t.w(t.address['position'],2),
       t.w(t.address['speed_run'],0),'sleep 200'],'verified_stop')
_,after=t.snapshot('after_stop')
print('AFTER',json.dumps(after),flush=True)
if after['moe']:raise RuntimeError('PWM is still ON; do not flash')
print('PWM OFF verified',flush=True)
