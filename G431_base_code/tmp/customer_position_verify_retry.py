from pathlib import Path
import sys,json,hashlib
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from measure_position_response import PositionTarget
from verify_encoder_artifacts import load_image
root=Path('software/control_analysis/results/customer_position_live_20261029')
image=Path('software/control_analysis/results/position_tuning_20261029/flash_r17/firmware.axf')
im=load_image(image);expected=bytes(im[a] for a in range(min(im),max(im)+1))
data=bytearray((root/'initial_flash.bin').read_bytes()[:len(expected)])
words=sorted({i//4*4 for i,(x,y) in enumerate(zip(data,expected)) if x!=y})
if len(words)>16:raise RuntimeError('Not a sparse read discrepancy; no RAM writes')
t=PositionTarget(image.resolve(),root,'150710982')
corrections=[]
for attempt in range(2):
    commands=[t.dump(0x08000000+i,4,root/f'retry_{attempt}_{i}.bin') for i in words]
    t.run(commands,f'identity_retry_{attempt}')
for i in words:
    a=(root/f'retry_0_{i}.bin').read_bytes();b=(root/f'retry_1_{i}.bin').read_bytes()
    if a!=b or a!=expected[i:i+4]:raise RuntimeError(f'Unresolved Flash word at {i}; no RAM writes')
    corrections.append(dict(offset=i,initial=data[i:i+4].hex(),retry=a.hex()))
    data[i:i+4]=a
assert data==expected
(root/'verified_initial_flash.bin').write_bytes(data)
(root/'identity.json').write_text(json.dumps(dict(image=str(image),verified_bytes=len(data),
    axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),read_retry_corrections=corrections),indent=2))
print('Exact R17 identity verified, bytes',len(data),'retried words',len(words),flush=True)
_,before=t.snapshot('before_stop');print('BEFORE',json.dumps(before),flush=True)
t.run([t.w(t.address['user'],0),t.w(t.address['position'],2),
       t.w(t.address['speed_run'],0),'sleep 200'],'verified_stop')
_,after=t.snapshot('after_stop');print('AFTER',json.dumps(after),flush=True)
if after['moe']:raise RuntimeError('PWM still enabled; do not flash')
print('PWM OFF verified',flush=True)
