"""Read-only verification after programming. No reset, resume or RAM writes."""
from pathlib import Path
import sys,json,hashlib
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from measure_position_response import PositionTarget
from verify_encoder_artifacts import load_image
root=Path('software/control_analysis/results/customer_position_live_20261029/flash_customer/recheck')
root.mkdir(exist_ok=False)
image=root.parent/'firmware.axf';im=load_image(image)
expected=bytes(im[a] for a in range(min(im),max(im)+1))
t=PositionTarget(image.resolve(),root,'150710982');t.swd_frequency=100000
data=bytearray();corrections=[]
for start in range(0,len(expected),16384):
    offsets=list(range(start,min(start+16384,len(expected)),4096))
    t.run([t.dump(0x08000000+i,min(4096,len(expected)-i),root/f'flash_{i}.bin') for i in offsets],f'flash_{start}')
    for i in offsets:data.extend((root/f'flash_{i}.bin').read_bytes())
    print('Read',len(data),'/',len(expected),flush=True)
words=sorted({i//4*4 for i,(a,b) in enumerate(zip(data,expected)) if a!=b})
if len(words)>64:raise RuntimeError('Flash mismatch exceeds read-retry envelope; leave OFF/halted')
t.swd_frequency=50000
for attempt in range(2):
    if words:t.run([t.dump(0x08000000+i,min(4,len(expected)-i),root/f'retry_{attempt}_{i}.bin') for i in words],f'retry_{attempt}')
for i in words:
    a=(root/f'retry_0_{i}.bin').read_bytes();b=(root/f'retry_1_{i}.bin').read_bytes()
    if a!=b or a!=expected[i:i+4]:raise RuntimeError(f'Flash word mismatch persists at {i}; leave OFF/halted')
    corrections.append(dict(offset=i,initial=data[i:i+4].hex(),reread=a.hex()));data[i:i+4]=a
if data!=expected:raise RuntimeError('Exact Flash verification failed')
(root/'verified_flash.bin').write_bytes(data)
(root/'result.json').write_text(json.dumps(dict(verified_bytes=len(data),retry_words=corrections,
    axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest()),indent=2))
print('VERIFIED new Flash',len(data),'bytes; corrected read words',len(words),flush=True)
print(t.run(['read32 0x40012c44 4; status'],'off_halted'),flush=True)
