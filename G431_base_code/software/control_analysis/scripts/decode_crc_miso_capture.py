import json, sys
from pathlib import Path
import numpy as np
p = Path(sys.argv[1])
m = json.loads((p/'waveforms.json').read_text())
a = np.load(p/'waveforms.npz')
v = {}
for ch in [1,2,3]:
    d=m['channels'][str(ch)]
    v[ch]=(a[f'ch{ch}'].astype(float)-d['YOFF'])*d['YMULT']+d['YZERO']
dt=m['channels']['1']['XINCR']; zero=m['channels']['1']['XZERO']
def rising(x): return np.flatnonzero((x[:-1]<1.65)&(x[1:]>=1.65))+1
tr=rising(v[2]); edges=rising(v[1]); edges=edges[np.r_[True,np.diff(edges)*dt>0.3e-6]]
groups=np.split(edges,np.flatnonzero(np.diff(edges)*dt>5e-6)+1)
trigger=int(tr[np.argmin(abs(tr*dt+zero))]) if len(tr) else None
records=[]
for g in groups:
    if not len(g) or trigger is None or abs((g[-1]-trigger)*dt)>0.0005: continue
    bits=(v[3][g]>=1.65).astype(np.uint8)
    values=np.packbits(bits[:len(bits)//8*8]).tolist()
    crc=0
    for b in values[2:5]:
        crc ^= b
        for _ in range(8): crc=((crc<<1)^7)&255 if crc&128 else (crc<<1)&255
    records.append({'start_us':(int(g[0])-trigger)*dt*1e6,'end_us':(int(g[-1])-trigger)*dt*1e6,'clocks':len(g),'bytes':' '.join(f'{b:02X}' for b in values),'computed_crc':crc,'crc_match':len(values)==6 and crc==values[5]})
out={'pd2_edges_s':(tr*dt+zero).tolist(), 'single_crc_latch_edge':len(tr)==1,
     'note':'CH3 physical identity must be verified; a calculated match alone does not identify an encoder response.', 'frames':records}
(p/'decoded.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
