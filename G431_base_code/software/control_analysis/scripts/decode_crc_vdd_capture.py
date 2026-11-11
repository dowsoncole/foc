import json, sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]); m=json.loads((p/'waveforms.json').read_text()); a=np.load(p/'waveforms.npz')
v={}
for ch in [1,2,3]:
 d=m['channels'][str(ch)]; v[ch]=(a[f'ch{ch}'].astype(float)-d['YOFF'])*d['YMULT']+d['YZERO']
dt=m['channels']['1']['XINCR']; zero=m['channels']['1']['XZERO']
edges=np.flatnonzero((v[1][:-1]<1.65)&(v[1][1:]>=1.65))+1
edges=edges[np.r_[True,np.diff(edges)*dt>0.3e-6]]
groups=np.split(edges,np.flatnonzero(np.diff(edges)*dt>5e-6)+1)
csrise=np.array([],dtype=int)
records=[]
for g in groups:
 if len(g)!=48: continue
 values=np.packbits((v[3][g]>=1.65).astype(np.uint8)).tolist(); crc=0
 for b in values[2:5]:
  crc^=b
  for _ in range(8): crc=((crc<<1)^7)&255 if crc&128 else (crc<<1)&255
 rises=csrise[(csrise>g[0])&(csrise<g[-1]+int(5e-6/dt))]
 records.append({'start_s':float(g[0]*dt+zero),'sample':int(g[0]),'bytes':' '.join(f'{b:02X}' for b in values),'crc_match':crc==values[5], 'computed_crc':crc,'vdd_min':float(v[2][g[0]:g[-1]+1].min()),'vdd_max':float(v[2][g[0]:g[-1]+1].max()),'vdd_median':float(np.median(v[2][g[0]:g[-1]+1]))})
bad=[r for r in records if not r['crc_match']]
out={'frames':len(records),'bad_frames':len(bad),'examples_bad':bad[:10],'examples_good':[r for r in records if r['crc_match']][:3],'records':records}
(p/'decoded_vdd.json').write_text(json.dumps(out,indent=2)); print(json.dumps({k:v for k,v in out.items() if k!='records'},indent=2))
