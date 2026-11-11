import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
p=Path('software/control_analysis/results/encoder_so_twoends_20261026_100853');result={};fig,axs=plt.subplots(3,3,figsize=(15,8),sharex=True)
for col,label in enumerate(['off','on','off_after']):
 m=json.loads((p/(label+'.json')).read_text());a=np.load(p/(label+'.npz'));v={}
 for c in [1,2,3]:
  d=m['channels'][str(c)];v[c]=(a[f'ch{c}'].astype(float)-d['YOFF'])*d['YMULT']+d['YZERO']
 dt=m['channels']['1']['XINCR'];logic={c:v[c]>1.65 for c in [1,2,3]};falls=np.flatnonzero(logic[1][:-1]&~logic[1][1:])+1;falls=falls[np.r_[True,np.diff(falls)*dt>50e-6]];falls=falls[(falls>int(3e-6/dt))&(falls<len(v[1])-int(44e-6/dt))]
 mask=~logic[1];stable=mask.copy();guard=max(1,round(.1e-6/dt))
 for c in [2,3]:
  es=np.flatnonzero(logic[c][1:]!=logic[c][:-1])+1
  for e in es:stable[max(0,e-guard):min(len(stable),e+guard+1)]=False
 rows=[]
 for f in falls:
  sl=slice(f+int(22e-6/dt),f+int(36e-6/dt));rows.append({'start_s':float(f*dt+m['channels']['1']['XZERO']),'chip_tail_high_fraction':float(logic[2][sl].mean()),'mcu_tail_high_fraction':float(logic[3][sl].mean())})
 result[label]={'cs_frames':len(falls),'stable_logic_match_fraction':float(np.mean(logic[2][stable]==logic[3][stable])),'median_voltage_difference_v':float(np.median((v[2]-v[3])[stable])),'chip_tail_high_median':float(np.median([r['chip_tail_high_fraction'] for r in rows])),'mcu_tail_high_median':float(np.median([r['mcu_tail_high_fraction'] for r in rows])),'examples':rows[:2]}
 n=int(falls[len(falls)//2]);lo=n-int(3e-6/dt);hi=n+int(44e-6/dt);t=(np.arange(lo,hi)-n)*dt*1e6
 for row,c in enumerate([1,2,3]):
  axs[row,col].plot(t,v[c][lo:hi],lw=.7);axs[row,col].grid(alpha=.25);axs[row,col].set_ylabel(['CS','Chip SO','MCU SO'][row]+' (V)')
 axs[0,col].set_title(label.upper());axs[2,col].set_xlabel('Time from CS falling (us)')
fig.tight_layout();fig.savefig(p/'twoends_comparison.png',dpi=150)
s=json.loads((p/'summary.json').read_text());result['mcu']={k:[s['before'][k],s['stopped'][k]] for k in ['completed','crc_errors','transfer_errors']};result['stopped_moe']=s['stopped']['moe'];(p/'analysis.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
