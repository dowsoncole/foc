"""Analyze frozen MCU trajectory records without accessing the motor."""
import argparse,json,math,struct
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);root=ap.parse_args().directory
summary=json.loads((root/'summary.json').read_text())
assert summary.get('position_sine_passed'), 'Hardware run did not pass'
metrics={'trials':[],'clock_source':summary['samples']['before']['clock_source']}
for trial in summary['trials']:
 amp=trial['amplitude_deg'];name=f'{amp:g}'
 dirs=list((root/'probe').glob(f'amp_{name}_*'));last=max(dirs,key=lambda p:int(p.name.rsplit('_',1)[1]))
 d=(last/'trajectory.bin').read_bytes();state,fault=struct.unpack_from('<2I',d,4)
 center,elapsed,peak,squared=struct.unpack_from('<4f',d,16);n,count=struct.unpack_from('<2I',d,32)
 assert state==2 and fault==0 and n>=7990 and count>=2
 raw=(root/f'trace_{name}.bin').read_bytes();rows=[struct.unpack_from('<4f',raw,i*16) for i in range(count)]
 interval=struct.unpack_from('<I',d,48)[0]*.001 if len(d)>=52 else .4
 interval=interval or .5
 for i,(t,target,actual,rpm) in enumerate(rows):
  assert all(math.isfinite(x) for x in (t,target,actual,rpm))
  assert abs(t-i*interval)<.015 and abs(target-actual)<=peak+.001
  if t<10 or t>=30:
   u=(t if t<10 else t-30)/10;q=u**3*(10+u*(-15+6*u));offset=amp*(-q if t<10 else q-1)
  else:offset=-amp*math.cos(2*math.pi*.1*(t-10))
  assert abs(target-center-offset)<.002, 'Trace corrupt or incorrect trajectory'
 result=dict(trial,rms_error_deg=math.sqrt(squared/n),center_deg=center,samples=n,trace_count=count)
 metrics['trials'].append(result)
 if amp==90:
  t,target,actual,rpm=zip(*rows)
  fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,gridspec_kw={'height_ratios':[2,1]})
  axes[0].plot(t,target,label='Target',color='#2563eb',lw=2)
  axes[0].plot(t,actual,label='MPT feedback',color='#ea580c',ls='--',lw=1.5)
  axes[0].set_ylabel('Output angle (deg)');axes[0].legend(loc='upper right')
  axes[0].set_title('Output position: +/-90 deg, 0.1 Hz, two cycles')
  axes[1].plot(t,[a-b for a,b in zip(target,actual)],color='#0f766e')
  axes[1].set_ylabel('Target - feedback (deg)');axes[1].set_xlabel('Time (s)')
  for ax in axes:
   ax.grid(alpha=.25);ax.axvspan(10,30,color='#2563eb',alpha=.06)
  fig.text(.5,.01,f'{n/elapsed:.0f} Hz error statistics: peak {peak:.3f} deg / RMS {math.sqrt(squared/n):.3f} deg. Plot decimated to {1/interval:g} Hz.',ha='center',fontsize=9)
  fig.tight_layout(rect=(0,.035,1,1));fig.savefig(root/'position_90deg_01hz.png',dpi=180);plt.close(fig)
first=(root/'probe/position_hold/mpt.bin').read_bytes();last=(root/'probe/stopped/mpt.bin').read_bytes()
metrics['mpt_read_hz']=(struct.unpack_from('<I',last,12)[0]-struct.unpack_from('<I',first,12)[0])*1000/(struct.unpack_from('<I',last,44)[0]-struct.unpack_from('<I',first,44)[0])
metrics['mpt_errors']=summary['samples']['stopped']['mpt_errors']
metrics['final_moe']=summary['samples']['stopped']['moe']
(root/'metrics.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
