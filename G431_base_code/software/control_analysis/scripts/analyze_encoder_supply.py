import json,sys,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
p=Path(sys.argv[1]);r={};fig,axs=plt.subplots(3,1,figsize=(12,7),sharex=True)
for ax,label in zip(axs,['off','on','off_after']):
 m=json.loads((p/(label+'.json')).read_text());raw=np.load(p/(label+'.npz'))['ch2'];d=m['channels']['2'];v=(raw.astype(float)-d['YOFF'])*d['YMULT']+d['YZERO'];dt=d['XINCR'];n=max(1,round(1e-6/dt));avg=v[:len(v)//n*n].reshape(-1,n).mean(axis=1)
 r[label]={'mean_v':float(v.mean()),'min_v':float(v.min()),'max_v':float(v.max()),'p01_v':float(np.percentile(v,1)),'p99_v':float(np.percentile(v,99)),'one_us_average_min_v':float(avg.min()),'one_us_average_max_v':float(avg.max()),'clip_fraction':float(np.mean((raw<=-127)|(raw>=127))),'duration_s':len(v)*dt}
 t=(np.arange(len(avg))*n*dt+d['XZERO'])*1e3;ax.plot(t,avg,lw=.6);ax.set_ylabel(label+' VDD (V)');ax.grid(alpha=.3);ax.set_ylim(min(3.2,float(avg.min())-.02),max(3.5,float(avg.max())+.02))
axs[-1].set_xlabel('Time (ms)');fig.suptitle('Encoder supply, 1 us block averages; Ud=1 V, 10 Hz');fig.tight_layout();fig.savefig(p/'supply_comparison.png',dpi=150);(p/'supply_analysis.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));s=json.loads((p/'summary.json').read_text());print('stopped_moe',s['stopped']['moe'])
