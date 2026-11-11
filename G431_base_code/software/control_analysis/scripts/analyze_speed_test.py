"""Analyze raw binary motor commissioning records without altering source captures."""
import sys,struct,json,re
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]); log=p.with_suffix('.read.log').read_text(encoding='utf-8',errors='replace')
n=int(re.search(r'\$3 = (\d+)',log)[1])
a=np.array(list(struct.iter_unpack('<8f',p.read_bytes())))[:n]
if n<3: raise SystemExit('Insufficient samples')
t,ref,w,id_,iq,iqref,angle,uq=a.T
tail=slice(max(0,n-n//4),n)
out=dict(samples=n,duration_recorded_s=float(t[-1]),
 speed_mean_rad_s=float(np.mean(w[tail])),speed_std_rad_s=float(np.std(w[tail])),
 angle_start_rad=float(angle[0]),angle_end_rad=float(angle[-1]),
 angle_change_rad=float(np.unwrap(angle)[-1]-np.unwrap(angle)[0]),
 peak_speed_rad_s=float(np.max(np.abs(w))),iq_peak_A=float(np.max(np.abs(iq))))
kind=int(re.search(r'kind = (\d+)',log)[1])
for key in ('kp','ki','current_limit_a','bias','amplitude'):
 out[key]=float(re.search(r'\b'+key+r'\s*=\s*([\d.eE+-]+)',log)[1])
if kind==3:
 target=ref[-1]; err=ref-w
 out.update(target_rad_s=float(target),error_mean_rad_s=float(np.mean(err[tail])),
            iae_rad=float(np.trapezoid(abs(err),t)))
 if abs(target)>1e-9:
  after=t>=.2; out['overshoot_pct']=float(max(0,np.max(w[after]/target)-1)*100)
  for f,name in ((.1,'t10_s'),(.9,'t90_s')):
   hit=np.where(after & (w/target>=f))[0]; out[name]=float(t[hit[0]]-.2) if len(hit) else None
  band=max(.02*abs(target),.05); bad=np.where(after & (abs(err)>band))[0]
  out['settling_band_rad_s']=float(band)
  candidate=float(t[bad[-1]+1]-.2) if len(bad) and bad[-1]<n-1 else (0 if not len(bad) else None)
  out['last_band_entry_s']=candidate
  out['settling_min_hold_s']=.5
  out['settling_time_s']=candidate if candidate is not None and t[-1]-(candidate+.2)>=.5 else None
if kind==4:
 f=float(re.search(r'frequency_hz = ([\d.eE+-]+)',log)[1]); keep=t>=max(.2,t[-1]*.25)
 X=np.column_stack((np.sin(2*np.pi*f*t[keep]),np.cos(2*np.pi*f*t[keep]),np.ones(sum(keep))))
 cr=np.linalg.lstsq(X,ref[keep],rcond=None)[0]; cy=np.linalg.lstsq(X,w[keep],rcond=None)[0]
 hr=complex(cr[0],cr[1]); hy=complex(cy[0],cy[1]); h=hy/hr if abs(hr)>1e-8 else complex('nan')
 residual=w[keep]-X@cy
 out.update(frequency_hz=f,closed_loop_gain_db=float(20*np.log10(abs(h))),
   closed_loop_phase_deg=float(np.angle(h,deg=True)),residual_rms_rad_s=float(np.sqrt(np.mean(residual**2))),
   measured_cycles=float((t[-1]-t[keep][0])*f))
out['note']='Encoder-derived speed; motor current calibration assumed. No independent tachometer validation.'
p.with_suffix('.json').write_text(json.dumps(out,indent=2),encoding='utf-8'); print(json.dumps(out,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(3,1,figsize=(9,8),sharex=True)
ax[0].plot(t,w,label='speed rad/s');
if kind>=3: ax[0].plot(t,ref,label='reference rad/s')
ax[1].plot(t,iq,label='Iq A'); ax[1].plot(t,iqref,label='Iq reference A'); ax[1].plot(t,id_,label='Id A')
ax[2].plot(t,np.unwrap(angle),label='mechanical angle rad')
for x in ax: x.legend(); x.grid(alpha=.3)
ax[2].set_xlabel('MCU elapsed time (s)'); fig.suptitle(p.stem); fig.tight_layout(); fig.savefig(p.with_suffix('.png'),dpi=140)
