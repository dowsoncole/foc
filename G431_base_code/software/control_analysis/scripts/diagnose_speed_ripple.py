"""Quantify angle-related feedback ripple, without claiming independent speed truth."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def diagnose(root):
    tags=['final_step_r2','final_step_1000rpm','balanced_default_1000']
    fig,axes=plt.subplots(len(tags),2,figsize=(12,9))
    results=[]
    for index,tag in enumerate(tags):
        path=root/tag
        if not (path/'summary.json').exists(): continue
        s=json.loads((path/'summary.json').read_text())
        a=np.fromfile(path/'samples.bin',dtype='<f4').reshape(-1,8).astype(float)
        t=a[:,1]-s['delay_s']
        keep=t>=.15
        angle=np.unwrap(a[keep,6]*2*np.pi/2097152.)
        error=a[keep,4]-a[keep,3]
        x=np.column_stack([np.ones(len(angle))]+
            [wave(n*angle) for n in range(1,7) for wave in (np.sin,np.cos)])
        fit=np.linalg.lstsq(x,error,rcond=None)[0]
        predicted=x@fit
        residual=error-predicted
        harmonics=[float(np.hypot(*fit[1+2*n:3+2*n])) for n in range(6)]
        r=dict(tag=tag,feedback_window_s=[float(t[keep][0]),float(t[keep][-1])],
               mechanical_revolutions=float(abs(angle[-1]-angle[0])/(2*np.pi)),
               speed_ripple_std_rpm=float(np.std(error)),
               harmonic_amplitudes_rpm_orders_1_to_6=harmonics,
               fitted_periodic_variance_fraction=float(1-np.var(residual)/np.var(error)),
               iq_tracking_rms_error_a=float(np.sqrt(np.mean((a[keep,5]-a[keep,7])**2))),
               iq_reference_rms_a=float(np.sqrt(np.mean(a[keep,7]**2))))
        results.append(r)
        axes[index,0].plot(t,a[:,4]-a[:,3],label='encoder speed error')
        axes[index,0].set_title(tag);axes[index,0].set_xlabel('Seconds after step')
        axes[index,0].set_ylabel('rpm');axes[index,0].grid(alpha=.3)
        axes[index,1].scatter(np.rad2deg(angle%(2*np.pi)),error,s=5,alpha=.35)
        axes[index,1].set_xlabel('Same encoder mechanical angle, deg')
        axes[index,1].set_ylabel('Speed error rpm');axes[index,1].grid(alpha=.3)
    fig.suptitle('Angle-related correlation is not proof of encoder error or true shaft ripple')
    fig.tight_layout();fig.savefig(root/'speed_ripple_diagnostics.png',dpi=140);plt.close(fig)
    output=dict(records=results,note='Same-encoder angle/speed correlation is not independent metrology. '
        'Periodic fit is descriptive only and must not be used as unverified correction.')
    (root/'ripple_diagnostics.json').write_text(json.dumps(output,indent=2))
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    diagnose(p.parse_args().root)
