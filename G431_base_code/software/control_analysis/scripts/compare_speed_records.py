"""Offline, equal-window comparison; no debugger access or motor operation."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def compare(root):
    tags=['baseline_step_fenced','final_step_r2','final_step_repeat',
          'final_step_down','final_step_reverse','final_step_user_ramp',
          'final_step_1000rpm','robust_p02_i05_1000','robust_p01_i05_1000',
          'robust_p02_i05_400','robust_p01_i05_400',
          'balanced_default_step','balanced_default_ramp','balanced_default_1000']
    metrics=[]
    fig, axes=plt.subplots(2,2,figsize=(12,8))
    for tag in tags:
        folder=root/tag
        if not (folder/'summary.json').exists(): continue
        s=json.loads((folder/'summary.json').read_text())
        a=np.fromfile(folder/'samples.bin',dtype='<f4').reshape(-1,8)
        t=a[:,1].astype(float)-s['delay_s']; ref=a[:,3]; y=a[:,4]
        keep=(t>=0)&(t<=.4)
        tail=(t>=.3)&(t<=.4)
        if tail.sum()<10 or t[-1]<.4: raise ValueError('Insufficient common window '+tag)
        err=y[tail]-ref[tail]
        m=dict(tag=tag,common_window_s=[0,.4],steady_window_s=[.3,.4],
               iae_400ms_rpm_s=float(np.trapezoid(abs(y[keep]-ref[keep]),t[keep])),
               command_iae_400ms_rpm_s=float(np.trapezoid(abs(y[keep]-a[keep,2]),t[keep])),
               steady_error_rpm=float(np.mean(err)),steady_std_rpm=float(np.std(err)),
               steady_peak_to_peak_rpm=float(np.ptp(err)),
               steady_abs_peak_rpm=float(np.max(abs(err))))
        metrics.append(m)
        labels={'baseline_step_fenced':'baseline', 'final_step_r2':'fast PI',
                'balanced_default_step':'balanced PI, fast ramp',
                'balanced_default_ramp':'balanced PI, normal ramp'}
        if tag in labels:
            axes[0,0].plot(t,y,label=labels[tag])
            axes[0,1].plot(t,y,label=labels[tag])
        if tag in ('final_step_1000rpm','balanced_default_1000'):
            axes[1,0].plot(t,y,label='fast PI' if tag=='final_step_1000rpm' else 'selected balanced PI')
    for prefix in ('baseline','final','balanced'):
        s=[json.loads(p.read_text()) for p in root.glob(prefix+'_f*/summary.json')]
        s.sort(key=lambda x:x['frequency_hz'])
        if s: axes[1,1].semilogx([x['frequency_hz'] for x in s],
                     [x['gain_db'] for x in s],'-o',label={
                         'baseline':'baseline','final':'fast PI (r2)',
                         'balanced':'selected balanced PI'}[prefix])
    axes[0,0].set_xlim(-.02,.08); axes[0,1].set_xlim(-.1,.42)
    axes[0,0].set_ylim(260,450); axes[0,1].set_ylim(260,450)
    axes[1,0].set_xlim(-.05,.42)
    for ax in axes.flat: ax.grid(alpha=.3); ax.legend(fontsize=7)
    for ax in axes.flat[:3]:
        ax.set_xlabel('Seconds relative to command step'); ax.set_ylabel('Motor rpm')
    axes[1,1].set_xlabel('Hz'); axes[1,1].set_ylabel('Measured closed-loop gain dB')
    axes[1,1].axhline(-3,color='gray',linestyle='--')
    fig.tight_layout();fig.savefig(root/'speed_comparison.png',dpi=150);plt.close(fig)
    (root/'common_window_metrics.json').write_text(json.dumps(metrics,indent=2))
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    compare(p.parse_args().root)
