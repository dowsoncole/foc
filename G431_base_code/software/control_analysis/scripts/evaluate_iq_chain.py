"""Effective recorded Iq-command -> Iq-feedback FRF during external speed sines."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def evaluate(root):
    records=[];fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for prefix in ('baseline','final','balanced'):
        dataset=[]
        for path in root.glob(prefix+'_f*/summary.json'):
            s=json.loads(path.read_text());f=s['frequency_hz']
            a=np.fromfile(path.parent/'samples.bin',dtype='<f4').reshape(-1,8).astype(float)
            keep=a[:,1]>=s['delay_s']+1/f;t=a[keep,1]
            x=np.column_stack([np.ones(len(t)),np.sin(2*np.pi*f*t),np.cos(2*np.pi*f*t)])
            y=np.linalg.lstsq(x,a[keep,5],rcond=None)[0]
            u=np.linalg.lstsq(x,a[keep,7],rcond=None)[0]
            ratio=complex(y[1],y[2])/complex(u[1],u[2])
            dataset.append(dict(prefix=prefix,frequency_hz=f,gain_db=float(20*np.log10(abs(ratio))),
                phase_deg=float(np.angle(ratio,deg=True)),
                iq_fundamental_peak_a=float(np.hypot(y[1],y[2])),
                command_fundamental_peak_a=float(np.hypot(u[1],u[2])),
                feedback_residual_rms_a=float(np.sqrt(np.mean((a[keep,5]-x@y)**2)))))
        dataset.sort(key=lambda r:r['frequency_hz']);records.extend(dataset)
        if dataset:
            f=[r['frequency_hz'] for r in dataset]
            axes[0].semilogx(f,[r['gain_db'] for r in dataset],'-o',label=prefix)
            axes[1].semilogx(f,np.rad2deg(np.unwrap(np.deg2rad([r['phase_deg'] for r in dataset]))),'-o',label=prefix)
    for ax in axes:ax.grid(alpha=.3);ax.legend()
    axes[0].set_ylabel('Recorded Iq feedback / Iq command, dB')
    axes[1].set_ylabel('Phase deg');axes[1].set_xlabel('Hz')
    fig.tight_layout();fig.savefig(root/'effective_iq_chain.png',dpi=140);plt.close(fig)
    result=dict(records=records,note='Includes current dynamics and command/publication/record timing. '
        'Not independent current-probe metrology or isolated current-loop bandwidth measurement.')
    (root/'effective_iq_chain.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    evaluate(p.parse_args().root)
