"""Plot same-session paired feedforward traces; never accesses hardware."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args(); root=args.directory
    data=json.loads((root/'summary.json').read_text())
    if not data['arguments'].get('compare_feedforward'):
        raise ValueError('Expected explicitly paired same-session data')
    trials=data['trials']; frequencies=sorted({p['frequency_hz'] for p in trials})
    fig,axes=plt.subplots(len(frequencies),2,figsize=(12,3.5*len(frequencies)),squeeze=False)
    for row,frequency in enumerate(frequencies):
        pair=sorted([p for p in trials if p['frequency_hz']==frequency],key=lambda p:p['feedforward'])
        if len(pair)!=2 or any(not p['linear_valid'] for p in pair):
            raise ValueError('Missing or rejected paired trace')
        arrays=[np.fromfile(root/p['trace_file'],dtype='<f4').reshape(-1,4) for p in pair]
        origin=min(a[0,0] for a in arrays)
        ax,err=axes[row]
        for index,(point,array) in enumerate(zip(pair,arrays)):
            t,x,y,_=array.T; center=(max(x)+min(x))/2
            label=f"FF={point['feedforward']:g}"
            if index==0:ax.plot(t-origin,x-center,'k--',lw=1,label='Reference')
            ax.plot(t-origin,y-center,label=label)
            err.plot(t-origin,y-x,label=f"{label}, RMS={point['steady_tracking_rms_deg']:.3f} deg")
        ax.set_title(f'+/-2 deg at {frequency:g} Hz')
        ax.set_ylabel('Output position (deg)');err.set_ylabel('Feedback - reference (deg)')
        err.axhline(0,color='gray',lw=.6)
        for panel in (ax,err):
            panel.set_xlabel('Steady trace time (s)');panel.grid(alpha=.25);panel.legend(fontsize=8)
    fig.suptitle(f"Paired feedforward comparison; Kp={data['arguments']['kp']:g}/s, same armed run")
    fig.tight_layout();fig.savefig(root/'feedforward_pair.png',dpi=170);plt.close(fig)


if __name__=='__main__':main()
