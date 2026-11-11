"""Rebuild measured current-loop comparison plots; no hardware access."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

DTYPE=np.dtype([('sequence','<u4'),('time','<f4'),('ref','<f4',(2,)),
                ('current','<f4',(2,)),('voltage','<f4',(2,))])


def load(root,name):
    return json.loads((root/name/'summary.json').read_text())


def bandwidth(data):
    sine=sorted((x for x in data if x['kind']=='sine'),key=lambda x:x['frequency_hz'])
    if not sine: return None
    if sine[0]['gain_db'] < -3:
        return {'upper_bound_hz':sine[0]['frequency_hz']}
    for a,b in zip(sine,sine[1:]):
        if a['gain_db'] >= -3 and b['gain_db'] < -3:
            weight=(-3-a['gain_db'])/(b['gain_db']-a['gain_db'])
            return {'interpolated_hz':float(a['frequency_hz'] *
                (b['frequency_hz']/a['frequency_hz'])**weight),
                'bracket_hz':[a['frequency_hz'],b['frequency_hz']]}
    return {'lower_bound_hz':sine[-1]['frequency_hz']}


def main():
    p=argparse.ArgumentParser(); p.add_argument('results',type=Path)
    a=p.parse_args(); root=a.results
    series=[('baseline_frequency',0,'Baseline D', '#999999'),
            ('baseline_q',1,'Baseline Q','#bbbbbb'),
            ('final_default_d',0,'Optimized D','#0969da'),
            ('final_default_q',1,'Optimized Q','#d85a00')]
    comparison={}
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    for name,axis,label,color in series:
        data=load(root,name)
        comparison[name]={'step':data[0],'bandwidth':bandwidth(data)}
        rows=np.fromfile(root/name/'step_0hz_samples.bin',dtype=DTYPE)
        t=rows['time']*1000
        current=rows['current'][:,axis]
        ax=axes[0,axis]
        ax.plot(t,current,color=color,alpha=.15,linewidth=.7)
        ax.plot(t[2:-2],np.convolve(current,np.ones(5)/5,mode='valid'),
                label=label,color=color,linewidth=1.5)
        sine=[x for x in data if x['kind']=='sine']
        if name.startswith('final_default'):
            # The full candidate sweep has identical gains; prefer the
            # reboot/default verification wherever that frequency exists.
            fallback='candidate4_d' if axis==0 else 'selected_q'
            merged={x['frequency_hz']:x for x in load(root,fallback) if x['kind']=='sine'}
            merged.update({x['frequency_hz']:x for x in sine})
            sine=[merged[f] for f in sorted(merged)]
        f=[x['frequency_hz'] for x in sine]
        axes[1,0].semilogx(f,[x['gain_db'] for x in sine],'-o',
                          label=label,color=color,markersize=4)
        axes[1,1].semilogx(f,[x['phase_deg'] for x in sine],'-o',
                          label=label,color=color,markersize=4)
        if axis==0 and name.startswith('final'):
            physical=[x for x in sine if 'physical_ch4' in x]
            axes[1,0].semilogx([x['frequency_hz'] for x in physical],
                [20*np.log10(x['physical_ch4']['peak_a']/abs(x['reference_peak_a']))
                 for x in physical],'x',color='#16823b',markersize=8,label='CH4 physical U')
    for index,ax in enumerate(axes[0]):
        ax.axhline(.3,color='black',linestyle='--',linewidth=1,label='0.30 A target')
        ax.axvline(2,color='black',linestyle=':',linewidth=.8)
        ax.set(title=('D' if index==0 else 'Q')+' step: 4-sample records; 5-record display mean',
               xlabel='Time from capture start (ms)',ylabel='Current (A)',xlim=(0,80))
    axes[1,0].axhline(-3,color='black',linestyle='--',linewidth=1,label='-3 dB')
    axes[1,0].set(title='Measured closed-loop amplitude',xlabel='Frequency (Hz)',ylabel='Gain (dB)')
    axes[1,1].set(title='Measured closed-loop phase (not open-loop margin)',
                  xlabel='Frequency (Hz)',ylabel='Phase (degrees)')
    for ax in axes.flat:
        ax.grid(True,which='both',alpha=.25); ax.legend(fontsize=8)
    fig.suptitle('20 kHz current loop: Kp 0.0025 -> 0.05, Ki 1.6 -> 100 (pu units)')
    fig.tight_layout(); fig.savefig(root/'current_response_comparison.png',dpi=170)
    plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,frequency in zip(axes,(100,800)):
        path=root/'final_default_d'/f'sine_{frequency}hz_ch4.npz'
        scope=np.load(path); y=scope['current_a']; dt=float(scope['sample_interval_s'])
        t=np.arange(len(y))*dt; mask=(t>=t[-1]-.4*(t[-1]+dt))
        t=t[mask]; y=y[mask]
        z=np.column_stack((np.ones_like(t),np.sin(2*np.pi*frequency*t),np.cos(2*np.pi*frequency*t)))
        fit=z@np.linalg.lstsq(z,y,rcond=None)[0]
        stride=max(1,len(y)//10000)
        ax.plot(t[::stride]*1000,y[::stride],alpha=.35,linewidth=.6,label='CH4 raw A')
        ax.plot(t[::stride]*1000,fit[::stride],linewidth=1.5,label='Fundamental + DC fit')
        ax.set(title=f'{frequency} Hz, 0.30 A command',xlabel='Scope time (ms)',ylabel='U current (A)')
        ax.grid(alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(root/'physical_current_validation.png',dpi=170)
    plt.close(fig)
    (root/'comparison.json').write_text(json.dumps(comparison,indent=2))
    print(json.dumps({name:val['bandwidth'] for name,val in comparison.items()},indent=2))


if __name__=='__main__': main()
