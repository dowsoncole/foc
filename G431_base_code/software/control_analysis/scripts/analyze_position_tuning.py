"""Aggregate frozen position trials, including clipped large-signal responses."""
import argparse
import json
import math
import struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from measure_position_response import fit_trace, analyze_step


def crossing(points):
    for first,second in zip(points,points[1:]):
        if first is None or second is None:continue
        (f1,g1),(f2,g2)=first,second
        if g1>=-3 and g2<-3:
            return math.exp(math.log(f1)+(-3-g1)/(g2-g1)*math.log(f2/f1))
    return None


def json_finite(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:json_finite(v) for k,v in value.items()}
    if isinstance(value,list):return [json_finite(v) for v in value]
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--plot-groups',default='')
    p.add_argument('--focus',default='')
    p.add_argument('--step-groups',default='')
    args=p.parse_args();root=args.directory
    plot_names=set(args.plot_groups.split(',')) if args.plot_groups else set()
    groups=[];steps=[]
    for summary in sorted(root.glob('*/summary.json')):
        data=json.loads(summary.read_text())
        trials=data.get('trials',[])
        if not trials:continue
        final_bdtr=data.get('final',{}).get('bdtr',[None])[0]
        final_file=summary.parent/'final_bdtr.bin'
        if final_bdtr is None and final_file.exists() and final_file.stat().st_size==4:
            final_bdtr=struct.unpack('<I',final_file.read_bytes())[0]
        group=dict(name=summary.parent.name,arguments=data['arguments'],points=[],
                   final_off=final_bdtr is not None and not bool(final_bdtr&0x8000),
                   error=data.get('error'),cleanup_error=data.get('cleanup_error'))
        armed_file=summary.parent/'armed.json'
        group['center_deg']=json.loads(armed_file.read_text()).get('mpt_deg') if armed_file.exists() else None
        context_file=summary.parent/'experiment_context.json'
        group['experiment_context']=json.loads(context_file.read_text()) if context_file.exists() else {'load_condition':'not_recorded'}
        for point in trials:
            f=point['frequency_hz'];path=summary.parent/point.get('trace_file',f'trace_{f:g}.bin')
            if not path.exists():continue
            if f:
                try:
                    fit,_=fit_trace(path.read_bytes(),f)
                    trace_error=None
                except RuntimeError as exc:
                    fit=dict(trace_gain_db=float('nan'),trace_phase_deg=float('nan'),
                             thd_2_to_5_pct=float('nan'))
                    trace_error=str(exc)
                frd=point.get('firmware_frd',{})
                long_valid=bool(frd.get('valid'))
                gain=frd['gain_db'] if long_valid else fit['trace_gain_db']
                phase=frd['phase_deg'] if long_valid else fit['trace_phase_deg']
                group['points'].append(dict(frequency_hz=f,**fit,
                    feedforward=point['feedforward'],trace_file=path.name,
                    tracking_gain_db=gain,tracking_phase_deg=phase,
                    gain_source='firmware_frd' if long_valid else 'trace',trace_error=trace_error,
                    saturated_samples=point['saturated'],sample_count=point['samples'],
                    slew_limited_samples=point.get('slew_limited'),
                    firmware_valid=point.get('linear_valid',False),
                    phase_peak_since_arm_a=point['phase_peak_a'],
                    phase_rms_a=point.get('phase_rms_a'),
                    current_samples=point.get('current_samples'),
                    trajectory_rms_error_deg=point['rms_error_deg'],
                    requests_hz=point['requests_hz'],updates_hz=point['updates_hz']))
            else:
                rows=np.frombuffer(path.read_bytes(),dtype='<f4').reshape(-1,4).astype(float)
                steps.append(dict(name=group['name'],**analyze_step(rows)))
        points=sorted(group['points'],key=lambda x:x['frequency_hz'])
        if points:
            group['amplitude_dependent_3db_hz']=crossing([(x['frequency_hz'],x['tracking_gain_db']) for x in points])
            group['unclipped_3db_hz']=crossing([(x['frequency_hz'],x['tracking_gain_db']) if x['firmware_valid'] else None for x in points])
            if data['arguments'].get('compare_feedforward'):
                group['amplitude_dependent_3db_hz']=group['unclipped_3db_hz']=None
                group['bandwidth_note']='Paired different feedforward values; not one transfer function.'
            groups.append(group)
    (root/'comparison.json').write_text(json.dumps(json_finite(dict(groups=groups,steps=steps)),indent=2,allow_nan=False))
    fig,axes=plt.subplots(4,1,figsize=(10,12),sharex=True)
    for group in groups:
        if plot_names and group['name'] not in plot_names:continue
        pts=sorted(group['points'],key=lambda x:x['frequency_hz']);arg=group['arguments']
        label=f"Kp={arg['kp']:g}, A={arg['amplitude']:g} deg, {arg.get('max_motor_rpm',2500):g} rpm, FF={arg['feedforward']:g}"
        if arg.get('compare_feedforward'):label+=' (paired with FF=0)'
        if group['center_deg'] is not None:label+=f", center={group['center_deg']:.1f} deg"
        x=[v['frequency_hz'] for v in pts]
        line=axes[0].semilogx(x,[v['tracking_gain_db'] for v in pts],'-o',ms=4,label=label)[0]
        c=line.get_color()
        for pt in pts:
            if not pt['firmware_valid']:
                axes[0].plot(pt['frequency_hz'],pt['tracking_gain_db'],'x',color=c,ms=10,mew=2)
        axes[1].semilogx(x,np.rad2deg(np.unwrap(np.deg2rad([v['tracking_phase_deg'] for v in pts]))),'-o',ms=4,color=c)
        axes[2].semilogx(x,[v['phase_peak_since_arm_a'] for v in pts],'-o',ms=4,color=c)
        axes[3].semilogx(x,[max(v['phase_rms_a']) if v['phase_rms_a'] else np.nan for v in pts],'-o',ms=4,color=c)
    axes[0].axhline(-3,color='gray',ls='--');axes[0].axhline(0,color='gray',lw=.5)
    axes[0].set_ylabel('Tracking gain (dB)');axes[1].set_ylabel('Tracking phase (deg)')
    axes[2].set_ylabel('Peak phase current since arm (A)')
    axes[2].axhline(15,color='gray',ls='--')
    axes[3].set_ylabel('Max phase RMS current (A)');axes[3].set_xlabel('Frequency (Hz)')
    axes[3].axhline(5,color='gray',ls='--')
    axes[0].legend(fontsize=8,loc='lower left')
    for ax in axes:ax.grid(True,which='both',alpha=.25);ax.axvline(20,color='gray',ls=':')
    axes[0].set_title('Measured MPT response (long-window FRD where valid); x = validity rejected')
    fig.tight_layout();fig.savefig(root/'frequency_comparison.png',dpi=170);plt.close(fig)
    selected=[g for g in groups if g['arguments']['amplitude']==2 and g['points'] and
              (not args.focus or g['name']==args.focus)]
    if args.focus and selected:
        group=selected[0];selected=[]
        points=sorted(group['points'],key=lambda q:q['frequency_hz'])
        for index in sorted({0,len(points)//2,len(points)-1}):
            point=points[index]
            selected.append(dict(group,points=[point]))
    if selected:
        fig,axes=plt.subplots(len(selected),1,figsize=(10,3*len(selected)),squeeze=False)
        for row,group in enumerate(selected):
            target=min(group['points'],key=lambda q:abs(q['frequency_hz']-10))
            f=target['frequency_hz'];raw=root/group['name']/target.get('trace_file',f'trace_{f:g}.bin')
            rows=np.frombuffer(raw.read_bytes(),dtype='<f4').reshape(-1,4)
            t,x,y,_=rows.T;origin=(max(x)+min(x))/2
            ax=axes[row,0];ax.plot(t-t[0],x-origin,label='Target');ax.plot(t-t[0],y-origin,label='MPT feedback')
            ax.set_title(f"{group['name']}, {f:g} Hz, FF={target['feedforward']:g}, gain {target['trace_gain_db']:.2f} dB")
            ax.set_ylabel('Output deg');ax.grid(alpha=.25);ax.legend()
        axes[-1,0].set_xlabel('Time (s)');fig.tight_layout();fig.savefig(root/'sine_comparison.png',dpi=170);plt.close(fig)
    step_names=set(args.step_groups.split(',')) if args.step_groups else set()
    plot_steps=[s for s in steps if not step_names or s['name'] in step_names]
    if plot_steps:
        fig,axes=plt.subplots(2,1,figsize=(10,8))
        for step in plot_steps:
            rows=np.frombuffer((root/step['name']/'trace_0.bin').read_bytes(),dtype='<f4').reshape(-1,4)
            t,x,y,_=rows.T
            for ax in axes:ax.plot(t,y-y[0],label=step['name'])
        for ax in axes:
            ax.plot(t,x-x[0],'k--',label='Target');ax.set_xlabel('Time (s)');ax.set_ylabel('Output deg')
            ax.legend(fontsize=8);ax.grid(alpha=.25)
        axes[1].set_xlim(0,.3);axes[1].set_title('Step detail (5 ms fast-trace interval)')
        fig.tight_layout();fig.savefig(root/'step_comparison.png',dpi=170);plt.close(fig)
    print(json.dumps({g['name']:dict(large_signal_3db_hz=g['amplitude_dependent_3db_hz'],
        unclipped_3db_hz=g['unclipped_3db_hz'],final_off=g['final_off']) for g in groups},indent=2))
    print(json.dumps(steps,indent=2))


if __name__=='__main__':main()
