"""Explicit bounded motor step/sine tests, using the production speed PI.

Requires the speed-response recorder firmware and a verified matching AXF.
Never resets the board or clears a fault. Output is disabled in finally and
independently by the MCU's recorder and pre-existing host guard.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess

import numpy as np
from measure_current_response import Target, DEBUG, PACK
from verify_encoder_artifacts import load_image


class SpeedTarget(Target):
    def __init__(self, axf, out, probe):
        self.probe=probe
        super().__init__(axf,out)
        expressions={
            'response':'&g_speed_response', 'response_size':'sizeof(g_speed_response)',
            'speed_requested':'&g_speed_test.requested',
            'speed_active':'&g_speed_test.active',
            'speed_test':'&g_speed_test', 'speed_run':'&g_speed_run',
            'speed_user':'&g_user_control.speed', 'guard':'&g_speed_run_guard',
            'diagnostics':'&g_encoder_diagnostics',
        }
        cmd=[str(DEBUG/'gdb/bin/arm-none-eabi-gdb.exe'),'-batch',str(axf)]
        for expr in expressions.values(): cmd+=['-ex','p/x '+expr]
        result=subprocess.run(cmd,capture_output=True,text=True,check=True)
        values=re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$',result.stdout,re.M)
        if len(values)!=len(expressions): raise RuntimeError(result.stdout+result.stderr)
        self.address.update(zip(expressions,(int(x,16) for x in values)))
        if self.address['response_size'] not in (88,92): raise RuntimeError('Recorder ABI changed')
        (out/'symbols.json').write_text(json.dumps(self.address,indent=2))

    def run(self, commands, name):
        fenced=[]
        for command in commands:
            if command.startswith('sleep '):
                # J-Link can defer a debug write until the next read. Sleeping
                # before that flush starts the MCU test AFTER the intended wait.
                fenced.append('read32 0xe0001004 4')
            fenced.append(command)
        result=subprocess.run([str(DEBUG/'pyocd/pyocd.exe'),'commander','--no-config',
            '--pack',PACK,'-t','stm32g431vbtx','-u',self.probe,'-M','attach','-W',
            '-f',str(getattr(self,'swd_frequency',100000)),'-c','; '.join(fenced)],capture_output=True,text=True,timeout=35)
        (self.out/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode or re.search(r'Error|No connected|No available',result.stdout+result.stderr):
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout


def analyze(rows, kind, frequency, delay):
    t=rows['time'].astype(float); ref=rows['applied'].astype(float)
    y=rows['rpm'].astype(float); iq=rows['iq'].astype(float)
    raw=np.unwrap(-rows['raw_angle'].astype(float)*2*np.pi/2097152.)
    raw_rpm=np.gradient(raw,t)*30/np.pi
    dt=np.diff(t)
    if not np.all(np.isfinite(rows.view('<f4').reshape(-1,8)[:,1:])):
        raise RuntimeError('Nonfinite capture')
    if not np.all(dt>0) or np.max(dt)>1.25*np.median(dt):
        raise RuntimeError('Missing recorder ticks')
    result=dict(kind=kind,samples=len(rows),sample_interval_s=float(np.median(dt)),
                duration_s=float(t[-1]-t[0]),iq_peak_a=float(np.max(abs(iq))),
                iq_reference_peak_a=float(np.max(abs(rows['iq_ref']))))
    if kind=='step':
        before=(t>=delay-.08)&(t<delay)
        initial=float(np.mean(y[before])); target=float(ref[-1]); amplitude=target-initial
        if abs(amplitude)<1: raise RuntimeError('Step amplitude too small')
        norm=(y-initial)/amplitude; after=t>=delay
        hits=[np.flatnonzero(after & (norm>=v)) for v in (.1,.9)]
        crossings=[float(t[x[0]]-delay) if len(x) else None for x in hits]
        band=max(.02*abs(amplitude),2.)
        bad=np.flatnonzero(after & (abs(y-target)>band))
        settling=None
        if len(bad) and bad[-1]<len(t)-1 and t[-1]-t[bad[-1]+1]>=.1:
            settling=float(t[bad[-1]+1]-delay)
        elif not len(bad): settling=0.
        result.update(initial_rpm=initial,target_rpm=target,
            rise_10_90_s=(crossings[1]-crossings[0] if None not in crossings else None),
            first_10_s=crossings[0],first_90_s=crossings[1],
            overshoot_pct=float(max(0.,np.max(norm[after])-1)*100),
            settling_s=settling,settling_band_rpm=band,settling_min_hold_s=.1,
            steady_error_rpm=float(np.mean(y[-100:])-target),
            steady_std_rpm=float(np.std(y[-100:])),
            iae_rpm_s=float(np.trapezoid(abs(ref[after]-y[after]),t[after])))
    else:
        mask=t>=delay+1/frequency
        phase=2*np.pi*frequency*t[mask]
        x=np.column_stack([np.ones(mask.sum()),np.sin(phase),np.cos(phase)])
        fit=lambda data: np.linalg.lstsq(x,data[mask],rcond=None)[0]
        r=fit(ref); v=fit(y); u=fit(rows['iq_ref'].astype(float))
        rc=complex(r[1],r[2]); vc=complex(v[1],v[2]); uc=complex(u[1],u[2])
        if abs(rc)<1 or abs(uc)<1e-5: raise RuntimeError('Insufficient excitation')
        h=vc/rc; plant=vc*(np.pi/30)/uc
        result.update(frequency_hz=frequency,gain_db=float(20*np.log10(abs(h))),
            phase_deg=float(np.angle(h,deg=True)),reference_peak_rpm=abs(rc),
            feedback_peak_rpm=abs(vc),iq_command_peak_a=abs(uc),
            residual_rms_rpm=float(np.sqrt(np.mean((y[mask]-x@v)**2))),
            measured_cycles=float((t[-1]-t[mask][0])*frequency),
            plant_real=float(plant.real),plant_imag=float(plant.imag))
    return result,raw_rpm


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path); p.add_argument('--probe',required=True)
    p.add_argument('--axf',type=Path,required=True)
    p.add_argument('--kp',type=float,default=.032); p.add_argument('--ki',type=float,default=1.)
    p.add_argument('--bias',type=float,default=300.); p.add_argument('--amplitude',type=float,default=100.)
    p.add_argument('--frequency',type=float,default=0.)
    p.add_argument('--ramp',type=float,default=100000.)
    p.add_argument('--step-delay',type=float,default=.3)
    p.add_argument('--sine-delay',type=float,default=1.0,
                   help='settle at bias before sine; 1 s recommended for low Ki')
    p.add_argument('--divider',type=int,choices=range(1,7))
    p.add_argument('--estimator-tau-ms',type=float,choices=[1.,10.],default=10.,
                   help='metadata for the verified firmware image, not a RAM filter change')
    p.add_argument('--run',action='store_true',help='explicitly allow motor actuation')
    p.add_argument('--use-default',action='store_true',help='verify boot PI without RAM tuning overrides')
    args=p.parse_args()
    if not (.001<=args.kp<=.2 and 0<=args.ki<=2 and abs(args.bias)<=1000 and
            1<=abs(args.amplitude)<=200 and 0<args.ramp<=100000 and
            .05<=args.step_delay<=2.8 and .1<=args.sine_delay<=1.1 and
            (args.frequency==0 or 2<=args.frequency<=160)):
        p.error('Commissioning envelope exceeded')
    out=args.output.resolve(); out.mkdir(parents=True,exist_ok=False)
    target=SpeedTarget(args.axf.resolve(),out,args.probe); a=target.address
    image=load_image(args.axf)
    expected=bytes(image[x] for x in range(min(image),max(image)+1))
    chunks=out/'flash_chunks'; chunks.mkdir()
    offsets=list(range(0,len(expected),4096))
    for start in range(0,len(offsets),4):
        group=offsets[start:start+4]
        target.run([target.dump(min(image)+i,min(4096,len(expected)-i),chunks/str(i))
                    for i in group],f'verify_flash_{start}')
        for i in group:
            if (chunks/str(i)).read_bytes()!=expected[i:i+4096]:
                raise RuntimeError('Board image mismatch: refusing ALL RAM writes')
        print('Flash verified',min(start+4,len(offsets)),'/',len(offsets),flush=True)
    (out/'verified_flash.bin').write_bytes(b''.join((chunks/str(i)).read_bytes() for i in offsets))
    if not args.run:
        print('Image verified; read-only invocation. --run is required for motion.')
        return
    if 'Running' not in target.run(['status'],'cpu_preflight'):
        raise RuntimeError('CPU must already be running; no RAM writes issued')
    kind='sine' if args.frequency else 'step'; delay=args.sine_delay if args.frequency else args.step_delay
    divider=max(1,min(6,math.ceil(6/(args.frequency*.512)))) if args.frequency else 2
    if args.divider is not None: divider=args.divider
    original_timeout=None
    try:
        target.checkpoint('before')
        target.run([target.dump(a['guard'],4,out/'guard_before.bin')],'guard_before')
        original_timeout=struct.unpack('<I',(out/'guard_before.bin').read_bytes())[0]
        target.run([target.dump(a['diagnostics'],128,out/'diag_before.bin')],'diag_before')
        commands=[target.w(a['user'],0),target.w(a['user']+4,4)]
        for index,value in enumerate((args.bias,args.ramp,args.ramp,1500.,1500.)):
            commands.append(target.wf(a['speed_user']+index*4,value))
        if not args.use_default:
            for offset,value in ((24,args.kp),(28,args.ki)):
                commands.append(target.wf(a['speed_requested']+offset,value))
        commands += [target.w(a['guard'],5000)]
        cfg=a['response']+16
        for index,value in enumerate((int(bool(args.frequency)),divider,512)):
            commands.append(target.w(cfg+index*4,value))
        for index,value in enumerate((args.bias,args.amplitude,args.frequency,delay)):
            commands.append(target.wf(cfg+12+index*4,value))
        commands += [target.w(a['response'],1),'sleep 60',
                     target.dump(a['response'],a['response_size'],out/'armed.bin')]
        target.run(commands,'arm')
        if struct.unpack_from('<I',(out/'armed.bin').read_bytes(),4)[0]!=1:
            raise RuntimeError('Recorder arm refused')
        warmup_ms=int(max(0,delay-.1)*1000) if a['response_size']==92 else 0
        target.run([target.w(a['user'],1),f'sleep {warmup_ms+512*divider+350}',
            target.dump(a['response'],a['response_size'],out/'finished.bin'),
            target.dump(a['samples'],16384,out/'samples.bin'),
            target.dump(a['diagnostics'],128,out/'diag_after.bin'),
            target.dump(a['speed_active'],48,out/'actual_pi.bin'),
            target.dump(a['speed_run'],64,out/'run.bin'),
            target.dump(a['speed_test'],192,out/'executor.bin'),target.w(a['user'],0)],'capture')
        target.checkpoint('after')
        state,count,fault=struct.unpack_from('<3I',(out/'finished.bin').read_bytes(),4)
        if (state,count,fault)!=(3,512,0): raise RuntimeError(f'{state=} {count=} {fault=}')
        before=np.fromfile(out/'diag_before.bin',dtype='<u4')
        after=np.fromfile(out/'diag_after.bin',dtype='<u4')
        if any(after[i]!=before[i] for i in (2,3,4,5,6,21,26)):
            raise RuntimeError('Encoder/control error counters increased')
        gains=struct.unpack_from('<2f',(out/'actual_pi.bin').read_bytes(),24)
        if abs(gains[0]-args.kp)>1e-6 or abs(gains[1]-args.ki)>1e-5:
            raise RuntimeError('Actual latched speed PI differs')
        dtype=np.dtype([('sequence','<u4'),('time','<f4'),('command','<f4'),
            ('applied','<f4'),('rpm','<f4'),('iq','<f4'),('raw_angle','<f4'),('iq_ref','<f4')])
        rows=np.fromfile(out/'samples.bin',dtype=dtype)
        result,raw=analyze(rows,kind,args.frequency,delay)
        result.update(kp=args.kp,ki=args.ki,bias_rpm=args.bias,amplitude_rpm=args.amplitude,
            estimator_tau_ms=args.estimator_tau_ms,
            ramp_rpm_s=args.ramp,
            recorder_version=2 if a['response_size']==92 else 1,
            boot_defaults_without_override=args.use_default,
            phase_peak_a=struct.unpack_from('<f',(out/'run.bin').read_bytes(),48)[0],
            delay_s=delay,divider=divider,saturated_rows=struct.unpack_from('<I',
            (out/'finished.bin').read_bytes(),80)[0])
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
        axes[0].plot(rows['time'],raw,alpha=.25,label='encoder derivative, unfiltered')
        for field in ('command','applied','rpm'): axes[0].plot(rows['time'],rows[field],label=field)
        axes[1].plot(rows['time'],rows['iq'],label='Iq feedback A')
        axes[1].plot(rows['time'],rows['iq_ref'],label='Iq reference A')
        for ax in axes: ax.legend(); ax.grid(alpha=.3)
        axes[0].set_ylabel('Motor rpm'); axes[1].set_xlabel('MCU time s')
        fig.tight_layout(); fig.savefig(out/'response.png',dpi=140); plt.close(fig)
        print(json.dumps(result,indent=2))
    finally:
        target.run([target.w(a['user'],0),'sleep 80'],'finally_off')
        target.checkpoint('final')
        if original_timeout is not None:
            target.run([target.w(a['guard'],original_timeout),
                        target.dump(a['guard'],4,out/'guard_restored.bin')],'restore_guard_off')
            if struct.unpack('<I',(out/'guard_restored.bin').read_bytes())[0]!=original_timeout:
                raise RuntimeError('Could not restore OFF-only watchdog setting')


if __name__=='__main__': main()
