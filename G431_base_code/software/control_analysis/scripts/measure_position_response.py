"""Measured MPT position response. Default is read-only; --run permits motion.

Resolve ABI from ELF, verify complete flash before any RAM writes, retain raw
captures, and always disable output. No fault clearing or automatic retries.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import time

import numpy as np
from measure_speed_response import SpeedTarget, DEBUG
from position_abi import position_symbols
from verify_encoder_artifacts import load_image


class PositionTarget(SpeedTarget):
    def __init__(self, axf, out, probe):
        self.swd_frequency=50000  # Live RAM reads require conservative SWD speed.
        super().__init__(axf, out, probe)
        self.address.update(position_symbols(axf, DEBUG/'gdb/bin/arm-none-eabi-gdb.exe'))
        expr = dict(cal_ok='&g_speed_test.calibration_ok', tick='&uwTick',
                    user_position='&g_user_control.position.target_deg',
                    position_updates='&g_output_position.updates')
        cmd = [str(DEBUG/'gdb/bin/arm-none-eabi-gdb.exe'), '-batch', str(axf)]
        for v in expr.values(): cmd += ['-ex', 'p/x '+v]
        r = subprocess.run(cmd, capture_output=True, text=True, check=True)
        vals = re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$', r.stdout, re.M)
        if len(vals) != len(expr): raise RuntimeError(r.stdout+r.stderr)
        self.address.update(zip(expr, (int(x, 16) for x in vals)))
        self.beat = 1
        (out/'symbols.json').write_text(json.dumps(self.address, indent=2))

    def heartbeat(self):
        self.beat += 1
        return self.w(self.address['guard']+4, self.beat)

    def run(self,commands,name):
        # Fence posted AP transactions before each burst; a J-Link burst was
        # observed with the previous block's final word prepended to it.
        fenced=[]
        for command in commands:
            if command.startswith('savemem '):fenced.append('read32 0xe0001004 4')
            fenced.append(command)
        return super().run(fenced,name)

    def snapshot(self, name, heartbeat=False):
        for attempt in range(2):
            reason='invalid state/count encoding'
            try:
                b,s=self._snapshot_once(name,heartbeat)
                if snapshot_sane(b,s,self.address['trace_size']//16):return b,s
            except (struct.error,ValueError,OverflowError) as exc:
                reason=repr(exc)
            if attempt:raise RuntimeError('RAM read ABI validation failed twice: '+reason)
            rejected=self.out/('rejected_'+name)
            rejected.mkdir()
            for path in list(self.out.iterdir()):
                if path.is_file() and (path.name.startswith(name+'_') or path.name in (name+'.json',name+'.log')):
                    path.rename(rejected/path.name)
            (rejected/'reason.txt').write_text(reason)
            self.swd_frequency=min(self.swd_frequency,50000)
        raise RuntimeError('Unreachable snapshot retry')

    def _snapshot_once(self, name, heartbeat=False):
        a = self.address
        keys = ('mpt', 'position', 'trajectory', 'frd')
        commands = [self.heartbeat()] if heartbeat else []
        commands += [self.dump(a[k], a[k+'_size'], self.out/f'{name}_{k}.bin') for k in keys]
        commands += [self.dump(a['tick'], 4, self.out/f'{name}_tick.bin'),
                     self.dump(a['speed_run'], 64, self.out/f'{name}_run.bin'),
                     self.dump(a['status'], 36, self.out/f'{name}_user.bin'),
                     self.dump(a['diagnostics'], 128, self.out/f'{name}_diag.bin'),
                     self.dump(a['cal_ok'], 4, self.out/f'{name}_cal.bin'),
                     self.dump(a['guard'], 16, self.out/f'{name}_guard.bin'),
                     self.dump(0x40012c44, 4, self.out/f'{name}_bdtr.bin')]
        self.run(commands, name)
        b = {k: (self.out/f'{name}_{k}.bin').read_bytes()
             for k in (*keys, 'tick', 'run', 'user', 'diag', 'cal', 'guard', 'bdtr')}
        u = lambda k, o: struct.unpack_from('<I', b[k], o)[0]
        f = lambda k, o: struct.unpack_from('<f', b[k], o)[0]
        s = dict(tick_ms=u('tick',0), requests=u('mpt',8), frames=u('mpt',12),
                 errors=list(struct.unpack_from('<5I', b['mpt'],16)),
                 mpt_valid=u('mpt',40), mpt_deg=f('mpt',68),
                 latency_max_ms=u('mpt',88), latency_sum_ms=u('mpt',92),
                 latency_samples=u('mpt',96), updates=u('position',48),
                 position_state=u('position',4), position_fault=u('position',8),
                 error_deg=f('position',28), trajectory_state=u('trajectory',4),
                 trajectory_fault=u('trajectory',8), trace_count=u('trajectory',36),
                 samples=u('trajectory',32), saturated=u('trajectory',52),
                 peak_error_deg=f('trajectory',24),
                 rms_error_deg=math.sqrt(max(0,f('trajectory',28))/max(1,u('trajectory',32))),
                 peak_command_rpm=f('trajectory',56), calibration_ok=u('cal',0),
                 run_state=u('run',40), run_fault=u('run',44),
                 phase_peak_a=f('run',48), user_fault=u('user',4),
                 moe=bool(u('bdtr',0)&0x8000))
        s['slew_limited']=u('trajectory',64) if len(b['trajectory'])>=68 else None
        s['phase_rms_a'],s['current_samples']=phase_rms_from_trajectory(b['trajectory'])
        s['latency_last_us']=u('mpt',100) if len(b['mpt'])>=108 else None
        s['latency_max_us']=u('mpt',104) if len(b['mpt'])>=108 else None
        s['diagnostic_errors']=[u('diag',4*i) for i in (2,3,4,5,6,21,26)]
        s['host_guard']=dict(zip(('timeout_ms','heartbeat','active_timeout_ms','expired'),
                                struct.unpack('<4I',b['guard'])))
        (self.out/f'{name}.json').write_text(json.dumps(s, indent=2))
        return b, s


def phase_rms_from_trajectory(raw):
    if len(raw)<84:return None,0
    sums=struct.unpack_from('<3f',raw,68)
    n=struct.unpack_from('<I',raw,80)[0]
    if any(not math.isfinite(v) or v<0 for v in sums):
        raise RuntimeError('Invalid phase RMS accumulator')
    return ([math.sqrt(v/n) for v in sums] if n else None),n


def snapshot_sane(blocks,state,capacity):
    u=lambda key,offset:struct.unpack_from('<I',blocks[key],offset)[0]
    # Word customer protocol lasts 120 s at 20 kHz (Excel lasts 60 s).
    return ((len(blocks['trajectory'])<84 or u('trajectory',80)<=2404000) and
            u('trajectory',0)<=2 and u('position',0)<=3 and
            state['position_state']<=2 and state['position_fault']<=32 and
            state['trajectory_state']<=3 and state['trajectory_fault']<=32 and
            state['trace_count']<=capacity and state['samples']<=120000 and
            state['mpt_valid']<=1 and state['run_state']<=4 and state['run_fault']<=64 and
            state['user_fault']<=3 and all(math.isfinite(state[k]) for k in
                ('mpt_deg','error_deg','peak_error_deg','rms_error_deg','peak_command_rpm','phase_peak_a')))


def rates(before, after):
    dt = ((after['tick_ms']-before['tick_ms']) & 0xffffffff)*.001
    if dt <= 0: raise RuntimeError('MCU tick stopped')
    return {k+'_hz': ((after[k]-before[k]) & 0xffffffff)/dt
            for k in ('requests', 'frames', 'updates')}


def fit_trace(raw, frequency):
    rows = np.frombuffer(raw, dtype='<f4').reshape(-1,4).astype(float)
    if len(rows)<16 or not np.isfinite(rows).all() or not np.all(np.diff(rows[:,0])>0):
        raise RuntimeError('Invalid trace')
    t, x, y, rpm = rows.T
    basis = np.column_stack((np.ones(len(t)), np.cos(2*np.pi*frequency*t),
                             np.sin(2*np.pi*frequency*t)))
    fx, fy = (np.linalg.lstsq(basis, z, rcond=None)[0] for z in (x, y))
    reference_residual=x-basis@fx
    if np.max(np.abs(reference_residual))>max(.002,.01*np.hypot(fx[1],fx[2])):
        raise RuntimeError('Reference trace is not a consistent sine; possible RAM transport corruption')
    if np.hypot(fx[1],fx[2])<1e-4 or np.hypot(fy[1],fy[2])<1e-8:
        raise RuntimeError('Degenerate reference/feedback excitation')
    h = complex(fy[1],-fy[2])/complex(fx[1],-fx[2])
    residual = y-basis@fy
    harmonics=np.column_stack([np.ones(len(t))]+[v for k in range(1,6)
        for v in (np.cos(k*2*np.pi*frequency*t),np.sin(k*2*np.pi*frequency*t))])
    harmonic_fit=np.linalg.lstsq(harmonics,y,rcond=None)[0]
    thd=float(np.linalg.norm(harmonic_fit[3:])/np.linalg.norm(harmonic_fit[1:3]))
    return dict(trace_gain_db=float(20*np.log10(abs(h))),
                trace_phase_deg=float(np.angle(h,deg=True)),
                feedback_amplitude_deg=float(np.hypot(fy[1],fy[2])),
                reference_amplitude_deg=float(np.hypot(fx[1],fx[2])),
                steady_tracking_rms_deg=float(np.sqrt(np.mean((y-x)**2))),
                residual_rms_deg=float(np.sqrt(np.mean(residual**2))),
                thd_2_to_5_pct=100*thd,
                sample_interval_s=float(np.median(np.diff(t))),
                trace_cycles=float((t[-1]-t[0])*frequency)), rows


def healthy(s, before):
    if (s['errors'] != before['errors'] or s['diagnostic_errors']!=before['diagnostic_errors'] or not s['mpt_valid'] or
            s['position_fault'] or s['run_fault'] or s['user_fault']):
        raise RuntimeError('Feedback/control fault: '+json.dumps(s))


def retry_flash_words(target,expected,offset,raw,out):
    """Re-read sparse burst discrepancies twice. No writes; real mismatches fail."""
    if len(raw)!=len(expected):raise RuntimeError('Truncated Flash read; no RAM writes issued')
    words=sorted({i//4*4 for i,(a,b) in enumerate(zip(raw,expected)) if a!=b})
    if len(words)>16:raise RuntimeError('Flash mismatch exceeds retry envelope; no RAM writes issued')
    old_frequency=target.swd_frequency
    target.swd_frequency=50000
    corrected=bytearray(raw)
    try:
        for attempt in range(2):
            target.run([target.dump(0x08000000+offset+i,min(4,len(raw)-i),
                        out/f'flash_word_{offset+i}_{attempt}.bin') for i in words],
                       f'flash_word_retry_{offset}_{attempt}')
        for i in words:
            a=(out/f'flash_word_{offset+i}_0.bin').read_bytes()
            b=(out/f'flash_word_{offset+i}_1.bin').read_bytes()
            if a!=b or a!=expected[i:i+4]:
                raise RuntimeError(f'Persistent Flash mismatch at {offset+i}; no RAM writes issued')
            corrected[i:i+4]=a
    finally:
        target.swd_frequency=old_frequency
    if corrected!=expected:raise RuntimeError('Flash verification failed; no RAM writes issued')
    return bytes(corrected)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('--axf', type=Path, required=True)
    p.add_argument('--probe', default='150710982')
    p.add_argument('--swd-frequency',type=int,choices=(50000,100000),default=50000)
    p.add_argument('--run', action='store_true')
    p.add_argument('--customer-protocol', action='store_true',
                   help='customer 0..2..0 deg protocol, 2 s work + 2 s hold per frequency')
    p.add_argument('--customer-source',choices=('xlsx','word'),default='xlsx',
                   help='xlsx=literal 300 Hz frames, 1..15 Hz; word=40 slots/cycle, 1..30 Hz')
    p.add_argument('--kp', type=float, default=80)
    p.add_argument('--amplitude', type=float, default=2)
    p.add_argument('--frequencies', default='1,2,3,5,7,8,9,10')
    p.add_argument('--feedforward', type=float, default=.6, help='use 0 for feedback-only bandwidth')
    p.add_argument('--compare-feedforward', action='store_true',
                   help='pair FF=0 and selected FF at each frequency within one armed session')
    p.add_argument('--max-motor-rpm', type=float, default=5500)
    p.add_argument('--duration', type=float, default=0, help='0=automatic, otherwise 2..60 seconds per sine')
    p.add_argument('--max-phase-peak',type=float,default=0,
                   help='optional host trial-stop criterion in A; does not change MCU current protection')
    p.add_argument('--max-phase-rms',type=float,default=0,
                   help='optional per-phase steady RMS trial stop in A; requires RMS telemetry firmware')
    p.add_argument('--step', type=float, default=0, help='relative step degrees; no sweep')
    args=p.parse_args()
    if args.customer_protocol:
        if args.step or args.compare_feedforward or args.duration:
            p.error('Customer protocol has a fixed schedule; step/FF comparison/duration are incompatible')
        args.feedforward=0
        args.amplitude=2
    if args.compare_feedforward and (args.step or args.feedforward==0):
        p.error('Feedforward comparison requires sine motion and nonzero feedforward')
    if args.step:args.feedforward=0
    frequencies = [float(x) for x in args.frequencies.split(',')]
    if (not .1<=args.kp<=200 or not .05<=args.amplitude<=2 or
        not all(.5<=x<=40 for x in frequencies) or not 0<=args.feedforward<=1 or
        not 0<=args.step<=2 or not 100<=args.max_motor_rpm<=8500 or
        not (args.duration==0 or 2<=args.duration<=60) or not 0<=args.max_phase_peak<=15 or
        not 0<=args.max_phase_rms<=5):
        p.error('Invalid trial envelope')
    if args.duration and not args.step and any((args.duration-2*max(.5,2/f))*f<20 for f in frequencies):
        p.error('Duration too short for settled FRD')
    if not args.step and any(2*math.pi*f*args.amplitude*args.feedforward>args.max_motor_rpm*6/202 for f in frequencies):
        p.error('Feedforward velocity exceeds firmware envelope')
    out=args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    t=PositionTarget(args.axf.resolve(),out,args.probe); a=t.address
    if args.customer_protocol and 'customer' not in a:
        raise RuntimeError('Customer protocol firmware required; no RAM writes issued')
    if a['trajectory_size'] not in (64,68,84): raise RuntimeError('Expected feedforward-selectable firmware')
    if args.max_phase_rms and a['trajectory_size']<84:raise RuntimeError('RMS criterion requires RMS telemetry; no motion issued')
    image=load_image(args.axf)
    expected=bytes(image[x] for x in range(min(image),max(image)+1))
    t.swd_frequency=1000000 # Fast flash reads are checked byte-for-byte, never trusted on faith.
    offsets=list(range(0,len(expected),4096))
    verified_flash=bytearray()
    for start in range(0,len(offsets),4):
        group=offsets[start:start+4]
        t.run([t.dump(min(image)+i,min(4096,len(expected)-i),out/f'flash_{i}.bin')
               for i in group],f'verify_{start}')
        if any((out/f'flash_{i}.bin').read_bytes()!=expected[i:i+4096] for i in group):
            t.swd_frequency=100000
            t.run([t.dump(min(image)+i,min(4096,len(expected)-i),out/f'flash_{i}.bin')
                   for i in group],f'verify_retry_{start}')
        for i in group:
            raw=(out/f'flash_{i}.bin').read_bytes()
            block=expected[i:i+4096]
            if raw!=block:raw=retry_flash_words(t,block,i,raw,out)
            verified_flash.extend(raw)
    if verified_flash!=expected:raise RuntimeError('Full Flash verification failed; no RAM writes issued')
    (out/'verified_flash.bin').write_bytes(verified_flash)
    t.swd_frequency=args.swd_frequency
    t.checkpoint('preflight')
    _, before=t.snapshot('before')
    time.sleep(1)
    _, after=t.snapshot('rate_off')
    result=dict(arguments=vars(args)|{'output':str(out),'axf':str(args.axf)},
                off_rates=rates(before,after),trials=[])
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    print('OFF rates',result['off_rates'],'calibration',before['calibration_ok'],flush=True)
    if not args.run: return
    if not before['calibration_ok']: raise RuntimeError('Calibration required; no motion issued')
    if not before['mpt_valid'] or any(before['errors']):
        raise RuntimeError('Unqualified MPT; no motion issued')
    original_guard=None
    try:
        t.run([t.dump(a['guard'],4,out/'guard_before.bin')],'guard_before')
        original_guard=struct.unpack('<I',(out/'guard_before.bin').read_bytes())[0]
        mode_reset=[t.w(a['customer'],0)] if 'customer' in a else []
        t.run(mode_reset+[t.w(a['user'],0),t.w(a['user']+4,5),t.wf(a['user_position'],0),
               t.wf(a['position']+36,args.kp),t.wf(a['position']+40,args.max_motor_rpm),
               t.wf(a['position']+44,.0055),
               t.w(a['guard'],10000),t.heartbeat(),t.w(a['user'],1),'sleep 400'],'arm')
        _, armed=t.snapshot('armed',True); healthy(armed,before)
        if not armed['moe'] or armed['position_state']!=1:
            raise RuntimeError('Position arm failed: '+json.dumps(armed))
        if armed['host_guard']['active_timeout_ms']!=10000 or armed['host_guard']['expired']:
            raise RuntimeError('MCU host guard was not latched; no trajectory issued')
        if args.customer_protocol:
            result['customer_protocol']=True
            t.run([t.heartbeat(),t.w(a['customer'],2 if args.customer_source=='xlsx' else 1),
                   t.w(a['trajectory'],1)],'customer_start')
            start=time.monotonic()
            j=0
            while True:
                _,s=t.snapshot(f'customer_{j}',True); healthy(s,before)
                if args.max_phase_peak and s['phase_peak_a']>args.max_phase_peak:
                    raise RuntimeError('Host experimental phase-peak stop criterion exceeded')
                if args.max_phase_rms and s['phase_rms_a'] and max(s['phase_rms_a'])>args.max_phase_rms:
                    raise RuntimeError('Host experimental whole-protocol RMS stop criterion exceeded')
                if not s['moe'] or s['trajectory_fault'] or s['trajectory_state'] not in (1,2):
                    raise RuntimeError('Customer trajectory interrupted: '+json.dumps(s))
                if s['trajectory_state']==2: break
                timeout=70 if args.customer_source=='xlsx' else 130
                if time.monotonic()-start>timeout: raise RuntimeError('Customer trajectory timeout')
                t.run([t.heartbeat(),'sleep 500'],f'customer_wait_{j}')
                j+=1
            return  # outer finally disables PWM, saves raw results and report
        schedule=[(f,ff) for f in (frequencies if not args.step else [0])
                  for ff in ([0,args.feedforward] if args.compare_feedforward else [args.feedforward])]
        for index, (frequency,feedforward) in enumerate(schedule):
            if frequency:
                duration=args.duration or max(4,2*max(.5,2/frequency)+21/frequency)
                if (duration-2*max(.5,2/frequency))*frequency<20:raise RuntimeError('Duration too short for settled FRD')
                interval=max(1,round(4000/(frequency*(a['trace_size']//16-1))))
                commands=[t.wf(a['trajectory']+12,args.amplitude),
                          t.wf(a['trajectory']+40,frequency),t.wf(a['trajectory']+44,duration),
                          t.w(a['trajectory']+48,interval),
                          t.wf(a['trajectory']+60,feedforward),t.w(a['trajectory'],1)]
            else:
                duration=args.duration or max(1,6/args.kp)
                interval=max(5,math.ceil(5000/(args.kp*(a['trace_size']//16-1))))
                commands=[t.wf(a['trajectory']+12,abs(args.step)),
                          t.wf(a['trajectory']+40,-1),t.wf(a['trajectory']+44,duration),
                          t.w(a['trajectory']+48,interval),t.wf(a['trajectory']+60,0),
                          t.w(a['trajectory'],1)]
            # Frequent heartbeats; MCU deadman remains independent of host finally.
            t.run([t.heartbeat()]+commands+['sleep 500'],f'start_{index}')
            start=time.monotonic()
            j=0
            while True:
                b,s=t.snapshot(f'f{index}_{j}',True); healthy(s,before)
                if args.max_phase_peak and s['phase_peak_a']>args.max_phase_peak:
                    raise RuntimeError('Host experimental phase-peak stop criterion exceeded')
                if args.max_phase_rms and s['phase_rms_a'] and max(s['phase_rms_a'])>args.max_phase_rms:
                    raise RuntimeError('Host experimental phase-RMS stop criterion exceeded')
                if not s['moe']: raise RuntimeError('Unexpected output OFF')
                if s['trajectory_fault']: raise RuntimeError('Trajectory failed: '+json.dumps(s))
                if s['trajectory_state']==2: break
                if time.monotonic()-start>duration+5: raise RuntimeError('Trajectory timeout')
                t.run([t.heartbeat(),'sleep 500'],f'wait_{index}_{j}')
                j+=1
            trial=dict(frequency_hz=frequency,kp=args.kp,feedforward=feedforward,
                       amplitude_deg=args.amplitude,**s,**rates(armed,s))
            n=s['trace_count']
            if not 0<n<=a['trace_size']//16: raise RuntimeError('Invalid trace count')
            suffix=f'_ff{feedforward:g}' if args.compare_feedforward else ''
            path=out/f'trace_{frequency:g}{suffix}.bin'
            trial['trace_file']=path.name
            t.run([t.heartbeat(),t.dump(a['trace'],n*16,path)],f'trace_{index}')
            if frequency:
                try:
                    fitted,rows=fit_trace(path.read_bytes(),frequency)
                except RuntimeError:
                    path.rename(path.with_suffix('.rejected.bin'))
                    t.run([t.heartbeat(),t.dump(a['trace'],n*16,path)],f'trace_retry_{index}')
                    fitted,rows=fit_trace(path.read_bytes(),frequency)
                trial.update(fitted)
            else:
                rows=np.frombuffer(path.read_bytes(),dtype='<f4').reshape(-1,4).astype(float)
                trial.update(analyze_step(rows))
            np.savetxt(path.with_suffix('.csv'), rows, delimiter=',',
                       header='time_s,target_deg,actual_deg,motor_rpm',comments='')
            if frequency:
                frd=struct.unpack('<6f2IfI',b['frd'])
                if not math.isclose(frd[4],frequency,rel_tol=1e-5) or not math.isclose(frd[5],args.amplitude,rel_tol=1e-5):
                    raise RuntimeError('Stale FRD frequency/amplitude')
                trial['firmware_frd']=dict(zip(('gain_db','phase_deg','coherence','cycles',
                    'frequency_hz','amplitude_deg','valid','reject','coherence_bin','segments'),frd))
                trial['linear_valid']=bool(frd[6] and not s['saturated'] and not s['slew_limited'])
            result['trials'].append(trial)
            (out/'summary.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(trial),flush=True)
            if not frequency: break
            t.run([t.heartbeat(),'sleep 600'],f'settle_{index}')
    except BaseException as exc:
        result['error']=repr(exc)
        raise
    finally:
        try:
            t.run([t.w(a['user'],0),t.w(a['position'],2),t.w(a['speed_run'],0),
                   'sleep 100'],'finally_off')
            try:
                result['final']=t.checkpoint('final')
            finally:
                if result.get('customer_protocol'):
                    from analyze_customer_position_test import write_report
                    capture=out/'customer_results.bin'
                    t.run([t.dump(a['trace'],30*32,capture)],'customer_results')
                    result['customer_results']=write_report(capture,out,args.customer_source)
            if original_guard is not None: t.run([t.w(a['guard'],original_guard)],'restore_guard')
        except BaseException as exc:
            result['cleanup_error']=repr(exc)
            raise
        finally:
            (out/'summary.json').write_text(json.dumps(result,indent=2))


def analyze_step(rows):
    if len(rows)<16 or not np.isfinite(rows).all() or not np.all(np.diff(rows[:,0])>0):
        raise RuntimeError('Invalid step trace')
    t,x,y,_=rows.T
    edges=np.flatnonzero(abs(x-x[0])>.01)
    if not len(edges):raise RuntimeError('Step edge missing')
    start=edges[0]
    amplitude=x[start]-x[0]; normalized=(y-y[0])/amplitude
    crossings=[np.flatnonzero((np.arange(len(t))>=start)&(normalized>=v)) for v in (.1,.9)]
    rise=float(t[crossings[1][0]]-t[crossings[0][0]]) if all(len(v) for v in crossings) else None
    err=abs(y-x[start]); bad=np.flatnonzero((np.arange(len(t))>=start)&(err>.02*abs(amplitude)))
    settling=(float(t[bad[-1]+1]-t[start]) if len(bad) and bad[-1]<len(t)-1
              and t[-1]-t[bad[-1]+1]>=.05 else None)
    return dict(rise_10_90_s=rise,overshoot_pct=float(max(0,np.max(normalized[start:])-1)*100),
                settling_2pct_s=settling,final_trace_error_deg=float(x[-1]-y[-1]),
                step_amplitude_deg=float(amplitude))


if __name__=='__main__': main()
