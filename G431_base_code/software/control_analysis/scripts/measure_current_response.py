"""Bounded, non-halting current-loop step/sine experiments via user control.

Resolve every RAM address from the supplied AXF. Firmware must route the
armed recorder through USER_MODE_OPEN_CURRENT and disable at completion.
Motor output is explicitly disabled in finally; no fault flags are cleared.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEBUG = Path(r'C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools')
PACK = r'C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0'


class Target:
    def __init__(self, axf, out):
        self.out = out
        expressions = {
            'user': '&g_user_control', 'status': '&g_user_control.status',
            'open': '&g_user_control.open_current',
            'test': '&g_current_loop_test', 'requested': '&g_current_loop_test.requested',
            'state': '&g_current_loop_test.state', 'reason': '&g_current_loop_test.abort_reason',
            'kind': '&g_current_loop_test.requested.kind',
            'axis': '&g_current_loop_test.requested.axis',
            'amplitude': '&g_current_loop_test.requested.amplitude_a',
            'frequency': '&g_current_loop_test.requested.frequency_hz',
            'settle': '&g_current_loop_test.requested.settle_s',
            'delay': '&g_current_loop_test.requested.step_delay_s',
            'average': '&g_current_loop_test.requested.average_count',
            'capture': '&g_current_loop_test.requested.capture_count',
            'samples': '&g_current_loop_test.samples',
            'count': '&g_current_loop_test.sample_count',
            'sat': '&g_current_loop_test.saturation_count',
            'timing': '&g_current_test_timing',
            'tuning': '&g_current_tuning', 'tune_d': '&g_current_tuning.requested_d',
            'tune_q': '&g_current_tuning.requested_q',
            'apply': '&g_current_tuning.apply_request',
            'tune_status': '&g_current_tuning.status',
            'tune_valid': '&g_current_tuning.valid',
            'runtime_fault': '&g_global_mode_control.runtime.fault_flags',
            'foc_fault': '&g_foc_controller.fault_flags',
            'power_fault': '&g_global_mode_control.runtime.first_fault_status_raw',
            'pwm_fault': '&g_tim1_pwm_commit_fault',
            'params': '&g_foc_controller.algorithm.parameters',
            'pi_d': '&g_foc_controller.algorithm.parameters.d_controller',
            'pi_q': '&g_foc_controller.algorithm.parameters.q_controller',
            'tune_size': 'sizeof(g_current_tuning)',
        }
        command = [str(DEBUG / 'gdb/bin/arm-none-eabi-gdb.exe'), '-batch',
                   '-ex', f'file {axf.as_posix()}']
        for expression in expressions.values():
            command += ['-ex', 'p/x ' + expression]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        values = re.findall(r'\$\d+ = (0x[0-9a-fA-F]+)', result.stdout)
        if len(values) != len(expressions):
            raise RuntimeError(result.stdout + result.stderr)
        self.address = dict(zip(expressions, (int(x, 16) for x in values)))
        (out / 'symbols.json').write_text(json.dumps(self.address, indent=2))

    def run(self, commands, name):
        result = subprocess.run(
            [str(DEBUG / 'pyocd/pyocd.exe'), 'commander', '-t', 'stm32g431cbtx',
             '--pack', PACK, '-u', 'E1007200D0D2139393740544', '-M', 'attach', '-W'],
            input='\n'.join(commands + ['exit']) + '\n', text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=25, check=True)
        (self.out / (name + '.log')).write_text(result.stdout, encoding='utf-8')
        if 'Error:' in result.stdout or 'Traceback' in result.stdout:
            raise RuntimeError(result.stdout)
        return result.stdout

    def w(self, address, value):
        return f'write32 {address:#x} {value:#x}'

    def wf(self, address, value):
        return self.w(address, struct.unpack('<I', struct.pack('<f', value))[0])

    def dump(self, address, size, path):
        return f'savemem {address:#x} {size} {path.as_posix()}'

    def checkpoint(self, name):
        a = self.address
        fields = [('bdtr', 0x40012c44, 4), ('runtime_fault', a['runtime_fault'], 4),
                  ('foc_fault', a['foc_fault'], 4), ('status', a['status'], 36)]
        self.run([self.dump(addr, size, self.out / f'{name}_{key}.bin')
                  for key, addr, size in fields], name)
        result = {key: struct.unpack('<'+'I'*(size//4),
                    (self.out / f'{name}_{key}.bin').read_bytes())
                  for key, _, size in fields}
        if result['bdtr'][0] & 0x8000 or result['runtime_fault'][0] or result['foc_fault'][0]:
            raise RuntimeError(f'output not OFF or backend fault: {result}')
        return result


class PhysicalScope:
    def __init__(self):
        import pyvisa
        self.s = pyvisa.ResourceManager().open_resource('USB::0x0699::0x052C::C064843::INSTR')
        self.s.timeout = 15000
        self.s.chunk_size = 1048576
        queries = ['ACQUIRE:STATE', 'ACQUIRE:STOPAFTER', 'HORIZONTAL:SCALE',
                   'TRIGGER:A:MODE','TRIGGER:A:EDGE:SOURCE','TRIGGER:A:LEVEL:CH4']
        self.original = {q:self.s.query(q+'?').strip() for q in queries}

    def arm(self, frequency):
        s = self.s
        s.write('ACQUIRE:STATE STOP')
        s.write(f'HORIZONTAL:SCALE {0.5/frequency}')
        s.write('TRIGGER:A:MODE NORMAL')
        s.write('TRIGGER:A:EDGE:SOURCE CH4')
        s.write('TRIGGER:A:LEVEL:CH4 0')
        s.write('ACQUIRE:STOPAFTER SEQUENCE')
        s.write('ACQUIRE:STATE RUN')

    def read(self, frequency, path):
        s = self.s
        if int(s.query('ACQUIRE:STATE?')):
            s.write('ACQUIRE:STATE STOP')
            raise RuntimeError('physical CH4 trigger did not complete')
        s.write('DATA:SOURCE CH4'); s.write('DATA:ENCDG RIBINARY')
        s.write('DATA:WIDTH 1'); s.write('DATA:START 1')
        s.write('DATA:STOP '+s.query('HORIZONTAL:RECORDLENGTH?').strip())
        dt=float(s.query('WFMOUTPRE:XINCR?'))
        ym=float(s.query('WFMOUTPRE:YMULT?')); yo=float(s.query('WFMOUTPRE:YOFF?'))
        yz=float(s.query('WFMOUTPRE:YZERO?'))
        if s.query('WFMOUTPRE:YUNIT?').strip().strip('"') != 'A':
            raise RuntimeError('CH4 is not in amperes')
        raw=s.query_binary_values('CURVE?', datatype='b',container=np.array)
        y=(raw.astype(float)-yo)*ym+yz
        t=np.arange(len(y))*dt
        # Post-trigger tail excludes acquisition prehistory and startup.
        mask=np.arange(len(y))>=int(.60*len(y))
        z=np.column_stack((np.ones(mask.sum()), np.sin(2*np.pi*frequency*t[mask]),
                           np.cos(2*np.pi*frequency*t[mask])))
        v=np.linalg.lstsq(z,y[mask],rcond=None)[0]
        np.savez_compressed(path,current_a=y,sample_interval_s=np.array(dt))
        return {'peak_a':float(np.hypot(v[1],v[2])), 'offset_a':float(v[0]),
                'residual_rms_a':float(np.sqrt(np.mean((y[mask]-z@v)**2)))}

    def close(self):
        self.s.write('ACQUIRE:STATE STOP')
        for key,value in self.original.items():
            if key != 'ACQUIRE:STATE': self.s.write(key+' '+value)
        self.s.write('ACQUIRE:STATE '+self.original['ACQUIRE:STATE'])
        self.s.close()


def analyze(rows, axis, kind, frequency):
    t = rows['time'].astype(float)
    ref = rows['ref'][:, axis].astype(float)
    y = rows['current'][:, axis].astype(float)
    result = {'samples': len(t), 'duration_s': float(t[-1]-t[0]),
              'sample_interval_s': float(np.median(np.diff(t)))}
    if kind == 'step':
        start = int(np.flatnonzero(abs(ref) > 1e-6)[0])
        target = float(ref[-1])
        tail = y[-128:]
        base = float(y[:start].mean())
        # Five-record moving mean is analysis only; retain all raw samples.
        smooth = np.convolve(y, np.ones(5)/5, mode='valid')
        ts = t[2:-2]
        norm = (smooth-base)/(target-base)
        active = ts >= t[start]
        crossing = []
        for level in (.1, .9):
            ix = np.flatnonzero(active & (norm >= level))
            crossing.append(float(ts[ix[0]]-t[start]) if len(ix) else None)
        late = active & (ts < t[-1]-10*result['sample_interval_s'])
        result.update(target_a=target, tail_mean_a=float(tail.mean()),
            tail_std_a=float(tail.std()), steady_error_a=float(tail.mean()-target),
            first_10_s=crossing[0], first_90_s=crossing[1],
            rise_10_90_s=(crossing[1]-crossing[0] if None not in crossing else None),
            smoothed_overshoot_pct=float(100*(norm[late].max()-1)),
            iae_a_s=float(np.sum(abs(ref[start:]-y[start:]))*result['sample_interval_s']),
            cross_axis_tail_std_a=float(rows['current'][-128:,1-axis].std()))
    else:
        # Discard startup and retain at least the last four cycles when available.
        mask = t >= max(t[0]+1/frequency, t[-1]-6/frequency)
        z = np.column_stack((np.ones(mask.sum()), np.sin(2*np.pi*frequency*t[mask]),
                             np.cos(2*np.pi*frequency*t[mask])))
        r = np.linalg.lstsq(z, ref[mask], rcond=None)[0]
        v = np.linalg.lstsq(z, y[mask], rcond=None)[0]
        rr = complex(r[1], r[2]); yy = complex(v[1], v[2])
        h = yy/rr
        result.update(frequency_hz=frequency, gain_db=float(20*np.log10(abs(h))),
            phase_deg=float(np.angle(h, deg=True)), reference_peak_a=abs(rr),
            current_peak_a=abs(yy), offset_a=float(v[0]),
            residual_rms_a=float(np.sqrt(np.mean((y[mask]-z@v)**2))))
    result['peak_voltage_pu'] = float(np.max(abs(rows['voltage'][:,axis])))
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    p.add_argument('--axf', type=Path, default=ROOT/'software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf')
    p.add_argument('--kp', type=float, default=.0025)
    p.add_argument('--ki', type=float, default=1.6)
    p.add_argument('--axis', type=int, choices=[0,1], default=0)
    p.add_argument('--amplitude', type=float, default=.3)
    p.add_argument('--frequencies', type=float, nargs='*', default=[])
    p.add_argument('--step-only', action='store_true')
    p.add_argument('--scope', action='store_true', help='also capture CH4 physical U-phase sine')
    p.add_argument('--use-default', action='store_true', help='verify boot gains without applying a tuning override')
    a = p.parse_args()
    if not (0 < abs(a.amplitude) <= .5 and 0 < a.kp <= .1 and 0 < a.ki <= 100):
        p.error('bounded commissioning range exceeded')
    a.output = a.output.resolve(); a.output.mkdir(parents=True, exist_ok=True)
    target = Target(a.axf.resolve(), a.output); ad = target.address
    summary = []
    scope = None
    try:
        if a.scope:
            if a.axis != 0: raise ValueError('CH4 U-phase scope capture requires D axis at angle zero')
            scope=PhysicalScope()
        target.run([target.w(ad['user'], 0), 'sleep 100'], 'disable')
        target.checkpoint('before')
        if a.use_default:
            target.run([target.dump(ad['tune_valid'],4,a.output/'default_tuning_valid.bin')],
                       'verify_no_override')
            if (a.output/'default_tuning_valid.bin').read_bytes()[0] != 0:
                raise RuntimeError('boot-default test requires no accepted RAM tuning override; restart with user enable OFF')
        commands = [target.w(ad['user']+4, 2), target.w(ad['test'], 3), 'sleep 100']
        if not a.use_default:
            for addr in (ad['tune_d'], ad['tune_q']):
                for index, value in enumerate((a.kp,a.ki,0,-.02,.02)):
                    commands.append(target.wf(addr+index*4, value))
            commands += [target.w(ad['apply'],2), 'sleep 150',
                         target.dump(ad['tune_status'], 4, a.output/'apply_status.bin')]
        target.run(commands, 'apply')
        if not a.use_default and struct.unpack('<I',(a.output/'apply_status.bin').read_bytes())[0] != 1:
            raise RuntimeError('PI application rejected')
        trials = [('step',0.)] + ([] if a.step_only else [('sine',f) for f in a.frequencies])
        for kind, frequency in trials:
            if kind=='sine' and not 10 <= frequency <= 2000:
                raise ValueError('sine frequency outside commissioned range')
            average = 4 if kind=='step' else max(1, math.ceil(8/(frequency*512*50e-6)))
            name = f'{kind}_{frequency:g}hz'
            command = [target.w(ad['test'],3), 'sleep 80']
            # ARMClang uses short enums: resolve members rather than assuming
            # host sizeof(enum) or packing. Both enum/axis are single bytes.
            command += [f'write8 {ad["kind"]:#x} {int(kind=="sine")}',
                        f'write8 {ad["axis"]:#x} {a.axis}',
                        f'write16 {ad["average"]:#x} {average}',
                        f'write16 {ad["capture"]:#x} 512']
            for field,value in [('amplitude',a.amplitude),('frequency',frequency),
                                ('settle',0.),('delay',.002 if kind=='step' else 0.)]:
                command.append(target.wf(ad[field],value))
            for n in range(3):
                command.append(target.wf(ad['open']+4*n,0.))
            command += [target.w(ad['test'],1),'sleep 80',
                        target.dump(ad['state'],2,a.output/f'{name}_armed.bin')]
            target.run(command, name+'_arm')
            if struct.unpack('<BB',(a.output/f'{name}_armed.bin').read_bytes()) != (1,0):
                raise RuntimeError('bounded test did not arm')
            if scope is not None and kind=='sine':
                scope.arm(frequency)
            command = [target.w(ad['user'],1),f'sleep {int(512*average*.05)+250}']
            for field,size in [('samples',16384),('timing',16),('count',8),('sat',8),
                               ('pi_d',20),('pi_q',20)]:
                command.append(target.dump(ad[field],size,a.output/f'{name}_{field}.bin'))
            command.append(target.dump(ad['state'],2,a.output/f'{name}_state.bin'))
            command.append(target.w(ad['user'],0))
            command += ['sleep 50']
            target.run(command,name+'_capture')
            state,reason = struct.unpack('<BB',(a.output/f'{name}_state.bin').read_bytes())
            if (state,reason) != (3,0):
                raise RuntimeError(f'test failed {state=} {reason=}')
            target.checkpoint(name+'_off')
            dtype=np.dtype([('sequence','<u4'),('time','<f4'),('ref','<f4',(2,)),
                            ('current','<f4',(2,)),('voltage','<f4',(2,))])
            rows=np.fromfile(a.output/f'{name}_samples.bin',dtype=dtype)
            if not np.all(np.diff(rows['sequence'])==average):
                raise RuntimeError('sample sequence gap')
            first,last,count,maximum=struct.unpack('<4I',(a.output/f'{name}_timing.bin').read_bytes())
            period=((last-first)&0xffffffff)/(168e6*(count-1))
            if abs(period-50e-6)>1e-6:
                raise RuntimeError(f'actual control period {period} invalid')
            result=analyze(rows,a.axis,kind,frequency)
            for field in ('pi_d','pi_q'):
                gain=struct.unpack('<5f',(a.output/f'{name}_{field}.bin').read_bytes())
                if abs(gain[0]-a.kp)>1e-6 or abs(gain[1]-a.ki)>1e-4:
                    raise RuntimeError(f'actual {field} gains differ from experiment request: {gain}')
            result['boot_defaults_without_override']=a.use_default
            if scope is not None and kind=='sine':
                result['physical_ch4']=scope.read(frequency,a.output/f'{name}_ch4.npz')
            saturation,maximum_error=struct.unpack('<If',(a.output/f'{name}_sat.bin').read_bytes())
            result.update(kind=kind,kp=a.kp,ki=a.ki,axis=a.axis,average_count=average,
                          measured_control_period_s=period,maximum_step_cycles=maximum,
                          saturation_count=saturation,maximum_error_a=maximum_error)
            summary.append(result)
            (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
            print(json.dumps(result),flush=True)
    finally:
        target.run([target.w(ad['user'],0),'sleep 100',target.w(ad['test'],3)],'finally_disable')
        target.checkpoint('final')
        if scope is not None: scope.close()


if __name__=='__main__':
    main()
