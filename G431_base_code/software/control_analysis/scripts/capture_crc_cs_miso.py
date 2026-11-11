"""Bounded encoder capture with boot motor enable explicitly disabled at main."""
import os, json, time, shutil, subprocess, sys
from pathlib import Path
import pyvisa
import numpy as np
import re
from concurrent.futures import ThreadPoolExecutor
os.environ['SERVO_PYOCD'] = 'C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe'
from test_highspeed_closed_loop import ProbeRun, base

class HoldProbeRun(ProbeRun):
    def command(self, commands):
        result = subprocess.run([str(base.PROBE), 'commander', '--no-config', '--pack', base.PACK,
            '-t', 'stm32g431vbtx', '-u', 'E1007200D0D2139393740544', '-M', 'attach', '-W',
            '-O', 'resume_on_disconnect=false', '-f', '500000', '-c', '; '.join(commands)],
            capture_output=True, text=True, timeout=40)
        with (self.directory / 'probe.log').open('a', encoding='utf-8') as log:
            log.write('; '.join(commands) + '\n' + result.stdout + result.stderr + '\n')
        if result.returncode or re.search(r'Error|No connected|No available', result.stdout + result.stderr):
            raise RuntimeError(result.stdout + result.stderr)
        return result.stdout

root = Path('software/control_analysis/results') / time.strftime('crc_cs_miso_%Y%m%d_%H%M%S')
root.mkdir()
image = (root / 'firmware.axf').resolve()
shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf', image)
symbols = subprocess.check_output(['arm-none-eabi-nm', str(image)], text=True)
main_address = next(line.split()[0] for line in symbols.splitlines() if line.split()[-1:] == ['main'])
p = HoldProbeRun(image, root / 'trial')
r = {'verification': p.verify_flash(), 'channels': {'1': 'SCK', '2': 'CS_PA4', '3': 'MISO_PA6'}, 'samples': []}
run_requested = '--run' in sys.argv
actuated = False
s = pyvisa.ResourceManager().open_resource('USB::0x0699::0x052C::C064843::INSTR')
s.timeout = 30000
s.chunk_size = 1024 * 1024
try:
    _, r['before'] = p.snapshot('before', full=True)
    base.require_off(r['before'])
    actuated = True
    executor = ThreadPoolExecutor(max_workers=1)
    startup = executor.submit(p.command, ['reset halt', f'break 0x{main_address}', 'go', 'sleep 50', 'reg pc', f'write32 0x{p.addresses["run"]:x} {int(run_requested)}', f'write32 0x{p.addresses["guard"]:x} {8000 if run_requested else 0}', f'write32 0x{p.addresses["guard"]+4:x} 1', f'rmbreak 0x{main_address}', 'sleep 20000', 'go'])
    time.sleep(3)
    for c in ['ACQUIRE:STATE STOP', 'SELECT:CH1 ON', 'SELECT:CH2 ON', 'SELECT:CH3 ON', 'SELECT:CH4 OFF', 'HORIZONTAL:RECORDLENGTH 10000000', 'HORIZONTAL:SCALE 0.01', 'HORIZONTAL:POSITION 10', 'ACQUIRE:MODE SAMPLE', 'TRIGGER:A:TYPE EDGE', 'TRIGGER:A:MODE NORMAL', 'TRIGGER:A:EDGE:SOURCE CH2', 'TRIGGER:A:EDGE:SLOPE FALL', 'TRIGGER:A:EDGE:COUPLING DC', 'TRIGGER:A:LEVEL:CH2 1.65', 'ACQUIRE:STOPAFTER SEQUENCE']:
        s.write(c)
    for ch in [1, 2, 3]:
        for setting in ['BANDWIDTH FULL', 'SCALE 1', 'POSITION -2', 'COUPLING DC']:
            s.write(f'CH{ch}:{setting}')
    s.query('*OPC?')
    s.write('HORIZONTAL:SCALE 0.005')
    r['scope_horizontal_before'] = s.query('HORIZONTAL?').strip()
    print('HORIZONTAL', r['scope_horizontal_before'], flush=True)
    s.write('ACQUIRE:STATE RUN')
    deadline = time.monotonic() + 12
    while True:
        trigger_state = s.query('TRIGGER:STATE?').strip()
        if trigger_state in ('READY', 'ARMED'):
            break
        if time.monotonic() > deadline:
            raise RuntimeError('Scope not ready: ' + trigger_state)
        time.sleep(0.2)
    print('SCOPE', trigger_state, flush=True)
    if run_requested:
        actuated = True
        p.keepalive = True
        p.sequence = 1
    startup.result(timeout=30)
    executor.shutdown()
    end_acquire = time.monotonic() + 5
    while s.query('ACQUIRE:STATE?').strip() != '0' and time.monotonic() < end_acquire:
        time.sleep(0.2)
    for i in range(6):
        _, st = p.snapshot(f'sample_{i}', full=True)
        if not run_requested:
            base.require_off(st)
        st['scope_acquire'] = s.query('ACQUIRE:STATE?').strip()
        r['samples'].append(st)
        print(json.dumps({k: st[k] for k in ['moe', 'crc_errors', 'transfer_errors', 'scope_acquire']}), flush=True)
        if st['crc_errors'] or st['run_fault'] or st['controller_fault']:
            break
    if actuated:
        try:
            r['stopped'] = p.stop(0x02002054)
        except Exception as stop_error:
            r['stop_error'] = repr(stop_error)
            p.command([f'write32 0x{p.addresses["run"]:x} 0', f'write32 0x{base.BDTR:x} 0x02002054'])
            _, r['stopped'] = p.snapshot('stop_verified', full=True)
            base.require_off(r['stopped'])
        actuated = False
    if s.query('ACQUIRE:STATE?').strip() == '0' and r['samples'][-1]['crc_errors']:
        meta = {'channels': {}, 'signals': r['channels']}
        arrays = {}
        for c in ['DATA:ENCDG RIBINARY', 'DATA:WIDTH 1', 'DATA:START 1', 'DATA:STOP 10000000']:
            s.write(c)
        for ch in [1, 2, 3]:
            s.write(f'DATA:SOURCE CH{ch}')
            meta['channels'][str(ch)] = {k: float(s.query('WFMOUTPRE:' + k + '?')) for k in ['XINCR', 'XZERO', 'PT_OFF', 'YMULT', 'YOFF', 'YZERO']}
            arrays[f'ch{ch}'] = np.asarray(s.query_binary_values('CURVE?', datatype='b', container=np.array), dtype=np.int8)
        np.savez_compressed(root / 'waveforms.npz', **arrays)
        (root / 'waveforms.json').write_text(json.dumps(meta, indent=2))
        r['captured'] = True
    else:
        r['captured'] = False
except Exception as e:
    r['error'] = repr(e)
    raise
finally:
    if 'startup' in globals() and not startup.done():
        try:
            startup.result(timeout=40)
        except Exception as e:
            r['startup_error'] = repr(e)
    if actuated:
        try:
            p.command([f'write32 0x{p.addresses["run"]:x} 0', f'write32 0x{base.BDTR:x} 0x02002054'])
        except Exception as e:
            r['emergency_stop_error'] = repr(e)
    s.write('ACQUIRE:STATE STOP')
    s.close()
    (root / 'summary.json').write_text(json.dumps(r, indent=2))
    print(root, flush=True)
