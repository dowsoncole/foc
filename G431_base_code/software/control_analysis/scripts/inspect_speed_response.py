"""Read-only, non-halting speed-response preflight. Never resets or enables PWM."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

from measure_current_response import DEBUG, PACK
from verify_encoder_artifacts import load_image


def inspect(axf, output, probe):
    output.mkdir(parents=True, exist_ok=False)
    expressions = {
        'user': '&g_user_control', 'run': '&g_speed_run',
        'test': '&g_speed_test', 'runtime': '&g_global_mode_control.runtime',
        'controller_fault': '&g_foc_controller.fault_flags',
        'encoder': '&g_encoder_snapshot', 'diagnostics': '&g_encoder_diagnostics',
        'capture': '&g_encoder_fault_capture',
        'calibration': '&g_encoder_calibration',
        'position': '&g_output_position', 'current_test': '&g_current_loop_test',
        'guard': '&g_speed_run_guard', 'clock': '&SystemCoreClock',
        'feedback_fault': '&g_speed_test.feedback_fault',
        'encoder_calibration_ok': '&g_speed_test.calibration_ok',
        'active': '&g_speed_test.active',
    }
    for name in ('user', 'run', 'test', 'runtime', 'encoder', 'diagnostics',
                 'capture', 'calibration', 'position', 'current_test', 'guard'):
        expressions[name + '_size'] = 'sizeof(' + expressions[name][1:] + ')'
    command = [str(DEBUG/'gdb/bin/arm-none-eabi-gdb.exe'), '-batch', str(axf)]
    for expression in expressions.values():
        command += ['-ex', 'p/x ' + expression]
    result = subprocess.run(command, capture_output=True, text=True, timeout=25)
    values = re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$', result.stdout, re.M)
    if result.returncode or len(values) != len(expressions):
        raise RuntimeError(result.stdout + result.stderr)
    addresses = dict(zip(expressions, (int(x, 16) for x in values)))
    (output/'symbols.json').write_text(json.dumps(addresses, indent=2))
    image = load_image(axf)
    first, end = min(image), max(image)+1
    if first != 0x08000000 or len(image) != end-first:
        raise RuntimeError('Unexpected Flash image layout')
    lengths = {name: addresses[name+'_size'] for name in expressions if name+'_size' in addresses}
    lengths.update(controller_fault=4, clock=4, feedback_fault=8,
                   encoder_calibration_ok=4, active=48)
    commands = ['status']
    for name, size in lengths.items():
        commands.append(f'savemem {addresses[name]:#x} {size} {(output/(name+".bin")).as_posix()}')
    commands += [f'savemem 0x40012c44 4 {(output/"bdtr.bin").as_posix()}',
                 f'savemem {first:#x} {end-first} {(output/"flash.bin").as_posix()}', 'status']
    result = subprocess.run([str(DEBUG/'pyocd/pyocd.exe'), 'commander', '--no-config',
        '--pack', PACK, '-t', 'stm32g431vbtx', '-u', probe, '-M', 'attach', '-W',
        '-f', '100000', '-c', '; '.join(commands)], capture_output=True, text=True, timeout=60)
    (output/'probe.log').write_text(result.stdout+result.stderr, encoding='utf-8')
    if result.returncode or re.search(r'Error|No connected|No available', result.stdout+result.stderr):
        raise RuntimeError(result.stdout+result.stderr)
    blocks = {name: (output/(name+'.bin')).read_bytes() for name in lengths}
    def word(name, offset=0): return struct.unpack_from('<I', blocks[name], offset)[0]
    def floats(name, offset, count): return struct.unpack_from('<'+'f'*count, blocks[name], offset)
    actual = (output/'flash.bin').read_bytes()
    expected = bytes(image[x] for x in range(first, end))
    # Some probe backends repeat a word on large cross-page reads. Keep the
    # original evidence and independently reread short blocks; never accept a
    # mismatched image merely because its vector table or symbols look right.
    if actual != expected:
        chunks=output/'flash_chunks'
        chunks.mkdir()
        offsets=list(range(0,len(expected),256))
        for start in range(0,len(offsets),32):
            commands=[f'savemem {first+i:#x} {min(256,len(expected)-i)} '
                      f'{(chunks/str(i)).as_posix()}' for i in offsets[start:start+32]]
            retry=subprocess.run([str(DEBUG/'pyocd/pyocd.exe'),'commander','--no-config',
                '--pack',PACK,'-t','stm32g431vbtx','-u',probe,'-M','attach','-W',
                '-f','100000','-c','; '.join(commands)],capture_output=True,text=True,timeout=35)
            (output/f'flash_retry_{start}.log').write_text(retry.stdout+retry.stderr)
            if retry.returncode or re.search(r'Error|No connected|No available',retry.stdout+retry.stderr):
                raise RuntimeError(retry.stdout+retry.stderr)
            print('Flash chunks',min(start+32,len(offsets)), '/',len(offsets),flush=True)
        rechecked=output/'flash_rechecked'
        rechecked.mkdir()
        verified=[]
        for i in offsets:
            data=(chunks/str(i)).read_bytes()
            want=expected[i:i+256]
            if data != want:
                retry=subprocess.run([str(DEBUG/'pyocd/pyocd.exe'),'commander','--no-config',
                    '--pack',PACK,'-t','stm32g431vbtx','-u',probe,'-M','attach','-W',
                    '-f','100000','-c',f'savemem {first+i:#x} {len(want)} '
                    f'{(rechecked/str(i)).as_posix()}'],capture_output=True,text=True,timeout=15)
                (rechecked/f'{i}.log').write_text(retry.stdout+retry.stderr)
                if retry.returncode: raise RuntimeError(retry.stdout+retry.stderr)
                data=(rechecked/str(i)).read_bytes()
                print('Independent recheck',hex(first+i),'match',data==want,flush=True)
            verified.append(data)
        actual=b''.join(verified)
        (output/'flash_short_reads.bin').write_bytes(actual)
    state = dict(axf=str(axf), axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
        flash_matches=actual==expected, flash_bytes=len(actual), probe_id=probe,
        bdtr=hex(struct.unpack('<I',(output/'bdtr.bin').read_bytes())[0]),
        user_enable=word('user'), user_mode=word('user',4),
        user_status=struct.unpack_from('<3I',blocks['user'],76),
        user_speed=floats('user',52,5), run_enable=word('run'),
        run_telemetry=floats('run',4,9), run_state=word('run',40), run_fault=word('run',44),
        start_block=word('run',56), runtime_bytes=list(blocks['runtime'][:6]),
        runtime_fault=word('runtime',8), controller_fault=word('controller_fault'),
        test_state=word('test',4), test_fault=word('test',8),
        feedback_fault=word('feedback_fault'), feedback_age_cycles=word('feedback_fault',4),
        diagnostics=struct.unpack('<'+'I'*(len(blocks['diagnostics'])//4),blocks['diagnostics']),
        capture_count=word('capture'), clock=word('clock'),
        encoder_valid=list(blocks['encoder'][40:44]), encoder_flags=word('encoder',36),
        calibration_ok=word('encoder_calibration_ok'))
    (output/'summary.json').write_text(json.dumps(state,indent=2))
    print(json.dumps(state,indent=2))
    if not state['flash_matches']:
        raise RuntimeError('Firmware does not match AXF. No RAM writes are permitted.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--axf',type=Path,default=Path('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf'))
    parser.add_argument('--probe',required=True)
    args=parser.parse_args()
    inspect(args.axf.resolve(),args.output.resolve(),args.probe)
