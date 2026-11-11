"""Read-only MPT communication check. Requires the supplied AXF already flashed.
No flash/reset/halt, motor start, encoder reset, or commissioning unlock.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time
from mt6701_probe import Probe, base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=10)
    args = parser.parse_args()
    if not 2 <= args.samples <= 60:
        parser.error('--samples must be 2..60')
    directory = Path('software/control_analysis/results') / time.strftime('mpt_comm_%Y%m%d_%H%M%S')
    p = Probe(args.image, directory)
    result = {'hardware_verified': False, 'samples': []}
    try:
        _, before = p.snapshot('off')
        base.require_off(before)
        result['flash'] = p.verify_flash()
        symbols = subprocess.run([str(base.GDB), '-batch', str(args.image),
                                  '-ex', 'p/x &g_mpt_t485', '-ex', 'p/x sizeof(g_mpt_t485)'],
                                 capture_output=True, text=True, check=True, timeout=30)
        values = re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$', symbols.stdout, re.M)
        if len(values) != 2:
            raise RuntimeError(symbols.stdout + symbols.stderr)
        address, size = map(lambda x: int(x, 16), values)
        if size != 88:
            raise RuntimeError('Unexpected MPT diagnostic ABI')
        names = 'initialized enabled requests good_frames crc_errors format_errors status_errors timeouts uart_errors consecutive_good valid last_good_ms sequence angle_raw turns_raw status alarm'.split()
        for i in range(args.samples):
            raw = directory / f'mpt_{i:03}.bin'
            p.command([f'savemem 0x{address:x} {size} {raw.as_posix()}'])
            data = raw.read_bytes()
            sample = dict(zip(names, struct.unpack_from('<17I', data)))
            sample['angle_deg'] = struct.unpack_from('<f', data, 68)[0]
            sample['frame_hex'] = data[72:83].hex(' ')
            _, state = p.snapshot(f'off_{i:03}')
            base.require_off(state)
            result['samples'].append(sample)
            if not sample['initialized'] or not sample['valid']:
                raise RuntimeError('MPT has not qualified 10 consecutive good frames')
            if i:
                first = result['samples'][0]
                if sample['good_frames'] <= result['samples'][-2]['good_frames']:
                    raise RuntimeError('MPT samples stopped')
                for key in names[4:9]:
                    if sample[key] != first[key]:
                        raise RuntimeError(f'{key} increased')
            print(json.dumps(sample), flush=True)
            time.sleep(0.5)
        result['hardware_verified'] = True
        result['scope'] = 'Static communication only; motion direction, timing under PWM and position accuracy unverified'
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        (directory / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(directory)


if __name__ == '__main__':
    main()
