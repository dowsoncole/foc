"""Re-establish ADC offsets with encoder SPI active and motor output OFF.

No flash/reset, motor command, encoder calibration or fault acknowledgement.
--run permits two contiguous noise captures, one atomic U/V-offset write, and
an independent residual capture. All three captures are preserved. No retries.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import struct

from encoder_logic_capture import make_probe, base


def raw_capture(data, scale, expected_offsets):
    if len(data) != 4128:
        raise ValueError('Unexpected noise capture ABI')
    command, count, first, last, ou, ov, ratio, profile = struct.unpack_from('<8I', data)
    if (command != 3 or count != 1024 or (last-first) & 0xffffffff != 1023
            or (ou, ov) != expected_offsets or ratio != 1 or profile != 1):
        raise ValueError('Incomplete capture or changed ADC calibration/profile')
    rows = list(struct.iter_unpack('<HH', data[32:]))
    if any(not 1 < x < 4094 for row in rows for x in row):
        raise ValueError('ADC rail sample')
    means = [statistics.mean(row[k] for row in rows) for k in (0, 1)]
    stds = [statistics.pstdev(row[k] for row in rows)*scale for k in (0, 1)]
    if any(x > .2 for x in stds):
        raise ValueError('Excessive OFF noise; do not calibrate it away')
    return dict(mean_codes=means, std_a=stds, offsets=[ou, ov],
                mean_a=[(offset-mean)*scale for offset, mean in zip((ou, ov), means)])


def propose_offsets(a, b, old, scale):
    if not math.isfinite(scale) or not 0 < scale < .1:
        raise ValueError('Invalid ADC scale')
    if any(abs(x-y)*scale > .02 for x, y in zip(a['mean_codes'], b['mean_codes'])):
        raise ValueError('OFF baseline changed between independent windows')
    proposed = [round((x+y)/2) for x, y in zip(a['mean_codes'], b['mean_codes'])]
    if any(not 1 < x < 4094 or abs(x-o)*scale > .5 for x, o in zip(proposed, old)):
        raise ValueError('Offset correction exceeds bounded OFF recalibration envelope')
    return tuple(proposed)


def require_baseline(blocks, state):
    base.require_off(state)
    if (state['calibration'] != 3 or state['calibration_ok'] != 0
            or state['runtime_fault'] or state['controller_fault']
            or state['current_trip_count'] or not math.isfinite(state['mechanical_speed_rad_s'])
            or abs(state['mechanical_speed_rad_s']) >= .2):
        raise RuntimeError('Not a stationary, fault-free, uncalibrated OFF baseline')
    fields = struct.unpack('<10I8s', blocks['logic_trigger'])
    if fields[:6] != (1, 2, 0, 1, 0, 0) or blocks['paused'] != b'\x01':
        raise RuntimeError('Encoder SPI must remain healthy ARMED')


def current_scale(config, timer):
    shunt, gain, vref, full, ratio, rate = struct.unpack('<If4I', config)
    psc, arr, rcr = struct.unpack('<3I', timer)
    # Include the 10 kHz candidate and the earlier 16/20 kHz images; no
    # arbitrary rate acceptance and no mixed ADC/PWM clock contracts.
    expected_arr = {10000: 8400, 16000: 5250, 20000: 4199}.get(rate)
    if ((shunt, vref, full, ratio) != (7000, 3300, 4095, 1)
            or abs(gain-6.1) > 1e-5 or not math.isfinite(gain)
            or expected_arr is None or (psc, arr, rcr) != (0, expected_arr, 1)):
        raise RuntimeError('Unexpected current conversion or ADC/PWM timing parameters')
    return (vref/1000)/(full*(shunt/1e6)*gain*ratio)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    base.EXPRESSIONS.update(adc_noise='&g_adc_noise', size_adc_noise='sizeof(g_adc_noise)',
                            adc_offsets='&g_current_sampling.offset_u')
    p = make_probe(args.image, args.directory)
    if p.addresses['size_adc_noise'] != 4128 or p.addresses['adc_offsets'] % 4:
        raise RuntimeError('Unsupported ADC ABI')
    p.lengths.update(adc_noise=4128, adc_offsets=4)
    p.addresses['tim1_timebase'] = 0x40012c28  # PSC, ARR, RCR, read-only.
    p.lengths['tim1_timebase'] = 12
    report = p.verify_flash()
    blocks, initial = p.snapshot('before', full=True)
    require_baseline(blocks, initial)
    config = blocks['current_config']
    scale = current_scale(config, blocks['tim1_timebase'])
    old = struct.unpack('<2H', blocks['adc_offsets'])
    report.update(applied=False, old_offsets=old, scale_a_per_code=scale,
                  sample_rate_hz=base.u32(config, 20),
                  tim1_timebase=list(struct.unpack('<3I', blocks['tim1_timebase'])))
    output = p.directory/'summary.json'
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    if not args.run:
        print('Read-only OFF/SPI-active preflight passed; no writes')
        return

    def capture(name, expected):
        p.command([f'write32 0x{p.addresses["adc_noise"]:x} 1'])
        b, s = p.snapshot(name, full=True)
        require_baseline(b, s)
        base.require_clean_counters(initial, s)
        if (b['current_config'] != config or b['tim1_timebase'] != blocks['tim1_timebase']
                or struct.unpack('<2H', b['adc_offsets']) != expected):
            raise RuntimeError('Current scale/offsets changed externally')
        return raw_capture(b['adc_noise'], scale, expected)

    report['first'] = capture('first', old)
    report['second'] = capture('second', old)
    proposed = propose_offsets(report['first'], report['second'], old, scale)
    report['proposed_offsets'] = proposed
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    # Exactly two uint16_t offsets in one aligned atomic write; calibration
    # flags, motor zero, PI gains, protection limits and ADC timing stay intact.
    packed = struct.unpack('<I', struct.pack('<2H', *proposed))[0]
    p.command([f'write32 0x{p.addresses["adc_offsets"]:x} 0x{packed:x}'])
    report['applied'] = True
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    report['verification'] = capture('verification', proposed)
    report['verified'] = all(abs(x) <= .03 for x in report['verification']['mean_a'])
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['verified']:
        raise RuntimeError('New offset residual failed; leave OFF, no retry or motion')


if __name__ == '__main__':
    main()
