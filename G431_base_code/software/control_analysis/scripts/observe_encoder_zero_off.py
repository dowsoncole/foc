"""Finite OFF-only observation of a verified zero-trace firmware.

No target writes, resets, fault acknowledgements, sensor writes or actuation.
Stops on the first failed read or unhealthy sample; never retries a probe.
"""
import argparse
import json
import struct
import time
from pathlib import Path

from test_highspeed_closed_loop import ProbeRun, base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        raise ValueError('Observation must be finite: 1..60 seconds')
    base.EXPRESSIONS.update(sensor_zero_guard='&g_encoder_zero_guard',
                            size_sensor_zero_guard='sizeof(g_encoder_zero_guard)',
                            zero_fault_capture='&g_encoder_zero_fault_capture',
                            size_zero_fault_capture='sizeof(g_encoder_zero_fault_capture)')
    probe = ProbeRun(args.image.resolve(), args.directory.resolve())
    if (probe.addresses['size_sensor_zero_guard'],
            probe.addresses['size_zero_fault_capture']) != (32, 328):
        raise RuntimeError('Unexpected zero guard/capture ABI')
    probe.lengths.update(sensor_zero_guard=32, zero_fault_capture=328)
    result = dict(flash=probe.verify_flash(), actuated=False, samples=[])
    previous = baseline = None
    start = time.monotonic()
    for index in range(80):
        blocks, state = probe.snapshot('sample_%02d' % index, full=True)
        zero = dict(zip(('expected', 'observed', 'verified_mask', 'latched',
                         'checks', 'read_errors', 'mismatches', 'last_check_cycle'),
                        struct.unpack('<8I', blocks['sensor_zero_guard'])))
        trace_sequence, trace_count = struct.unpack_from('<2I', blocks['zero_fault_capture'])
        elapsed = time.monotonic() - start
        result['samples'].append(dict(elapsed_s=elapsed, state=state, zero=zero,
                                      trace_sequence=trace_sequence, trace_count=trace_count))
        (probe.directory/'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        base.require_ready(state, calibration_required=False)
        if (zero['expected'] or zero['observed'] or zero['latched'] or
                zero['read_errors'] or zero['mismatches'] or zero['verified_mask'] != 3 or
                trace_sequence or trace_count):
            raise RuntimeError('Zero fault evidence present; no clearing/retry')
        if baseline is None:
            baseline, first_time = state, time.monotonic()
        base.require_clean_counters(baseline, state)
        if previous is not None and state['encoder_sequence'] == previous['encoder_sequence']:
            raise RuntimeError('Encoder publication stopped')
        previous = state
        span = time.monotonic() - first_time
        print(json.dumps(dict(seconds=round(span, 3), crc=state['crc_errors'],
                              step=state['step_errors'], zero_mismatches=zero['mismatches'],
                              checks=zero['checks'], sequence=state['encoder_sequence'])), flush=True)
        if span >= args.seconds:
            print('PASS OFF-only observation:', probe.directory, flush=True)
            return
        time.sleep(1)
    raise RuntimeError('Observation did not complete within sample bound')


if __name__ == '__main__':
    main()
