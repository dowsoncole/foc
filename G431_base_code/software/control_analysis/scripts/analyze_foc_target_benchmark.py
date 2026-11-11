"""Decode the OFF-only Cortex-M4 DWT comparison; writes no target or files."""
import argparse
import json
from pathlib import Path
import struct


def decode(path):
    data = Path(path).read_bytes()
    if len(data) != 312:
        raise ValueError('Unexpected benchmark ABI size')
    words = struct.unpack('<78I', data)
    version, state, hz, mismatches, count, iterations = words[:6]
    if version != 1 or state != 2 or hz != 168000000 or count != 6:
        raise ValueError('Benchmark incomplete or unexpected clock/version')
    names = ['linear_unsaturated', 'linear_saturated', 'ovm1', 'ovm2',
             'negative_angle', 'invalid_angle']
    cases = []
    for index, name in enumerate(names):
        w = words[6 + index * 12:18 + index * 12]
        if w[0] != iterations or w[4] != iterations:
            raise ValueError('Incorrect timed iteration count')
        before = w[3] / w[0]
        after = w[7] / w[4]
        cases.append(dict(
            name=name, timed_count=w[0],
            before_cycles=dict(minimum=w[1], maximum=w[2], mean=before),
            after_cycles=dict(minimum=w[5], maximum=w[6], mean=after),
            before_mean_us=before * 1e6 / hz,
            after_mean_us=after * 1e6 / hz,
            saved_mean_us=(before - after) * 1e6 / hz,
            reduction_percent=(before - after) * 100 / before,
            mismatches=w[8], valid_count=w[9], saturated_count=w[10]))
    if mismatches or any(case['mismatches'] for case in cases):
        raise ValueError('Target algorithm state differs from baseline')
    return dict(version=version, state='complete', clock_hz=hz,
                scope='algorithm_only_not_complete_ADC_IRQ',
                mismatch_count=mismatches, cases=cases)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    print(json.dumps(decode(args.capture), indent=2))
