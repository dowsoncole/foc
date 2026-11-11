"""Offline audit of angle bursts AND 24-clock ZERO_POS register reads.

Consumes analyze_spi_logic.py output, not hardware. The angle-only decoder's
24-clock exclusions are not bus faults. Unknown/ambiguous frames remain visible.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics


def classify(row):
    if row['partial_start'] == '1' or row['partial_end'] == '1':
        return 'partial'
    if row['included'] == '1':
        return 'angle'
    allowed = {'clock_count_not_48', 'not_expected_a003_angle_burst'}
    if (row['rising_clocks'] == row['falling_clocks'] == '24'
            and row['tx_hex'] in ('30 09 00', '30 0A 00')
            and set(filter(None, row['excluded_reasons'].split(';'))) <= allowed
            and len(bytes.fromhex(row['rx_hex'])) == 3):
        return 'zero_read'
    return 'unrecognized'


def distribution(values):
    return dict(count=len(values), min=min(values), median=statistics.median(values),
                max=max(values)) if values else None


def audit(rows):
    categories, commands, angles, responses = (Counter() for _ in range(4))
    registers = {'30 09 00': Counter(), '30 0A 00': Counter()}
    unknown = []
    timings = {k: [] for k in ('cs_setup_ns', 'cs_hold_ns')}
    bad_crc = alarms = ambiguities = zero_width = 0
    for r in rows:
        category = classify(r)
        categories[category] += 1
        if category == 'partial':
            continue
        commands[r['tx_hex']] += 1
        ambiguities += int(r['sample_edge_ambiguities'])
        zero_width += int(r['zero_width_events'])
        for k in timings:
            if r[k]:
                timings[k].append(int(r[k]))
        if category == 'angle':
            bad_crc += r['crc_ok'] == '0'
            alarms += r['sensor_alarm_valid'] == '1'
            responses[r['rx_hex']] += 1
            if r['crc_ok'] == '1':
                angles[r['angle_raw']] += 1
        elif category == 'zero_read':
            registers[r['tx_hex']][r['rx_hex']] += 1
        elif len(unknown) < 16:
            unknown.append(r)
    zero = None
    # Combine only constant, correctly framed repeated H/L responses; these
    # register reads themselves have no CRC, and are not simultaneous samples.
    if all(len(v) == 1 for v in registers.values()):
        hi = bytes.fromhex(next(iter(registers['30 09 00'])))
        lo = bytes.fromhex(next(iter(registers['30 0A 00'])))
        if hi[:2] == lo[:2] == b'\xff\xff':
            zero = (hi[2] << 4) | (lo[2] >> 4)
    return dict(categories=dict(categories), command_histogram=dict(commands),
                angle_responses=dict(responses), valid_raw_angles=dict(angles),
                crc_errors=bad_crc, crc_valid_sensor_alarms=alarms,
                zero_register_responses={k: dict(v) for k, v in registers.items()},
                constant_zero_pos=zero,
                constant_zero_mechanical_deg=zero*360/4096 if zero is not None else None,
                unknown_examples=unknown, sample_edge_ambiguities=ambiguities,
                zero_width_events=zero_width,
                timing_ns={k: distribution(v) for k, v in timings.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--last-good-raw', type=int)
    args = parser.parse_args()
    summary = json.loads(Path(str(args.prefix)+'.summary.json').read_text(encoding='utf-8'))
    source = Path(summary['source'])
    with source.open('rb') as f:
        digest = hashlib.file_digest(f, 'sha256').hexdigest()
    with Path(summary['frames_csv']).open(encoding='utf-8', newline='') as f:
        result = audit(csv.DictReader(f))
    result.update(source=str(source), source_bytes=source.stat().st_size,
                  source_sha256=digest, channels=summary['channels'],
                  capture_span_ns=summary['capture_end_ns']-summary['capture_start_ns'],
                  hardware_accessed=False, source_modified=False)
    if args.last_good_raw is not None:
        if not 0 <= args.last_good_raw < 2**21:
            raise ValueError('Invalid 21-bit comparison reference')
        z = result['constant_zero_pos']
        if z is not None and len(result['valid_raw_angles']) == 1:
            raw = int(next(iter(result['valid_raw_angles'])))
            restored = (raw + z*512) % 2**21
            difference = (restored-args.last_good_raw+2**20) % 2**21-2**20
            result['reference_comparison'] = dict(
                last_good_raw=args.last_good_raw, captured_raw=raw,
                reconstructed_zero0_raw=restored, difference_counts=difference,
                difference_mechanical_deg=difference*360/2**21,
                note='Register-offset consistency, not independent proof of a locked rotor or trigger cause.')
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
