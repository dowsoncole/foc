"""Offline correlation of decoded SPI frames and a confirmed FOC probe channel.

Re-decodes the raw capture to avoid accidentally correlating stale frame files.
All timestamps are integer ns. A probe pulse is the registered callback window,
NOT the ADC hardware trigger or the whole ADC ISR. No hardware access.
"""
import argparse
from bisect import bisect_right
from collections import Counter
from contextlib import closing
import csv
import hashlib
import json
from pathlib import Path

from analyze_spi_logic import analyze_csv, read_events
from audit_encoder_transactions import audit, classify, distribution


def read_pulses(source, probe_channel, channels):
    # Reuse the raw reader's timestamp merging and malformed-input checks.
    # Only probe and NSS levels are interpreted; SCK/MOSI fill its four inputs.
    if probe_channel in channels:
        raise ValueError('probe channel must be distinct from SPI channels')
    events = read_events(source, (probe_channel, channels[2], channels[0], channels[3]))
    pulses, previous, active = [], None, None
    start = end = None
    # Explicitly close the generator on validation errors too (Windows locks).
    with closing(events):
        for event in events:
            if event.zero_width:
                raise ValueError('ambiguous same-timestamp transitions in timing capture')
            probe = event.levels[0]
            if probe is None:
                raise ValueError('unknown probe level; cannot time callback windows')
            if previous is None:
                start = event.ns
                if probe:
                    active = {'rise_ns': None, 'fall_ns': None, 'width_ns': None}
            elif probe != previous:
                if probe:
                    active = {'rise_ns': event.ns, 'fall_ns': None, 'width_ns': None}
                else:
                    active['fall_ns'] = event.ns
                    if active['rise_ns'] is not None:
                        active['width_ns'] = event.ns - active['rise_ns']
                    pulses.append(active)
                    active = None
            previous, end = probe, event.ns
    if start is None:
        raise ValueError('no digital events')
    if active is not None:
        pulses.append(active)
    return pulses, start, end


def correlate(frames, pulses, capture_end):
    starts = [int(row['start_ns']) for row in frames]
    if any(b <= a for a, b in zip(starts, starts[1:])):
        raise ValueError('SPI frame starts must increase strictly')
    matches, unmatched = [], []
    for pulse in pulses:
        rise = pulse['rise_ns']
        if rise is None:
            unmatched.append(dict(pulse, reason='partial_start'))
            continue
        i = bisect_right(starts, rise) - 1
        if i < 0:
            unmatched.append(dict(pulse, reason='no_preceding_frame'))
            continue
        row = frames[i]
        end = int(row['end_ns'])
        if rise <= end or classify(row) == 'partial':
            unmatched.append(dict(pulse, reason='overlap_or_partial_frame'))
            continue
        next_start = starts[i + 1] if i + 1 < len(starts) else capture_end
        fall = pulse['fall_ns'] if pulse['fall_ns'] is not None else capture_end
        matches.append(dict(pulse, frame_index=int(row['frame_index']),
                            category=classify(row), tx_hex=row['tx_hex'],
                            cs_start_ns=starts[i], cs_end_ns=end,
                            cs_duration_ns=end-starts[i],
                            cs_assert_to_rise_ns=rise-starts[i],
                            cs_release_to_rise_ns=rise-end,
                            overlaps_next_spi=fall > next_start))
    counts = Counter(m['frame_index'] for m in matches)
    # Multiple callback edges in one SPI slot are not a one-to-one timing proof.
    unique = [m for m in matches if counts[m['frame_index']] == 1]
    groups = {}
    fields = ('cs_duration_ns', 'cs_assert_to_rise_ns', 'cs_release_to_rise_ns', 'width_ns')
    for command in sorted({m['tx_hex'] for m in unique}):
        group = [m for m in unique if m['tx_hex'] == command]
        groups[command] = {key: distribution([m[key] for m in group if m[key] is not None])
                           for key in fields}
    by_frame = {m['frame_index']: m for m in unique}
    shifts = []
    for m in unique:
        if m['category'] != 'zero_read':
            continue
        left = by_frame.get(m['frame_index']-1)
        right = by_frame.get(m['frame_index']+1)
        if left and right and left['category'] == right['category'] == 'angle':
            # Equal-slot interpolation, not a measurement of ADC trigger delay.
            shifts.append(dict(frame_index=m['frame_index'],
                               early_vs_adjacent_angle_rises_ns=
                               (left['rise_ns']+right['rise_ns'])/2-m['rise_ns']))
    rises = [p['rise_ns'] for p in pulses if p['rise_ns'] is not None]
    return dict(pulses=pulses, associations=matches, unmatched_pulses=unmatched,
                frames_with_multiple_rises=[i for i, n in counts.items() if n > 1],
                frames_without_association=[int(r['frame_index']) for r in frames
                                            if int(r['frame_index']) not in counts],
                command_timing_ns=groups, zero_slot_early_shifts=shifts,
                rise_period_ns=distribution([b-a for a, b in zip(rises, rises[1:])]),
                cs_start_period_ns=distribution([b-a for a, b in zip(starts, starts[1:])]))


def analyze(source, prefix, probe_channel=5, channels=(2, 3, 1, 0)):
    output = Path(str(prefix)+'.timing.json')
    if output.exists():
        raise FileExistsError('refusing to overwrite timing evidence')
    pulses, start, end = read_pulses(source, probe_channel, channels)
    summary = analyze_csv(source, prefix, channels=channels)
    with Path(summary['frames_csv']).open(encoding='utf-8', newline='') as stream:
        frames = list(csv.DictReader(stream))
    result = correlate(frames, pulses, end)
    with source.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    result.update(source=str(source.resolve()), source_sha256=digest,
                  source_bytes=source.stat().st_size,
                  channels=dict(summary['channels'], foc_probe=probe_channel),
                  capture_start_ns=start, capture_end_ns=end, capture_span_ns=end-start,
                  hardware_accessed=False, spi_audit=audit(frames),
                  interpretation='Probe is callback window, not ADC trigger. CS release is not DMA IRQ exit. '
                  'Correlations do not alone establish running-mode timing or a causal repair.')
    with output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output-prefix', required=True, type=Path)
    parser.add_argument('--probe-channel', type=int, default=5)
    for name, value in zip(('mosi', 'miso', 'sck', 'nss'), (2, 3, 1, 0)):
        parser.add_argument('--'+name, type=int, default=value)
    args = parser.parse_args()
    result = analyze(args.source, args.output_prefix, args.probe_channel,
                     (args.mosi, args.miso, args.sck, args.nss))
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ('pulses', 'associations')}, indent=2))


if __name__ == '__main__':
    main()
