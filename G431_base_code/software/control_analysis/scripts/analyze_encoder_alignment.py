"""Offline analysis of bounded kind-1 alignment captures; never accesses hardware.

The firmware generates a fixed field for the first quarter, sweeps to angle_end
over the middle half, then holds. Id/Iq are in that commanded frame, not an
independent measurement of rotor flux alignment. No offset is applied by this tool.
"""
import argparse
import json
import math
from pathlib import Path
import struct

TAU = 2 * math.pi


def wrap_error(angle):
    return (angle + math.pi) % TAU - math.pi


def circular_stats(angles):
    if not angles or any(not math.isfinite(a) for a in angles):
        raise ValueError('Finite nonempty angle samples required')
    sn = sum(math.sin(a) for a in angles)
    cs = sum(math.cos(a) for a in angles)
    if math.hypot(sn, cs) / len(angles) < 0.9:
        raise ValueError('Angles do not define a unique, concentrated offset')
    mean = math.atan2(sn, cs) % TAU
    deviations = [wrap_error(a - mean) for a in angles]
    return mean, math.sqrt(sum(x*x for x in deviations) / len(deviations))


def field_angle(time_s, duration_s, angle_end, angle_start=0):
    p = max(0.0, min(1.0, (time_s / duration_s - 0.25) * 2))
    return angle_start + p*(angle_end-angle_start)


def alignment_current_ceiling(kind, duration, amplitude, alignment, version=None):
    if 'requested_rms_a' not in alignment:
        return 3.0
    rms = alignment['requested_rms_a']
    if (kind != 1 or version != 2 or alignment.get('current_contract_version') != 2
            or not math.isfinite(rms) or not 0 < rms <= 5 or duration != 5
            or not math.isclose(amplitude, rms*math.sqrt(2), abs_tol=1e-6)):
        raise ValueError('Invalid RMS alignment contract')
    return 5*math.sqrt(2)+1e-6


def analyze_run(directory):
    directory = Path(directory)
    metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
    summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
    alignment = metadata['alignment']
    data = (directory / 'stopped/test.bin').read_bytes()
    if len(data) != 4416:
        raise ValueError('Unsupported capture ABI')
    count = struct.unpack_from('<I', data, 12)[0]
    kind, duration, amplitude, start, _, end = struct.unpack_from('<I5f', data, 64)
    direction, poles, stored = struct.unpack_from('<iIf', data, 120)
    version = None
    if 'requested_rms_a' in alignment:
        version = struct.unpack('<I', (directory/'before/alignment_profile_version.bin').read_bytes())[0]
    voltage_mode = kind == 10
    if voltage_mode:
        voltage_version = struct.unpack('<I', (directory/'before/voltage_alignment_profile_version.bin').read_bytes())[0]
        if (alignment.get('method') != 'voltage' or voltage_version != 1
                or alignment.get('voltage_contract_version') != 1 or duration != 5
                or not math.isfinite(amplitude) or not 0 < amplitude <= 1
                or not math.isclose(amplitude, alignment['voltage_v'], abs_tol=1e-6)
                or 'current_a' in alignment or 'requested_rms_a' in alignment):
            raise ValueError('Invalid voltage alignment contract or mixed units')
        current_ceiling = 1.0  # Config amplitude is volts for kind 10, not amperes.
    else:
        current_ceiling = alignment_current_ceiling(kind, duration, amplitude, alignment, version)
    if (kind not in (1, 9, 10) or not 1 <= count <= 128 or poles != 2 or direction not in (-1, 1)
            or not 0.05 <= duration <= (12 if kind == 9 else 5) or not 0 < amplitude <= current_ceiling
            or not math.isclose(end, alignment['angle_end_rad'], abs_tol=1e-6)
            or not math.isclose(duration, alignment['duration_s'], abs_tol=1e-6)
            or not math.isclose(amplitude, alignment['voltage_v' if voltage_mode else 'current_a'], abs_tol=1e-6)):
        raise ValueError('Unsupported alignment configuration')
    if kind in (1, 10) and not math.isclose(start, alignment.get('start_field_rad', 0), abs_tol=1e-6):
        raise ValueError('Start-field metadata does not match active configuration')
    if kind == 9 and (duration != 12 or not math.isclose(end, 4*math.pi, abs_tol=1e-6)
                      or amplitude != 3):
        raise ValueError('Calibration fit requires the reviewed 3 A / 12 s sweep')
    armed, after = summary['armed'], summary['after']
    if (after['speed_test_state'] != 3 or after['speed_test_fault']
            or after['runtime_fault'] or after['controller_fault'] or after['moe']):
        raise ValueError('Faulted or energized capture cannot establish calibration')
    for key in ('crc_errors', 'step_errors', 'alarms', 'start_errors', 'transfer_errors'):
        if armed[key] != after[key]:
            raise ValueError(key + ' increased during alignment')
    # Old metadata omitted this field; the original diagnostics retain it.
    diag_before = (directory / 'armed/diagnostics.bin').read_bytes()
    diag_after = (directory / 'stopped/diagnostics.bin').read_bytes()
    if struct.unpack_from('<I', diag_before, 84)[0] != struct.unpack_from('<I', diag_after, 84)[0]:
        raise ValueError('Current-loop deadline overrun during alignment')
    rows = [struct.unpack_from('<8f', data, 320 + i*32) for i in range(count)]
    if (any(not all(math.isfinite(v) for v in row) for row in rows)
            or any(not 0 <= row[0] < duration for row in rows)
            or any(b[0] <= a[0] for a, b in zip(rows, rows[1:]))):
        raise ValueError('Nonfinite or unordered records')
    if kind == 9:
        for row in rows:
            expected = max(0, min(1, (row[0]-3)/3 if row[0] < 6 else (9-row[0])/3))*end
            if abs(row[1]-expected) > .002:
                raise ValueError('Recorded field does not match the reviewed sweep profile')
    elif 'start_field_rad' in alignment:
        if any(abs(r[1]-field_angle(r[0], duration, end, start)) > .002 for r in rows):
            raise ValueError('Recorded field does not match the near-rotor start profile')
    tail = [r for r in rows if duration - 0.5 < r[0] < duration]
    if len(tail) < 5:
        raise ValueError('Insufficient final hold samples')
    final_field = 0.0 if kind == 9 else end
    zero, std = circular_stats([final_field - poles*r[6] for r in tail])
    mechanical, mechanical_std = circular_stats([r[6] for r in tail])
    # Skip the low-current ramp and the field-transition endpoints. Moving
    # errors include friction/dynamic lag and must NOT be called sensor INL.
    moving = [r for r in rows if max(2.1, duration*0.30) < r[0] < duration*0.73] if kind in (1, 10) else []
    bins = []
    for index in range(12):
        selected = [r for r in moving if int((r[6] % TAU) / TAU * 12) == index]
        if selected:
            z, spread = circular_stats([
                field_angle(r[0], duration, end, start) - poles*r[6] for r in selected])
            bins.append(dict(mechanical_bin_deg=index*30, count=len(selected),
                             apparent_offset_deg=math.degrees(z),
                             spread_deg=math.degrees(spread)))
    result = dict(run=directory.name, kind=kind, direction=direction, pole_pairs=poles,
                  commanded_id_amplitude_a=amplitude, equivalent_three_phase_rms_a=amplitude/math.sqrt(2),
                  field_end_deg=math.degrees(end), stored_offset_rad=stored,
                  tail_count=len(tail), estimated_offset_rad=zero,
                  estimated_offset_deg=math.degrees(zero),
                  tail_std_electrical_deg=math.degrees(std),
                  tail_mechanical_deg=math.degrees(mechanical),
                  tail_std_mechanical_deg=math.degrees(mechanical_std),
                  mean_id_a=sum(r[3] for r in tail)/len(tail),
                  mean_iq_command_frame_a=sum(r[4] for r in tail)/len(tail),
                  peak_tail_id_error_a=max(abs(r[3]-amplitude) for r in tail),
                  peak_tail_iq_command_frame_a=max(abs(r[4]) for r in tail),
                  rms_tail_id_error_a=math.sqrt(sum((r[3]-amplitude)**2 for r in tail)/len(tail)),
                  rms_tail_iq_command_frame_a=math.sqrt(sum(r[4]**2 for r in tail)/len(tail)),
                  peak_tail_speed_rad_s=max(abs(r[2]) for r in tail),
                  moving_bins=bins)
    if kind == 9:
        result['sweep'] = evaluate_sweep(rows, poles)
    if voltage_mode:
        for key in ('commanded_id_amplitude_a', 'equivalent_three_phase_rms_a',
                    'peak_tail_id_error_a', 'rms_tail_id_error_a'):
            del result[key]
        result.update(method='voltage', commanded_ud_v=amplitude,
            mean_ud_v=sum(r[5] for r in tail)/len(tail),
            mean_uq_v=sum(r[7] for r in tail)/len(tail),
            rms_tail_ud_error_v=math.sqrt(sum((r[5]-amplitude)**2 for r in tail)/len(tail)),
            rms_tail_uq_v=math.sqrt(sum(r[7]**2 for r in tail)/len(tail)),
            measured_tail_rms_a=math.sqrt(sum((r[3]**2+r[4]**2)/2 for r in tail)/len(tail)))
    return result


def evaluate_static_holds(runs):
    """Commissioning gates for settled 3 A holds, not moving-sweep acceptance.

    This independent method excludes motion lag but still rejects position bias,
    poor settling and approach-dependent offsets. It never writes calibration.
    Current error is RMS over the settled records (5% of the 3 A command), not
    an instantaneous peak gate. Peaks remain reported. This distinguishes
    sampled current ripple from a persistent field bias; it is not a change to
    any firmware current protection or the existing moving-sweep criteria.
    Callers must first validate the captures with analyze_run, and verify the
    3 A/5 s metadata and zero-guard/secondary-bus snapshots separately.
    """
    rejects = []
    if len(runs) < 7 or any(r['kind'] != 1 or r['pole_pairs'] != 2 or
                            r['direction'] != -1 for r in runs):
        return dict(accepted=False, rejections=['Insufficient or incompatible static captures'])
    phases = set()
    for r in runs:
        angle = r['field_end_deg']
        quarter = round(angle / 90)
        if abs(angle - quarter*90) > .001:
            rejects.append('Non-quarter-turn field endpoint')
        phases.add(quarter % 4)
        for key, limit in (('tail_std_electrical_deg', 1.0),
                           ('rms_tail_id_error_a', .15),
                           ('rms_tail_iq_command_frame_a', .15),
                           ('peak_tail_speed_rad_s', .2)):
            if not math.isfinite(r[key]) or r[key] > limit:
                rejects.append(r['run'] + ': ' + key + ' exceeds ' + str(limit))
    if phases != {0, 1, 2, 3}:
        rejects.append('Missing static electrical quadrants')
    endpoints = [r['field_end_deg'] for r in runs]
    if not all(any(abs(x - wanted) < .001 for x in endpoints) for wanted in (-360, 360)):
        rejects.append('Missing positive/negative same-field approaches')
    direct = [r for r in runs if abs(r['field_end_deg']) < .001]
    if len(direct) < 2:
        rejects.append('Missing repeated zero-field holds')
    offsets = [r['estimated_offset_rad'] for r in runs]
    try:
        offset, _ = circular_stats(offsets)
    except ValueError as exc:
        return dict(accepted=False, rejections=rejects + [str(exc)])
    residual = max(abs(math.degrees(wrap_error(a-offset))) for a in offsets)
    if residual > 3:
        rejects.append('Static position residual exceeds 3 electrical degrees')
    same_field = [r['estimated_offset_rad'] for r in runs
                  if abs(wrap_error(math.radians(r['field_end_deg']))) < 1e-5]
    separation = max(abs(math.degrees(wrap_error(a-b))) for a in same_field for b in same_field)
    if separation > 2:
        rejects.append('Repeated/opposite-approach zero separation exceeds 2 electrical degrees')
    return dict(accepted=not rejects, rejections=rejects, offset_rad=offset,
                offset_deg=math.degrees(offset), max_position_residual_deg=residual,
                max_zero_separation_deg=separation)


def evaluate_sweep(rows, poles=2):
    """Symmetric forward/reverse fit, with explicit rejection of bad calibration.

    Bounds are commissioning acceptance gates, not a sensor accuracy rating.
    Never apply an average if hysteresis/nonuniformity/coverage checks fail.
    """
    forward = [r for r in rows if 3.15 < r[0] < 5.85]
    reverse = [r for r in rows if 6.15 < r[0] < 8.85]
    errors = []
    paired = []
    rejections = []
    for i in range(8):
        pair = []
        for direction_rows in (forward, reverse):
            selected = [r for r in direction_rows if int(r[1] / (2*TAU) * 8) == i]
            if len(selected) < 2:
                rejections.append('Insufficient bidirectional field coverage in bin %d' % i)
                continue
            mean, std = circular_stats([r[1] - poles*r[6] for r in selected])
            if math.degrees(std) > 3:
                rejections.append('Moving spread exceeds 3 electrical degrees in bin %d' % i)
            pair.append(mean)
        if len(pair) == 2:
            mean, _ = circular_stats(pair)
            hysteresis = math.degrees(wrap_error(pair[0] - pair[1]))
            paired.append(dict(bin=i, offset_rad=mean, hysteresis_deg=hysteresis))
            errors.append(mean)
            if abs(hysteresis) > 6:
                rejections.append('Forward/reverse separation exceeds 6 electrical degrees in bin %d' % i)
    if len(errors) != 8:
        return dict(accepted=False, rejections=rejections, bins=paired)
    offset, _ = circular_stats(errors)
    residual = max(abs(math.degrees(wrap_error(x-offset))) for x in errors)
    if residual > 3:
        rejections.append('Position-dependent residual exceeds 3 electrical degrees')
    moving = forward + reverse
    if any(abs(r[3]-3) > .15 or abs(r[4]) > .15 for r in moving):
        rejections.append('Command-frame current tracking is outside 0.15 A')
    if any(abs(r[2]) > 5 for r in moving):
        rejections.append('Sweep speed exceeds diagnostic expectation of 5 rad/s')
    return dict(accepted=not rejections, offset_rad=offset, offset_deg=math.degrees(offset),
                max_position_residual_deg=residual, rejections=rejections, bins=paired)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = dict(note='Estimates, not proof of physical zero drift or encoder INL.',
                  runs=[analyze_run(p) for p in args.directories])
    rendered = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        with args.output.open('x', encoding='utf-8') as handle:
            handle.write(rendered + '\n')
    print(rendered)
    if any(r.get('sweep', {}).get('accepted') is False for r in result['runs']):
        raise SystemExit(2)  # Automation must not mistake a diagnostic fit for a valid calibration.


if __name__ == '__main__':
    main()
