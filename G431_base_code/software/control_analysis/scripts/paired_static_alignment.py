"""Bounded, adjacent-position bidirectional static calibration diagnostics.

Does not flash, reset, clear a trigger, apply an offset, or start speed mode.
Each excitation uses the profile-2 trigger runner: 5 s, 20 rad/s guard.
Default is kind-10 Ud=1 V, Uq=0, no current PI. --rms-current explicitly
selects the legacy current method; never mix current and voltage contracts.
Default prints the schedule. --run explicitly performs the 19 bounded holds.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys

from analyze_encoder_alignment import analyze_run, circular_stats, wrap_error
from test_encoder_neutral_pwm import require_voltage_alignment


def schedule():
    return ([('anchor', 0, 0)] + [('forward', k % 8, 1) for k in range(1, 9)]
            + [('reverse', k % 8, -1) for k in range(7, -1, -1)]
            + [('return_neighbor', 7, -1), ('repeat_forward_zero', 0, 1)])


def point_rejections(point, anchor):
    r, rejects = point['analysis'], []
    if r['kind'] not in (1, 10) or (r['pole_pairs'], r['direction']) != (2, -1):
        rejects.append(r['run'] + ': incompatible capture')
    error = math.degrees(wrap_error(math.radians(r['tail_mechanical_deg'] - anchor - 45*point['bin'])))
    if not math.isfinite(error) or abs(error) > 10:
        rejects.append(r['run'] + ': wrong mechanical position/half-turn')
    limits = [('tail_std_electrical_deg', 1), ('peak_tail_speed_rad_s', .2)]
    if r['kind'] == 10:
        limits += [('rms_tail_ud_error_v', .01), ('rms_tail_uq_v', .01)]
        if not math.isfinite(r['measured_tail_rms_a']) or not .1 <= r['measured_tail_rms_a'] <= 6:
            rejects.append(r['run'] + ': measured excitation outside 0.1..6 A RMS envelope')
    else:
        limits += [('rms_tail_id_error_a', .15), ('rms_tail_iq_command_frame_a', .15)]
    for key, limit in limits:
        if not math.isfinite(r[key]) or r[key] > limit:
            rejects.append(r['run'] + ': ' + key + ' exceeds ' + str(limit))
    return rejects


def evaluate_pairs(points):
    rejects, pairs = [], []
    if [(p['role'], p['bin'], p['step_direction']) for p in points] != schedule():
        return dict(accepted=False, rejections=['Incomplete or reordered mechanical path'])
    if len({p['analysis']['kind'] for p in points}) != 1:
        return dict(accepted=False, rejections=['Mixed voltage/current calibration methods'])
    anchor = points[0]['analysis']['tail_mechanical_deg']
    for point in points:
        rejects.extend(point_rejections(point, anchor))
    for index in range(8):
        selected = [p['analysis'] for p in points if p['bin'] == index and p['role'] in ('forward', 'reverse')]
        a, b = (r['estimated_offset_rad'] for r in selected)
        mean, _ = circular_stats([a, b])
        separation = abs(math.degrees(wrap_error(a-b)))
        if separation > 6:
            rejects.append('Opposite-approach separation exceeds 6 electrical degrees in bin ' + str(index))
        pairs.append(dict(bin=index, offset_rad=mean, separation_deg=separation))
    mean, _ = circular_stats([p['offset_rad'] for p in pairs])
    residual = max(abs(math.degrees(wrap_error(p['offset_rad']-mean))) for p in pairs)
    if residual > 3:
        rejects.append('Paired position residual exceeds 3 electrical degrees')
    first = next(p['analysis']['estimated_offset_rad'] for p in points if p['role']=='forward' and p['bin']==0)
    last = points[-1]['analysis']['estimated_offset_rad']
    repeat = abs(math.degrees(wrap_error(first-last)))
    if repeat > 2:
        rejects.append('Same-position, same-approach zero repeat exceeds 2 electrical degrees')
    return dict(accepted=not rejects, rejections=rejects, offset_rad=mean,
                offset_deg=math.degrees(mean), max_position_residual_deg=residual,
                zero_repeat_deg=repeat, pairs=pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--rough-offset', required=True, type=float)
    excitation = parser.add_mutually_exclusive_group()
    excitation.add_argument('--rms-current', type=float)
    excitation.add_argument('--voltage', type=float, help='Ud volts, defaults to 1.0; Uq=0, no current PI')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch('[A-Za-z0-9_-]+', args.prefix) or not 0 <= args.rough_offset < 2*math.pi:
        raise ValueError('Invalid prefix/rough offset')
    if args.rms_current is not None and (not math.isfinite(args.rms_current) or not 0 < args.rms_current <= 5):
        raise ValueError('RMS current must be within (0,5] A')
    voltage = args.rms_current is None
    ud = 1.0 if args.voltage is None else args.voltage
    if voltage:
        require_voltage_alignment(ud)
    current_args = ['--alignment-voltage', str(ud)] if voltage else ['--alignment-rms-current', str(args.rms_current)]
    expected_id = None if voltage else args.rms_current*math.sqrt(2)
    if not args.run:
        print(json.dumps(dict(method='voltage' if voltage else 'current',
                              ud_v=ud if voltage else None, schedule=schedule())))
        return
    root = Path('software/control_analysis/results')
    output = root/(args.prefix+'.json')
    zero_directory = root/(args.prefix+'_adc_zero')
    if output.exists() or zero_directory.exists() or any((root/(args.prefix+'_%02d'%i)).exists() for i in range(19)):
        raise ValueError('Existing evidence; no implicit resume or overwrite')
    report = dict(applied=False, method='paired adjacent mechanical holds; unchanged 3-degree residual / 6-degree approach / 2-degree repeat gates', points=[])
    # WAIT firmware calibrates ADC before SPI starts. Re-establish its OFF
    # baseline with SPI ARMED before any alignment; never mask this as an
    # encoder-offset correction or permit an already-valid motor calibration.
    subprocess.run([sys.executable, str(Path(__file__).with_name('calibrate_adc_zero_off.py')),
                    '--image', str(args.image), '--directory', str(zero_directory), '--run'], check=True)
    zero_result = json.loads((zero_directory/'summary.json').read_text(encoding='utf-8'))
    if not zero_result.get('verified'):
        raise RuntimeError('SPI-active ADC zero not verified; no excitation')
    report['adc_zero_evidence'] = str(zero_directory)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for i, (role, index, direction) in enumerate(schedule()):
        tag = args.prefix+'_%02d'%i
        command = [sys.executable, str(Path(__file__).with_name('run_triggered_commission.py')),
                   'alignment', '--image', str(args.image), '--tag', tag,
                   *current_args, '--duration', '5',
                   '--alignment-start-offset', str(args.rough_offset),
                   '--angle-end', str((index%4)*math.pi/2), '--run']
        if direction:
            command += ['--alignment-step-direction', str(direction)]
        print('POINT', i, role, 'mechanical_bin', index, flush=True)
        subprocess.run(command, check=True)
        metadata = json.loads((root/tag/'metadata.json').read_text(encoding='utf-8'))
        excitation_ok = (metadata['alignment'].get('method') == 'voltage' and
                         math.isclose(metadata['alignment']['voltage_v'], ud, abs_tol=1e-6)) if voltage else (
                         math.isclose(metadata['alignment']['current_a'], expected_id, abs_tol=1e-6))
        if not excitation_ok or metadata['alignment']['duration_s'] != 5:
            raise RuntimeError('Wrong excitation in completed capture')
        r = analyze_run(root/tag)
        report['points'].append(dict(role=role, bin=index, step_direction=direction, analysis=r))
        output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(dict(point=i, offset_deg=r['estimated_offset_deg'], mechanical_deg=r['tail_mechanical_deg'],
                              std_deg=r['tail_std_electrical_deg'])), flush=True)
        rejects = point_rejections(report['points'][-1], report['points'][0]['analysis']['tail_mechanical_deg'])
        if rejects:
            report['gate'] = dict(accepted=False, complete=False, rejections=rejects)
            output.write_text(json.dumps(report, indent=2), encoding='utf-8')
            raise RuntimeError('Static point rejected; no subsequent excitation: ' + '; '.join(rejects))
    report['gate'] = evaluate_pairs(report['points'])
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report['gate'], indent=2), flush=True)
    if not report['gate']['accepted']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
