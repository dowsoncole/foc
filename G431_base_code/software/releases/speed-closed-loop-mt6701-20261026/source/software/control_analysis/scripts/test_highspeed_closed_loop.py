"""Non-halting, guarded staged speed test. No flashing, resets or fault retries.

Run from the repository root. Default is read-only; --run explicitly actuates.
The exact full Flash image and ABI must match. Firmware host-watchdog support
is mandatory. All exit paths request OFF and verify MOE; disconnected hosts
are covered by the firmware's latched timeout, not by Python finally alone.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import time

import test_encoder_neutral_pwm as base

MAX_TEST_RPM = 9000.0  # Experimental envelope; firmware overspeed remains 950 rad/s.
MAX_ACTUAL_RPM = 950.0 * 30.0 / math.pi


def parse_stages(value, dwell):
    stages = [float(x) for x in value.split(',')]
    if (not stages or any(not math.isfinite(x) or not 100 <= x <= MAX_TEST_RPM for x in stages)
            or any(b <= a for a, b in zip(stages, stages[1:]))
            or not 2 <= dwell <= 30):
        raise ValueError('Invalid stages/dwell')
    return stages


def parse_initial_energy(value, enabled):
    if (not math.isfinite(value) or value < 0 or math.ceil(value * 1024) >= 1440 * 1024
            or (value and not enabled)):
        raise ValueError('Invalid initial I2t energy or I2t mode is disabled')
    return value


def preserve_initial_energy(requested, enabled, *states):
    energy = parse_initial_energy(requested, enabled)
    if not enabled:
        return energy
    # A diagnostic rollback may leave a recorded minimum in the public seed
    # while the freshly booted private accumulator is zero. Never overwrite
    # that minimum with the CLI's default zero, or reduce recorded live debt.
    for state in states:
        for key in ('i2t_energy_a2s', 'i2t_initial_energy_a2s'):
            energy = max(energy, parse_initial_energy(state.get(key, 0.0), True))
    return energy


def parse_final_dwell(value, dwell):
    final = dwell if value is None else value
    if not (math.isfinite(dwell) and math.isfinite(final) and 2 <= dwell <= final <= 30):
        raise ValueError('Final dwell must be between the normal dwell and 30 seconds')
    return final


def check_running(state, baseline, previous=None, overload_mode=False, initial_energy=0.0):
    for key in ('crc_errors', 'alarms', 'step_errors', 'start_errors', 'transfer_errors',
                'deadline_overruns', 'pwm_window_rejections', 'pwm_deadline_rejections'):
        if state[key] != baseline[key]:
            raise RuntimeError(f'{key} increased: {baseline[key]} -> {state[key]}')
    if (state['runtime_fault'] or state['controller_fault'] or state['run_fault'] or
            state['encoder_flags'] or state['guard_expired'] or not state['enable'] or
            not state['moe'] or state['run_state'] != 2):
        raise RuntimeError('Run fault or output unexpectedly OFF')
    for key in ('actual_rpm', 'applied_rpm', 'iq_a', 'id_a', 'iq_reference_a', 'uq_pu', 'phase_peak_a'):
        if not math.isfinite(state[key]):
            raise RuntimeError(f'Nonfinite telemetry: {key}')
    if state['actual_rpm'] < -100 or state['actual_rpm'] > MAX_ACTUAL_RPM:
        raise RuntimeError('Wrong direction or speed outside test envelope')
    if overload_mode and (not state.get('overload_armed') or not state.get('overload_used') or
                              state.get('overload_expired') or state.get('overload_invalid')):
        raise RuntimeError('One-shot overload permission is not active')
    if overload_mode:
        for key in ('i2t_energy_a2s', 'i2t_budget_a2s', 'current_rms_a'):
            if not math.isfinite(state[key]) or state[key]<0:
                raise RuntimeError(f'Invalid I2t telemetry: {key}')
        if state['i2t_budget_a2s'] != 1440.0 or state['i2t_energy_a2s'] >= 1440.0:
            raise RuntimeError('I2t budget changed or exhausted')
        if state['i2t_energy_a2s'] + .001 < initial_energy:
            raise RuntimeError('Recorded heat carry was not latched')
    if previous is not None:
        if state['encoder_sequence'] == previous['encoder_sequence']:
            raise RuntimeError('Encoder publication stopped')
        if state['run_elapsed_s'] <= previous['run_elapsed_s']:
            raise RuntimeError('Control elapsed-time stopped; stale speed cannot pass a stage')


class ProbeRun:
    def __init__(self, image, directory):
        self.image, self.directory = image, directory
        directory.mkdir(exist_ok=False)
        base.EXPRESSIONS.update(guard='&g_speed_run_guard', size_guard='sizeof(g_speed_run_guard)',
                                output='&g_foc_controller.algorithm.output',
                                size_output='sizeof(g_foc_controller.algorithm.output)',
                                current_config='&g_current_sampling',
                                pwm_sampling='&g_tim1_pwm_sampling',
                                size_pwm_sampling='sizeof(g_tim1_pwm_sampling)',
                                overload='&g_speed_run_overload', size_overload='sizeof(g_speed_run_overload)')
        self.addresses = base.symbols(image)
        if (self.addresses['size_guard'] != 16 or self.addresses['size_output'] != 92 or
                self.addresses['size_pwm_sampling'] != 32 or self.addresses['size_overload'] not in (40, 44)):
            raise RuntimeError('Unexpected guard/output ABI')
        self.lengths = dict(base.LENGTHS, guard=16, output=92, current_config=24, pwm_sampling=32,
                            overload=self.addresses['size_overload'])
        self.sequence = 0
        self.keepalive = False
        self.origin = time.monotonic()

    def command(self, commands):
        result = subprocess.run([
            str(base.PROBE), 'commander', '--no-config', '--pack', base.PACK,
            '-t', 'stm32g431vbtx', '-u', 'E1007200D0D2139393740544',
            '-M', 'attach', '-W', '-f', '500000', '-c', '; '.join(commands),
        ], capture_output=True, text=True, timeout=6)
        with (self.directory / 'probe.log').open('a', encoding='utf-8') as log:
            log.write('; '.join(commands) + '\n' + result.stdout + result.stderr + '\n')
        if result.returncode or re.search(r'Error|No connected|No available', result.stdout + result.stderr):
            raise RuntimeError(result.stdout + result.stderr)
        return result.stdout

    def heartbeat(self):
        self.sequence = (self.sequence + 1) & 0xffffffff
        return f'write32 0x{self.addresses["guard"] + 4:x} {self.sequence}'

    def snapshot(self, name, full=False):
        folder = self.directory / name
        folder.mkdir()
        lengths = dict(self.lengths)
        if not full:
            lengths['test'] = 192
        commands = [self.heartbeat()] if self.keepalive else []
        commands += [f'savemem 0x{self.addresses[k]:x} {n} {(folder / (k + ".bin")).as_posix()}'
                     for k, n in lengths.items()]
        self.command(commands + ['status'])
        blocks = {k: (folder / (k + '.bin')).read_bytes() for k in lengths}
        if any(len(blocks[k]) != n for k, n in lengths.items()):
            raise RuntimeError('Incomplete snapshot')
        state = base.decode(blocks)
        state.update(host_elapsed_s=time.monotonic() - self.origin,
                     guard_timeout_ms=base.u32(blocks['guard']),
                     guard_active_timeout_ms=base.u32(blocks['guard'], 8),
                     guard_expired=base.u32(blocks['guard'], 12),
                     calibration_offset=base.f32(blocks['test'], 128),
                     encoder_sequence=base.u32(blocks['encoder'], 32),
                     turns=base.f32(blocks['run'], 16),
                     run_elapsed_s=base.f32(blocks['run'], 20),
                     iq_reference_a=base.f32(blocks['run'], 24), iq_a=base.f32(blocks['run'], 28),
                     id_a=base.f32(blocks['run'], 32), uq_pu=base.f32(blocks['run'], 36),
                     foc_isr_max_cycles=base.u32(blocks['diagnostics'], 80),
                     deadline_overruns=base.u32(blocks['diagnostics'], 84),
                     resync_count=base.u32(blocks['diagnostics'], 88),
                     voltage_limit=base.f32(blocks['parameters'], 8),
                     d_voltage=base.f32(blocks['output'], 36),
                     q_voltage=base.f32(blocks['output'], 40),
                     output_saturated=blocks['output'][91],
                     adc_oversampling_ratio=base.u32(blocks['current_config'], 16),
                     adc_sample_rate_hz=base.u32(blocks['current_config'], 20),
                     pwm_sequence_ticks=base.u32(blocks['pwm_sampling'], 4)-base.u32(blocks['pwm_sampling']),
                     pwm_window_rejections=base.u32(blocks['pwm_sampling'], 20),
                     pwm_deadline_rejections=base.u32(blocks['pwm_sampling'], 24),
                     overload_armed=base.u32(blocks['overload'], 4),
                     overload_started=base.u32(blocks['overload'], 8),
                     overload_elapsed_ms=base.u32(blocks['overload'], 12),
                     overload_expired=base.u32(blocks['overload'], 16),
                     overload_used=base.u32(blocks['overload'], 20),
                     i2t_energy_a2s=base.f32(blocks['overload'], 24),
                     i2t_budget_a2s=base.f32(blocks['overload'], 28),
                     current_rms_a=base.f32(blocks['overload'], 32),
                     overload_invalid=base.u32(blocks['overload'], 36),
                     i2t_warmstart_supported=self.lengths['overload']==44,
                     i2t_initial_energy_a2s=base.f32(blocks['overload'], 40)
                         if self.lengths['overload']==44 else 0.0)
        (folder / 'state.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
        return blocks, state

    def verify_flash(self):
        expected = base.flash_section(self.image)
        path = self.directory / 'flash.bin'
        self.command([f'read32 0x{base.BDTR:x} 4',
                      f'savemem 0x08000000 {len(expected)} {path.as_posix()}', 'status'])
        if path.read_bytes() != expected:
            raise RuntimeError('Full Flash mismatch; no test writes allowed')
        return dict(image=str(self.image), axf_sha256=hashlib.sha256(self.image.read_bytes()).hexdigest(),
                    flash_bytes=len(expected), flash_sha256=hashlib.sha256(expected).hexdigest(),
                    addresses=self.addresses,
                    note='Sequential non-halting RAM snapshots, not simultaneous waveform or independent tachometry.')

    def stop(self, saved_bdtr):
        self.keepalive = False
        self.command([f'write32 0x{self.addresses["run"]:x} 0',
                      f'write8 0x{self.addresses["runtime"] + 1:x} 0',
                      f'write32 0x{base.BDTR:x} 0x{saved_bdtr & ~0x8000:x}'])
        _, stopped = self.snapshot('stopped', full=True)
        base.require_off(stopped)
        return stopped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--watchdog-test', action='store_true')
    parser.add_argument('--i2t', action='store_true',
                        help='Sampled-RMS I2t: rated 6 A, reference 18 A/5 s, 1440 A^2*s; no cooling')
    parser.add_argument('--initial-i2t-energy-a2s', type=float, default=0,
                        help='Recorded heat to carry into a new image; rounded upward by firmware')
    parser.add_argument('--stages', default='500,1000,2000,3000,4000,5000,6000,7000,7500')
    parser.add_argument('--dwell', type=float, default=5)
    parser.add_argument('--final-dwell', type=float,
                        help='Optional longer hold for only the final stage, at most 30 s; same one-shot trial.')
    args = parser.parse_args()
    if args.i2t and args.watchdog_test:
        raise ValueError('Overload permission cannot be combined with the watchdog self-test')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.tag):
        raise ValueError('Invalid tag')
    stages = parse_stages(args.stages, args.dwell)
    final_dwell = parse_final_dwell(args.final_dwell, args.dwell)
    initial_energy = parse_initial_energy(args.initial_i2t_energy_a2s, args.i2t)
    probe = ProbeRun(args.image, Path('software/control_analysis/results') / args.tag)
    result = probe.verify_flash()
    before_blocks, before = probe.snapshot('before', full=True)
    base.require_ready(before)
    if before['runtime_fault'] or before['controller_fault']:
        raise RuntimeError('Latched fault requires separate investigation, no auto-acknowledge')
    _, baseline = probe.snapshot('baseline')
    base.require_ready(baseline)
    if baseline['completed'] <= before['completed']:
        raise RuntimeError('Encoder stopped updating')
    for k in ('crc_errors', 'alarms', 'step_errors', 'start_errors', 'transfer_errors'):
        if baseline[k] != before[k]:
            raise RuntimeError('Unhealthy idle encoder')
    initial_energy = preserve_initial_energy(initial_energy, args.i2t, before, baseline)
    result.update(before=before, baseline=baseline, actuated=False, stages=[], samples=[])
    overload_mode = args.i2t
    if args.run and args.i2t and baseline['overload_used']:
        raise RuntimeError('One-shot overload permission already consumed; no automatic reset/retry')
    if initial_energy and not baseline['i2t_warmstart_supported']:
        raise RuntimeError('Firmware lacks warm-start heat carry; no actuation allowed')
    result['acceptance'] = dict(tracking_fraction=.01, tracking_floor_rpm=50,
                                minimum_dwell_s=args.dwell, final_stage_minimum_dwell_s=final_dwell,
                                minimum_samples=3,
                                sampled_phase_peak_limit_a=None, command_ceiling_rpm=MAX_TEST_RPM,
                                overload_mode=args.i2t, rated_rms_a=6.0, reference_rms_a=18.0,
                                reference_time_s=5.0, i2t_budget_a2s=1440.0, cooling=False,
                                initial_i2t_energy_a2s=initial_energy,
                                requested_initial_i2t_energy_a2s=args.initial_i2t_energy_a2s)
    summary_path = probe.directory / 'summary.json'

    def save():
        summary_path.write_text(json.dumps(result, indent=2), encoding='utf-8')

    save()
    if not args.run:
        print('Read-only preflight passed; no target writes.', flush=True)
        return
    address = probe.addresses
    configured = False
    try:
        configured = True
        if baseline['speed_test_state'] == 3:
            # A completed alignment/test is explicitly acknowledged while OFF.
            # Never acknowledge a fault, or combine this with the ON write.
            probe.command([f'write32 0x{address["test"]:x} 3'])
            _, idle = probe.snapshot('idle')
            base.require_ready(idle)
            if idle['speed_test_state'] != 0 or idle['speed_test_fault']:
                raise RuntimeError('Completed test did not enter idle before startup')
            result['idle_before_start'] = idle
            save()
        timeout_ms = 1500 if args.watchdog_test else 8000
        target = 100.0 if args.watchdog_test else stages[0]
        seed_commands = ([f'write32 0x{address["overload"] + 40:x} '
                          f'0x{struct.unpack("<I", struct.pack("<f", initial_energy))[0]:x}']
                         if baseline['i2t_warmstart_supported'] else [])
        probe.command(seed_commands + [f'write32 0x{address["overload"]:x} {int(args.i2t)}',
                       f'write32 0x{address["guard"]:x} {timeout_ms}', probe.heartbeat(),
                       f'write32 0x{address["run"] + 4:x} 0x{struct.unpack("<I", struct.pack("<f", target))[0]:x}',
                       f'write32 0x{address["run"]:x} 1'])
        result['actuated'] = True
        save()
        if args.watchdog_test:
            time.sleep(2)
            _, state = probe.snapshot('watchdog_expired')
            base.require_off(state)
            if state['speed_test_fault'] != 13 or state['guard_expired'] != 1:
                raise RuntimeError('Firmware watchdog did not trip')
            for k in ('crc_errors', 'alarms', 'step_errors', 'start_errors', 'transfer_errors'):
                if state[k] != baseline[k]:
                    raise RuntimeError('Encoder error during watchdog test; do not acknowledge')
            result['watchdog_test'] = state
            result['outcome'] = 'watchdog_pass'
        else:
            probe.keepalive = True
            sample_index = 0
            previous_target = 0.0
            previous_sample = None
            for stage_index, target in enumerate(stages):
                stage_dwell = final_dwell if stage_index == len(stages)-1 else args.dwell
                probe.command([probe.heartbeat(), f'write32 0x{address["run"] + 4:x} '
                               f'0x{struct.unpack("<I", struct.pack("<f", target))[0]:x}'])
                stage_start = time.monotonic()
                deadline = stage_start + (target - previous_target) / 300 + 15 + stage_dwell
                stable_since = None
                accepted = []
                while time.monotonic() < deadline:
                    _, state = probe.snapshot(f'sample_{sample_index:04d}')
                    sample_index += 1
                    result['samples'].append(state)
                    save()
                    check_running(state, baseline, previous_sample, overload_mode, initial_energy)
                    previous_sample = state
                    print(f'target={target:.0f} applied={state["applied_rpm"]:.1f} '
                          f'actual={state["actual_rpm"]:.1f} Iq={state["iq_a"]:.2f} '
                          f'Uq={state["uq_pu"]:.3f} CRC={state["crc_errors"]-baseline["crc_errors"]} '
                          f'phase_peak={state["phase_peak_a"]:.2f} RMS={state["current_rms_a"]:.2f} '
                          f'I2t={state["i2t_energy_a2s"]:.2f}/1440', flush=True)
                    reached = abs(state['applied_rpm'] - target) < 1
                    tracking = abs(state['actual_rpm'] - target) <= max(50, target * .01)
                    if reached and tracking:
                        if stable_since is None:
                            stable_since = time.monotonic()
                            accepted = []
                        accepted.append(state['actual_rpm'])
                        if time.monotonic() - stable_since >= stage_dwell and len(accepted) >= 3:
                            result['stages'].append(dict(target_rpm=target, samples=len(accepted),
                                                         actual_min=min(accepted), actual_max=max(accepted),
                                                         actual_mean=sum(accepted) / len(accepted),
                                                         dwell_s=time.monotonic()-stable_since,
                                                         required_dwell_s=stage_dwell))
                            save()
                            break
                    else:
                        stable_since = None
                    time.sleep(.15)
                else:
                    raise RuntimeError(f'Could not hold {target:g} rpm within 1% / 50 rpm tracking tolerance')
                previous_target = target
            result['outcome'] = 'all_stages_pass'
    except BaseException as error:
        result['outcome'] = 'stopped_on_error'
        result['error'] = repr(error)
        raise
    finally:
        if configured:
            try:
                result['stopped'] = probe.stop(base.u32(before_blocks['bdtr']))
                # Restore commands only after OFF is confirmed. Never restore enable.
                probe.command([f'write32 0x{address["guard"]:x} 0',
                               f'write32 0x{address["overload"]:x} 0',
                               f'write32 0x{address["run"] + 4:x} 0x{base.u32(before_blocks["run"], 4):x}'])
                print('Verified output OFF. Faults, if any, remain latched.', flush=True)
            except BaseException as error:
                result['cleanup_error'] = repr(error)
                result['outcome'] = 'off_not_verified'
                print('Host could not verify OFF; firmware watchdog remains the fallback.', flush=True)
            save()
    if 'cleanup_error' in result:
        raise RuntimeError('Final OFF verification failed; see cleanup_error in summary')
    print(result['outcome'], flush=True)


if __name__ == '__main__':
    main()
