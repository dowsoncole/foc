"""Bounded zero-reference isolation using the existing verified firmware.

No flashing, halt, reset, or sensor writes. The default test uses kind 1 with
zero amplitude/angle and zero current-PI gains to produce neutral PWM; firmware
owns its 100 ms deadline and stops on invalid feedback. --continuous instead
tests the real g_speed_run.enable at 500 rpm, with host-side finally shutdown.
Run from the repository root. Without --run the invocation is read-only.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import time
from verify_encoder_artifacts import load_image

# Allow an explicit tool override after a CMSIS extension upgrade. This does
# not discover/select a target, connect, reset, or change any test interlock.
PROBE = Path(os.environ.get('SERVO_PYOCD') or
             'C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.8.0-win32-x64/tools/pyocd/pyocd.exe')
GDB = Path(os.environ.get('SERVO_GDB') or
           str(PROBE.parents[1] / 'gdb/bin/arm-none-eabi-gdb.exe'))
PACK = 'C:/Users/penghao/AppData/Local/Arm/Packs/Keil/STM32G4xx_DFP/2.2.0'
IMAGE = Path('software/control_analysis/results/firmware_crc_startup_before_20261022.axf')
BDTR = 0x40012C44
CYCCNT = 0xE0001004
EXPRESSIONS = {
    'runtime': '&g_global_mode_control.runtime',
    'mode': '&g_global_mode_control',
    'stage': '&g_global_mode_control.current_open_loop',
    'run': '&g_speed_run',
    'test': '&g_speed_test',
    'parameters': '&g_foc_controller.algorithm.parameters',
    'controller_running': '&g_foc_controller.running',
    'controller_fault': '&g_foc_controller.fault_flags',
    'calibration': '&g_current_sampling.calibration_state',
    'current_test_state': '&g_current_loop_test.state',
    'tuning_request': '&g_current_tuning.apply_request',
    'trip': '&g_current_trip',
    'encoder': '&g_encoder_snapshot',
    'diagnostics': '&g_encoder_diagnostics',
    'capture': '&g_encoder_fault_capture',
    'clock': '&SystemCoreClock',
    'size_parameters': 'sizeof(g_foc_controller.algorithm.parameters)',
    'size_test': 'sizeof(g_speed_test)',
    'size_encoder': 'sizeof(g_encoder_snapshot)',
    'size_capture': 'sizeof(g_encoder_fault_capture)',
}
LENGTHS = dict(runtime=36, mode=4, stage=72, run=60, test=4416,
               parameters=68, controller_running=1, controller_fault=4,
               calibration=1, current_test_state=1, tuning_request=4,
               trip=28, encoder=48, diagnostics=128, capture=312, clock=4,
               bdtr=4, cycles=4)


def u32(data, offset=0):
    return struct.unpack_from('<I', data, offset)[0]


def f32(data, offset=0):
    return struct.unpack_from('<f', data, offset)[0]


def flash_section(path):
    # Check every Flash load byte, including compressed RW initializers that
    # are outside ER_IROM1. Preserve the public helper name for older callers.
    image = load_image(path)
    base, end = min(image), max(image) + 1
    assert base == 0x08000000 and len(image) == end - base
    return bytes(image[address] for address in range(base, end))


def symbols(image=IMAGE):
    args = [str(GDB), '-batch', str(image)]
    for expr in EXPRESSIONS.values():
        args += ['-ex', 'p/x ' + expr]
    result = subprocess.run(args, capture_output=True, text=True, timeout=30)
    values = re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$', result.stdout, re.M)
    assert result.returncode == 0 and len(values) == len(EXPRESSIONS), result.stdout + result.stderr
    addresses = dict(zip(EXPRESSIONS, (int(v, 16) for v in values)))
    assert [addresses[k] for k in ('size_parameters', 'size_test', 'size_encoder', 'size_capture')] == [68, 4416, 48, 312]
    addresses.update(bdtr=BDTR, cycles=CYCCNT)
    return addresses


def decode(blocks):
    runtime, test, encoder, diag = (blocks[k] for k in ('runtime', 'test', 'encoder', 'diagnostics'))
    capture = blocks['capture']
    frame = capture[4:32]
    return dict(
        moe=bool(u32(blocks['bdtr']) & 0x8000),
        runtime_bytes=list(runtime[:6]), runtime_fault=u32(runtime, 8),
        controller_running=blocks['controller_running'][0],
        controller_fault=u32(blocks['controller_fault']),
        mode=list(blocks['mode'][:2]), stage=list(blocks['stage'][:2]),
        calibration=blocks['calibration'][0],
        current_test_state=blocks['current_test_state'][0],
        tuning_request=u32(blocks['tuning_request']), current_trip_count=u32(blocks['trip']),
        enable=u32(blocks['run']), speed_test_state=u32(test, 4),
        target_rpm=f32(blocks['run'], 4), applied_rpm=f32(blocks['run'], 8),
        actual_rpm=f32(blocks['run'], 12), run_state=u32(blocks['run'], 40),
        run_fault=u32(blocks['run'], 44), phase_peak_a=f32(blocks['run'], 48),
        speed_test_fault=u32(test, 8), active_kind=u32(test, 64),
        feedback_fault=u32(test, 176), feedback_age_cycles=u32(test, 180),
        ready_cycle=u32(test, 184), ready_count=u32(test, 188),
        first_control_cycle=u32(test, 152), elapsed_s=f32(test, 132),
        calibration_ok=u32(test, 116), encoder_flags=u32(encoder, 36),
        encoder_valid=list(encoder[40:44]), mechanical_speed_rad_s=f32(encoder, 20),
        completed=u32(diag, 4), busy=u32(diag, 8), start_errors=u32(diag, 12),
        transfer_errors=u32(diag, 16), crc_errors=u32(diag, 20),
        alarms=u32(diag, 24), step_errors=u32(diag, 104),
        capture_count=u32(capture), first_error_cycle=u32(frame),
        first_error_reason=u32(frame, 4), first_error_rx=frame[20:26].hex(' ').upper(),
        first_error_computed_crc=frame[26],
        d_pi=struct.unpack_from('<5f', blocks['parameters'], 28),
        q_pi=struct.unpack_from('<5f', blocks['parameters'], 48),
        deadline_overruns=u32(diag, 84),
        clock_hz=u32(blocks['clock']))


def require_off(state, reserved=False):
    assert not state['moe'] and not state['enable'], 'Output/continuous enable active'
    assert not any(state['runtime_bytes'][i] for i in (0, 1, 2, 5)), 'Runtime not stopped'
    assert state['runtime_bytes'][3] == int(reserved), 'Unexpected precharge/reservation'
    assert not state['controller_running'], 'Controller active'


def require_ready(state, reserved=False, calibration_required=True):
    require_off(state, reserved)
    assert state['runtime_bytes'][4] == 1, 'Power stage not safe'
    assert state['calibration'] == 3, 'ADC current offsets are not calibrated'
    if calibration_required:
        assert state['calibration_ok'] == 1, 'Motor electrical calibration is not valid'
    assert state['mode'] == [1, 1] and state['stage'] == [2, 2]
    assert state['encoder_valid'] == [0, 1, 1, 1] and state['encoder_flags'] == 0
    assert math.isfinite(state['mechanical_speed_rad_s']) and abs(state['mechanical_speed_rad_s']) < 1
    assert not state['current_test_state'] and not state['tuning_request'] and not state['current_trip_count']
    faults = state['runtime_fault'], state['controller_fault']
    assert faults == (0, 0) or (
        faults == (6, 6) and state['speed_test_state'] == 4
        and state['speed_test_fault'] in (4, 8) and state['feedback_fault'] in (2, 8, 10)
    ), 'Unreviewed fault: cannot acknowledge for this test'


def require_clean_counters(before, after):
    for key in ('crc_errors', 'step_errors', 'alarms', 'start_errors', 'transfer_errors',
                'deadline_overruns'):
        assert before[key] == after[key], key + ' increased; reject calibration'


def adjacent_quarter_endpoint(start_field, target_phase, direction):
    """Unwrap a cardinal target near the requested +/-90 degree field step.

    With two pole pairs, adding an unnecessary electrical revolution moves
    the rotor to the OTHER mechanical half-turn. Never silently take it.
    """
    if (direction not in (-1, 1) or not math.isfinite(start_field)
            or not -math.pi <= start_field <= math.pi
            or not math.isfinite(target_phase)
            or abs(target_phase/(math.pi/2)-round(target_phase/(math.pi/2))) > 1e-6):
        raise ValueError('Invalid adjacent-quarter alignment request')
    endpoint = target_phase + 2*math.pi*round(
        (start_field + direction*math.pi/2 - target_phase)/(2*math.pi))
    if not math.pi/4 <= direction*(endpoint-start_field) <= 3*math.pi/4 or abs(endpoint) > 2*math.pi:
        raise ValueError('Not an adjacent quarter-turn in the requested direction')
    return endpoint


def alignment_id_amplitude(id_current, rms_current=None):
    """Explicit units: legacy Id amps or equivalent three-phase RMS amps."""
    if rms_current is None:
        if not math.isfinite(id_current) or not 0 <= id_current <= 3:
            raise ValueError('Legacy Id command must be within 0..3 A')
        return id_current
    if id_current != 0 or not math.isfinite(rms_current) or not 0 < rms_current <= 5:
        raise ValueError('RMS command must be within (0,5] A and not mixed with Id amps')
    return rms_current * math.sqrt(2)


def require_alignment_profile(version, rms_current=None):
    if version not in (1, 2) or (rms_current is not None and version != 2):
        raise ValueError('Unsupported alignment profile / RMS-current contract')


def require_voltage_alignment(voltage, version=1, bus=24.0):
    if (version != 1 or not math.isfinite(voltage) or not 0 < voltage <= 1.0
            or not math.isfinite(bus) or not 18 <= bus <= 28 or voltage > .05*bus):
        raise ValueError('Voltage alignment requires profile 1, Ud in (0,1] V, and adequate nominal Vdc')


def snapshot_lengths(name):
    lengths = dict(LENGTHS)
    # All status/config fields end at byte 192. Only the final stopped dump
    # needs the 4 KiB records; retain every transport/zero fault block always.
    if name != 'stopped':
        lengths['test'] = 192
    return lengths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--continuous', action='store_true',
                        help='Test actual g_speed_run.enable at 500 rpm; finally stops output.')
    parser.add_argument('--alignment-current', type=float, default=0,
                        help='Legacy kind-1 Id amplitude, NOT phase RMS (0 uses neutral PWM). Maximum 3 A.')
    parser.add_argument('--alignment-rms-current', type=float,
                        help='Explicit equivalent three-phase RMS, maximum 5 A (Id = RMS*sqrt(2)); profile 2 required.')
    parser.add_argument('--alignment-voltage', type=float,
                        help='Kind 10: positive physical Ud in (0,1] V, Uq=0, no current PI; 5 seconds.')
    parser.add_argument('--bidirectional-sweep', action='store_true',
                        help='Kind 9: 12 s, full mechanical turn forward/reverse; no automatic offset application.')
    parser.add_argument('--duration', type=float, default=0.1)
    parser.add_argument('--angle-end', type=float, default=0)
    parser.add_argument('--alignment-start-offset', type=float,
                        help='Kind-1/10: rough electrical offset used to start the field near the rotor. '
                             'Does not apply/validate motor calibration; requires alignment profile 1 or 2.')
    parser.add_argument('--alignment-step-direction', type=int, choices=(-1, 1),
                        help='Unwrap the cardinal --angle-end to an adjacent +/-90 electrical degree step.')
    parser.add_argument('--bounded-speed-rpm', type=float,
                        help='Kind-3 bounded speed step, at most +/-90 rpm, 0.3 A cap.')
    parser.add_argument('--calibration-offset', type=float,
                        help='Apply measured motor electrical offset in RAM while OFF, not sensor EEPROM.')
    parser.add_argument('--image', type=Path, default=IMAGE,
                        help='AXF whose full Flash load image must match the board before any test write.')
    parser.add_argument('--wait-for-capture', action='store_true',
                        help='Wait OFF for capture_go.flag after writing capture_ready.flag (180 s limit).')
    args = parser.parse_args()
    assert re.fullmatch(r'[A-Za-z0-9_-]+', args.tag)
    args.alignment_current = alignment_id_amplitude(args.alignment_current, args.alignment_rms_current)
    voltage_alignment = args.alignment_voltage is not None
    if voltage_alignment:
        require_voltage_alignment(args.alignment_voltage)
        assert args.alignment_current == 0 and args.alignment_rms_current is None
        assert not args.bidirectional_sweep and not args.continuous and args.bounded_speed_rpm is None
        assert args.duration == 5 and args.alignment_start_offset is not None and args.calibration_offset is None
        EXPRESSIONS['voltage_alignment_profile_version'] = '&g_speed_voltage_alignment_profile_version'
        LENGTHS['voltage_alignment_profile_version'] = 4
        EXPRESSIONS['alignment_nominal_bus'] = '&g_global_mode_control.current_open_loop.dc_bus_voltage_v'
        LENGTHS['alignment_nominal_bus'] = 4
    if args.alignment_rms_current is not None:
        assert not args.bidirectional_sweep and args.alignment_start_offset is not None
        assert args.duration == 5, 'RMS alignment uses the reviewed 5 s static profile'
    assert 0.05 <= args.duration <= (12 if args.bidirectional_sweep else 5)
    if args.bidirectional_sweep:
        assert args.duration == 12 and args.alignment_current > 0
        assert args.angle_end == 0 and args.bounded_speed_rpm is None
        assert args.calibration_offset is None and not args.continuous
    # One electrical revolution permits controlled positive/negative approach
    # to the SAME modulo-2pi field, for alignment hysteresis diagnostics.
    assert math.isfinite(args.angle_end) and abs(args.angle_end) <= 2 * math.pi
    assert not (args.continuous and args.alignment_current)
    if args.bounded_speed_rpm is not None:
        assert math.isfinite(args.bounded_speed_rpm) and abs(args.bounded_speed_rpm) <= 90
        assert not args.continuous and not args.alignment_current
    if args.calibration_offset is not None:
        assert math.isfinite(args.calibration_offset) and 0 <= args.calibration_offset < 2 * math.pi
    if args.alignment_start_offset is not None:
        assert math.isfinite(args.alignment_start_offset) and 0 <= args.alignment_start_offset < 2*math.pi
        assert (args.alignment_current > 0 or voltage_alignment) and not args.bidirectional_sweep and not args.continuous
        assert args.bounded_speed_rpm is None and args.calibration_offset is None
        EXPRESSIONS['alignment_profile_version'] = '&g_speed_alignment_profile_version'
        LENGTHS['alignment_profile_version'] = 4
    if args.alignment_step_direction is not None:
        assert args.alignment_start_offset is not None
    # Synthetic-angle alignment establishes motor calibration; it must work
    # while the OLD profile is deliberately invalidated. Closed-loop tests
    # still require calibration, including after an explicit RAM update.
    needs_motor_calibration = args.alignment_current <= 0 and not voltage_alignment
    needs_initial_calibration = needs_motor_calibration and args.calibration_offset is None
    directory = Path('software/control_analysis/results') / args.tag
    directory.mkdir(exist_ok=False)
    addresses = symbols(args.image)

    def command(commands):
        result = subprocess.run([
            str(PROBE), 'commander', '--no-config', '--pack', PACK,
            '-t', 'stm32g431vbtx', '-u', 'E1007200D0D2139393740544',
            '-M', 'attach', '-W', '-f', '500000', '-c', commands,
        ], capture_output=True, text=True, timeout=30)
        with (directory / 'probe.log').open('a', encoding='utf-8') as log:
            log.write(commands + '\n' + result.stdout + result.stderr + '\n')
        assert result.returncode == 0 and not re.search(r'Error|No connected|No available', result.stdout + result.stderr), result.stdout + result.stderr
        return result.stdout

    def snapshot(name):
        folder = directory / name
        folder.mkdir()
        lengths = snapshot_lengths(name)
        commands = [f'savemem 0x{addresses[k]:08x} {length} {(folder / (k + ".bin")).as_posix()}'
                    for k, length in lengths.items()]
        command('; '.join(commands + ['status']))
        blocks = {k: (folder / (k + '.bin')).read_bytes() for k in lengths}
        assert all(len(blocks[k]) == length for k, length in lengths.items())
        state = decode(blocks)
        (folder / 'state.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
        return blocks, state

    def write_words(address, data):
        assert len(data) % 4 == 0
        return [f'write32 0x{address + i:08x} 0x{u32(data, i):08x}' for i in range(0, len(data), 4)]

    expected = flash_section(args.image)
    flash_path = directory / 'flash.bin'
    command(f'read32 0x{BDTR:08x} 4; savemem 0x08000000 {len(expected)} {flash_path.as_posix()}; status')
    assert flash_path.read_bytes() == expected, 'Board Flash does not match symbol image'
    before_blocks, before = snapshot('before')
    require_ready(before, calibration_required=needs_initial_calibration)
    time.sleep(0.2)
    _, baseline = snapshot('baseline')
    require_ready(baseline, calibration_required=needs_initial_calibration)
    assert baseline['completed'] > before['completed'] and baseline['crc_errors'] == before['crc_errors'], 'Idle communication not healthy'
    require_clean_counters(before, baseline)
    start_field = 0.0
    if args.alignment_start_offset is not None:
        require_alignment_profile(u32(before_blocks['alignment_profile_version']), args.alignment_rms_current)
        if voltage_alignment:
            # Resolve by symbol: target uses short enums, unlike host C tests.
            nominal_bus = f32(before_blocks['alignment_nominal_bus'])
            require_voltage_alignment(args.alignment_voltage,
                                      u32(before_blocks['voltage_alignment_profile_version']), nominal_bus)
        direction, poles = struct.unpack_from('<iI', before_blocks['test'], 120)
        assert direction == -1 and poles == 2
        # Snapshot angle already includes configured direction; do not apply it twice.
        mechanical = f32(before_blocks['encoder'], 16)
        assert math.isfinite(mechanical)
        start_field = (poles*mechanical + args.alignment_start_offset + math.pi) % (2*math.pi) - math.pi
    nominal_phase = args.angle_end
    if args.alignment_step_direction is not None:
        args.angle_end = adjacent_quarter_endpoint(start_field, nominal_phase, args.alignment_step_direction)
    metadata = dict(image=str(args.image), image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
                    flash_verified_bytes=len(expected), flash_sha256=hashlib.sha256(expected).hexdigest(),
                    addresses=addresses, actuation_requested=args.run, actuated=False,
                    note='RAM blocks are non-halting sequential reads, not a synchronized electrical capture.')
    (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print('Verified image; OFF, stationary, ADC offsets ready; idle CRC counter stable; '
          f'motor calibration={baseline["calibration_ok"]}.', flush=True)
    if not args.run:
        print('Inspect-only: no target writes.', flush=True)
        return

    # Gain validation permits zero. Keep modulation limits and integral bounds
    # intact; clear both PI state/gain triplets so command and noise produce 0.
    neutral_parameters = bytearray(before_blocks['parameters'])
    struct.pack_into('<3f', neutral_parameters, 28, 0, 0, 0)
    struct.pack_into('<3f', neutral_parameters, 48, 0, 0, 0)
    requested = struct.pack('<I9f2I', 1, args.duration, args.alignment_current, start_field, 0,
                            args.angle_end, 0, 0, max(0.3, args.alignment_current), 20,
                            max(1, math.ceil(args.duration * 1000 / 120)), 16)
    if voltage_alignment:
        requested = struct.pack('<I9f2I', 10, 5, args.alignment_voltage, start_field, 0,
                                args.angle_end, 0, 0, 15, 20, 40, 16)
    if args.bidirectional_sweep:
        requested = struct.pack('<I9f2I', 9, 12, args.alignment_current, 0, 0,
                                4*math.pi, 0, 0, args.alignment_current, 20, 100, 16)
    if args.bounded_speed_rpm is not None:
        requested = struct.pack('<I9f2I', 3, args.duration, args.bounded_speed_rpm * math.pi / 30,
                                0, 0, 0, .032, 1, .3, 20,
                                max(1, math.ceil(args.duration * 1000 / 120)), 16)
    neutral = not args.alignment_current and not voltage_alignment and args.bounded_speed_rpm is None
    configured = False
    try:
        configured = True
        commands = [f'write32 0x{addresses["controller_fault"]:08x} 0',
                    f'write32 0x{addresses["runtime"] + 8:08x} 0',
                    f'write32 0x{addresses["test"]:08x} 3']
        command('; '.join(commands))
        _, idle = snapshot('acknowledged')
        require_ready(idle, calibration_required=needs_initial_calibration)
        assert idle['speed_test_state'] == 0 and idle['runtime_fault'] == idle['controller_fault'] == 0
        if args.calibration_offset is not None:
            command('; '.join(write_words(addresses['test'] + 128, struct.pack('<f', args.calibration_offset))
                              + [f'write32 0x{addresses["test"] + 112:08x} 1']))
            calibrated_blocks, calibrated = snapshot('calibrated')
            require_ready(calibrated)
            assert abs(f32(calibrated_blocks['test'], 128) - args.calibration_offset) < 1e-5
            assert u32(calibrated_blocks['test'], 112) == 0
            metadata['ram_calibration_offset'] = args.calibration_offset
        if args.continuous:
            # Use the application's normal startup, ramp, PI, current cap and
            # feedback interlocks. There is no continuous-mode firmware timer;
            # the short host dwell is always followed by finally shutdown.
            command('; '.join(write_words(addresses['run'] + 4, struct.pack('<f', 500))
                              + [f'write32 0x{addresses["run"]:08x} 1']))
            metadata['actuated'] = True
            (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
            print('Requested g_speed_run.enable=1, target 500 rpm; bounded host observation.', flush=True)
            time.sleep(1.0)
            _, running = snapshot('running')
            summary = dict(before=baseline, running=running,
                           crc_delta=(running['crc_errors'] - baseline['crc_errors']) & 0xffffffff)
            (directory / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
            print(json.dumps({k: running[k] for k in ('enable', 'run_state', 'run_fault', 'applied_rpm', 'actual_rpm', 'crc_errors')}, indent=2), flush=True)
            assert running['enable'] == 1 and running['run_state'] == 2 and running['run_fault'] == 0, 'Continuous startup failed'
            assert summary['crc_delta'] == 0, 'CRC occurred during continuous startup'
            return
        command('; '.join(write_words(addresses['test'] + 16, requested)
                          + [f'write32 0x{addresses["capture"]:08x} 0',
                             f'write32 0x{addresses["test"]:08x} 1']))
        _, armed = snapshot('armed')
        require_ready(armed, calibration_required=needs_motor_calibration)
        assert armed['speed_test_state'] == 1 and armed['speed_test_fault'] == 0 and armed['active_kind'] == u32(requested)
        # While armed but OFF, MainCurrentTest_Service reloads the normal PI
        # gains. Reserve the existing precharge gate before overriding gains;
        # this keeps ADC CCR writes and background PI reloads blocked. Keep
        # output_request=0 and MOE=0 until the final checked start operation.
        commands = [f'write8 0x{addresses["runtime"] + 3:08x} 1']
        if neutral:
            commands += write_words(addresses['parameters'], neutral_parameters)
        command('; '.join(commands))
        _, reserved = snapshot('reserved')
        require_ready(reserved, reserved=True, calibration_required=needs_motor_calibration)
        if neutral:
            assert reserved['d_pi'][:3] == reserved['q_pi'][:3] == (0, 0, 0), 'Zero-gain isolation not established'
        elif not voltage_alignment:
            assert reserved['d_pi'][0] > 0 and reserved['q_pi'][0] > 0, 'Alignment PI gains missing'
        metadata['alignment'] = dict(current_a=args.alignment_current, duration_s=args.duration,
                                     angle_end_rad=4*math.pi if args.bidirectional_sweep else args.angle_end,
                                     bidirectional_sweep=args.bidirectional_sweep)
        metadata['alignment'].update(current_unit='Id amplitude A',
                                     equivalent_three_phase_rms_a=args.alignment_current/math.sqrt(2))
        if voltage_alignment:
            metadata['alignment'] = dict(method='voltage', voltage_v=args.alignment_voltage,
                voltage_unit='Ud physical V; Uq=0; current PI bypassed', voltage_contract_version=1,
                nominal_dc_bus_voltage_v=nominal_bus, duration_s=5,
                angle_end_rad=args.angle_end, bidirectional_sweep=False)
        if args.alignment_rms_current is not None:
            metadata['alignment'].update(requested_rms_a=args.alignment_rms_current,
                                         current_contract_version=2)
        if args.alignment_start_offset is not None:
            metadata['alignment'].update(start_field_rad=start_field,
                                         start_offset_estimate_rad=args.alignment_start_offset)
        if args.alignment_step_direction is not None:
            metadata['alignment'].update(step_direction=args.alignment_step_direction,
                                         nominal_phase_rad=nominal_phase)
        metadata['bounded_speed_rpm'] = args.bounded_speed_rpm
        excitation = (f'Ud={args.alignment_voltage:g} V, Uq=0, NO current PI' if voltage_alignment else
                      f'Id={args.alignment_current:g} A (equivalent RMS={args.alignment_current/math.sqrt(2):g} A)')
        print(f'Armed: kind {u32(requested)}, {excitation}, {f32(requested,20):g} rad, '
              f'{args.duration:g} s firmware deadline.', flush=True)
        if args.wait_for_capture:
            (directory / 'capture_ready.flag').write_text('PWM OFF; ready for analyzer\n', encoding='utf-8')
            wait_start = time.monotonic()
            while not (directory / 'capture_go.flag').exists():
                assert time.monotonic() - wait_start < 180, 'Capture coordination timeout; cancelling while OFF'
                time.sleep(0.1)
        command(f'read32 0x{CYCCNT:08x} 4; write8 0x{addresses["runtime"] + 1:08x} 1; write8 0x{addresses["runtime"] + 3:08x} 0; read32 0x{CYCCNT:08x} 4; status')
        metadata['actuated'] = True
        (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        time.sleep(args.duration + 0.4)
        _, after = snapshot('after')
        require_off(after)
        assert after['speed_test_state'] in (3, 4), 'Firmware did not terminate bounded test'
        summary = dict(before=baseline, armed=armed, reserved=reserved, after=after)
        summary['crc_delta'] = (after['crc_errors'] - armed['crc_errors']) & 0xffffffff
        if after['capture_count'] and after['ready_cycle']:
            delta = (after['first_error_cycle'] - after['ready_cycle']) & 0xffffffff
            if delta >= 0x80000000:
                delta -= 0x100000000
            summary['first_error_relative_to_post_precharge_feedback_us'] = delta * 1e6 / after['clock_hz']
        (directory / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps({k: summary[k] for k in ('crc_delta', 'first_error_relative_to_post_precharge_feedback_us') if k in summary}, indent=2), flush=True)
        print('Test ended with output OFF; state', after['speed_test_state'], 'fault', after['speed_test_fault'], flush=True)
        assert after['speed_test_state'] == 3 and after['speed_test_fault'] == 0, 'Bounded test faulted; do not use its records as calibration'
        assert summary['crc_delta'] == 0, 'CRC occurred during bounded test'
        require_clean_counters(armed, after)
    finally:
        if configured:
            # Emergency cleanup is unconditional, including failed SWD reads.
            cleanup = [
                f'write32 0x{addresses["run"]:08x} 0',
                f'write8 0x{addresses["runtime"] + 1:08x} 0',
                f'write32 0x{BDTR:08x} 0x{u32(before_blocks["bdtr"]) & ~0x8000:08x}',
                f'write8 0x{addresses["runtime"] + 3:08x} 0',
            ]
            if not args.continuous and ('after' not in locals() or after['speed_test_state'] not in (3, 4)):
                cleanup.append(f'write32 0x{addresses["test"]:08x} 2')
            command('; '.join(cleanup))
            _, stopped = snapshot('stopped')
            require_off(stopped)
            command('; '.join(write_words(addresses['parameters'], before_blocks['parameters'])
                              + write_words(addresses['test'] + 16, before_blocks['test'][16:64])
                              + write_words(addresses['run'] + 4, before_blocks['run'][4:8])))
            restored_blocks, restored = snapshot('restored')
            require_off(restored)
            assert restored_blocks['parameters'][8:] == before_blocks['parameters'][8:], 'PI/limits restoration failed'
            assert restored_blocks['test'][16:64] == before_blocks['test'][16:64], 'Requested test restoration failed'
            if 'after' in locals():
                require_clean_counters(after, restored)
            if 'summary' in locals():
                summary['restored'] = restored
                summary['crc_delta_until_off'] = (restored['crc_errors'] - baseline['crc_errors']) & 0xffffffff
                (directory / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
            print('Original PI parameters and requested test restored; output OFF.', flush=True)


if __name__ == '__main__':
    main()
