"""MT6835 fault trigger preparation while OFF. Default is read-only inspection.

--arm starts encoder SPI acquisition, NEVER motor output. Use only after the
operator has armed the analyzer. Profile 1 prohibits motion; profile 2 permits
later, separately commanded motion through the existing interlocks. No reset,
flash, fault clearing or retries.
"""
import argparse
import json
import math
import struct
from pathlib import Path

from test_highspeed_closed_loop import ProbeRun, base

START_COMMAND = 0x43415031
# Accepted bidirectional calibration, also compiled into speed_commission.h.
BOARD_ENCODER_OFFSET_RAD = 5.560209515459482


def make_probe(image, directory):
    base.EXPRESSIONS.update(
        logic_trigger='&g_encoder_logic_trigger',
        size_logic_trigger='sizeof(g_encoder_logic_trigger)',
        sensor_zero_guard='&g_encoder_zero_guard',
        zero_fault_capture='&g_encoder_zero_fault_capture',
        paused='&s_encoder.initialized', busy_flag='&s_encoder.transfer_busy')
    probe = ProbeRun(image.resolve(), directory.resolve())
    if probe.addresses['size_logic_trigger'] != 48:
        raise RuntimeError('Unexpected trigger ABI')
    probe.addresses.update(gpioa_idr=0x48000010, gpioa_odr=0x48000014,
                           gpioa_moder=0x48000000)
    probe.lengths.update(logic_trigger=48, sensor_zero_guard=32,
                         zero_fault_capture=328, paused=1, busy_flag=1,
                         gpioa_idr=4, gpioa_odr=4, gpioa_moder=4)
    return probe


def require_waiting(blocks, state, expected_mode=1):
    base.require_off(state)
    fields = struct.unpack('<10I8s', blocks['logic_trigger'])
    if expected_mode not in (1, 2):
        raise RuntimeError('Unsupported capture mode')
    if fields[:5] != (1, expected_mode, 0, 0, 0) or any(fields[5:10]) or any(fields[10]):
        raise RuntimeError('Not a clean diagnostic WAIT state; no rearm/reset')
    if blocks['paused'] != b'\x01' or blocks['busy_flag'] != b'\x00':
        raise RuntimeError('Encoder not initialized and idle')
    # PA0 now marks the complete ADC IRQ and can be HIGH even in WAIT/OFF.
    # The validated RAM trigger/zero-guard records, not this pulse, gate arm.
    if base.u32(blocks['gpioa_moder']) & 3 != 1:
        raise RuntimeError('PA0 timing probe not configured as output')
    if (any(blocks['sensor_zero_guard']) or any(blocks['zero_fault_capture'])
            or base.u32(blocks['capture']) or base.u32(blocks['diagnostics'])
            or state['encoder_sequence']):
        raise RuntimeError('Acquisition already ran or old evidence present')
    # Boot now loads an accepted board profile without sampling or moving.
    # Only that exact profile may accompany a clean WAIT; do not admit an
    # arbitrary RAM calibration or the rejected historical 5.3988 rad seed.
    if state['calibration_ok']:
        test = blocks.get('test', b'')
        if state['calibration_ok'] != 1 or len(test) < 132:
            raise RuntimeError('Missing or invalid boot calibration record')
        command, valid, direction, pole_pairs, offset = struct.unpack_from('<IIiIf', test, 112)
        if (command != 0 or valid != 1 or direction != -1 or pole_pairs != 2
                or not math.isclose(offset, BOARD_ENCODER_OFFSET_RAD, rel_tol=0, abs_tol=1e-6)):
            raise RuntimeError('Unexpected boot calibration; no rearm/reset')
    if state['i2t_initial_energy_a2s'] < 83:
        raise RuntimeError('Recorded thermal debt not restored')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--arm', action='store_true',
                        help='Only after operator confirms analyzer waiting')
    parser.add_argument('--motion-capable', action='store_true',
                        help='Expect profile 2. This command still only starts SPI; it never starts the motor.')
    args = parser.parse_args()
    probe = make_probe(args.image, args.directory)
    meta = probe.verify_flash()
    (probe.directory/'verified.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    blocks, state = probe.snapshot('before', full=True)
    expected_mode = 2 if args.motion_capable else 1
    require_waiting(blocks, state, expected_mode)
    if not args.arm:
        print('WAIT verified: output OFF, no encoder sampling; PA0 is ADC IRQ timing; no target writes')
        return
    probe.command([f'write32 0x{probe.addresses["logic_trigger"]+8:x} 0x{START_COMMAND:x}'])
    blocks, state = probe.snapshot('after_arm', full=True)
    base.require_off(state)
    trigger = struct.unpack('<10I8s', blocks['logic_trigger'])
    if trigger[:3] != (1, expected_mode, 0) or trigger[3] not in (1, 2) or trigger[4]:
        raise RuntimeError('Start command not accepted; no retry')
    if not base.u32(blocks['diagnostics']):
        raise RuntimeError('No encoder transactions; leave evidence unchanged')
    print(json.dumps(dict(state='FIRED' if trigger[3] == 2 else 'ARMED',
                          reason=trigger[5], cycle=trigger[6], motor_output=False,
                          directory=str(probe.directory)), indent=2))


if __name__ == '__main__':
    main()
