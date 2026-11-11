"""One-off guarded OFF checkpoints for the 2026-10-24 phase-10 candidate.

No flashing/reset/arming/motion here. 'before' verifies and saves the old fault,
then pauses SPI. 'after' verifies the new image/clean WAIT and restores only the
recorded I2t seed. Hard-pinned images: not a general-purpose fault reset tool.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

from encoder_logic_capture import make_probe, require_waiting
from test_highspeed_closed_loop import base


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoint', choices=('before', 'after'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]/'results'
    before = args.checkpoint == 'before'
    folder = 'mt6835_16khz_fusedvoltage_20261024' if before else 'mt6835_16khz_phase10_20261024'
    expected = ('da5bd30ec58ce31ba042402bb6bc87539c5ddda163f42f7e4ed624d702aa690a'
                if before else '1b5c298f5bbbc1d6b51455842b8199b62365ff0e28bf7c8292c386ae801c0532')
    image = root/folder/'firmware.axf'
    require(hashlib.sha256(image.read_bytes()).hexdigest() == expected, 'Unexpected image')
    base.EXPRESSIONS.update(pwm_fault='&g_tim1_pwm_commit_fault',
                            pwm_timing='&g_tim1_pwm_commit_timing',
                            control_period='&g_foc_controller.algorithm.parameters.sample_period_s',
                            voltage_profile='&g_speed_voltage_alignment_profile_version',
                            nominal_bus='&g_global_mode_control.current_open_loop.dc_bus_voltage_v')
    directory = root/('phase10_preflash_20261024' if before else 'phase10_capture_ready_20261024')
    probe = make_probe(image, directory)
    probe.lengths.update(pwm_fault=88, pwm_timing=20, control_period=4,
                         voltage_profile=4, nominal_bus=4, tim1_timebase=12, tim16_arr=4)
    probe.addresses.update(tim1_timebase=0x40012c28, tim16_arr=0x4001442c)
    verification = probe.verify_flash()
    blocks, state = probe.snapshot('frozen_before' if before else 'downloaded_wait', full=True)
    base.require_off(state)
    require(state['clock_hz'] == 168000000 and state['adc_sample_rate_hz'] == 16000, 'Wrong clocks')
    require(struct.unpack('<3I', blocks['tim1_timebase']) == (0, 5250, 1), 'Wrong TIM1 cadence')
    require(base.u32(blocks['voltage_profile']) == 1 and base.f32(blocks['nominal_bus']) == 24,
            'Wrong voltage-alignment contract')
    require(not state['calibration_ok'], 'Unexpected valid motor calibration')
    if before:
        fault = struct.unpack('<22I', blocks['pwm_fault'])
        require(fault[:3] == (1, 1, 2) and fault[3] == 936319427,
                'Prior first-fault changed or target reset; preserve evidence and stop')
        require(state['controller_fault'] == state['runtime_fault'] == 64,
                'Unexpected prior controller fault')
        require(struct.unpack_from('<4I', blocks['logic_trigger']) == (1, 2, 0, 1),
                'Prior image no longer ARMED')
        seed = max(83.0, state['i2t_initial_energy_a2s'], state['i2t_energy_a2s'])
        require(math.isfinite(seed) and seed < 1440, 'Invalid thermal carry')
        # Stop periodic scheduling only after preserving the unchanged OFF fault.
        probe.command([f'write8 0x{probe.addresses["paused"]:x} 0'])
        paused, final = probe.snapshot('off_spi_paused', full=True)
        base.require_off(final)
        require(paused['paused'] == paused['busy_flag'] == b'\x00', 'SPI not stopped')
        require(paused['pwm_fault'] == blocks['pwm_fault'], 'First fault changed')
    else:
        seed = json.loads((root/'phase10_preflash_20261024'/'verified.json').read_text())['thermal_seed_a2s']
        require(math.isfinite(seed) and 83 <= seed < 1440, 'Invalid prior thermal record')
        # Check the WAIT contract before any write, except its not-yet-restored seed.
        require_waiting(blocks, dict(state, i2t_initial_energy_a2s=seed), expected_mode=2)
        require(not state['runtime_fault'] and not state['controller_fault'] and
                not any(blocks['pwm_fault']) and not any(blocks['pwm_timing']), 'New fault present')
        require(base.u32(blocks['tim16_arr']) == 1679, 'Not the 10 us candidate')
        require(abs(base.f32(blocks['control_period'])-0.0000625) < 1e-10, 'Wrong control period')
        seed_bits = struct.unpack('<I', struct.pack('<f', seed))[0]
        probe.command([f'write32 0x{probe.addresses["overload"]+40:x} 0x{seed_bits:x}'])
        blocks, final = probe.snapshot('final_wait', full=True)
        require_waiting(blocks, final, expected_mode=2)
        require(final['i2t_initial_energy_a2s'] >= seed and not final['runtime_fault'] and
                not final['controller_fault'] and not any(blocks['pwm_fault']) and
                not any(blocks['pwm_timing']), 'Final WAIT verification failed')
    verification.update(thermal_seed_a2s=seed, checkpoint=args.checkpoint,
                        output_off=True, motor_started=False, capture_armed_by_script=False,
                        note='Old first fault preserved before explicit candidate download; no sensor register writes.')
    with (directory/'verified.json').open('x', encoding='utf-8') as stream:
        json.dump(verification, stream, indent=2)
    print(json.dumps(verification, indent=2))


if __name__ == '__main__':
    main()
