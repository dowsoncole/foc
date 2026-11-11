"""Profile-2 wrapper around the existing bounded alignment/speed runners.

Adds raw first-trigger/zero-register evidence to every snapshot and refuses
to arm motion after any trigger. It never flashes, resets or rearms PA0.
Usage: run_triggered_commission.py alignment|speed <existing runner options>
Add --pwm-diagnostics for firmware with first-fault and successful-commit timing.
"""
import sys
import struct

import test_encoder_neutral_pwm as base
import test_highspeed_closed_loop as speed


def require_trigger_healthy(state):
    if (state['logic_version'], state['logic_mode'], state['logic_state'],
            state['logic_reason']) != (1, 2, 1, 0):
        raise RuntimeError('Motion capture is not healthy ARMED; no reset/retry')
    if state.get('pwm_commit_fault_count', 0):
        raise RuntimeError('PWM first-fault is latched; no reset/retry')


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('alignment', 'speed'):
        raise SystemExit(__doc__)
    kind = sys.argv.pop(1)
    pwm_diagnostics = '--pwm-diagnostics' in sys.argv
    if pwm_diagnostics:
        sys.argv.remove('--pwm-diagnostics')
        base.EXPRESSIONS.update(pwm_fault='&g_tim1_pwm_commit_fault',
                                pwm_timing='&g_tim1_pwm_commit_timing')
        base.LENGTHS.update(pwm_fault=88, pwm_timing=20)
    base.EXPRESSIONS.update(logic_trigger='&g_encoder_logic_trigger',
                            sensor_zero_guard='&g_encoder_zero_guard',
                            zero_fault_capture='&g_encoder_zero_fault_capture')
    base.LENGTHS.update(logic_trigger=48, sensor_zero_guard=32, zero_fault_capture=328)
    decode, ready, clean, running = base.decode, base.require_ready, base.require_clean_counters, speed.check_running

    def decode_with_trigger(blocks):
        state = decode(blocks)
        fields = struct.unpack('<10I8s', blocks['logic_trigger'])
        state.update(logic_version=fields[0], logic_mode=fields[1], logic_state=fields[3],
                     logic_reason=fields[5], logic_cycle=fields[6],
                     zero_fields=list(struct.unpack('<8I', blocks['sensor_zero_guard'])))
        if pwm_diagnostics:
            fault = struct.unpack('<22I', blocks['pwm_fault'])
            timing = struct.unpack('<5I', blocks['pwm_timing'])
            state.update(pwm_commit_fault_count=fault[1], pwm_commit_fault_reason=fault[2],
                         pwm_commit_success_count=timing[0], pwm_commit_min_remaining_ticks=timing[1],
                         pwm_commit_max_adc_cycles=timing[2], pwm_commit_last_remaining_ticks=timing[3],
                         pwm_commit_last_adc_cycles=timing[4])
        return state

    def ready_with_trigger(state, *args, **kwargs):
        ready(state, *args, **kwargs)
        require_trigger_healthy(state)

    def clean_with_trigger(before, after):
        clean(before, after)
        require_trigger_healthy(before)
        require_trigger_healthy(after)

    def running_with_trigger(state, *args, **kwargs):
        running(state, *args, **kwargs)
        require_trigger_healthy(state)

    base.decode, base.require_ready = decode_with_trigger, ready_with_trigger
    base.require_clean_counters, speed.check_running = clean_with_trigger, running_with_trigger
    (base.main if kind == 'alignment' else speed.main)()


if __name__ == '__main__':
    main()
