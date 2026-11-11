"""Offline tests only: never constructs ProbeRun or connects hardware."""
import unittest
from test_highspeed_closed_loop import (check_running, parse_stages, parse_initial_energy,
                                       parse_final_dwell, preserve_initial_energy)
from test_encoder_neutral_pwm import require_ready


class GuardTests(unittest.TestCase):
    def test_preserve_recorded_heat_floor(self):
        self.assertEqual(preserve_initial_energy(0, True, {'i2t_initial_energy_a2s': 82.567}), 82.567)
        self.assertEqual(preserve_initial_energy(90, True, {'i2t_initial_energy_a2s': 82.567}), 90)
        self.assertEqual(preserve_initial_energy(0, True, {'i2t_energy_a2s': 100},
                                                {'i2t_energy_a2s': 101}), 101)
        self.assertEqual(preserve_initial_energy(0, True, {}), 0)
        self.assertEqual(preserve_initial_energy(0, False, {'i2t_initial_energy_a2s': 82.567}), 0)

    def test_invalid_recorded_heat_rejected_before_actuation(self):
        for key in ('i2t_energy_a2s', 'i2t_initial_energy_a2s'):
            for value in (-1, float('nan'), float('inf'), 1440, 1439.9999):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    preserve_initial_energy(100, True, {key: value})
        with self.assertRaises(ValueError):
            preserve_initial_energy(1, False, {})

    def test_final_hold(self):
        self.assertEqual(parse_final_dwell(None, 10), 10)
        self.assertEqual(parse_final_dwell(30, 10), 30)
        self.assertEqual(parse_final_dwell(10, 10), 10)
        for final, dwell in ((9, 10), (31, 10), (float('nan'), 10),
                             (float('inf'), 10), (None, float('nan')), (None, 1)):
            with self.subTest(final=final, dwell=dwell), self.assertRaises(ValueError):
                parse_final_dwell(final, dwell)

    def test_heat_carry_input(self):
        self.assertEqual(parse_initial_energy(82.563, True), 82.563)
        self.assertEqual(parse_initial_energy(0, False), 0)
        for value, enabled in ((-1, True), (float('nan'), True), (float('inf'), True),
                               (1440, True), (1439.9999, True), (82.563, False)):
            with self.subTest(value=value, enabled=enabled), self.assertRaises(ValueError):
                parse_initial_energy(value, enabled)
    def test_uncalibrated_alignment_keeps_other_interlocks(self):
        state = dict(moe=False, enable=0, runtime_bytes=[0, 0, 0, 0, 1, 0],
                     controller_running=0, calibration=3, calibration_ok=0,
                     mode=[1, 1], stage=[2, 2], encoder_valid=[0, 1, 1, 1],
                     encoder_flags=0, mechanical_speed_rad_s=0, current_test_state=0,
                     tuning_request=0, current_trip_count=0, runtime_fault=0, controller_fault=0)
        with self.assertRaises(AssertionError):
            require_ready(state)
        require_ready(state, calibration_required=False)
        for change in ({'moe': True}, {'calibration': 2}, {'encoder_flags': 2},
                       {'current_trip_count': 1}, {'mechanical_speed_rad_s': 2},
                       {'runtime_bytes': [0, 0, 0, 0, 0, 0]}):
            with self.subTest(change=change), self.assertRaises(AssertionError):
                require_ready(dict(state, **change), calibration_required=False)

    def setUp(self):
        self.baseline = dict(crc_errors=0, alarms=0, step_errors=0, start_errors=0, transfer_errors=0,
                             deadline_overruns=0, pwm_window_rejections=0, pwm_deadline_rejections=0)
        self.good = dict(self.baseline, runtime_fault=0, controller_fault=0, run_fault=0,
                         encoder_flags=0, guard_expired=0, enable=1, moe=True, run_state=2,
                         actual_rpm=500, applied_rpm=500, iq_a=.2, id_a=0,
                         iq_reference_a=.2, uq_pu=.03, phase_peak_a=.4,
                         encoder_sequence=100, run_elapsed_s=2)

    def test_good(self):
        check_running(self.good, self.baseline)
        check_running(self.good, self.baseline, dict(self.good, encoder_sequence=90, run_elapsed_s=1))

    def test_stage_envelope(self):
        self.assertEqual(parse_stages('500,7500,8500,9000', 5), [500, 7500, 8500, 9000])
        for value, dwell in (('9001', 5), ('10000', 5), ('nan', 5), ('inf', 5),
                             ('-500', 5), ('500,500', 5), ('1000,500', 5),
                             ('500', float('nan')), ('500', 31), ('500', 1)):
            with self.subTest(value=value, dwell=dwell), self.assertRaises(ValueError):
                parse_stages(value, dwell)

    def test_failures(self):
        failures = dict(crc_errors=1, alarms=1, step_errors=1, start_errors=1, transfer_errors=1, deadline_overruns=1,
                        pwm_window_rejections=1, pwm_deadline_rejections=1,
                        runtime_fault=6, controller_fault=6, run_fault=13, encoder_flags=2,
                        guard_expired=1, enable=0, moe=False, run_state=4, actual_rpm=-2000,
                        iq_a=float('nan'))
        for key, value in failures.items():
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                check_running(dict(self.good, **{key: value}), self.baseline)
        with self.assertRaises(RuntimeError):
            check_running(dict(self.good, actual_rpm=9073), self.baseline)

    def test_stale(self):
        for previous in (self.good, dict(self.good, encoder_sequence=90),
                         dict(self.good, run_elapsed_s=1)):
            with self.assertRaises(RuntimeError):
                check_running(self.good, self.baseline, previous)

    def test_overload_envelope(self):
        active = dict(self.good, phase_peak_a=25.5, overload_armed=1, overload_used=1, overload_expired=0,
                      overload_invalid=0, i2t_energy_a2s=1000, i2t_budget_a2s=1440, current_rms_a=18)
        check_running(active, self.baseline, overload_mode=True)
        with self.assertRaises(RuntimeError):
            check_running(dict(active, i2t_energy_a2s=0), self.baseline,
                          overload_mode=True, initial_energy=82.563)
        for changes in ({'overload_armed': 0},
                        {'overload_used': 0}, {'overload_expired': 1}, {'overload_invalid': 1},
                        {'i2t_energy_a2s': 1440}, {'i2t_energy_a2s': float('nan')},
                        {'i2t_budget_a2s': 2000}):
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                check_running(dict(active, **changes), self.baseline, overload_mode=True)
        check_running(dict(active, phase_peak_a=40), self.baseline)
        check_running(dict(active, phase_peak_a=40), self.baseline, overload_mode=True)


if __name__ == '__main__':
    unittest.main()
