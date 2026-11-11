"""Synthetic/offline regression only. No probe imports or hardware access."""
import math
import unittest

from analyze_encoder_alignment import circular_stats, evaluate_sweep, evaluate_static_holds, field_angle, wrap_error
from test_encoder_neutral_pwm import require_clean_counters


def sweep_rows(offset=5.6, hysteresis_deg=2, ripple_deg=0):
    rows = []
    for i in range(120):
        t = i*.1
        field = max(0, min(1, (t-3)/3 if t < 6 else (9-t)/3))*4*math.pi
        sign = 1 if 3 < t < 6 else -1 if 6 < t < 9 else 0
        error = math.radians(hysteresis_deg)*sign
        error += math.radians(ripple_deg)*math.sin(field/2)
        mech = ((field-offset-error)/2) % (2*math.pi)
        rows.append((t, field, sign*2.0944, 3, 0, 0, mech, 0))
    return rows


class AlignmentTests(unittest.TestCase):
    def static_runs(self):
        return [dict(run=str(i), kind=1, pole_pairs=2, direction=-1,
                     field_end_deg=x, estimated_offset_rad=5.6,
                     tail_std_electrical_deg=.05, rms_tail_id_error_a=.02,
                     rms_tail_iq_command_frame_a=.02, peak_tail_speed_rad_s=.01)
                for i, x in enumerate((0, 90, 180, 270, 360, -360, 0))]

    def test_static_requires_coverage_and_repeatability(self):
        runs = self.static_runs()
        self.assertTrue(evaluate_static_holds(runs)['accepted'])
        self.assertFalse(evaluate_static_holds(runs[:-1])['accepted'])
        runs[1]['field_end_deg'] = 0
        self.assertFalse(evaluate_static_holds(runs)['accepted'])
        runs = self.static_runs()
        runs[-1]['estimated_offset_rad'] += math.radians(2.1)
        self.assertFalse(evaluate_static_holds(runs)['accepted'])

    def test_static_rejects_position_error_or_unsettled_tail(self):
        for key, value in [('estimated_offset_rad', 5.8),
                           ('tail_std_electrical_deg', 1.1),
                           ('rms_tail_id_error_a', .16),
                           ('rms_tail_iq_command_frame_a', .16),
                           ('peak_tail_speed_rad_s', .21)]:
            runs = self.static_runs()
            runs[2][key] = value
            with self.subTest(key=key):
                self.assertFalse(evaluate_static_holds(runs)['accepted'])

    def test_circular_mean_across_wrap(self):
        mean, std = circular_stats([math.radians(359), math.radians(1)])
        self.assertAlmostEqual(wrap_error(mean), 0)
        self.assertAlmostEqual(math.degrees(std), 1)
        for angles in ([], [0, math.pi], [float('nan')]):
            with self.assertRaises(ValueError):
                circular_stats(angles)

    def test_legacy_field_is_float_not_integer_steps(self):
        self.assertAlmostEqual(field_angle(2, 5, 2*math.pi), .6*math.pi)
        self.assertEqual(field_angle(0, 5, 1), 0)
        self.assertEqual(field_angle(5, 5, 1), 1)
        self.assertEqual(field_angle(0, 5, -4.7, 3), 3)
        self.assertEqual(field_angle(1.25, 5, -4.7, 3), 3)
        self.assertAlmostEqual(field_angle(2.5, 5, -4.7, 3), -.85)
        self.assertAlmostEqual(field_angle(5, 5, -4.7, 3), -4.7)

    def test_bidirectional_bias_cancels(self):
        result = evaluate_sweep(sweep_rows())
        self.assertTrue(result['accepted'], result)
        self.assertAlmostEqual(result['offset_rad'], 5.6)
        self.assertLess(result['max_position_residual_deg'], 1e-10)

    def test_bad_hysteresis_and_position_error_rejected(self):
        self.assertFalse(evaluate_sweep(sweep_rows(hysteresis_deg=4))['accepted'])
        self.assertFalse(evaluate_sweep(sweep_rows(ripple_deg=6))['accepted'])

    def test_missing_coverage_and_current_tracking_rejected(self):
        self.assertFalse(evaluate_sweep(sweep_rows()[:60])['accepted'])
        rows = sweep_rows()
        r = list(rows[40]); r[3] = 2.5; rows[40] = tuple(r)
        self.assertFalse(evaluate_sweep(rows)['accepted'])

    def test_all_transport_counters_gate_calibration(self):
        good = dict(crc_errors=0, step_errors=40, alarms=0, start_errors=0, transfer_errors=0,
                    deadline_overruns=0)
        require_clean_counters(good, good)
        for key in good:
            bad = dict(good); bad[key] += 1
            with self.subTest(key=key), self.assertRaises(AssertionError):
                require_clean_counters(good, bad)


if __name__ == '__main__':
    unittest.main()
