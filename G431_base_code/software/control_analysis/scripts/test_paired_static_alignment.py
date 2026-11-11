import copy
import math
import unittest
from test_encoder_neutral_pwm import adjacent_quarter_endpoint, alignment_id_amplitude, require_alignment_profile, require_voltage_alignment, snapshot_lengths, LENGTHS
from analyze_encoder_alignment import alignment_current_ceiling
from paired_static_alignment import evaluate_pairs, schedule, point_rejections


class PairedTests(unittest.TestCase):
    def test_voltage_contract_and_independent_gates(self):
        require_voltage_alignment(1)
        for v, version, bus in ((0,1,24),(-1,1,24),(1.01,1,24),(math.nan,1,24),
                                (1,2,24),(1,1,math.nan),(1,1,18)):
            with self.assertRaises(ValueError):
                require_voltage_alignment(v, version, bus)
        points=self.points()
        for p in points:
            r=p['analysis']; r['kind']=10
            del r['rms_tail_id_error_a']
            r.update(rms_tail_ud_error_v=.001,rms_tail_uq_v=.001,measured_tail_rms_a=4)
        self.assertTrue(evaluate_pairs(points)['accepted'])
        points[0]['analysis']['rms_tail_ud_error_v']=.02
        self.assertFalse(evaluate_pairs(points)['accepted'])
        points[0]['analysis']['rms_tail_ud_error_v']=.001
        points[0]['analysis']['measured_tail_rms_a']=6.1
        self.assertFalse(evaluate_pairs(points)['accepted'])
        points[0]=self.points()[0]
        self.assertFalse(evaluate_pairs(points)['accepted'])

    def test_compact_status_preserves_records_and_fault_blocks(self):
        self.assertEqual(snapshot_lengths('stopped')['test'], 4416)
        for name in ('before', 'baseline', 'armed', 'reserved', 'after', 'restored'):
            lengths = snapshot_lengths(name)
            self.assertEqual(lengths['test'], 192)
            self.assertEqual({k: v for k, v in lengths.items() if k != 'test'},
                             {k: v for k, v in LENGTHS.items() if k != 'test'})
        self.assertEqual(LENGTHS['test'], 4416)

    def test_explicit_rms_contract(self):
        self.assertEqual(alignment_id_amplitude(3), 3)
        self.assertAlmostEqual(alignment_id_amplitude(0, 5), 7.0710678118654755)
        for old, rms in ((3, 5), (0, 5.01), (0, -1), (0, 0), (0, float('nan')), (0, float('inf')), (3.01, None)):
            with self.assertRaises(ValueError):
                alignment_id_amplitude(old, rms)
        require_alignment_profile(1)
        require_alignment_profile(2)
        require_alignment_profile(2, 5)
        for version in (0, 1, 3):
            with self.assertRaises(ValueError):
                require_alignment_profile(version, 5)

    def test_offline_analysis_rejects_wrong_rms_units_or_firmware(self):
        config = dict(requested_rms_a=5, current_contract_version=2)
        self.assertGreater(alignment_current_ceiling(1, 5, 5*math.sqrt(2), config, 2), 7.071)
        self.assertEqual(alignment_current_ceiling(1, 5, 3, {}), 3)
        for kind, duration, current, version in ((9, 5, 5*math.sqrt(2), 2),
                (1, 4, 5*math.sqrt(2), 2), (1, 5, 5, 2), (1, 5, 5*math.sqrt(2), 1)):
            with self.assertRaises(ValueError):
                alignment_current_ceiling(kind, duration, current, config, version)

    def test_adjacent_wrap_and_direction(self):
        for step in (-1, 1):
            for n in range(-2, 3):
                for error in (-.1, .1):
                    start = (n*math.pi/2 + error + math.pi) % (2*math.pi) - math.pi
                    phase = ((n+step)%4)*math.pi/2
                    end = adjacent_quarter_endpoint(start, phase, step)
                    self.assertAlmostEqual(end-start, step*math.pi/2-error)
                    self.assertLess(abs(end), 2*math.pi)
        for start, phase, step in ((0, 0, 1), (0, math.pi, 1), (4, 0, -1), (0, .3, 1), (0, math.pi/2, 0)):
            with self.assertRaises(ValueError):
                adjacent_quarter_endpoint(start, phase, step)

    def points(self):
        points = []
        for i, (role, b, direction) in enumerate(schedule()):
            offset = 5.6+direction*math.radians(2)
            points.append(dict(role=role, bin=b, step_direction=direction,
                analysis=dict(run=str(i), kind=1, pole_pairs=2, direction=-1,
                    tail_mechanical_deg=(20+45*b-direction)%360, estimated_offset_rad=offset,
                    tail_std_electrical_deg=.05, rms_tail_id_error_a=.05,
                    rms_tail_iq_command_frame_a=.05, peak_tail_speed_rad_s=.02)))
        return points

    def test_pair_cancels_symmetric_friction(self):
        result = evaluate_pairs(self.points())
        self.assertTrue(result['accepted'], result)
        self.assertAlmostEqual(result['offset_rad'], 5.6)

    def test_single_point_quality_is_checked_before_next_motion(self):
        point = self.points()[0]
        self.assertEqual(point_rejections(point, 20), [])
        point['analysis']['rms_tail_iq_command_frame_a'] = .151
        self.assertTrue(point_rejections(point, 20))
        point = self.points()[0]
        point['analysis']['tail_mechanical_deg'] = float('nan')
        self.assertTrue(point_rejections(point, 20))

    def test_bad_geometry_cannot_average_away(self):
        points = self.points()
        points[3]['analysis']['tail_mechanical_deg'] += 180
        self.assertFalse(evaluate_pairs(points)['accepted'])
        self.assertFalse(evaluate_pairs(points[:-1])['accepted'])

    def test_existing_quality_bounds_preserved(self):
        for key, value in (('estimated_offset_rad', 5.9), ('tail_std_electrical_deg', 1.1),
                           ('rms_tail_id_error_a', .16), ('rms_tail_iq_command_frame_a', .16),
                           ('peak_tail_speed_rad_s', .21)):
            points = self.points()
            points[4]['analysis'][key] = value
            self.assertFalse(evaluate_pairs(points)['accepted'])
        points = self.points()
        points[-1]['analysis']['estimated_offset_rad'] += math.radians(2.1)
        self.assertFalse(evaluate_pairs(points)['accepted'])
        points = self.points()
        for p in points:
            if p['bin'] == 4:
                p['analysis']['estimated_offset_rad'] += math.radians(4)
        self.assertFalse(evaluate_pairs(points)['accepted'])


if __name__ == '__main__':
    unittest.main()
