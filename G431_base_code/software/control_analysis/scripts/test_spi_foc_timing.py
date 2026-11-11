"""Pure offline tests; no hardware/probe or firmware changes."""
import tempfile
import unittest
from pathlib import Path

from analyze_spi_foc_timing import correlate, read_pulses


def frame(i, start, end):
    return dict(frame_index=str(i), start_ns=str(start), end_ns=str(end),
                partial_start='0', partial_end='0', included='1', tx_hex='A0 03 00 00 00 00')


def pulse(rise, fall):
    return dict(rise_ns=rise, fall_ns=fall,
                width_ns=fall-rise if rise is not None and fall is not None else None)


class TimingTests(unittest.TestCase):
    def read(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'capture.txt'
            path.write_text('Time[s],CH0,CH1,CH2,CH3,CH5\n'+rows, encoding='utf-8')
            return read_pulses(path, 5, (2, 3, 1, 0))

    def test_negative_time_partial_pulses(self):
        pulses, start, end = self.read('-0.000000020,1,1,0,1,1\n'
                                      '-0.000000010,1,1,0,1,0\n'
                                      '0,1,1,0,1,1\n'
                                      '0.000000010,1,1,0,1,0\n'
                                      '0.000000020,1,1,0,1,1\n')
        self.assertEqual((start, end), (-20, 20))
        self.assertEqual(pulses, [pulse(None, -10), pulse(0, 10), pulse(20, None)])

    def test_reject_unknown_backwards_and_same_time_toggle(self):
        for rows in ('0,1,1,0,1,x\n',
                     '0.1,1,1,0,1,0\n0,1,1,0,1,1\n',
                     '0,1,1,0,1,0\n0,1,1,0,1,1\n0,1,1,0,1,0\n'):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.read(rows)

    def test_partial_end_not_a_zero_width(self):
        result = correlate([frame(0, -100, -50)], [pulse(-10, None)], 10)
        self.assertEqual(result['associations'][0]['cs_release_to_rise_ns'], 40)
        self.assertIsNone(result['command_timing_ns']['A0 03 00 00 00 00']['width_ns'])
        self.assertEqual(result['unmatched_pulses'], [])

    def test_fall_at_zero_is_not_missing(self):
        result = correlate([frame(0, -100, -50), frame(1, 10, 20)],
                           [pulse(-10, 0)], 100)
        self.assertFalse(result['associations'][0]['overlaps_next_spi'])

    def test_overlaps_missing_and_multiple(self):
        result = correlate([frame(0, 0, 10), frame(1, 100, 110), frame(2, 200, 210)],
                           [pulse(-20, -10), pulse(5, 15), pulse(20, 30),
                            pulse(40, 50), pulse(220, 230)], 250)
        self.assertEqual(len(result['unmatched_pulses']), 2)
        self.assertEqual(result['frames_with_multiple_rises'], [0])
        self.assertEqual(result['frames_without_association'], [1])
        self.assertEqual(result['command_timing_ns']['A0 03 00 00 00 00']['width_ns']['count'], 1)

    def test_zero_slot_comparison_and_next_overlap(self):
        zero = dict(frame(1, 100, 110), included='0', rising_clocks='24', falling_clocks='24',
                    tx_hex='30 09 00', rx_hex='FF FF 00', excluded_reasons='clock_count_not_48')
        result = correlate([frame(0, 0, 10), zero, frame(2, 200, 210)],
                           [pulse(30, 105), pulse(120, 130), pulse(230, 240)], 250)
        self.assertTrue(result['associations'][0]['overlaps_next_spi'])
        self.assertEqual(result['zero_slot_early_shifts'],
                         [dict(frame_index=1, early_vs_adjacent_angle_rises_ns=10)])


if __name__ == '__main__':
    unittest.main()
