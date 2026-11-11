import math
import struct
import unittest

from calibrate_adc_zero_off import propose_offsets, raw_capture, current_scale


class OffCalibrationTests(unittest.TestCase):
    def test_rate_and_hardware_timebase_must_match(self):
        for rate, arr in ((10000, 8400), (16000, 5250), (20000, 4199)):
            config = struct.pack('<If4I', 7000, 6.1, 3300, 4095, 1, rate)
            self.assertAlmostEqual(current_scale(config, struct.pack('<3I', 0, arr, 1)), .018872619, places=8)
            for timer in ((1, arr, 1), (0, arr, 0), (0, arr+1, 1)):
                with self.assertRaises(RuntimeError):
                    current_scale(config, struct.pack('<3I', *timer))
        for rate in (0, 10000, 32000):
            with self.assertRaises(RuntimeError):
                current_scale(struct.pack('<If4I', 7000, 6.1, 3300, 4095, 1, rate),
                              struct.pack('<3I', 0, 5250, 1))

    def capture(self):
        return struct.pack('<8I', 3, 1024, 0xfffffc10, 0xf, 2106, 2117, 1, 1)+struct.pack('<2048H', *([2112, 2113]*1024))

    def test_contiguous_wrap_and_stable_correction(self):
        sample = raw_capture(self.capture(), .01887, (2106, 2117))
        self.assertEqual(propose_offsets(sample, sample, (2106, 2117), .01887), (2112, 2113))

    def test_reject_bad_capture(self):
        for offset, value in ((0, 2), (4, 1023), (12, 0x10), (16, 2105), (24, 4), (28, 2)):
            data = bytearray(self.capture())
            struct.pack_into('<I', data, offset, value)
            with self.assertRaises(ValueError):
                raw_capture(data, .01887, (2106, 2117))
        data = bytearray(self.capture())
        struct.pack_into('<H', data, 32, 4095)
        with self.assertRaises(ValueError):
            raw_capture(data, .01887, (2106, 2117))

    def test_reject_drift_large_shift_or_bad_scale(self):
        a = dict(mean_codes=[2112, 2113])
        for b, old, scale in ((dict(mean_codes=[2114, 2113]), (2106, 2117), .01887),
                              (a, (2000, 2000), .01887), (a, (2106, 2117), math.nan)):
            with self.assertRaises(ValueError):
                propose_offsets(a, b, old, scale)


if __name__ == '__main__':
    unittest.main()
