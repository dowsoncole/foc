"""Hardware-free tests for the firmware V2 variable waveform protocol."""

import math
import unittest

from servo_transport import (
    ADC_SAMPLES_PER_FRAME,
    CHANNEL_DEFINITIONS,
    DEFAULT_BAUDRATE,
    SAMPLE_RATE_HZ,
    SineWaveformSource,
    SourceStats,
    TransportError,
    WaveformParser,
    build_frame,
    crc8,
    estimate_frequency,
    frame_length,
    parse_baudrate,
    to_scope_sample,
)


class ProtocolV2Tests(unittest.TestCase):
    def test_variable_frame_lengths_and_crc(self) -> None:
        for count in range(1, 5):
            with self.subTest(count=count):
                channels = tuple((channel_id, channel_id / 10) for channel_id in range(1, count + 1))
                frame = build_frame(3, 0x1234, 0x12345678, channels)
                self.assertEqual(len(frame), 12 + 5 * count)
                self.assertEqual(len(frame), frame_length(count))
                self.assertEqual(frame[:5], bytes((0xA5, 0x5A, 2, 3, count)))
                self.assertEqual(frame[5:7], b"\x34\x12")
                self.assertEqual(crc8(frame[:-1]), frame[-1])

    def test_decodes_mode_ids_values_names_and_units(self) -> None:
        encoded = build_frame(2, 7, 140, ((4, 1.25), (5, -0.5), (6, 2.0), (7, 1.75)))
        frame = WaveformParser().feed(encoded)[0]
        self.assertEqual(frame.mode, 2)
        self.assertEqual(frame.mode_name, "电流开环")
        self.assertEqual(frame.channel_ids, (4, 5, 6, 7))
        self.assertEqual(frame.values, (1.25, -0.5, 2.0, 1.75))
        self.assertEqual([channel.unit for channel in frame.channels], ["A"] * 4)
        self.assertEqual(frame.channels[0].name, "D电流指令")

    def test_custom_mixed_units_are_bound_by_channel_id(self) -> None:
        frame = WaveformParser().feed(
            build_frame(3, 1, 20, ((1, -1.0), (8, 0.625), (13, 3.142), (14, 48.2)))
        )[0]
        self.assertEqual([channel.name for channel in frame.channels], ["U相电流", "D电压输出", "电角度", "直流母线电压"])
        self.assertEqual([channel.unit for channel in frame.channels], ["A", "pu", "rad", "V"])
        self.assertEqual(to_scope_sample(frame), (-1.0, 0.625, 3.142, 48.2))

    def test_mixed_lengths_chunking_corruption_and_resync(self) -> None:
        first = build_frame(1, 1, 20, ((1, 1.0), (2, -0.4), (3, -0.6)))
        damaged = bytearray(build_frame(3, 2, 40, ((14, 48.0),)))
        damaged[-1] ^= 0xFF
        last = build_frame(2, 3, 60, ((4, 0.5), (5, 0.4), (6, 1.0), (7, 0.9)))
        stream = b"\x00\x99" + first + bytes(damaged) + last
        for chunk_size in (1, 3, 11, 17, 27, 32):
            parser = WaveformParser()
            frames = []
            for offset in range(0, len(stream), chunk_size):
                frames.extend(parser.feed(stream[offset:offset + chunk_size]))
            self.assertEqual([frame.mode for frame in frames], [1, 2])
            self.assertEqual([len(frame.channels) for frame in frames], [3, 4])
            self.assertGreaterEqual(parser.stats.crc_errors, 1)
            self.assertEqual(parser.stats.sequence_gaps, 1)

    def test_invalid_version_count_mode_and_channel_id_are_rejected(self) -> None:
        mutations = []
        base = bytearray(build_frame(1, 1, 20, ((1, 1.0),)))
        for offset, value in ((2, 1), (3, 9), (4, 0), (11, 99)):
            invalid = bytearray(base)
            invalid[offset] = value
            invalid[-1] = crc8(invalid[:-1])
            mutations.append(bytes(invalid))
        for invalid in mutations:
            parser = WaveformParser()
            self.assertEqual(parser.feed(invalid), [])
            self.assertGreaterEqual(parser.stats.protocol_errors, 1)

    def test_adc_and_frame_sequence_rollovers(self) -> None:
        parser = WaveformParser()
        parser.feed(
            build_frame(1, 0xFFFF, 0xFFFFFFF0, ((1, 0.0),))
            + build_frame(1, 0, 4, ((1, 0.0),))
            + build_frame(1, 1, 99, ((1, 0.0),))
        )
        self.assertEqual(parser.stats.sequence_gaps, 0)
        self.assertEqual(parser.stats.adc_discontinuities, 1)

    def test_split_header_and_reset(self) -> None:
        parser = WaveformParser()
        encoded = build_frame(1, 1, 20, ((1, 1.0),))
        self.assertEqual(parser.feed(b"\x88\xA5"), [])
        self.assertEqual(len(parser.feed(encoded[1:])), 1)
        parser.reset()
        self.assertEqual(parser.stats, SourceStats())

    def test_channel_catalog_matches_documented_ids(self) -> None:
        self.assertEqual(set(CHANNEL_DEFINITIONS), set(range(1, 15)))
        self.assertEqual(CHANNEL_DEFINITIONS[8].unit, "pu")
        self.assertEqual(CHANNEL_DEFINITIONS[13].unit, "rad")
        self.assertEqual(CHANNEL_DEFINITIONS[14].unit, "V")

    def test_compact_statistics_include_mode_and_errors(self) -> None:
        stats = SourceStats(frames=12, crc_errors=1, protocol_errors=2, sequence_gaps=3, adc_discontinuities=4, dropped_samples=5, last_mode=2)
        self.assertEqual(stats.compact_summary(), "帧 12 | 电流开环 | CRC 1 | 协议 2 | 跳 3 | ADC 4 | 丢 5")


class SineSourceTests(unittest.TestCase):
    def test_demo_source_uses_v2_three_phase_frames(self) -> None:
        source = SineWaveformSource()
        source.open()
        frames = source.emit(int(SAMPLE_RATE_HZ * 2.5))
        drained = source.drain()
        source.close()
        self.assertEqual(frames, drained)
        self.assertTrue(all(frame.mode == 1 and frame.channel_ids == (1, 2, 3) for frame in frames))
        self.assertTrue(all(abs(sum(frame.values)) <= 0.002 for frame in frames))
        self.assertAlmostEqual(estimate_frequency([frame.values[0] for frame in frames], SAMPLE_RATE_HZ), 1.0, delta=0.05)
        self.assertEqual(frames[1].adc_sequence - frames[0].adc_sequence, ADC_SAMPLES_PER_FRAME)
        self.assertEqual(source.stats.crc_errors, 0)

    def test_sequence_rolls_over_at_16_bits(self) -> None:
        source = SineWaveformSource(rate_hz=10)
        source.open()
        source._emitted = 0xFFFF
        frames = source.emit(2)
        source.close()
        self.assertEqual([frame.frame_sequence for frame in frames], [0xFFFF, 0])


class UtilityTests(unittest.TestCase):
    def test_baudrate_labels(self) -> None:
        self.assertEqual(parse_baudrate("921600 bps"), DEFAULT_BAUDRATE)
        self.assertEqual(parse_baudrate("1 Mbps"), 1_000_000)
        with self.assertRaises(TransportError):
            parse_baudrate("invalid")

    def test_frequency_estimation(self) -> None:
        values = [100.0 * math.sin(2 * math.pi * 5.0 * i / 1000.0) for i in range(1000)]
        self.assertAlmostEqual(estimate_frequency(values, 1000.0), 5.0, delta=0.2)
        self.assertEqual(estimate_frequency([1.0, 2.0], 1000.0), 0.0)


if __name__ == "__main__":
    unittest.main()
