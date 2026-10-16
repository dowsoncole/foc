"""Tests for the RS485 waveform transport layer.

These run without Qt or hardware: they feed byte streams straight into the
parser and drive the local sine generator, so the decode path that the
oscilloscope depends on stays protected.
"""

import unittest

from servo_transport import (
    DEFAULT_BAUDRATE,
    FRAME_LENGTH,
    SAMPLE_RATE_HZ,
    SineWaveformSource,
    SourceStats,
    WaveformFrame,
    WaveformParser,
    build_frame,
    crc8,
    estimate_frequency,
    parse_baudrate,
    to_scope_samples,
)


class Crc8Tests(unittest.TestCase):
    def test_matches_known_polynomial_vectors(self) -> None:
        # CRC-8/poly 0x07 with zero init: "123456789" -> 0xF4.
        self.assertEqual(crc8(b"123456789"), 0xF4)
        self.assertEqual(crc8(b""), 0)

    def test_frame_roundtrip_validates(self) -> None:
        frame = build_frame(sequence=0x2A, sample=-1234)
        self.assertEqual(len(frame), FRAME_LENGTH)
        self.assertEqual(frame[:2], bytes((0xA5, 0x5A)))
        self.assertEqual(crc8(frame[:7]), frame[7])

    def test_negative_sample_survives_little_endian_packing(self) -> None:
        frame = build_frame(sequence=1, sample=-32768)
        self.assertEqual(frame[4:6], bytes((0x00, 0x80)))


class WaveformParserTests(unittest.TestCase):
    def test_parses_contiguous_frames_in_order(self) -> None:
        parser = WaveformParser()
        stream = b"".join(build_frame(index, index * 100) for index in range(5))
        frames = parser.feed(stream)
        self.assertEqual([frame.sequence for frame in frames], [0, 1, 2, 3, 4])
        self.assertEqual([frame.sample for frame in frames], [0, 100, 200, 300, 400])
        self.assertEqual(parser.stats.frames, 5)
        self.assertEqual(parser.stats.crc_errors, 0)
        self.assertEqual(parser.stats.sequence_gaps, 0)

    def test_handles_frames_split_across_reads(self) -> None:
        stream = build_frame(7, 512) + build_frame(8, 1024)
        for chunk_size in (1, 3, 5, 8):
            with self.subTest(chunk_size=chunk_size):
                parser = WaveformParser()
                collected = []
                for offset in range(0, len(stream), chunk_size):
                    collected.extend(parser.feed(stream[offset:offset + chunk_size]))
                self.assertEqual([frame.sequence for frame in collected], [7, 8])
                self.assertEqual([frame.sample for frame in collected], [512, 1024])
                self.assertEqual(parser.stats.crc_errors, 0)

    def test_corrupted_frame_is_skipped_and_stream_resyncs(self) -> None:
        parser = WaveformParser()
        good_before = build_frame(1, 111)
        damaged = bytearray(build_frame(2, 222))
        damaged[7] ^= 0xFF  # break the CRC only
        good_after = build_frame(3, 333)
        frames = parser.feed(good_before + bytes(damaged) + good_after)
        self.assertEqual([frame.sample for frame in frames], [111, 333])
        self.assertGreaterEqual(parser.stats.crc_errors, 1)
        # The bad frame is dropped, so sequence 3 follows 1.
        self.assertEqual(parser.stats.sequence_gaps, 1)

    def test_leading_garbage_is_discarded(self) -> None:
        parser = WaveformParser()
        frames = parser.feed(b"\x00\xff\x13" + build_frame(9, 4242))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sample, 4242)
        self.assertGreaterEqual(parser.stats.resync_bytes, 3)

    def test_truncated_header_byte_is_kept_for_next_chunk(self) -> None:
        parser = WaveformParser()
        frame = build_frame(4, 8)
        first = parser.feed(b"\x99\xA5")  # 0xA5 may start a split header
        second = parser.feed(frame[1:])
        self.assertEqual(first, [])
        self.assertEqual(len(second), 1)
        self.assertEqual(second[0].sample, 8)

    def test_reset_clears_buffer_and_statistics(self) -> None:
        parser = WaveformParser()
        parser.feed(build_frame(1, 5))
        parser.reset()
        self.assertEqual(parser.stats, SourceStats())
        self.assertEqual(parser.feed(build_frame(2, 6))[0].sample, 6)
        self.assertEqual(parser.stats.sequence_gaps, 0)


class FrameValueTests(unittest.TestCase):
    def test_normalization_maps_full_scale_to_scope_range(self) -> None:
        self.assertAlmostEqual(WaveformFrame(1, 0, 32768).normalized, 100.0, places=2)
        self.assertAlmostEqual(WaveformFrame(1, 0, -32768).normalized, -100.0, places=2)
        self.assertAlmostEqual(WaveformFrame(1, 0, 0).normalized, 0.0, places=6)

    def test_scope_conversion_lands_on_channel_one(self) -> None:
        samples = to_scope_samples([WaveformFrame(1, 0, 16384), WaveformFrame(1, 1, -16384)])
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[0][0], 50.0, places=1)
        self.assertAlmostEqual(samples[1][0], -50.0, places=1)
        self.assertEqual(samples[0][1:], (0.0, 0.0, 0.0))


class SineSourceTests(unittest.TestCase):
    def test_generates_one_cycle_per_second_at_sample_rate(self) -> None:
        source = SineWaveformSource()
        source.open()
        # emit() derives phase from its own counter, so exactly rate_hz frames
        # span one cycle with no dependence on wall-clock timing. The window
        # mirrors the scope buffer (about 2.5 cycles) so that the estimator has
        # at least two rising zero crossings to measure between.
        frames = source.emit(int(SAMPLE_RATE_HZ * 2.5))
        source.close()
        self.assertEqual(len(frames), int(SAMPLE_RATE_HZ * 2.5))
        normalized = [frame.normalized for frame in frames]
        self.assertGreater(max(normalized), 90.0)
        self.assertLess(min(normalized), -90.0)
        # One cycle spans rate_hz frames, so the measured frequency is 1 Hz.
        self.assertAlmostEqual(
            estimate_frequency(normalized, SAMPLE_RATE_HZ), 1.0, delta=0.05
        )

    def test_frequency_estimate_needs_a_complete_cycle(self) -> None:
        """A single cycle from phase zero contains no measurable crossing pair."""
        source = SineWaveformSource()
        source.open()
        frames = source.emit(SAMPLE_RATE_HZ)
        source.close()
        normalized = [frame.normalized for frame in frames]
        self.assertEqual(estimate_frequency(normalized, SAMPLE_RATE_HZ), 0.0)

    def test_sequence_numbers_roll_over_at_eight_bits(self) -> None:
        source = SineWaveformSource()
        source.open()
        frames = source.emit(260)
        source.close()
        sequences = [frame.sequence for frame in frames]
        self.assertTrue(all(0 <= value <= 255 for value in sequences))
        self.assertEqual(sequences[0], 0)
        self.assertEqual(sequences[255], 255)
        self.assertEqual(sequences[256], 0)

    def test_closed_source_yields_nothing(self) -> None:
        source = SineWaveformSource()
        source.open()
        source.close()
        self.assertEqual(source.drain(), [])

    def test_emit_returns_pending_frames_via_drain(self) -> None:
        source = SineWaveformSource()
        source.open()
        source.emit(16)
        drained = source.drain()
        self.assertEqual(len(drained), 16)
        self.assertEqual(source.drain(), [])  # already consumed
        source.close()

    def test_generated_frames_pass_the_firmware_parser(self) -> None:
        """The simulator must emit bytes the real parser accepts."""
        parser = WaveformParser()
        frames = _sine_bytes_roundtrip(parser)
        self.assertGreater(len(frames), 0)
        self.assertEqual(parser.stats.crc_errors, 0)


def _sine_bytes_roundtrip(parser: WaveformParser) -> list[WaveformFrame]:
    """Re-encode generator output as bytes, then parse it back."""
    source = SineWaveformSource()
    source.open()
    frames = source.emit(40)
    source.close()
    stream = b"".join(build_frame(frame.sequence, frame.sample) for frame in frames)
    return parser.feed(stream)


class BaudrateParsingTests(unittest.TestCase):
    def test_parses_ui_labels(self) -> None:
        self.assertEqual(parse_baudrate("921600 bps"), 921600)
        self.assertEqual(parse_baudrate("115200 bps"), 115200)
        self.assertEqual(parse_baudrate("1 Mbps"), 1_000_000)
        self.assertEqual(parse_baudrate("500 kbps"), 500_000)

    def test_default_matches_firmware_configuration(self) -> None:
        self.assertEqual(DEFAULT_BAUDRATE, 921600)

    def test_invalid_label_raises_transport_error(self) -> None:
        from servo_transport import TransportError

        with self.assertRaises(TransportError):
            parse_baudrate("not-a-baud")


class FrequencyEstimateTests(unittest.TestCase):
    def test_recovers_frequency_from_synthetic_sine(self) -> None:
        import math

        rate = 1000.0
        values = [
            100.0 * math.sin(2 * math.pi * 5.0 * index / rate)
            for index in range(int(rate))
        ]
        self.assertAlmostEqual(estimate_frequency(values, rate), 5.0, delta=0.2)

    def test_degenerate_input_returns_zero(self) -> None:
        self.assertEqual(estimate_frequency([], 1000.0), 0.0)
        self.assertEqual(estimate_frequency([1.0, 2.0], 1000.0), 0.0)
        self.assertEqual(estimate_frequency([1.0, 2.0, 3.0], 0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
