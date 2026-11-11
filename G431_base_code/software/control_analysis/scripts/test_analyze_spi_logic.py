"""Synthetic electrical-edge fixtures; no probe, target, or Kingst UI access."""
import csv
import json
import tempfile
import tracemalloc
import unittest
from pathlib import Path

from analyze_spi_logic import EXPECTED_TX, analyze_csv, crc8, read_events


GOOD_RX = bytes.fromhex("FF FF 61 E4 08 81")  # Preserved known-good GPIO read.
BAD_RX = bytes.fromhex("FF FF 03 18 51 1E")  # RAM capture: correct CRC is F2.
ALARM_RX = bytes.fromhex("FF FE 04 25 81 CA")  # Captured valid status=1 alarm.


def bits(data):
    return [(byte >> bit) & 1 for byte in data for bit in range(7, -1, -1)]


class Fixture:
    def __init__(self):
        self.rows = []
        self.state = [0, 1, 1, 1]

    def at(self, ns, **changes):
        for key, value in changes.items():
            self.state[int(key[1:])] = value
        self.rows.append((ns, *self.state))

    def burst(self, start=-10000, rx=GOOD_RX, clocks=48, setup=100, hold=200,
              split_falling=False, ambiguous_sample=False, release=True):
        if not self.rows:
            self.at(start - 1000)
        self.at(start, c3=0)
        tx_bits, rx_bits = bits(EXPECTED_TX), bits(rx)
        for index in range(clocks):
            fall = start + setup + index * 400
            tx, data = tx_bits[index] if index < 48 else 0, rx_bits[index] if index < 48 else 0
            if split_falling:
                self.at(fall, c2=0)
                self.at(fall, c0=tx, c1=data)
            else:
                self.at(fall, c2=0, c0=tx, c1=data)
            self.at(fall + 200, c2=1)
            if ambiguous_sample and index == 20:
                self.at(fall + 200, c1=1 - data)
        if release:
            self.at(start + setup + (clocks - 1) * 400 + 200 + hold, c3=1)
        return self

    def save(self, path):
        with path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("Time[s]", "通道 0", "通道 1", "通道 2", "通道 3"))
            for ns, *levels in self.rows:
                sign = "-" if ns < 0 else ""
                value = abs(ns)
                writer.writerow((f"{sign}{value // 1000000000}.{value % 1000000000:09d}", *levels))


class AnalyzerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="spi-logic-test-")
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def run_fixture(self, fixture, **kwargs):
        source = self.root / "capture.csv"
        fixture.save(source)
        summary = analyze_csv(source, self.root / "out", **kwargs)
        with (self.root / "out.frames.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(json.loads((self.root / "out.summary.json").read_text()), summary)
        return summary, rows

    def test_known_crc_vectors(self):
        self.assertEqual(crc8(GOOD_RX[2:5]), 0x81)
        self.assertEqual(crc8(BAD_RX[2:5]), 0xF2)
        self.assertNotEqual(crc8(BAD_RX[2:5]), BAD_RX[-1])
        self.assertEqual(crc8(ALARM_RX[2:5]), 0xCA)

    def test_valid_mode3_crosses_negative_pretrigger_and_timing(self):
        summary, rows = self.run_fixture(Fixture().burst(), match_rx=GOOD_RX)
        row = rows[0]
        self.assertEqual(summary["frames_valid_crc"], 1)
        self.assertEqual(summary["frames_bad_crc"], 0)
        self.assertEqual(summary["matches"][0]["start_ns"], -10000)
        self.assertEqual(row["end_ns"], "9300")
        self.assertEqual(row["tx_hex"], "A0 03 00 00 00 00")
        self.assertEqual(row["rx_hex"], "FF FF 61 E4 08 81")
        self.assertEqual(row["angle_raw"], "801921")
        self.assertEqual(row["cs_setup_ns"], "100")
        self.assertEqual(row["cs_hold_ns"], "200")
        self.assertEqual(row["cs_hold_violation"], "0")
        self.assertEqual(row["sck_high_min_ns"], "200")
        self.assertEqual(row["sck_low_min_ns"], "200")
        self.assertEqual(row["miso_setup_min_ns"], "200")
        self.assertEqual(row["miso_hold_min_ns"], "200")
        self.assertGreater(int(row["miso_hold_censored_samples"]), 0)

    def test_bad_crc_alarm_separation_and_consecutive_run(self):
        fixture = Fixture()
        for index, rx in enumerate((BAD_RX, BAD_RX, GOOD_RX, BAD_RX, ALARM_RX)):
            fixture.burst(start=-10000 + index * 25000, rx=rx)
        summary, rows = self.run_fixture(fixture, match_rx=BAD_RX, max_matches=1)
        self.assertEqual(summary["frames_bad_crc"], 3)
        self.assertEqual(summary["frames_valid_crc_alarm"], 1)
        self.assertEqual(summary["longest_bad_crc_run"], {"length": 2, "first_frame": 0, "last_frame": 1})
        self.assertEqual(summary["matched_frames_total"], 3)
        self.assertTrue(summary["matches_truncated"])
        self.assertEqual(rows[0]["status_raw"], "1")
        self.assertEqual(rows[0]["sensor_alarm_valid"], "0")
        self.assertEqual(rows[-1]["sensor_alarm_valid"], "1")

    def test_47_and_49_clocks_are_excluded_not_bad_crc(self):
        fixture = Fixture().burst(clocks=47).burst(start=20000, clocks=49).burst(start=50000)
        summary, rows = self.run_fixture(fixture)
        self.assertEqual(summary["frames_included"], 1)
        self.assertEqual(summary["frames_excluded"], 2)
        self.assertEqual(summary["frames_bad_crc"], 0)
        self.assertEqual(summary["complete_frames_missing_clocks"], 1)
        self.assertEqual(summary["complete_frames_extra_clocks"], 1)
        self.assertEqual(rows[0]["missing_clocks"], "1")
        self.assertEqual(rows[1]["extra_clocks"], "1")
        self.assertEqual(rows[0]["crc_ok"], "")

    def test_same_time_rows_merge_at_falling_edge(self):
        summary, rows = self.run_fixture(Fixture().burst(split_falling=True))
        self.assertEqual(summary["frames_valid_crc"], 1)
        self.assertEqual(rows[0]["rising_clocks"], "48")
        self.assertEqual(rows[0]["sample_edge_ambiguities"], "0")

    def test_data_changes_on_rising_edge_are_ambiguous(self):
        summary, rows = self.run_fixture(Fixture().burst(ambiguous_sample=True))
        self.assertEqual(summary["frames_excluded"], 1)
        self.assertIn("data_changes_at_sampling_edge", rows[0]["excluded_reasons"])
        self.assertEqual(summary["frames_bad_crc"], 0)

    def test_zero_width_pulse_is_flagged_not_silently_resampled(self):
        fixture = Fixture().burst()
        index = 12  # An existing transition time, during NSS low.
        ns, *values = fixture.rows[index]
        opposite = values.copy()
        opposite[2] = 1 - opposite[2]
        fixture.rows[index + 1:index + 1] = [(ns, *opposite), (ns, *values)]
        summary, rows = self.run_fixture(fixture)
        self.assertEqual(summary["frames_excluded"], 1)
        self.assertEqual(rows[0]["zero_width_events"], "1")
        self.assertIn("multiple_same_timestamp_transitions", rows[0]["excluded_reasons"])

    def test_constant_miso_setup_and_hold_are_censored_not_infinite(self):
        summary, rows = self.run_fixture(Fixture().burst(rx=b"\xff" * 6))
        self.assertEqual(summary["frames_included"], 1)
        self.assertEqual(rows[0]["miso_setup_min_ns"], "")
        self.assertEqual(rows[0]["miso_hold_min_ns"], "")
        self.assertEqual(rows[0]["miso_setup_censored_samples"], "48")
        self.assertEqual(rows[0]["miso_hold_censored_samples"], "48")
        self.assertEqual(rows[0]["miso_hold_end_lower_bound_ns"], "200")

    def test_timing_violations_report_without_hiding_crc(self):
        summary, rows = self.run_fixture(Fixture().burst(setup=50, hold=100))
        self.assertEqual(summary["frames_valid_crc"], 1)
        self.assertEqual(summary["frames_cs_setup_violation"], 1)
        self.assertEqual(summary["frames_cs_hold_violation"], 1)
        self.assertEqual(rows[0]["cs_hold_required_ns"], "200.0")

    def test_partial_start_and_end_are_excluded(self):
        fixture = Fixture().burst()
        fixture.rows = fixture.rows[15:]
        fixture.burst(start=25000, release=False)
        summary, rows = self.run_fixture(fixture)
        self.assertEqual(summary["frames_partial"], 2)
        self.assertEqual(summary["frames_included"], 0)
        self.assertEqual(rows[0]["partial_start"], "1")
        self.assertEqual(rows[1]["partial_end"], "1")

    def test_missing_channel_and_bad_time_are_rejected(self):
        path = self.root / "bad.csv"
        path.write_text("Time[s],Channel 0,Channel 2,Channel 3\n0,0,1,1\n")
        with self.assertRaisesRegex(ValueError, "need four distinct"):
            analyze_csv(path, self.root / "out")
        self.assertFalse((self.root / "out.frames.csv").exists())
        path.write_text("Time[s],Channel 0,Channel 1,Channel 2,Channel 3\n0,0,1,1,1\n-0.1,0,1,1,1\n")
        with self.assertRaisesRegex(ValueError, "backwards"):
            list(read_events(path, (0, 1, 2, 3)))

    def test_existing_evidence_is_not_overwritten(self):
        fixture = Fixture().burst()
        self.run_fixture(fixture)
        original = (self.root / "out.frames.csv").read_bytes()
        with self.assertRaises(FileExistsError):
            analyze_csv(self.root / "capture.csv", self.root / "out")
        self.assertEqual((self.root / "out.frames.csv").read_bytes(), original)

    def test_streaming_memory_does_not_scale_with_rows(self):
        # Generate ~98k edge rows without keeping the aggregate fixture in RAM.
        source = self.root / "large.csv"
        with source.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("Time[s]", "Channel 0", "Channel 1", "Channel 2", "Channel 3"))
            for index in range(1000):
                fixture = Fixture().burst(start=index * 25000)
                for ns, *values in fixture.rows:
                    writer.writerow((f"{ns / 1e9:.9f}", *values))
        tracemalloc.start()
        summary = analyze_csv(source, self.root / "large", max_matches=0)
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        self.assertEqual(summary["frames_valid_crc"], 1000)
        self.assertLess(peak, 4_000_000, f"streaming memory unexpectedly high: {peak}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
