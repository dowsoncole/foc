"""Offline, streaming Kingst edge-CSV decoder for the MT6835 Mode-3 burst.

Default wiring: channel 0=MOSI, 1=MISO, 2=SCK, 3=NSS. No hardware access.
CSV timestamps are change times, not a uniformly sampled sequence. Output files
are created exclusively; existing evidence is never overwritten.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import re
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterator


EXPECTED_TX = bytes.fromhex("A0 03 00 00 00 00")
FIELDS = (
    "frame_index", "start_ns", "end_ns", "duration_ns", "included", "excluded_reasons",
    "partial_start", "partial_end", "rising_clocks", "falling_clocks", "missing_clocks",
    "extra_clocks", "tx_hex", "rx_hex", "tx_expected", "received_crc", "computed_crc",
    "crc_ok", "angle_raw", "status_raw", "sensor_alarm_valid", "match_rx",
    "cs_setup_ns", "cs_setup_violation", "cs_hold_ns", "cs_hold_required_ns",
    "cs_hold_violation", "sck_high_min_ns", "sck_low_min_ns", "sck_period_min_ns",
    "sck_period_max_ns", "miso_setup_min_ns", "miso_setup_observed_samples",
    "miso_setup_censored_samples", "miso_setup_lower_bound_min_ns", "miso_hold_min_ns",
    "miso_hold_observed_samples", "miso_hold_censored_samples",
    "miso_hold_end_lower_bound_ns", "sample_edge_ambiguities", "zero_width_events",
)


def crc8(data: bytes) -> int:
    """CRC-8/poly 0x07, init 0, MSB first, no reflection/final XOR."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value


def minimum(previous: int | None, value: int) -> int:
    return value if previous is None else min(previous, value)


def time_ns(value: str) -> int:
    try:
        scaled = Decimal(value.strip()) * 1_000_000_000
        if not scaled.is_finite() or scaled != scaled.to_integral_value():
            raise ValueError("timestamp must resolve to integer nanoseconds")
        return int(scaled)
    except InvalidOperation as exc:
        raise ValueError(f"invalid timestamp: {value!r}") from exc


def level(value: str) -> int | None:
    text = value.strip().lower()
    if text in ("0", "1"):
        return int(text)
    if text in ("x", "z", "?", ""):
        return None
    raise ValueError(f"invalid digital level: {value!r}")


@dataclass(frozen=True)
class Event:
    ns: int
    levels: tuple[int | None, ...]
    changes: frozenset[int]
    zero_width: bool = False


def read_events(path: Path, channels: tuple[int, int, int, int]) -> Iterator[Event]:
    """Merge equal timestamps before interpreting edges; bounded-memory CSV."""
    # Kingst localized headers may be UTF-8 or an ANSI codepage. ASCII channel
    # suffixes and numeric data survive replacement, so decoding is not tied to
    # Windows' active codepage or the Chinese word for "channel".
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as stream:
        reader = csv.reader(stream, skipinitialspace=True)
        header = next(reader, None)
        if not header or not re.fullmatch(r"time\s*\[s\]", header[0].strip(), re.I):
            raise ValueError("expected Kingst raw CSV header Time[s], channel columns")
        columns: dict[int, int] = {}
        for index, name in enumerate(header[1:], 1):
            match = re.search(r"(\d+)\s*$", name)
            if match:
                channel = int(match.group(1))
                if channel in columns:
                    raise ValueError(f"duplicate channel {channel} in header")
                columns[channel] = index
        if len(set(channels)) != 4 or any(channel not in columns for channel in channels):
            raise ValueError(f"need four distinct MOSI/MISO/SCK/NSS channels {channels}; have {sorted(columns)}")
        positions = [columns[channel] for channel in channels]
        previous: tuple[int | None, ...] | None = None
        current: tuple[int | None, ...] | None = None
        stamp: int | None = None
        changes: set[int] = set()
        toggles = [0, 0, 0, 0]
        for line_number, row in enumerate(reader, 2):
            if not row or not any(cell.strip() for cell in row):
                continue
            if len(row) <= max(positions):
                raise ValueError(f"line {line_number}: incomplete row")
            ns = time_ns(row[0])
            values = tuple(level(row[index]) for index in positions)
            if stamp is not None and ns < stamp:
                raise ValueError(f"line {line_number}: timestamps go backwards")
            if stamp is not None and ns != stamp:
                assert current is not None
                yield Event(stamp, current, frozenset(changes), max(toggles) > 1)
                changes = set()
                toggles = [0, 0, 0, 0]
            if previous is not None:
                for channel in range(4):
                    if values[channel] != previous[channel]:
                        changes.add(channel)
                        toggles[channel] += 1
            previous = current = values
            stamp = ns
        if stamp is not None:
            assert current is not None
            yield Event(stamp, current, frozenset(changes), max(toggles) > 1)


class Frame:
    def __init__(self, index: int, start: int, partial: bool, sck: int | None):
        self.index, self.start, self.partial = index, start, partial
        self.reasons: set[str] = set()
        if partial:
            self.reasons.add("capture_started_with_nss_low")
        if sck != 1:
            self.reasons.add("sck_not_idle_high_at_nss_assert")
        self.rises = self.falls = self.ambiguous = self.zero_width = 0
        self.tx = bytearray()
        self.rx = bytearray()
        self.tx_byte = self.rx_byte = 0
        self.first_fall: int | None = None
        self.last_edge: tuple[int, int] | None = None
        self.last_rise: int | None = None
        self.last_period: int | None = None
        self.period_min: int | None = None
        self.period_max: int | None = None
        self.high_min: int | None = None
        self.low_min: int | None = None
        self.setup_min: int | None = None
        self.setup_observed = self.setup_censored = 0
        self.setup_bound: int | None = None
        self.hold_min: int | None = None
        self.hold_observed = self.pending_hold = 0
        self.pending_rise: int | None = None

    def data_change(self, ns: int) -> None:
        if self.pending_rise is not None:
            # The latest pending rising edge has the smallest hold time when
            # data finally changes: no per-edge queue is necessary.
            self.hold_min = minimum(self.hold_min, ns - self.pending_rise)
            self.hold_observed += self.pending_hold
            self.pending_rise, self.pending_hold = None, 0

    def clock(self, ns: int, rising: bool, event: Event,
              last_miso_change: int | None, capture_start: int) -> None:
        if self.last_edge is not None:
            old_ns, old_level = self.last_edge
            if old_level == 1:
                self.high_min = minimum(self.high_min, ns - old_ns)
            else:
                self.low_min = minimum(self.low_min, ns - old_ns)
        self.last_edge = ns, int(rising)
        if not rising:
            self.falls += 1
            if self.first_fall is None:
                self.first_fall = ns
            return
        self.rises += 1
        if self.last_rise is not None:
            self.last_period = ns - self.last_rise
            self.period_min = minimum(self.period_min, self.last_period)
            self.period_max = max(self.period_max or 0, self.last_period)
        self.last_rise = ns
        if 0 in event.changes or 1 in event.changes:
            self.ambiguous += 1
            self.reasons.add("data_changes_at_sampling_edge")
        if last_miso_change is None:
            self.setup_censored += 1
            self.setup_bound = minimum(self.setup_bound, ns - capture_start)
        else:
            self.setup_observed += 1
            self.setup_min = minimum(self.setup_min, ns - last_miso_change)
        self.pending_rise, self.pending_hold = ns, self.pending_hold + 1
        mosi, miso = event.levels[:2]
        if mosi is None or miso is None:
            self.reasons.add("unknown_level_at_sampling_edge")
        # Retain only the first six bytes even if NSS is stuck low for an
        # enormous file. Clock counts still account for every edge.
        if self.rises <= 48:
            self.tx_byte = (self.tx_byte << 1) | int(mosi or 0)
            self.rx_byte = (self.rx_byte << 1) | int(miso or 0)
            if self.rises % 8 == 0:
                self.tx.append(self.tx_byte)
                self.rx.append(self.rx_byte)
                self.tx_byte = self.rx_byte = 0

    def finish(self, end: int, partial_end: bool, match_rx: bytes | None) -> dict:
        if partial_end:
            self.reasons.add("capture_ended_before_nss_release")
        if self.rises != 48 or self.falls != 48:
            self.reasons.add("clock_count_not_48")
        tx_expected = self.rises == 48 and bytes(self.tx) == EXPECTED_TX
        if not tx_expected:
            self.reasons.add("not_expected_a003_angle_burst")
        included = not self.reasons
        crc = crc8(bytes(self.rx[2:5])) if len(self.rx) == 6 else None
        crc_ok = (crc == self.rx[5]) if included else None
        setup = None if self.first_fall is None else self.first_fall - self.start
        hold = None if self.last_rise is None else end - self.last_rise
        required = None if self.last_period is None else self.last_period / 2
        return {
            "frame_index": self.index, "start_ns": self.start, "end_ns": end,
            "duration_ns": end - self.start, "included": int(included),
            "excluded_reasons": ";".join(sorted(self.reasons)),
            "partial_start": int(self.partial), "partial_end": int(partial_end),
            "rising_clocks": self.rises, "falling_clocks": self.falls,
            "missing_clocks": max(0, 48 - self.rises), "extra_clocks": max(0, self.rises - 48),
            "tx_hex": self.tx.hex(" ").upper(), "rx_hex": self.rx.hex(" ").upper(),
            "tx_expected": int(tx_expected),
            "received_crc": f"{self.rx[5]:02X}" if len(self.rx) == 6 else None,
            "computed_crc": f"{crc:02X}" if crc is not None else None,
            "crc_ok": None if crc_ok is None else int(crc_ok),
            "angle_raw": int.from_bytes(self.rx[2:5], "big") >> 3 if included else None,
            "status_raw": self.rx[4] & 7 if included else None,
            "sensor_alarm_valid": int(bool(included and crc_ok and (self.rx[4] & 7))),
            "match_rx": int(bool(self.rises == 48 and len(self.rx) == 6 and
                                  "unknown_level_at_sampling_edge" not in self.reasons and
                                  match_rx is not None and bytes(self.rx) == match_rx)),
            "cs_setup_ns": setup,
            "cs_setup_violation": None if setup is None or self.partial else int(setup < 100),
            "cs_hold_ns": hold, "cs_hold_required_ns": required,
            "cs_hold_violation": None if hold is None or required is None or partial_end else int(hold < required),
            "sck_high_min_ns": self.high_min, "sck_low_min_ns": self.low_min,
            "sck_period_min_ns": self.period_min, "sck_period_max_ns": self.period_max,
            "miso_setup_min_ns": self.setup_min,
            "miso_setup_observed_samples": self.setup_observed,
            "miso_setup_censored_samples": self.setup_censored,
            "miso_setup_lower_bound_min_ns": self.setup_bound,
            "miso_hold_min_ns": self.hold_min,
            "miso_hold_observed_samples": self.hold_observed,
            "miso_hold_censored_samples": self.pending_hold,
            "miso_hold_end_lower_bound_ns": None if self.pending_rise is None else end - self.pending_rise,
            "sample_edge_ambiguities": self.ambiguous, "zero_width_events": self.zero_width,
        }


def analyze_csv(path: Path, output_prefix: Path, channels=(0, 1, 2, 3),
                match_rx: bytes | None = None, max_matches: int = 1000) -> dict:
    if len(channels) != 4 or max_matches < 0 or (match_rx is not None and len(match_rx) != 6):
        raise ValueError("invalid channel mapping/match limit; match-rx must contain six bytes")
    frame_path = Path(str(output_prefix) + ".frames.csv")
    summary_path = Path(str(output_prefix) + ".summary.json")
    if frame_path.exists() or summary_path.exists():
        raise FileExistsError("refusing to overwrite existing analysis evidence")
    events = read_events(path, tuple(channels))
    first = next(events, None)  # Validate format/channel selection before creating output.
    if first is None:
        raise ValueError("no digital data rows")
    summary = {
        "source": str(path.resolve()), "frames_csv": str(frame_path.resolve()),
        "mode": 3, "sample_edge": "rising", "bit_order": "MSB first",
        "channels": dict(zip(("mosi", "miso", "sck", "nss"), channels)),
        "expected_tx_hex": EXPECTED_TX.hex(" ").upper(),
        "capture_start_ns": first.ns, "capture_end_ns": first.ns,
        "event_groups": 0, "observed_timestamp_quantum_ns": 0,
        "frames_total": 0, "frames_included": 0, "frames_excluded": 0,
        "frames_partial": 0, "complete_frames_missing_clocks": 0,
        "complete_frames_extra_clocks": 0, "frames_bad_crc": 0,
        "frames_valid_crc": 0, "frames_valid_crc_alarm": 0,
        "frames_cs_setup_violation": 0, "frames_cs_hold_violation": 0,
        "longest_bad_crc_run": {"length": 0, "first_frame": None, "last_frame": None},
        "sck_high_min_ns": None, "sck_low_min_ns": None,
        "miso_setup_min_ns": None, "miso_hold_min_ns": None,
        "match_rx_hex": None if match_rx is None else match_rx.hex(" ").upper(),
        "matched_frames_total": 0, "matches": [], "matches_truncated": False,
        "notes": [
            "Raw change-time CSV; equal timestamps are merged before decoding.",
            "Times use signed integer nanoseconds; no uniform-sample-grid assumption.",
            "Bad CRC is counted only for complete, unambiguous expected 48-clock bursts.",
            "Alarm counts require a valid CRC; status bits of corrupt data are not sensor alarms.",
            "MISO timings are observed digital transitions, not analog electrical timing proof.",
            "Missing transitions yield censored lower bounds, never an invented infinite setup/hold.",
            "CS hold requirement uses half the final measured rising-to-rising SCK period.",
            "Excluded frames break a consecutive-bad-CRC run; all frames remain in the CSV.",
            "Timestamp quantum is a GCD of exported changes, not a claimed analyzer sample rate.",
        ],
    }
    current: Frame | None = None
    previous: Event | None = None
    last_miso_change: int | None = None
    next_index = 0
    streak = 0
    streak_first: int | None = None

    with frame_path.open("x", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()

        def emit(frame: Frame, end: int, partial_end=False):
            nonlocal streak, streak_first
            row = frame.finish(end, partial_end, match_rx)
            writer.writerow(row)
            summary["frames_total"] += 1
            summary["frames_included" if row["included"] else "frames_excluded"] += 1
            partial = row["partial_start"] or row["partial_end"]
            summary["frames_partial"] += int(bool(partial))
            if not partial:
                summary["complete_frames_missing_clocks"] += int(row["missing_clocks"] > 0)
                summary["complete_frames_extra_clocks"] += int(row["extra_clocks"] > 0)
                summary["frames_cs_setup_violation"] += int(bool(row["cs_setup_violation"]))
                summary["frames_cs_hold_violation"] += int(bool(row["cs_hold_violation"]))
            if row["included"]:
                bad = not row["crc_ok"]
                summary["frames_bad_crc" if bad else "frames_valid_crc"] += 1
                summary["frames_valid_crc_alarm"] += row["sensor_alarm_valid"]
                if bad:
                    if not streak:
                        streak_first = frame.index
                    streak += 1
                    if streak > summary["longest_bad_crc_run"]["length"]:
                        summary["longest_bad_crc_run"] = {
                            "length": streak, "first_frame": streak_first, "last_frame": frame.index,
                        }
                else:
                    streak = 0
                for key in ("sck_high_min_ns", "sck_low_min_ns", "miso_setup_min_ns", "miso_hold_min_ns"):
                    if row[key] is not None:
                        summary[key] = minimum(summary[key], row[key])
            else:
                streak = 0
            if row["match_rx"]:
                summary["matched_frames_total"] += 1
                if len(summary["matches"]) < max_matches:
                    summary["matches"].append({"frame_index": frame.index, "start_ns": frame.start,
                                               "end_ns": end, "included": bool(row["included"])})
                else:
                    summary["matches_truncated"] = True

        for event in itertools.chain((first,), events):
            summary["event_groups"] += 1
            summary["capture_end_ns"] = event.ns
            if previous is None:
                if event.levels[3] == 0:
                    current = Frame(next_index, event.ns, True, event.levels[2])
                    next_index += 1
                previous = event
                continue
            summary["observed_timestamp_quantum_ns"] = math.gcd(
                summary["observed_timestamp_quantum_ns"], event.ns - previous.ns)
            if 1 in event.changes:
                last_miso_change = event.ns
                if current:
                    current.data_change(event.ns)
            old_nss, new_nss = previous.levels[3], event.levels[3]
            if new_nss == 0 and old_nss != 0:
                current = Frame(next_index, event.ns, old_nss is None, previous.levels[2])
                next_index += 1
            if current is not None:
                if event.zero_width:
                    current.zero_width += 1
                    current.reasons.add("multiple_same_timestamp_transitions")
                if new_nss is None:
                    current.reasons.add("unknown_nss_level")
                sck_change = previous.levels[2] != event.levels[2]
                if sck_change:
                    if previous.levels[2] is None or event.levels[2] is None:
                        current.reasons.add("unknown_sck_transition")
                    elif old_nss == 0 or new_nss == 0:
                        rising = event.levels[2] == 1
                        if rising and old_nss != new_nss:
                            current.reasons.add("sampling_edge_at_nss_boundary")
                        current.clock(event.ns, rising, event, last_miso_change, first.ns)
                if old_nss == 0 and new_nss != 0:
                    emit(current, event.ns, partial_end=new_nss is None)
                    current = None
            previous = event
        if current is not None:
            emit(current, summary["capture_end_ns"], partial_end=True)
    with summary_path.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    return summary


def parse_match(value: str) -> bytes:
    try:
        data = bytes.fromhex(value.replace(":", "").replace("_", ""))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("match-rx must be six hexadecimal bytes") from exc
    if len(data) != 6:
        raise argparse.ArgumentTypeError("match-rx must be six hexadecimal bytes")
    return data


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--mosi", type=int, default=0)
    parser.add_argument("--miso", type=int, default=1)
    parser.add_argument("--sck", type=int, default=2)
    parser.add_argument("--nss", type=int, default=3)
    parser.add_argument("--match-rx", type=parse_match)
    parser.add_argument("--max-matches", type=int, default=1000,
                        help="summary match cap; every match is still marked in frames.csv")
    args = parser.parse_args(argv)
    try:
        summary = analyze_csv(args.csv, args.output_prefix,
                              (args.mosi, args.miso, args.sck, args.nss),
                              args.match_rx, args.max_matches)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({key: summary[key] for key in (
        "frames_total", "frames_included", "frames_excluded", "frames_bad_crc",
        "frames_valid_crc_alarm", "longest_bad_crc_run", "matched_frames_total")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
