"""Decode a GDB dump of current_test_sample_t; pass the VALID sample count."""
import argparse
import csv
import json
import math
import struct
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("binary", type=Path)
p.add_argument("--count", type=int, required=True)
a = p.parse_args()
record = struct.Struct("<I7f")
data = a.binary.read_bytes()
if not 0 < a.count <= len(data) // record.size:
    p.error("count must match the frozen firmware sample_count")
rows = list(record.iter_unpack(data[:a.count * record.size]))
header = ["source_sequence", "time_s", "d_reference_a", "q_reference_a",
          "d_current_a", "q_current_a", "d_voltage_pu", "q_voltage_pu"]
with a.binary.with_suffix(".csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(rows)
summary = {"valid_samples": a.count, "last_sample_time_s": rows[-1][1],
           "sequence_continuous": all(b[0] - c[0] == 1 for c, b in zip(rows, rows[1:])),
           "note": "Aborted capture is not a completed step-response test."}
for name, col in [("d_current_a", 4), ("q_current_a", 5)]:
    values = [r[col] for r in rows]
    summary[name] = {"min": min(values), "max": max(values),
                     "rms": math.sqrt(sum(x*x for x in values)/len(values))}
print(json.dumps(summary, indent=2))
