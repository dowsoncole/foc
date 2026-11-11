"""Plot decoded RS485 V2 frames; frame index is NOT a uniform time axis."""
import csv
import dataclasses
import json
from pathlib import Path
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root.parent / "PythonExample"))
from servo_transport import WaveformParser

path = root / "software/control_analysis/results/uart_com11_capture.bin"
parser = WaveformParser()
frames = parser.feed(path.read_bytes())
dq = [f for f in frames if f.channel_ids == (4, 5, 6, 7)]
if not dq:
    raise SystemExit("No valid D/Q frames")
report = {"communication": dataclasses.asdict(parser.stats), "valid_dq_frames": len(dq)}
for i, name in enumerate(("id_ref", "id", "iq_ref", "iq")):
    values = [f.values[i] for f in dq]
    report[name] = {"mean_A": statistics.mean(values), "std_A": statistics.pstdev(values),
                    "min_A": min(values), "max_A": max(values)}
path.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
with path.with_suffix(".csv").open("w", newline="") as output:
    writer = csv.writer(output)
    writer.writerow(("frame_sequence", "adc_sequence", "id_ref_A", "id_A", "iq_ref_A", "iq_A"))
    writer.writerows((f.frame_sequence, f.adc_sequence, *f.values) for f in dq)
fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
for ax, ref, feedback, name in ((axes[0], 0, 1, "D"), (axes[1], 2, 3, "Q")):
    ax.plot([f.values[feedback] for f in dq], linewidth=.7, label=f"I{name.lower()} feedback")
    ax.plot([f.values[ref] for f in dq], color="black", linestyle="--", label="Reference")
    ax.set_ylabel(f"{name} current (A)")
    ax.grid(alpha=.25)
    ax.legend(loc="upper right")
axes[-1].set_xlabel("Decoded valid frame index (NOT uniform time)")
fig.suptitle(f"COM11 / 921600 / RS485 V2: {len(dq)} valid DQ frames\n"
             f"CRC rejects: {parser.stats.crc_errors}; sequence anomalies present")
fig.tight_layout()
fig.savefig(path.with_suffix(".png"), dpi=160)
print(json.dumps(report, indent=2))
