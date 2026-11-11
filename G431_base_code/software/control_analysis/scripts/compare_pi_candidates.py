"""Compare completed MCU captures; averaging is explicit and not a control filter."""
import csv
import json
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

results = Path(__file__).resolve().parents[1] / 'results'
reports = []
for path in sorted(results.glob('pi_*.json')):
    report = json.loads(path.read_text(encoding='utf-8'))
    report['tag'] = path.stem
    reports.append(report)
fields = ['tag', 'axis', 'kp', 'ki', 'average', 'target_A', 'error_A', 'std_A',
          'other_mean_A', 'other_std_A', 'iae_As', 'peak_voltage_pu']
with (results / 'pi_candidate_comparison.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
    writer.writeheader()
    writer.writerows(reports)

fig, axes = plt.subplots(2, 2, figsize=(12, 7))
for axis, name in enumerate(('D', 'Q')):
    for prefix, label, color in [('baseline', 'Kp=.005, Ki=.2', '#888888'),
                                 ('lowp_fast', 'Kp=.0025, Ki=.8', '#ce8b13'),
                                 ('lowp_strong', 'Kp=.0025, Ki=1.6', '#176ab3')]:
        path = results / f'pi_{prefix}_{name.lower()}.bin'
        rows = list(struct.iter_unpack('<I7f', path.read_bytes()))
        times = [r[1] * 1000 for r in rows]
        values = [r[4 + axis] * 1000 for r in rows]
        axes[axis, 0].plot(times, values, alpha=.25, linewidth=.6, color=color)
        # 20 recorded points = approx 4 ms; for trend comparison ONLY.
        trend_t = [sum(times[i:i+20])/20 for i in range(0, len(rows)-19, 20)]
        trend_y = [sum(values[i:i+20])/20 for i in range(0, len(rows)-19, 20)]
        axes[axis, 0].plot(trend_t, trend_y, label=label, color=color)
        axes[axis, 1].plot(times[-128:], values[-128:], linewidth=.7, label=label, color=color)
    axes[axis, 0].plot(times, [r[2+axis]*1000 for r in rows], 'k--', label='Reference')
    axes[axis, 1].axhline(50, color='black', linestyle='--')
    for col in range(2):
        axes[axis, col].set_ylabel(f'{name} current (mA)')
        axes[axis, col].set_xlabel('Time (ms)')
        axes[axis, col].grid(alpha=.2)
        axes[axis, col].legend(fontsize=8)
axes[0, 0].set_title('Step response: faint=4-sample capture; bold=4 ms trend')
axes[0, 1].set_title('Final 128 points: identical capture averaging for all runs')
fig.suptitle('Measured PI comparison: fixed angle 0, +50 mA step\n'
             'Better mean tracking does not establish reduced raw current ripple')
fig.tight_layout()
fig.savefig(results / 'pi_candidate_comparison.png', dpi=160)
print(f'Compared {len(reports)} captures; CSV and PNG saved in {results}')
