import io

PATH = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\temp\2026-10-13_16-57-49.csv'
lines = io.open(PATH, encoding='utf-8', errors='ignore').read().splitlines()
rows = [l.split(',') for l in lines[1:] if l.strip()]
t = [float(r[0]) for r in rows]
v = [int(r[1].strip()) for r in rows]

# time axis quantisation
steps = sorted(set(round(t[i + 1] - t[i], 9) for i in range(len(t) - 1)))
print('samples', len(v))
print('distinct time steps (us)', [round(s * 1e6, 4) for s in steps][:8])
res_us = steps[0] * 1e6
print('assumed resolution %.3f us' % res_us)

e = [(t[i], v[i]) for i in range(1, len(v)) if v[i] != v[i - 1]]
hi = [(e[i + 1][0] - e[i][0]) * 1e6 for i in range(len(e) - 1) if e[i][1] == 1]
lo = [(e[i + 1][0] - e[i][0]) * 1e6 for i in range(len(e) - 1) if e[i][1] == 0]
ris = [x[0] for x in e if x[1] == 1]
per = [(ris[i + 1] - ris[i]) * 1e6 for i in range(len(ris) - 1)]

print('cycles', len(ris))
print('high us distinct', sorted(set(round(x, 4) for x in hi)))
print('low  us distinct', sorted(set(round(x, 4) for x in lo)))
print('period us min/max/mean %.3f %.3f %.4f' % (min(per), max(per), sum(per) / len(per)))
print('high   us min/max/mean %.3f %.3f %.4f' % (min(hi), max(hi), sum(hi) / len(hi)))

P = sum(per) / len(per)
H = sum(hi) / len(hi)
print('measured duty = %.6f  (%.3f%%)' % (H / P, 100 * H / P))

# --- theory ---
SYS = 168_000_000.0
ARR = 4199
duty_cmd = 50
CCR = duty_cmd * (ARR + 1) // 1000
per_counts = 2 * (ARR + 1)
ocref_ns = 2 * CCR * 1e9 / SYS
dt_ns = 84 * 1e9 / SYS
print()
print('CCR=%d  OCREF high %.1f ns  deadtime %.1f ns  -> pin high %.1f ns'
      % (CCR, ocref_ns, dt_ns, ocref_ns - dt_ns))
pin_ns = ocref_ns - dt_ns
print('theory duty without deadtime = %.6f' % (ocref_ns / 1000 / (per_counts * 1e9 / SYS / 1000)))
print('theory duty with    deadtime = %.6f  (pin high %.1f ns / %.1f ns)'
      % (pin_ns / (per_counts * 1e9 / SYS), pin_ns, per_counts * 1e9 / SYS))
print('deadtime accounts for        = %.6f of duty' % (dt_ns / (per_counts * 1e9 / SYS)))

# quantisation of the measured pulse
print()
print('pin high in resolution units = %.3f steps of %.3f us'
      % ((pin_ns / 1000) / res_us, res_us))
for n in range(int((pin_ns / 1000) / res_us) - 1, int((pin_ns / 1000) / res_us) + 3):
    print('  if scope quantises high to %d steps -> duty %.6f (err %+.6f)'
          % (n, (n * res_us) / P, (n * res_us) / P - pin_ns / (per_counts * 1e9 / SYS)))
