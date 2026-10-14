import io
from collections import Counter

PATH = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\temp\2026-10-57-49.csv'.replace('2026-10-57-49', '2026-10-13_16-57-49')
lines = io.open(PATH, encoding='utf-8', errors='ignore').read().splitlines()
rows = [l.split(',') for l in lines[1:] if l.strip()]
t = [float(r[0]) for r in rows]
v = [int(r[1].strip()) for r in rows]

e = [(t[i], v[i]) for i in range(1, len(v)) if v[i] != v[i - 1]]
# drop the first partial/glitch cycle
hi = [(e[i + 1][0] - e[i][0]) * 1e6 for i in range(len(e) - 1) if e[i][1] == 1]
lo = [(e[i + 1][0] - e[i][0]) * 1e6 for i in range(len(e) - 1) if e[i][1] == 0]
hi = [x for x in hi if x > 1.0]
lo = [x for x in lo if x > 1.0]

def stats(name, xs):
    n = len(xs)
    mean = sum(xs) / n
    sd = (sum((x - mean) ** 2 for x in xs) / n) ** 0.5
    print('%s n=%d min=%.3f max=%.3f mean=%.4f us  sd=%.4f us  spread=%.3f us'
          % (name, n, min(xs), max(xs), mean, sd, max(xs) - min(xs)))
    return mean

mh = stats('high', hi)
ml = stats('low ', lo)
per = [(hi[i] + lo[i]) for i in range(min(len(hi), len(lo)))]
print('period mean %.4f us  sd %.4f us  min %.3f max %.3f'
      % (sum(per) / len(per),
         (sum((x - sum(per) / len(per)) ** 2 for x in per) / len(per)) ** 0.5,
         min(per), max(per)))

c = Counter(round(x, 2) for x in hi)
print('\nhigh-width histogram (us -> count), top 12:')
for k, n in sorted(c.items(), key=lambda kv: -kv[1])[:12]:
    print('  %.2f  %5d  %s' % (k, n, '#' * min(60, n // 40)))

print('\n--- theory vs measured ---')
SYS = 168e6
ARR = 4199
CCR = 50 * (ARR + 1) // 1000
T_us = 2 * (ARR + 1) / SYS * 1e6
ocref_us = 2 * CCR / SYS * 1e6
dt_us = 84 / SYS * 1e6
pin_us = ocref_us - dt_us
print('CCR=%d period=%.4f us' % (CCR, T_us))
print('commanded duty (CCR band) = %.4f  -> OCREF high %.3f us' % (CCR / (ARR + 1), ocref_us))
print('minus dead time %.3f us     -> CH1 pin high %.3f us = %.4f duty' % (dt_us, pin_us, pin_us / T_us))
print('measured high %.4f us       -> duty %.5f' % (mh, mh / T_us))
print('measured - pin theory  = %+.4f us  (%+.5f duty)' % (mh - pin_us, (mh - pin_us) / T_us))
print('measured - OCREF(5%%)   = %+.4f us  (%+.5f duty)' % (mh - ocref_us, (mh - ocref_us) / T_us))
print('dead time alone would  = %+.4f us  (%+.5f duty)  <- wrong sign & wrong size'
      % (-dt_us, -dt_us / T_us))
implied_ccr = (mh / 1e6 * SYS + 84) / 2
print('implied CCR if dead time were really 84 counts: %.1f  (duty %.1f)' % (implied_ccr, implied_ccr * 1000 / 4200))
