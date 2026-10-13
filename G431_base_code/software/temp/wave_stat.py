import sys

path = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\temp\2026-10-13_16-14-30.csv'
lines = open(path, encoding='utf-8', errors='ignore').read().splitlines()
rows = [l.split(',') for l in lines[1:] if l.strip()]
t = [float(r[0]) for r in rows]
v = [int(r[1].strip()) for r in rows]
edges = [(t[i], v[i]) for i in range(1, len(v)) if v[i] != v[i - 1]]
high = [(edges[i + 1][0] - edges[i][0]) * 1e6 for i in range(len(edges) - 1) if edges[i][1] == 1]
low = [(edges[i + 1][0] - edges[i][0]) * 1e6 for i in range(len(edges) - 1) if edges[i][1] == 0]
rises = [e[0] for e in edges if e[1] == 1]
periods = [(rises[i + 1] - rises[i]) * 1e6 for i in range(len(rises) - 1)]
print('samples', len(v))
print('cycles', len(rises))
print('high_us uniq', sorted(set(round(x, 3) for x in high)))
print('low_us uniq', sorted(set(round(x, 3) for x in low)))
if periods:
    print('period_us min/max/mean', round(min(periods), 3), round(max(periods), 3), round(sum(periods) / len(periods), 3))
if high:
    print('high_us min/max/mean', round(min(high), 3), round(max(high), 3), round(sum(high) / len(high), 3))
print('span_s', round(t[-1] - t[0], 4))
# first 20 edges for eyeballing
for e in edges[:20]:
    print(round(e[0], 6), e[1])
