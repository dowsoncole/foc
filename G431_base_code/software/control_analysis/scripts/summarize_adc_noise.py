import sys, struct, json, statistics
from pathlib import Path

path = Path(sys.argv[1])
data = path.read_bytes()
state, count, first, last, ou, ov, ratio, profile = struct.unpack_from("<8I", data)
assert state == 3 and count == 1024 and last-first == 1023, "Incomplete/noncontiguous capture"
raw = list(struct.iter_unpack("<HH", data[32:32+count*4]))
scale = 3.3/(4095*.007*6.1*ratio)
u = [(ou-a)*scale for a,b in raw]
v = [(ov-b)*scale for a,b in raw]
w = [-a-b for a,b in zip(u,v)]
result = dict(profile=profile, ratio=ratio, samples=count, first=first, last=last,
              offset_u=ou, offset_v=ov)
for name, values in zip(("u","v","w"), (u,v,w)):
    result[name] = dict(mean_A=statistics.mean(values), std_A=statistics.pstdev(values),
                       min_A=min(values), max_A=max(values), peak_peak_A=max(values)-min(values))
path.with_suffix(".json").write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
