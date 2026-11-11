import argparse, struct, json, statistics, re
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument("path",type=Path)
p.add_argument("--axis",type=int,required=True)
p.add_argument("--kp",type=float,required=True)
p.add_argument("--ki",type=float,required=True)
p.add_argument("--average",type=int,required=True)
a=p.parse_args()
rows=list(struct.iter_unpack("<I7f",a.path.read_bytes()))
assert len(rows)==512 and all(y[0]-x[0]==a.average for x,y in zip(rows,rows[1:])), "Missing samples"
ref=[r[2+a.axis] for r in rows]; val=[r[4+a.axis] for r in rows]
other=[r[5-a.axis] for r in rows]
target=ref[-1]; tail=val[-128:]
result=dict(kp=a.kp,ki=a.ki,axis=a.axis,average=a.average,
 target_A=target,duration_s=rows[-1][1],mean_A=statistics.mean(tail),
 std_A=statistics.pstdev(tail),error_A=statistics.mean(tail)-target,
 other_mean_A=statistics.mean(other[-128:]),other_std_A=statistics.pstdev(other[-128:]),
 peak_voltage_pu=max(abs(r[6+a.axis]) for r in rows),
 iae_As=sum(abs(r[2+a.axis]-r[4+a.axis])*(rows[i][1]-rows[i-1][1]) for i,r in enumerate(rows) if i))
if target:
    crossing=next((r[1]-.01 for r in rows if r[1]>=.01 and r[4+a.axis]/target>=.9),None)
    result["first_90pct_s"]=crossing
    result["overshoot_pct"]=100*(max(r[4+a.axis]/target for r in rows if r[1]>=.01)-1)
result["note"]="Tail 128 points; averaged feedback, not raw ADC ripple. First crossing/overshoot noise-sensitive."
log_path=a.path.with_suffix('.read.log')
if log_path.exists():
    log=log_path.read_text(encoding='utf-8',errors='replace').replace('\n',' ')
    timing=re.search(r'first_cycle = (\d+), last_cycle = (\d+), samples = (\d+)',log)
    if timing:
        first,last,count=map(int,timing.groups())
        interval=((last-first)&0xffffffff)/168000000/(count-1)
        result['measured_update_period_us']=interval*1e6
        result['timing_valid']=abs(interval-50e-6)<1e-6
        if not result['timing_valid']:
            result['note']+=' INVALID firmware timebase: do not rank time-based metrics.'
a.path.with_suffix(".json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
