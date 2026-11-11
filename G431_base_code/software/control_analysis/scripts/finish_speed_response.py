"""OFF-only commissioning handoff. Never enables, resets or clears faults."""
import argparse
import json
from pathlib import Path
import struct
from measure_speed_response import SpeedTarget


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--axf',type=Path,required=True)
    p.add_argument('--probe',default='150710982')
    p.add_argument('--calibration-before',type=Path,required=True)
    p.add_argument('--kp',type=float,default=.030637519)
    p.add_argument('--ki',type=float,default=.13476460)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    t=SpeedTarget(args.axf.resolve(),args.output.resolve(),args.probe);a=t.address
    t.checkpoint('before')
    if 'Running' not in t.run(['status'],'cpu_before'): raise RuntimeError('CPU not running')
    commands=[t.w(a['user'],0),t.w(a['speed_run'],0),t.w(a['user']+4,4)]
    for i,value in enumerate((0.,9000.,9000.,9000.,9000.)):
        commands.append(t.wf(a['speed_user']+4*i,value))
    commands += [t.wf(a['speed_requested']+24,args.kp),
                 t.wf(a['speed_requested']+28,args.ki),t.w(a['guard'],0),'sleep 100']
    t.run(commands,'restore_off')
    state=t.checkpoint('final')
    if tuple(state['status'][:2])!=(0,0):
        raise RuntimeError('User controller not idle/healthy at handoff')
    final_log=t.run([t.dump(a['user'],112,args.output/'user.bin'),
           t.dump(a['guard'],32,args.output/'guard.bin'),
           t.dump(0x0801f000,4096,args.output/'calibration_pages_after.bin'),
           'status'],'handoff_readback')
    if 'Running' not in final_log: raise RuntimeError('CPU not running at handoff')
    user=(args.output/'user.bin').read_bytes()
    if struct.unpack_from('<2I',user)!=(0,4): raise RuntimeError('User enable/mode differs')
    if struct.unpack_from('<5f',user,52)!=(0.,9000.,9000.,9000.,9000.):
        raise RuntimeError('Speed defaults not restored')
    if struct.unpack_from('<I',(args.output/'guard.bin').read_bytes())[0]:
        raise RuntimeError('Test-only timeout not removed')
    same=(args.output/'calibration_pages_after.bin').read_bytes()==args.calibration_before.read_bytes()
    if not same: raise RuntimeError('Calibration pages changed')
    evidence=dict(output_enabled=False,user_enable=0,mode=4,target_rpm=0,
                  acceleration_rpm_s=9000,deceleration_rpm_s=9000,
                  forward_limit_rpm=9000,reverse_limit_rpm=9000,
                  temporary_timeout_ms=0,calibration_pages_unchanged=same,
                  checkpoint=state)
    (args.output/'handoff.json').write_text(json.dumps(evidence,indent=2))
    print(json.dumps(evidence,indent=2))


if __name__=='__main__': main()
