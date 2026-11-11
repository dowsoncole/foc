"""Recheck completed speed experiments from immutable binary evidence."""
import argparse
import json
from pathlib import Path
import struct
import numpy as np


def audit(root):
    results=[]
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or not (folder/'finished.bin').exists(): continue
        state,count,fault=struct.unpack_from('<3I',(folder/'finished.bin').read_bytes(),4)
        record=dict(tag=folder.name,state=state,count=count,fault=fault,
                    qualified_summary=(folder/'summary.json').exists())
        if record['qualified_summary']:
            summary=json.loads((folder/'summary.json').read_text())
            before=np.fromfile(folder/'diag_before.bin',dtype='<u4')
            after=np.fromfile(folder/'diag_after.bin',dtype='<u4')
            names={2:'busy',3:'start',4:'transfer',5:'crc',6:'alarm',21:'deadline',26:'angle_step'}
            record['error_deltas']={name:int(after[i])-int(before[i]) for i,name in names.items()}
            record['final_moe']=bool(struct.unpack('<I',(folder/'final_bdtr.bin').read_bytes())[0]&0x8000)
            record['final_foc_fault']=struct.unpack('<I',(folder/'final_foc_fault.bin').read_bytes())[0]
            record['final_runtime_fault']=struct.unpack('<I',(folder/'final_runtime_fault.bin').read_bytes())[0]
            rows=np.fromfile(folder/'samples.bin',dtype='<f4').reshape(-1,8)
            if not np.all(np.diff(rows[:,1])>0): raise RuntimeError(folder.name+' time not monotonic')
            record['summary']=summary
            if (state,count,fault)!=(3,512,0) or any(record['error_deltas'].values()) or\
               record['final_moe'] or record['final_foc_fault'] or record['final_runtime_fault']:
                raise RuntimeError('Accepted summary has invalid evidence: '+folder.name)
            if summary['kind']=='sine':
                t=rows[:,1].astype(float); f=summary['frequency_hz']
                keep=np.flatnonzero(t>=summary['delay_s']+1/f)
                parts=np.array_split(keep,3 if summary['measured_cycles']>=6 else 2)
                gains=[]
                for part in parts:
                    x=np.column_stack([np.ones(len(part)),np.sin(2*np.pi*f*t[part]),np.cos(2*np.pi*f*t[part])])
                    r=np.linalg.lstsq(x,rows[part,3],rcond=None)[0]
                    y=np.linalg.lstsq(x,rows[part,4],rcond=None)[0]
                    gains.append(complex(y[1],y[2])/complex(r[1],r[2]))
                record['frf_window_gain_db']=[float(20*np.log10(abs(g))) for g in gains]
                record['frf_window_phase_deg']=[float(np.angle(g,deg=True)) for g in gains]
        results.append(record)
    output=dict(accepted=sum(r['qualified_summary'] for r in results),experiments=results,
                note='Window spread is repeatability within a record, not an independent confidence interval.')
    (root/'trial_audit.json').write_text(json.dumps(output,indent=2))
    print('Audited accepted experiments:',output['accepted'])
    for r in results:
        if r['qualified_summary'] and r['summary']['kind']=='step':
            s=r['summary'];print(r['tag'],s['rise_10_90_s'],s['overshoot_pct'],s['steady_std_rpm'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    audit(p.parse_args().root)
