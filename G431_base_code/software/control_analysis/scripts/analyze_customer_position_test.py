"""Offline decoder for the 30-row customer position protocol result table."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct

FIELDS=('frequency_hz','reference_amplitude_deg','actual_amplitude_deg',
        'gain_db','phase_deg','samples','flags','missed_commands')


def decode(data):
    if len(data) not in (960,1024):
        raise ValueError('Expected 960-byte result table or 1024-byte capture union')
    rows=[]
    for i in range(30):
        row=dict(zip(FIELDS,struct.unpack_from('<5f3I',data,i*32)))
        if row['frequency_hz'] not in (0,i+1) or row['flags'] & ~63:
            raise ValueError(f'Invalid row {i+1}; check exact ELF and capture mode')
        if row['samples']>2000 or bool(row['missed_commands']) != bool(row['flags'] & 8):
            raise ValueError(f'Inconsistent count/slot flags at row {i+1}')
        measured=row['frequency_hz']==i+1 and row['samples']>0
        finite=all(math.isfinite(row[k]) for k in FIELDS[:5])
        usable=bool(measured and finite and row['samples']>=1900 and
                    row['reference_amplitude_deg']>1e-5 and row['actual_amplitude_deg']>1e-5 and
                    not row['flags'] & 55)
        row.update(frequency_hz=i+1,estimate_usable=usable,
                   protocol_complete=bool(usable and row['flags']==0))
        for k,v in row.items():
            if isinstance(v,float) and not math.isfinite(v):row[k]=None
        rows.append(row)
    return rows


def write_report(capture,out,source='xlsx'):
    rows=decode(Path(capture).read_bytes())
    if source=='xlsx':rows=rows[:15]
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    (out/'customer_results.json').write_text(json.dumps(rows,indent=2,allow_nan=False),encoding='utf-8')
    with (out/'customer_results.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    description=('Excel逐帧模式：1–15 Hz，每秒300帧，原表一秒序列重复两次；16–30 Hz未提供。'
                 if source=='xlsx' else 'Word文字模式：1–30 Hz，每周期40点阶梯三角指令。')
    lines=['# 客户位置指令测试结果','',description,
           '每频点 2 秒运动 + 2 秒原点保持；0→2→0°；内部位置指令复现。',
           '幅值为基波峰值，相位为输出相对实际已下发指令的主值相位，负号表示滞后。',
           '包含启动暂态，未执行稳态相干性判定；此表不能直接作为闭环带宽验收。',
           'flags：1 拟合无效，2 速度限幅，4 斜坡限幅，8 指令漏点，16 中止，32 时间覆盖不足。',
           'estimate_usable 仅表示幅相数值及限幅/时间检查通过；protocol_complete 还要求没有指令漏点。','',
           '| Hz | 参考基波 ° | 输出基波 ° | 增益 dB | 相位 ° | 样本 | flags | 漏点 | 估计可用 | 指令完整 |',
           '|---:|---:|---:|---:|---:|---:|---:|---:|:---:|:---:|']
    for r in rows:
        values=[r[k] for k in FIELDS]+[r['estimate_usable'],r['protocol_complete']]
        lines.append('| '+' | '.join('—' if v is None else f'{v:.4f}' if isinstance(v,float) else str(v) for v in values)+' |')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(9,7),sharex=True)
    for ax,key,label in zip(axes,('gain_db','phase_deg'),('Fundamental gain (dB)','Principal phase (deg)')):
        observed=[r for r in rows if r['samples']>=1900 and not r['flags'] & 49 and
                  r['gain_db'] is not None and r['phase_deg'] is not None]
        ax.plot([r['frequency_hz'] for r in observed],[r[key] for r in observed],
                ':',color='gray',label='Observed response (may be limited)')
        ax.plot([r['frequency_hz'] for r in rows],
                [r[key] if r['estimate_usable'] else math.nan for r in rows],'.-',label='Usable finite-window estimate')
        limited=[r for r in observed if r['flags'] & 6]
        ax.scatter([r['frequency_hz'] for r in limited],[r[key] for r in limited],
                   color='red',marker='x',s=50,label='Speed / slew limited')
        missed=[r for r in rows if r['estimate_usable'] and not r['protocol_complete']]
        ax.scatter([r['frequency_hz'] for r in missed],[r[key] for r in missed],
                   facecolors='none',edgecolors='orange',s=80,label='Command slots skipped')
        ax.set_ylabel(label);ax.grid(True);ax.legend()
    axes[1].set_xlabel('Frequency (Hz)')
    fig.suptitle(f'Customer {source.upper()} protocol: 2 s motion / 2 s hold')
    fig.tight_layout();fig.savefig(out/'customer_response.png',dpi=160);plt.close(fig)
    lines+=['','灰线为实际短窗口响应，红叉表示速度/斜坡限幅；红叉点不作为线性频响证据。',
            '','![幅频与相频](customer_response.png)','']
    (out/'customer_results.md').write_text('\n'.join(lines),encoding='utf-8')
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--source',choices=('xlsx','word'),default='xlsx')
    a=p.parse_args();write_report(a.capture,a.output,a.source)
