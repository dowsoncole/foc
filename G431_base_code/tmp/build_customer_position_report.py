"""Build this session's measured comparison; no hardware access."""
from pathlib import Path
import json,math,hashlib,struct
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('software/control_analysis/results/customer_position_live_20261029')
names=('xlsx_run2','xlsx_repeat')
runs=[]
for name in names:
    folder=root/name
    points=json.loads((folder/'customer_results.json').read_text())
    summary=json.loads((folder/'summary.json').read_text())
    armed=json.loads((folder/'armed.json').read_text())
    snapshots=sorted(folder.glob('customer_[0-9]*.json'),key=lambda p:int(p.stem.split('_')[1]))
    final=json.loads(snapshots[-1].read_text())
    assert not summary.get('cleanup_error')
    assert not summary['final']['bdtr'][0]&0x8000
    assert summary['final']['runtime_fault']==summary['final']['foc_fault']==[0]
    assert len(points)==15 and all(p['samples']==2000 and p['missed_commands']==0 for p in points)
    # Check the redundant amplitude/gain fields independently of the decoder.
    for p in points:
        assert abs(20*math.log10(p['actual_amplitude_deg']/p['reference_amplitude_deg'])-p['gain_db'])<1e-4
    dt=(final['tick_ms']-armed['tick_ms'])*.001
    rates={k+'_hz':(final[k]-armed[k])/dt for k in ('requests','frames','updates')}
    runs.append(dict(name=name,points=points,center_deg=armed['mpt_deg'],
                     peak_a=final['phase_peak_a'],rms_a=final['phase_rms_a'],
                     slew_limited=final['slew_limited'],speed_limited=final['saturated'],
                     rates=rates,final_off=summary['final']))
fig,axes=plt.subplots(3,1,figsize=(10,10),sharex=True)
colors=('#2166ac','#d95f02')
for i,(run,color) in enumerate(zip(runs,colors),1):
    p=run['points'];f=[x['frequency_hz'] for x in p]
    label=f'Run {i}, center {run["center_deg"]:.3f} deg'
    axes[0].plot(f,[x['actual_amplitude_deg'] for x in p],'o-',color=color,label=label)
    axes[1].plot(f,[x['gain_db'] for x in p],'o-',color=color,label=label)
    axes[2].plot(f,[x['phase_deg'] for x in p],'o-',color=color,label=label)
    for ax,key in zip(axes,('actual_amplitude_deg','gain_db','phase_deg')):
        clipped=[x for x in p if x['flags'] & 6]
        ax.scatter([x['frequency_hz'] for x in clipped],[x[key] for x in clipped],
                   marker='x',color='red',s=48,zorder=5,label='Slew limited' if i==1 else None)
axes[0].plot([x['frequency_hz'] for x in runs[0]['points']],
             [x['reference_amplitude_deg'] for x in runs[0]['points']],
             '--',color='gray',label='Run 1 applied reference fundamental')
axes[1].axhline(0,color='gray',lw=.8)
for ax,label in zip(axes,('Fundamental peak amplitude (deg)','Output / applied reference (dB)','Phase lag: negative = lag (deg)')):
    ax.set_ylabel(label);ax.grid(True,alpha=.4);ax.legend(fontsize=9)
axes[-1].set_xlabel('Command frequency (Hz)');axes[-1].set_xticks(range(1,16))
fig.suptitle('Measured Excel position protocol | Kp=72, FF=0, 300 frames/s\n0..2..0 deg, 2 s motion + 2 s hold; red x: slew limited',fontsize=12)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(root/'measured_comparison.png',dpi=160);plt.close(fig)
comparison=dict(protocol='xlsx',runs=runs,
    maximum_gain_difference_db=max(abs(a['gain_db']-b['gain_db']) for a,b in zip(runs[0]['points'],runs[1]['points'])),
    maximum_phase_difference_deg=max(abs(a['phase_deg']-b['phase_deg']) for a,b in zip(runs[0]['points'],runs[1]['points'])),
    firmware_sha256=hashlib.sha256((root/'flash_customer/firmware.axf').read_bytes()).hexdigest())
(root/'measured_comparison.json').write_text(json.dumps(comparison,indent=2,ensure_ascii=False),encoding='utf8')
lines=['# 客户位置指令实测幅相报告','',
'测试日期：2026-10-29。硬件：J-Link 150710982，STM32G431伺服主控，MPT输出轴位置反馈。',
'本报告是本次真实电机测量；没有引用仿真曲线充当实测。','',
'## 结果','',
'相位滞后随频率增加，两个独立测试均约为：1 Hz 6°，5 Hz 26～27°，10 Hz 56～59°，15 Hz 86～87°。',
'幅值先出现抬升、随后回落，但转折频率不固定：第一轮在10 Hz为+1.69 dB，11 Hz转为−1.14 dB；第二轮在8 Hz为+1.30 dB，9 Hz转为−0.53 dB。',
'9～10 Hz的两轮幅值差约1.88～2.24 dB，相位最大差约5.05°。不能仅据单轮曲线断言固定10 Hz机械共振。',
'15 Hz输出基波为参考的约85%～88%，增益−1.42～−1.16 dB，仍有明显约87°相位滞后。','',
'![两轮实测幅值与相位](measured_comparison.png)','',
'蓝/橙线为两次独立实测，灰色虚线为第一轮实际下发参考的基波幅值，红叉为出现加速度斜坡限幅的点。红叉仍是实际跟踪响应，但不能作为未限幅的线性频响证据。','',
'## 测试条件','',
'- 按Excel可见表逐帧执行：300帧/秒，0→2→0°，1～15 Hz，每组运动2秒、保持2秒，总时长60秒。每频点采样2000次、执行600帧，两轮均无漏帧。',
'- Kp=72 /s，FF=0，转速上限5500 rpm，普通位置加速度斜坡300000 rpm/s；未修改控制参数以改善本次曲线。',
'- 主机试验中止条件：相电流峰值15 A、累计相RMS 5 A；硬件OCP/FLT_MAX策略不变。',
'- 两轮起点约256.437°和256.432°，相差约0.0055°；同一现场连接连续测试。负载沿用此前用户“下午带惯量盘”的记录，本轮未重新核实惯量和供电数值。',
'- 幅值为拟合基波峰值，不能与指令0～2°的2°行程直接相除；相位为实际输出相对实际下发指令的主值相位。',
'- 每组窗口包含启动暂态。没有执行至少20周期和Welch相干性稳态门槛，不能据此给出可靠−3 dB闭环带宽或证明20 Hz性能。Excel没有16～30 Hz逐帧列，本轮没有外推测试。','',
'## 逐频点结果','',
'| Hz | 输出基波1 ° | 输出基波2 ° | 增益1 dB | 增益2 dB | 相位1 ° | 相位2 ° | flags 1/2 |',
'|---:|---:|---:|---:|---:|---:|---:|:---:|']
for a,b in zip(runs[0]['points'],runs[1]['points']):
    lines.append(f'| {a["frequency_hz"]} | {a["actual_amplitude_deg"]:.4f} | {b["actual_amplitude_deg"]:.4f} | {a["gain_db"]:+.3f} | {b["gain_db"]:+.3f} | {a["phase_deg"]:.2f} | {b["phase_deg"]:.2f} | {a["flags"]}/{b["flags"]} |')
lines+=['','flags=0表示数值/覆盖/限幅检查均通过；flags=4表示该频点出现加速度斜坡限幅。两轮速度限幅均0，漏帧均0；第一轮仅1、2、3、5 Hz为flags=0，第二轮仅1、2、3 Hz为flags=0。','',
'## 电流、重复性与停机','',
f'- 第一轮峰值相电流{runs[0]["peak_a"]:.3f} A，累计最大相RMS {max(runs[0]["rms_a"]):.3f} A，斜坡限幅样本{runs[0]["slew_limited"]}。',
f'- 第二轮峰值相电流{runs[1]["peak_a"]:.3f} A，累计最大相RMS {max(runs[1]["rms_a"]):.3f} A，斜坡限幅样本{runs[1]["slew_limited"]}。',
'- RMS累计包含休息段，不能当成每个频点运动段RMS。两轮均未触发15 A/5 A主机停止条件。',
'- 最终BDTR=0x02002054，MOE=0；runtime、FOC、位置和速度故障均0，MPT通信错误计数均0。最终状态另行回读确认，第二轮结果表含原始导出共三次读取完全相同。',
'- 相位随频率增加的趋势清晰。幅值转折的重复性不足且多点限幅，下一步应先隔离斜坡影响，并在8～11 Hz逐点重复采集实际时域轨迹，再区分机械/摩擦非线性与控制器频响。此处为待验证方向，未在本次修改斜坡或归因于机械共振。','',
'## 固件身份和读取记录','',
'原板完整匹配R17后通过正常控制入口停机；首次批量读取的7个字出现差异，两次单字读取一致并匹配预期，保留原始捕获与修正记录。',
'新固件已写入，首次自动校验失败，没有直接进入运动。随后94760字节完整回读与新AXF装载镜像逐字节一致，再通过main断点关闭enable后初始化。早期未复位的旧执行上下文停在HardFault，受控复位后恢复，后续运行故障为0。',
'测试脚本仍重新验证完整Flash，批量读差异仅允许逐字双读均匹配预期后继续；真实持续差异拒绝RAM写入。该路径新增主机回归，当前Python共22项通过。',
f'固件AXF SHA256：`{comparison["firmware_sha256"]}`。','',
'证据：',
'- [第一轮原始结果](xlsx_run2/customer_results.md)、[第一轮状态与参数](xlsx_run2/summary.json)',
'- [第二轮原始结果](xlsx_repeat/customer_results.md)、[第二轮状态与参数](xlsx_repeat/summary.json)',
'- [最终OFF读回](final_off/result.json)、[完整镜像校验](flash_customer/recheck/result.json)',
'- [数值汇总](measured_comparison.json)、[冻结固件](flash_customer/firmware.axf)',
'- [操作说明](../../../docs/客户位置指令幅相测试说明.md)','']
(root/'客户位置指令_实测幅相报告.md').write_text('\n'.join(lines),encoding='utf8')
print('Measured comparison report generated:',root)
print('max gain repeat difference',comparison['maximum_gain_difference_db'],'max phase difference',comparison['maximum_phase_difference_deg'])
