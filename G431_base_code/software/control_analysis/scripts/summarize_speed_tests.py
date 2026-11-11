"""Generate auditable tables/plots from successful captured speed tests only."""
from pathlib import Path
import json,struct,re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).resolve().parents[1]/'results'
log_dir=Path(__file__).resolve().parents[2]/'docs'/'log'
log_dir.mkdir(parents=True,exist_ok=True)
steps=[]; freqs=[]
for p in sorted(r.glob('speed_*.json')):
 d=json.loads(p.read_text(encoding='utf-8')); d['tag']=p.stem
 if 'target_rad_s' in d: steps.append(d)
 if 'frequency_hz' in d: freqs.append(d)
lines=['# 速度测试实测汇总','',
 '> 开发调试记录；仅汇总历史采集数据，不代表当前固件的实机验收。',
 '> 数据来源：[原始结果目录](../../control_analysis/results/)。当前系统说明见[架构说明](../架构说明.md)。','',
 '自动从同名 JSON 生成。速度为编码器滤波反馈；不代表独立测速精度。',
 '', '## 时域','',
 '| 标签 | Kp / Ki | 给定 rad/s | 末段均值 | 标准差 rad/s | 采样超调 % | IAE rad | 保持进入误差带 |',
 '|---|---|---:|---:|---:|---:|---:|---|']
for d in steps:
 lines.append(f"|{d['tag']}|{d.get('kp','?')} / {d.get('ki','?')}|{d['target_rad_s']:.3f}|{d['speed_mean_rad_s']:.3f}|{d['speed_std_rad_s']:.3f}|{d['overshoot_pct']:.2f}|{d['iae_rad']:.3f}|{'未达到' if d['settling_time_s'] is None else str(round(d['settling_time_s'],3))+' s'}|")
lines+=['','初期 step_initial/step_damped 使用不同固件，不能只将改进归因于 PI。tune_* 为同一优化版本。',
 '', '## 频域','',
 '| 标签 | Kp / Ki | Hz | 闭环增益 dB | 相位 deg | 残差 RMS rad/s | 有效周期数 |',
 '|---|---|---:|---:|---:|---:|---:|']
freqs.sort(key=lambda d:d['frequency_hz'])
for d in freqs:
 lines.append(f"|{d['tag']}|{d.get('kp','?')} / {d.get('ki','?')}|{d['frequency_hz']:.2f}|{d['closed_loop_gain_db']:.2f}|{d['closed_loop_phase_deg']:.2f}|{d['residual_rms_rad_s']:.3f}|{d['measured_cycles']:.2f}|")
lines+=['','这是 2 rad/s 偏置、0.5 rad/s 正弦幅值下的经验闭环响应，包含测量滤波。',
 '残差反映未由基频解释的波动，不是独立重复试验的置信区间。无开环裕度结论。',
 '频率点连线仅辅助阅读，不代表连续扫频。相位表为主值，图对每组执行相位展开；稀疏点之间不能排除额外相位绕转。',
 '未测到交点时不报告确定带宽；测到两侧点也只能给局部跨越区间，需考虑短记录启动瞬态与非线性。',
 '', '![时域比较](../../control_analysis/results/speed_step_comparison.png)','', '![频率响应](../../control_analysis/results/speed_frequency_response.png)']
(log_dir/'未注明日期_速度测试时域与频域实测汇总.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
fig,ax=plt.subplots(2,1,figsize=(10,7),sharex=True)
for tag in ('tune_p10_i05','tune_p15_i05','tune_p20_i05','repeat_positive','repeat_negative'):
 p=r/f'speed_{tag}.bin'
 if not p.exists(): continue
 d=json.loads(p.with_suffix('.json').read_text()); a=np.array(list(struct.iter_unpack('<8f',p.read_bytes())))[:d['samples']]
 ax[0].plot(a[:,0],a[:,2],label=tag)
 ax[1].plot(a[:,0],a[:,5],label=tag)
ax[0].set_ylabel('Encoder speed (rad/s)');ax[1].set_ylabel('Iq reference (A)');ax[1].set_xlabel('DWT time (s)')
for x in ax: x.grid(alpha=.3);x.legend(fontsize=8)
fig.tight_layout();fig.savefig(r/'speed_step_comparison.png',dpi=140);plt.close(fig)
fig,ax=plt.subplots(3,1,figsize=(8,8),sharex=True)
if freqs:
 groups=sorted(set((round(d.get('kp',0),4),round(d.get('ki',0),4)) for d in freqs))
 for kp,ki in groups:
  group=[d for d in freqs if (round(d.get('kp',0),4),round(d.get('ki',0),4))==(kp,ki)]
  f=[d['frequency_hz'] for d in group]
  for a,k in zip(ax,('closed_loop_gain_db','closed_loop_phase_deg','residual_rms_rad_s')):
   values=[d[k] for d in group]
   if k=='closed_loop_phase_deg': values=np.rad2deg(np.unwrap(np.deg2rad(values)))
   a.semilogx(f,values,'o-',label=f'Kp={kp}, Ki={ki}');a.set_ylabel(k);a.grid(alpha=.3);a.legend()
ax[-1].set_xlabel('Frequency (Hz)');fig.suptitle('Measured closed-loop response; not open-loop stability margins')
fig.tight_layout();fig.savefig(r/'speed_frequency_response.png',dpi=140);plt.close(fig)
print('Saved measured summary and plots; steps=',len(steps),'frequency points=',len(freqs))
