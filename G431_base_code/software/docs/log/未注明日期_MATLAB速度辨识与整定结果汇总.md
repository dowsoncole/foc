> 开发档案 · 辨识结果 · 2026-10-26 统一归档
> 原文位置：`software/control_analysis/MATLAB辨识与整定结果.md`。
> 以下保留当时的配置、试验过程、失败结果和结论；“当前/最新/已验证”均指记录当时。现行说明见[架构说明](../架构说明.md)。
> 原文命令和代码块内的路径保持历史写法；Markdown 链接已按新目录调整。原路径基准为工程根目录或原文另行指定的目录。
> 原始数据与脚本：[control_analysis](../../control_analysis/)。原文裸写的 results/、scripts/ 均以该目录为基准。

# MATLAB 辨识与整定：本轮结果

流程已跑通；**候选 PI 尚未通过性能验收，不是最优参数**。电机输出已关闭。

## 最新复测：官网核查后重新辨识

- 修正激励更新与采样节拍不同步：重复周期的输入一致率由约 97%～98% 提高到 100%。
- 使用 10.4 秒 PRBS，舍弃首周期后分析 4 个完整周期；零均值 ±0.10 A 响应重复性约 97%，最佳独立仿真验证拟合 **65.05%**，仍未通过本轮 70% 等联合门槛。
- 幅值对照：±0.06 / ±0.10 / ±0.14 A 的速度峰值分别约 **2.45 / 5.21 / 8.62 rad/s**。最后一项接近 10 rad/s 保护线，不再增大。
- 带偏置 0.16±0.04 A 复测的最佳拟合只有 **16.05%**，不用于输出新 PI。
- 当前没有新的已验收 PI；下一步应提高实际 Iq/饱和状态记录质量，分清内环跟踪和工作点变化的影响，不能只继续增大激励或继续追求模型带宽。

![同步后的频域验证](../../control_analysis/results/speed_prbs_sync_analysis/frequency_validation.png)

完整过程见[详细日志第 13～19 节](2026-10-21_速度闭环详细工作日志.md#13-官网用法核查与本次复测范围)。以下保留上一轮结果，便于回溯。

## 做了什么

1. 参考 demo_coder 的已验证流程，原工程未改动。
2. 旁路速度 PI、保留电流内环，采集真实 Iq PRBS→速度响应。
3. MATLAB 完成 TF/ARX/OE 对照、独立数据验证、spafdr 和相干性分析。
4. MATLAB pidtune 自动生成 PI，再实测正反向阶跃和闭环 PRBS。

## 候选和实测

MATLAB 候选：`Kp=0.03205945`、`Ki=1.31987164`，单位分别为 A/(rad/s)、A/rad。试验 Iq 上限 .3 A，速度指令 ±2 rad/s。

| 指标 | 正向重复测试 | 反向测试 |
|---|---:|---:|
| 给定 rad/s | 2 | -2 |
| 末段平均 rad/s | 2.109 | -2.092 |
| 末段标准差 rad/s | .214 | .101 |
| 保存点超调 | 31.06% | 13.31% |
| MATLAB 上升时间 | 43.2 ms | 33.7 ms |
| 严格误差带保持 ≥.5 s | 未达到 | 未达到 |

上升时间基于 16 ms 记录点插值，不具备小数位所暗示的精度。平均速度接近给定不等于波动已经解决。

实测闭环 PRBS 的 spafdr 在约 **8.55 Hz 放大 9.10 dB**。相对 0 dB 的 -3 dB 跨越位于约 **12.57～12.92 Hz**，仅为本次有限数据的局部估计。MATLAB `bandwidth(g)` 返回 15.91 Hz，但其最低频增益估计异常偏低，不能把该值当成已验收带宽。

模型预测的 0% 超调、65° 相位裕度、11.08 Hz 带宽均为**模型预测**，不是实机已证实指标。模型验证拟合仅 52.55%，对其他工作点泛化不足；因此未将候选 PI 写入启动默认配置。

## MATLAB 生成的图

![实机阶跃](../../control_analysis/results/matlab_measured_evaluation/measured_steps.png)

![实测频响和模型预测](../../control_analysis/results/matlab_measured_evaluation/measured_vs_predicted_frequency.png)

[完整工作日志](2026-10-21_速度闭环详细工作日志.md) · [MATLAB 分析脚本](../../control_analysis/scripts/matlab_identify_speed.m) · [MATLAB 实测评估脚本](../../control_analysis/scripts/matlab_evaluate_speed.m)
