# 电流开环模式的 D/Q 电流闭环修复说明

## 1. 结论

工程中的 `FOC_CONTROL_MODE_CURRENT_OPEN_LOOP` 应理解为：

> **电角度开环，D/Q 电流闭环。**

它不使用真实编码器。为了避免相序、极性、坐标变换尚未确认时直接闭 PI，
现在按三个运行时阶段投运：

1. `FOC_CURRENT_STAGE_D_VOLTAGE_CHECK`：固定电角度，强制 `Uq=0`，PI 完全旁路；
2. `FOC_CURRENT_STAGE_D_CURRENT_P_CHECK`：只闭 D 轴低增益 P 环，Q 轴输出仍为 0；
3. `FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP`：D 轴 P 环稳定后，真实 Id/Iq 进入低增益 D/Q PI。

本次实机波形中：

```text
Id_ref = +0.100 A
Id     = -1.833 A
Iq_ref =  0.000 A
Iq     = -1.194 A
```

正 D 轴指令下 D 轴反馈持续向负方向增长，说明闭环符号错误。PI 代码虽然在执行，但反馈形成了正反馈，因此波形表现为“没有闭环”。同时，默认电角频率被改成了 0 Hz，虚拟角度固定为 0 rad，电流开环模式退化成了静止直流矢量注入。

第二次实机波形进一步显示，`Id/Iq` 为近似等幅、相差 90° 的周期量，且
`Uq` 已接近当前 `±0.02 pu`（24 V 时约 ±0.48 V）限幅。以截图瞬时值为例：

```text
Iq_ref = 0 A
Iq     = 1.308 A
eq     = -1.308 A
Kp×eq  = 0.05×(-1.308) = -0.0654 pu
```

因此 PI 确实读入了反馈并产生反向校正，但所需电压已经超过限制。问题不是
“PI 函数没执行”。检查中确认了一项会破坏反馈质量的配置：旧 U/V 两个 rank
不是同一时刻的电流快照，PWM 纹波和通道差异会被带入 dq，使控制器更容易
持续追赶并饱和。该问题已修复，但它是否是实机异常的唯一原因仍需新波形验证。

## 2. 本次修复

### 2.1 先固定电角度验证 Ud/Id 方向

默认配置改为：

```c
active_stage = FOC_CURRENT_STAGE_D_VOLTAGE_CHECK;
electrical_frequency_hz = 0.0f;
electrical_angle_rad = 0.0f;
d_voltage_check_v = 0.0f;
```

这个阶段每个 ADC 周期执行：

```text
读取 U/V 电流并换算 Id/Iq
Ud = 限幅和斜坡后的 d_voltage_check_v
Uq = 0（固件强制，不提供可写 Q 轴测试命令）
固定角度逆 Park + SVPWM
```

`FOC_ControllerStep()` 在这个阶段完全不调用，因此 D/Q PI 的比例项和积分项都
不会影响方向判断。只有确认正、负 Ud 对应的 Id 变化方向正确后，才能关闭输出，
把 `requested_stage` 改为 `FOC_CURRENT_STAGE_D_CURRENT_P_CHECK`。

阶段切换只在 `output_request=0`、MOE 已关闭、控制器停止时生效；切换时会清空
PI 积分器、测试电压、应用电流指令和角度。电流给定按 0.2 A/s 斜坡进入。
进入 P/PI 阶段后仍默认保持 0 Hz，先验证静止电流闭环；
静止闭环正确后，才可以逐步增加 `electrical_frequency_hz`。

### 2.2 修正相电流反馈极性

FOC 统一规定：电流流入电机相端为正。板上低侧分流采样的原始正方向与该定义相反，因此在采样换算边界设置：

```c
g_current_sampling.polarity_u = -1;
g_current_sampling.polarity_v = -1;
```

转换过程为：

```text
IU = (raw_u - offset_u) × polarity_u × ADC比例
IV = (raw_v - offset_v) × polarity_v × ADC比例
IW = -(IU + IV)
```

极性只在 ADC 物理量换算层修正，不修改 ADC rank、Clarke/Park 公式或 PWM 相序，避免在多个层次重复翻转。

### 2.3 增加闭环软件测试

主机测试新增对称 RL 电机的 D/Q 数值模型：

1. 给定 `Id_ref = 0.1 A`、`Iq_ref = 0 A`；
2. 虚拟电角度以 2 Hz 连续旋转；
3. 把模型 D/Q 电流逆变换为 U/V 反馈；
4. 执行正式 `FOC_AlgorithmStep()`；
5. 用 PI 输出的 D/Q 基波电压更新 RL 模型；
6. 验证稳态 Id 跟随 0.1 A，Iq 接近 0 A。

该测试保护控制算法的闭环符号、Clarke/Park 变换和旋转角度下的 D/Q 解耦。硬件相序和采样模拟链仍需实机波形验证。

### 2.4 修复 U/V 顺序过采样造成的时间错位

旧配置对 ADC1 注入组启用 16 倍硬件过采样。STM32 的执行顺序是：

```text
U 相连续转换 16 次（约 5.714 us）
    ↓
V 相连续转换 16 次（约 5.714 us）
```

U/V 两相采样中心相差约 5.714 us，相当于 20 kHz PWM 周期的约 41°。
这样的两个数值不能严格组成同一时刻的 Clarke 电流矢量，并会把 PWM 纹波和
U/V 差异转换为负序 dq 分量。

修复后注入组关闭逐 rank 硬件过采样：

```text
U 相单次转换（357 ns）→ V 相单次转换（357 ns）
```

两相中心只相差约 357 ns，完整序列约 0.714 us。ADC 数据改为显式右对齐，
`JDR1/JDR2` 范围恢复为 `0..4095`，换算中的 `oversampling_ratio` 同步改为 1。

### 2.5 加强零点校准

旧状态机只计划 20 点，并允许 1 ms 超时后用任意非零样本数计算 offset。
现在改为：

- 稳定等待 10 ms；
- 完整采集 400 点（20 kHz × 20 ms）；
- 50 ms 仍未采满才进入 `CAL_ERROR`；
- 只有采满 400 点才更新 `offset_u/offset_v`；
- 保存 U/V 校准窗口的 min/max，供 Watch 检查噪声和干扰。

采样电阻按实物确认值保持 **7 mΩ**；原理图对应标注与实物不一致，不能据此
把软件参数改为其它阻值。运放增益当前按软件既有配置保持 `6.1`，由于原理图
已确认存在错误，该增益仍应以后续实物网络或标定结果为准。

## 3. 修复后的完整实时链路

```text
TIM1 CH4 触发 ADC 注入转换
    ↓
JDR1/JDR2 得到 U/V 原始电流
    ↓ 减零点、乘 polarity=-1、换算安培
IU、IV，重构 IW=-(IU+IV)
    ↓
Clarke：IU/IV → Iα/Iβ
    ↓
Park：Iα/Iβ + 虚拟电角度 → Id/Iq
    ↓
ed = Id_ref - Id
eq = Iq_ref - Iq
    ↓
D/Q PI：ed/eq → Ud/Uq
    ↓
电压矢量限幅和抗积分饱和
    ↓
逆 Park + SVPWM
    ↓
TIM1 CCR1/CCR2/CCR3
    ↓
功率级和电机
    ↓
下一周期重新采样，形成真实闭环
```

## 4. 实机验证步骤

### 4.1 上电前

1. 实验电源设置较低限流；
2. 确认 `active_stage = FOC_CURRENT_STAGE_D_VOLTAGE_CHECK`；
3. `electrical_frequency_hz = 0`、`electrical_angle_rad = 0`；
4. `d_voltage_check_v` 从 0 开始，只做很小的正负阶跃；
5. 确认母线电压配置与实际一致。

### 4.2 Watch 应观察的变量

```text
g_global_mode_control.active_mode
g_global_mode_control.current_open_loop.d_current_reference_a
g_global_mode_control.current_open_loop.q_current_reference_a
g_global_mode_control.current_open_loop.requested_stage
g_global_mode_control.current_open_loop.active_stage
g_global_mode_control.current_open_loop.d_voltage_check_v
g_global_mode_control.current_open_loop.applied_d_voltage_check_v
g_global_mode_control.current_open_loop.electrical_frequency_hz
g_global_mode_control.current_open_loop.electrical_angle_rad
g_global_mode_control.current_open_loop.measured_d_current_a
g_global_mode_control.current_open_loop.measured_q_current_a
g_global_mode_control.current_open_loop.output_d_voltage_v
g_global_mode_control.current_open_loop.output_q_voltage_v
g_global_mode_control.runtime.output_request
g_global_mode_control.runtime.output_armed
g_global_mode_control.runtime.output_enabled
g_global_mode_control.runtime.fault_flags
g_foc_controller.algorithm.parameters.d_controller.integral
g_foc_controller.algorithm.parameters.q_controller.integral
g_foc_controller.algorithm.output.saturated
g_current_sampling.polarity_u
g_current_sampling.polarity_v
g_current_sampling.oversampling_ratio
g_current_sampling.offset_u
g_current_sampling.offset_v
g_current_sampling.calibration_sample_count
g_current_sampling.calibration_min_u
g_current_sampling.calibration_max_u
g_current_sampling.calibration_min_v
g_current_sampling.calibration_max_v
g_tim1_pwm_sampling.sampled_trigger
g_tim1_pwm_sampling.window_rejections
```

### 4.3 第一阶段正确现象

- `electrical_angle_rad` 固定不变；
- `cycle_count` 持续增加；
- `output_q_voltage_v` 始终接近 0；
- `d_voltage_check_v` 从 0 增大为正值时，Id 应沿同一方向稳定变化；
- `d_voltage_check_v` 回到 0 时，Id 应回落；改为小负值时 Id 应反向；
- D/Q PI 积分量保持 0，因为这个阶段 PI 没有执行；
- `oversampling_ratio = 1`，校准样本数达到 400；
- U/V 的校准 `max-min` 应较小且重复上电稳定；
- `output_d_voltage_v/output_q_voltage_v` 会随误差变化；
- 三相 duty 持续变化；
- `fault_flags = 0`，MOE 保持打开。

### 4.4 方向确认后先接入 D 轴 P 环

```text
1. runtime.output_request = 0，等待 output_enabled = 0
2. requested_stage = FOC_CURRENT_STAGE_D_CURRENT_P_CHECK
3. 等待 active_stage 同步，stage_switch_count 加 1
4. 保持 electrical_frequency_hz = 0
5. Id_ref 从 0 开始给很小阶跃，Iq_ref = 0
6. runtime.output_request = 1
```

这个阶段使用 `D.Kp=0.005 pu/A、D.Ki=0`，Q 轴 `Kp=Ki=0`，因此只有 D 轴
比例负反馈。若仍啸叫，应停止并排查采样纹波/延迟，而不是继续加积分。

### 4.5 D 轴 P 环稳定后接入 D/Q PI

再次关闭输出，把 `requested_stage` 改为
`FOC_CURRENT_STAGE_DQ_PI_CLOSED_LOOP`。未提供实测电机 R/L 时，默认使用较低的
`Kp=0.005 pu/A、Ki=0.2 pu/(A·s)`；原来的 `Kp=0.05、Ki=2` 不再自动启用。
确认 Id 收敛、Iq 接近 0、积分器不饱和后，再进行旋转虚拟角度测试。

同周期诊断变量位于：

```text
g_foc_controller.algorithm.output.d_current_error
g_foc_controller.algorithm.output.q_current_error
g_foc_controller.algorithm.output.d_pi_command
g_foc_controller.algorithm.output.q_pi_command
```

这些字段在一次 ADC 中断内同时锁存，暂停内核后可直接验证 PI 符号，避免 Watch
跨多个控制周期拼接出互相矛盾的反馈和输出。

### 4.6 仍异常时如何判断

| 现象 | 优先检查 |
| --- | --- |
| 正 Id 指令下 Id 再次稳定为负 | 固件是否重新烧录；Watch 中 `polarity_u/v` 是否为 -1；PWM 相序 |
| Id 能跟随但 Iq 很大 | U/V 模拟增益不平衡、实物相序或电机反电势；确认新固件 `oversampling_ratio=1` |
| D/Q 电压长期顶在限幅 | 电流比例错误、反馈方向仍错、电压限幅过低或电机 R/L 参数不合适 |
| 角度不变化 | `electrical_frequency_hz` 是否被 Watch 改成 0；是否运行了固定角度阶跃测试 |
| 指令和反馈都不更新 | `output_request/output_armed/running`、ADC 校准状态和故障标志 |

## 5. 当前边界

本次修复建立了软件负反馈、自动虚拟电角度，并消除了 ADC 单通道 16 倍顺序
过采样造成的 U/V 时间错位。烧录后仍必须低限流验证；若 `Id/Iq` 仍出现
明显等幅正交波形，应同步打印 U/V/W、dq 电流和 dq 电压，并用外部电流探头
区分真实负序电流与模拟采样链不平衡。当前不应通过提高 PI 增益或放宽电压
限幅来掩盖输入问题。
