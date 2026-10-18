# FOC 调制模块与过调制

`foc_algorithm.c` 负责 Clarke/Park、电流 PI、条件积分抗饱和；
`foc_modulation.c` 独立负责 αβ 基波指令到三相占空比；
`foc_interface.c` 负责运行门控、端口调用和快速故障锁存；
`tim1_pwm.c` 负责硬件允许的占空比范围及 CCR 写入；
`servo_manager.c` 负责故障状态、Servo ON/OFF 和后台上报。
调制模块不依赖 HAL，也不修改 running、MOE 或 ADC。

## 电压单位与理论上限

所有输入电压均为“目标基波相电压峰值 / 实际直流母线电压”。当前电流 PI
直接输出这种标幺量；本次没有加入母线电压 ADC 采样。若上层改用伏特，必须先
用有效的实测 Vdc 归一化，不能直接把伏特输入本接口。

设 s = duty_max - duty_min。目标旋转基波幅值的范围为：

| 区域 | 基波峰值 / Vdc | 方法 |
| --- | --- | --- |
| 线性 | 0 .. s/√3 | min/max 零序注入 |
| OVM1 | s/√3 .. 0.6056966996·s | 参考圆超出六边形时保角度径向投影 |
| OVM2 | 0.6056966996·s .. 2s/π | 逐渐延长最近有效矢量的保持区 |
| 六步极限 | 2s/π | 每 60° 选用相邻有效矢量，换相点等时间混合 |

对于允许 duty=0..1 的理想功率级，最大基波为 2Vdc/π ≈ 0.63662 Vdc，
比线性 SVPWM 的 Vdc/√3 高 10.27%。六边形顶点的 2Vdc/3 是瞬时矢量幅值，
不是基波幅值；不能把它与 Vdc/√3 相除后声称基波还有 15.47% 的增益。
此处六步指两电平逆变器的六个有效矢量轨迹，三相都由互补桥臂驱动，
不是让一相悬空的梯形 BLDC 六步换相。

## 计算与基波增益补偿

OVM1 先做三相投影，若 max(va,vb,vc)-min(va,vb,vc)>1，则按该差值等比缩小。
等效于 T1'=T1/(T1+T2)、T2'=T2/(T1+T2)、T0'=0；相位不变。

OVM2 对归一化占空比的中间相（即 q=T2/(T1+T2)）做：

```text
q' = clamp(0.5 + (q - 0.5)/(1 - h), 0, 1)
```

最大/最小相维持 1/0。h 从 0 增加到 1，混合区逐渐缩小至六步。
这是本模块采用的驻留时间展开法，不能把 ST SDK 的另一套保持角参数直接套用。

两段过调制均非线性。离线用独立的扇区 T1/T2 公式和 Fourier 积分生成单调
增益表，运行时二分查找、线性插值，使目标基波与输入幅值近似一致。
实时路径无 atan2、无数值积分、无内存分配；二分查找上界为 7/8 次比较。
这种基波增益定义适用于恒幅、匀速旋转输入；快速瞬态仍应按实际电压分析。

```powershell
python FOC/Tools/generate_modulation_lut.py --check
# 不带 --check 时向 stdout 输出表头，可用于审查和重新生成。
```

`fundamental_d/q_voltage` 是限幅后的基波目标；原有 `alpha/beta_voltage`、
`d/q_voltage`、`phase_*_voltage` 则从本周期实际 duty 重构，包含过调制畸变。
PI 以基波能力做条件积分：本次积分若进一步推向饱和区外就撤销，并重新计算
调制；向内退饱和时允许积分。不能把实际电压中的谐波全部回算给 PI，否则会
抵消过调制。`saturated` 表示限幅或进入 OVM，`modulation_region` 提供具体区域。

## 当前板级采样约束

两路低侧电流反馈保留 16 倍过采样，完整序列仍约 11.428 µs，原始码和 S16
电流尺度均未改变。`main.c` 在 ADC 启动前用 `ADC1_CS_SEQUENCE_CONV_NS`
调用 `Tim1Pwm_ConfigureCurrentSampling()`，将统一可用范围交给调制模块。

独立的 `Modules/Three_Phase_Driver/Src/pwm_sampling.c` 按下一组三相 CCR 规划
触发时刻，不访问 HAL。两路传感器固定在 U/V，不能按扇区随意切换到 W。
设 P=ARR、S=完整序列计数、G=死区与建立时间计数、C=max(CCR_U,CCR_V)：

```text
U/V 公共低侧稳定窗口 = [C+G, 2P-C-G]
可用起点 = [C+G, 2P-C-G-S] 与 [1, P-1] 的交集
另排除会让完整序列碰到 W 相开关边沿及其建立区的起点
优先起点 = P-S（序列在载波峰值完成），不可行时选最近的有效起点
统一保证可行的最大 CCR = P-G-ceil(S/2)
```

默认最小高侧脉宽 1 µs、死区 500 ns、额外建立时间 500 ns，向保守方向取整。
额外建立时间是调试起点，需用实测反馈波形复核，不能据此宣称采样已验证。
当前 P=4199、S=1920、G=169，最大 CCR=3070。参数约得到
duty=0.02023810..0.73095238，s≈0.71071429，对应最大基波约
0.45245475 Vdc；精确范围可看 `g_foc_controller.algorithm.parameters.modulation`，
精确基波上限可看 `output.fundamental_voltage_limit`。
这比理想全母线极限低，主要是较长的采样窗口造成的降额。仅移动触发点不能
缩短这个窗口，因此本次没有实质提高统一电压上限。这个数值是当前保守的
统一占空比跨度策略的结果，不是所有可能调制/重构策略的全局最优证明。此前只限幅
到 0.57735 的实现没有证明高调制比下的低侧采样有效，不能把该值当作已可用能力。

本次保留 ADC 转换时长，用所有桥臂共同的 duty 范围保证两路电流窗口。
要接近全母线 0.63662 Vdc，需要另行缩短转换窗口、引入电流重构/采样脉冲调制，
或使用不依赖低侧导通的相电流传感器，并复核高侧持续导通时的栅极供电。
只有这些条件成立后才可配置 duty=0..1。若 s<1，六步极限指缩放后的线电压
六步轨迹，桥臂仍在上下界 PWM 以保留采样和刷新窗口。

`Tim1Pwm_SetPhaseVoltage()` 直接把 float duty 换算成 CCR，不再经过 1000 级中间量；
独立的整数 `Tim1Pwm_SetPhaseDuty()` 测试接口保持原有行为，不承诺 ADC 窗口有效。
FOC 浮点接口先完成全组三相参数与采样窗口检查，再同步预装载 CCR1..4。
RCR=1 在启动前配置，更新在谷值；触发明确使用 TIM1_TRGO2=OC4REF
上升沿，每个完整载波周期触发一次，禁用空的 CC4 中断。不能混淆 CC4
比较事件、OC4REF 电平边沿以及峰值/谷值更新事件。

短临界区暂禁更新，整批写完后恢复；提交前后均检查距离谷值的 2 µs 余量，
越界或窗口不足均返回失败，FOC 锁存 PWM 写入故障并关闭 MOE、撤销 running。
这不是实测的最坏执行时间保证，上板仍需确认 ISR 耗时和模拟信号建立时间。
CubeMX `.ioc` 当前没有维护 TIM1/ADC1，配置仍由现有手工 L1 源码管理，
不能用该 `.ioc` 重新生成后直接覆盖这些配置。

ADC 完成回调先记录本周期已生效的 CCR4，再运行 FOC；PI 积分周期采用
`(2P+本次CCR4-上次CCR4)/TIM时钟`。校准暂停回调后的首周期用名义周期起算。
未来电角度快照也应对齐真实采样时刻；U/V 顺序过采样间约 5.714 µs 的时差仍存在。

调试器中的 `g_tim1_pwm_sampling` 提供触发点、序列结束点、有效窗口和窗口/
截止时间拒绝计数。`planned_*` 是预装载的下一周期计划，`sampled_trigger`
是当前已完成样本的触发点，两者不要混用。

## 配置与验证

应用已选择 `FOC_MODULATION_OVERMODULATION`，`voltage_limit=2/pi`；
实际上限由配置与板级跨度取小值。要只保留线性输出可切换到
`FOC_MODULATION_LINEAR_ONLY`。现有电角度业务未接入，`angle_valid` 仍为 0；
不能为了试验过调制把它直接强制为 1。

主机测试从真实 C 模块输出的 duty 独立重构电压并测量基波，覆盖六扇区、
区域边界、OVM1/OVM2/六步、占空比降额、NaN/Inf、PI 抗饱和和故障关断。
不连接电机，不写 MCU。Windows MSVC 开发者终端可执行：

```powershell
./Tests/Host/run_tests.ps1
# 也可用 -VcVarsPath 指定本机 vcvars64.bat；或用 -ZigPath 指定 zig.exe。
```

本次主机 Fourier 扫描使用 81 个幅值、每个幅值 32768 个相位点，外加非对称
占空比降额测试，最大基波误差为 0.000018244 Vdc（约 0.00183% Vdc）。
Keil Build Target 已得到 0 Error(s)、0 Warning(s)。这些是数值和构建结果，
不包含任何电机带载或实际 ADC 窗口测量。

动态采样测试逐个枚举小计数模型的 U/V/W 比较值、穷举所有合法触发时刻，
核对是否存在窗口及所选最接近峰值完成的起点；同时在真实计数参数下扫描
7200 个电压角度，验证完整序列未越出 U/V 稳定窗口或碰到 W 开关边沿。

Keil 工程和 CMSIS 工程清单均已加入新模块。数学测试和编译不能代替上板测量
ADC 窗口、采样相序、实际电流波形及电流环稳定性；OVM 会增加低次电流谐波。

参考：[ST MC SDK Overmodulation](https://wiki.st.com/stm32mcu/wiki/STM32MotorControl%3ASDK_Overmodulation)。
触发路由、RCR 与预装载语义见 [ST RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。
