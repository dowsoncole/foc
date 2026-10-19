# 2026-10-19 详细开发记录：FOC 电压开环实机运行与执行时间验证

> **今日节点结论：电压开环功能已经完成，电机已经能够正常转动。**  当前固件可在
> 20 kHz ADC 注入转换完成节拍下运行固定 D/Q 电压开环算法，连续推进电角度并生成
> 三相 PWM。实机最终使用 `D = 0.2 V`、`Q = 0 V`、母线 `24 V`、电角频率
> `2 Hz` 运行。PA0 示波测量确认 ADC 完成后的 FOC 软件链路能够在一个
> `50 µs` 控制周期内结束，满足当前 20 kHz 实时性要求。

## 1. 记录范围与版本基线

| 项目 | 内容 |
| --- | --- |
| 日期与时区 | 2026-10-19，Asia/Shanghai（UTC+08:00） |
| 工程 | `G431_base_code/software/servo_v1` |
| 工作分支 | `main` |
| 本次修改前 Git 基线 | `72e0033`，`feat: modularize servo control and record 2026-10-18 changes` |
| 当前功能模式 | `FOC_ALGORITHM_VOLTAGE_TEST_MODE = 1U` |
| MCU | STM32G431 |
| 三相驱动器 | STSPIN32G4 |
| PWM/控制频率 | 20 kHz，周期约 50 µs |
| 实机母线电压 | 24 V |
| 最终开环给定 | D 轴 0.2 V，Q 轴 0 V，电角频率 2 Hz |
| 最终实机结果 | 电机正常转动，驱动 READY/nFAULT 正常，无软件故障 |
| 最终 Keil 构建 | `0 Error(s), 0 Warning(s)` |

本记录描述从编译工程修复、开环算法接入、实时调度、功率级故障定位、启动安全改造、
电压量纲修正直到电机成功转动的完整过程。文中明确区分算法运行、PWM 寄存器更新和
功率级实际输出，避免再次把三个状态混为一谈。

## 2. 今日目标及完成情况

今天的目标不是建立完整电流闭环，而是先独立验证算法层及实时链路：

1. 暂时屏蔽正式 Servo Manager 状态管理。
2. 使用固定 D 轴电压和连续变化的电角度运行开环算法。
3. 由已有 FOC 调制模块输出三相 PWM。
4. 用一个统一数据结构管理开环给定、角度、斜坡和运行状态。
5. 测量 ADC 转换完成至 FOC 计算及 PWM 提交结束的执行时间。
6. 在真实功率板和电机上确认电压开环能够驱动电机正常转动。

上述目标均已完成。当前仍然是调试专用电压开环模式，不代表电流环、速度环、位置环
或量产状态机已经完成。

## 3. 最终运行参数

开环运行参数集中在 `g_foc_voltage_test.open_loop` 中：

| 字段 | 当前值 | 单位与含义 |
| --- | ---: | --- |
| `d_voltage_v` | 0.2 | D 轴目标电压，物理伏特 |
| `q_voltage_v` | 0 | Q 轴目标电压，物理伏特 |
| `dc_bus_voltage_v` | 24 | 母线电压，物理伏特 |
| `electrical_frequency_hz` | 2 | 电角频率，不是机械转速 |
| `voltage_ramp_rate_v_per_s` | 0.5 | D/Q 给定斜坡速度，V/s |
| `applied_d_voltage_v` | 最终为 0.2 | 斜坡后的实际 D 轴给定 |
| `applied_q_voltage_v` | 0 | 斜坡后的实际 Q 轴给定 |
| `voltage_ramp_complete` | 1 | 电压斜坡已到目标值 |

算法内部先把物理电压换算为母线标幺值：

```text
Ud_pu = Ud_V / Vdc_V = 0.2 / 24 = 0.008333333
Uq_pu = Uq_V / Vdc_V = 0 / 24 = 0
```

换算后的标幺 D/Q 电压再进入反 Park 变换和调制模块。这样 Watch 中的 0.2 明确表示
0.2 V，而不再可能被误解为 `0.2 × Vdc`。

## 4. 最终软件链路

实时执行链路如下：

```text
TIM1 CH4 / TRGO2
    ↓ 硬件触发
ADC1 注入组两通道、16 倍过采样
    ↓ JEOS
ADC1_2_IRQHandler
    ↓ PA0 = 1
HAL_ADC_IRQHandler
    ↓
HAL_ADCEx_InjectedConvCpltCallback
    ↓ 发布 JDR1/JDR2 快照并增加 sample_sequence
MainFoc_OnAdcConversion
    ↓
FOC_AlgorithmVoltageStep
    ↓ D/Q 伏特给定 → 母线标幺 → 反 Park → SVPWM/调制
Tim1Pwm_SetPhaseVoltage
    ↓ 写入 TIM1 CCR1/CCR2/CCR3，并同步规划 CCR4
ADC1_2_IRQHandler 返回前
    ↓ PA0 = 0
下一控制周期
```

开环算法重新挂在 ADC 注入序列完成回调上，而不是依赖主循环或低频监控任务。这样每次
FOC 都使用确定性的 20 kHz 硬件节拍，PA0 测得的也是实际中断路径。

## 5. 开环电压算法实现

### 5.1 统一状态结构

新增 `foc_voltage_open_loop_t`，统一保存：

- D/Q 物理电压目标值；
- 母线电压；
- 电角频率和当前电角度；
- 周期计数；
- 斜坡后的实际电压；
- 电压斜坡速率和完成标志。

应用层不再用多个互不关联的全局变量拼接开环测试。算法需要的给定和算法自身推进的状态
属于同一个对象，便于 Watch 观察、参数调整及后续协议映射。

### 5.2 电压斜坡

每个采样周期的最大电压变化量为：

```text
ΔU = ramp_rate × Ts
   = 0.5 V/s × 50 µs
   = 25 µV/周期
```

从 0 V 上升到 0.2 V 理论需要：

```text
0.2 V / 0.5 V/s = 0.4 s
```

斜坡避免功率级刚使能时第一帧就出现完整电压阶跃，也使电机建立磁场和开始跟随旋转磁场
的过程更平缓。

### 5.3 电角度推进

电角度每个控制周期的增量为：

```text
Δθ = 2π × fe × Ts
   = 2π × 2 Hz × 50 µs
   ≈ 0.0006283185 rad
```

20 kHz 控制频率和 2 Hz 电角频率对应每个电周期 10000 个控制点。角度超过 `2π` 后
回绕到 `[0, 2π)`，`cycle_count` 每次成功执行后递增。实机 Watch 中角度和计数持续
变化，证明算法不是只运行一次，而是在连续实时执行。

### 5.4 调制输出

`FOC_AlgorithmVoltageStep()` 跳过电流反馈、Park 电流变换和 PI，直接完成：

1. 物理伏特给定除以母线电压；
2. D/Q 电压按当前电角度变换到 α/β；
3. 调用正式 `FOC_ModulationCompute()`；
4. 发布三相电压、三相 duty、扇区、调制区域和饱和标志；
5. 由 `Tim1Pwm_SetPhaseVoltage()` 写入三相比较寄存器。

开环与正式闭环复用同一个调制模块和 PWM 写接口，因此本次验证覆盖了从坐标变换到实际
CCR 提交的完整后半段链路，而不是独立生成一套临时正弦表。

## 6. 算法运行与功率输出解耦

今天调试中反复验证了三个状态不能混用：

| 状态 | 表达内容 |
| --- | --- |
| `running` | 开环算法是否持续执行 |
| `output_armed` | 是否已经通过显式请求、预充电和安全检查 |
| `output_enabled` | TIM1 BDTR.MOE 是否真实置位，栅极是否可能收到PWM |

最终固件上电默认：

```text
running        = 1
output_request = 0
output_armed   = 0
output_enabled = 0
```

此时算法、角度和输出 duty 可在 RAM 中持续更新，但功率级不会自动输出。只有调试者明确写入：

```text
g_foc_voltage_test.output_request = 1
```

主循环才执行自举预充电、READY/nFAULT 检查和中性矢量稳定过程，随后置
`output_armed = 1`。写回 `output_request = 0` 会立即清除 MOE，并将斜坡实际值归零。

这一设计防止复位、重新下载或调试器重新运行时电机突然自动带电。

## 7. 功率级启动流程

### 7.1 自举预充电

新增 `Tim1Pwm_BootstrapPrecharge()`：

1. 无条件关闭 MOE；
2. 在输出关闭状态下把三相 CCR 预装为低桥导通矢量；
3. 等待至少一个 PWM 更新周期，确保预装值进入活动寄存器；
4. 打开 MOE并保持低桥状态 10 ms，为高桥自举电容充电；
5. 切换到三相 50% 中性矢量；
6. 再等待寄存器更新。

随后应用层保持中性矢量 20 ms，并连续检查 READY/nFAULT。任何异常都会立即关闭 MOE、
取消输出请求并锁存功率级故障。

### 7.2 快速引脚安全门

每个 ADC/FOC 周期读取一次 `GPIOE->IDR`，同时检查：

- PE14：READY，必须为高；
- PE15：nFAULT，必须为高。

一次端口读取得到两个引脚的同一时刻快照。发生异常时保存：

- `first_unsafe_pin_levels`；
- `unsafe_sample_count`；
- `ready_low_count`；
- `nfault_low_count`。

主循环还会在首次异常后尽快读取 STSPIN32G4 STATUS 寄存器，保存
`first_fault_status_raw`，用于区分 VCC 欠压、过温、VDS 保护和复位。

## 8. 调试过程与关键问题

### 8.1 工程可以打开但新增算法没有真正进入目标

首先核对并修复了 Keil 工程中的源文件和头文件注册问题，确保 FOC、Servo、PWM 采样及
相关模块参与目标构建。判断构建是否成功以 Keil 构建日志中的 `0 Error(s)` 为准，不能
只看命令行程序退出码。

### 8.2 `FOC_AlgorithmVoltageStep()` 不执行

早期测试把算法入口放在一个没有被实际触发的路径中，Watch 表现为角度不变、周期计数
只增加一次或完全不增加。最终确认 ADC 必须真实启动，并将开环回调注册到注入序列完成
事件。修正后 `electrical_angle_rad` 与 `cycle_count` 连续变化。

### 8.3 算法运行但没有PWM波形

当时观察到：

```text
running        = 1
output_enabled = 0
fault_flags    = 8
```

这证明算法调度已正常，缺少波形的原因是功率级安全故障关闭了 MOE，而不是算法没有执行。
随后加入 PE14/PE15 的 20 kHz 快速诊断计数，确认出现过 nFAULT 低电平；后续一次测试还
捕获到 READY 与 nFAULT 同时低约 39 个周期，即约 1.95 ms。

### 8.4 增加预充电后仍有功率级异常

针对首次高桥导通可能缺少自举电荷的问题，加入低桥预充电和中性矢量稳定阶段。该改动
解决了启动顺序不可控的问题，但之后实机出现电源达到 24 V、4 A限流的危险现象。

### 8.5 4 A问题的根因：电压量纲错误

当时 Watch 中的 `d_voltage = 0.2` 被算法解释为母线标幺值：

```text
0.2 pu × 24 V = 4.8 V
```

而调试意图实际是 0.2 V。电压开环又绕过了电流 PI，没有软件电流限制，因此低阻电机在
低电角频率下迅速把实验电源推到 4 A限流。这个现象不是 `0.2 V` 本身产生了 4.8 V，
而是字段单位定义与调试者理解不一致。

最终修复是让开环结构显式使用 `_v` 后缀和物理伏特，并把 `dc_bus_voltage_v` 放入同一
结构中统一换算。与此同时加入显式 `output_request` 安全门，禁止上电自动打开功率级。

## 9. PA0实时执行时间测量

### 9.1 测量点定义

PA0 配置为高速推挽输出，时序定义为：

```c
void ADC1_2_IRQHandler(void)
{
  GPIOA->BSRR = GPIO_PIN_0;                  /* JEOS中断入口：置高 */
  HAL_ADC_IRQHandler(&hadc1);                /* 快照、FOC、调制、CCR提交 */
  GPIOA->BSRR = ((uint32_t)GPIO_PIN_0 << 16U); /* 完成：置低 */
}
```

因此 PA0 高电平窗口覆盖：

- HAL ADC 中断分发；
- JDR1/JDR2 快照；
- 开环电压斜坡；
- 电角度三角函数；
- 反 Park 变换；
- SVPWM/过调制计算；
- 三相 CCR及下一次 ADC 触发位置提交。

它不包含 ADC 模拟采样和16倍过采样本身的时间，因为 JEOS产生后CPU才进入中断；也不包含
JEOS置位到第一条 PA0寄存器写之间的少量异常入口延迟。

### 9.2 实测结果

逻辑分析仪截图给出的周期测量为：

| 项目 | 实测值 |
| --- | ---: |
| 周期 | 50.01 µs |
| 频率 | 19.996 kHz |
| 窄脉冲宽度 | 14.86 µs |
| 显示占空比 | 70.2859428% |

仪器截图同时显示约 70.29% 的高电平占空比和 14.86 µs 的窄低电平，因此按照固件中
“中断入口置高、结束置低”的极性，高电平执行窗口约为：

```text
50.01 µs × 70.2859% ≈ 35.15 µs
```

对应剩余空闲时间约为：

```text
50.01 µs - 35.15 µs ≈ 14.86 µs
```

如果逻辑分析仪通道设置了反相，两个数值的物理含义会互换；但无论按哪种显示极性，已经
确认 PA0在下一个50 µs周期到来前恢复，故可以确定 ADC完成后的FOC软件链路小于50 µs。
按照当前代码输出极性和截图占空比，较合理的执行时间估计是约35.15 µs，周期余量约
14.86 µs。

### 9.3 实时性结论边界

当前结论适用于本次 Keil 调试构建、当前算法参数和当前中断负载。它证明平均/当前观测路径
满足20 kHz周期，但还不是所有工况下的严格最坏执行时间证明。正式闭环接入后，电流读取、
Clarke/Park、两个PI以及其他高优先级中断可能改变执行时间，需要再次测量最大脉宽和抖动。

## 10. 实机验证证据

最终实机状态由用户确认：电机已经正常转动。调试观察同时表明：

- `running = 1`，开环算法持续运行；
- `electrical_angle_rad` 连续变化；
- `cycle_count` 持续增加；
- `applied_d_voltage_v = 0.2`；
- `applied_q_voltage_v = 0`；
- `voltage_ramp_complete = 1`；
- `power_stage_safe = 1`；
- `fault_flags = 0`；
- STSPIN32G4 `state = DRV_STATE_READY`；
- READY = 1，nFAULT = 1，STATUS快照为 `0x80`，仅LOCK位有效；
- PA0 周期约50.01 µs，频率约19.996 kHz，FOC在周期内结束。

提供的 Watch 截图中 `output_request/output_armed/output_enabled` 为0，是停止输出后的安全状态
快照；电机正常运行结论来自实际使能阶段的实机观察，不应把停止后的单帧 Watch 值误解为
“从未输出”。

## 11. 主要修改文件

| 文件 | 今日主要修改 |
| --- | --- |
| `Core/Src/main.c` | 开环测试模式、统一运行对象、ADC回调入口、显式输出请求、预充电状态、安全诊断和快速STATUS抓取 |
| `Core/Src/gpio.c` | PA0高速推挽时序探针初始化 |
| `Core/Src/stm32g4xx_it.c` | ADC1_2 IRQ入口/出口翻转PA0 |
| `FOC/Inc/foc_algorithm.h` | 定义物理伏特单位的 `foc_voltage_open_loop_t` 和开环算法接口 |
| `FOC/Src/foc_algorithm.c` | 电压斜坡、母线归一化、角度推进、反Park及调制调用 |
| `Modules/Three_Phase_Driver/Inc/tim1_pwm.h` | 增加启动预充电接口声明 |
| `Modules/Three_Phase_Driver/Src/tim1_pwm.c` | 实现低桥自举预充电及中性矢量切换 |
| `Modules/Driver_Communication/Src/drv_monitor.c` | TIM1周期回调分发兼容处理 |
| `Tests/Host/test_foc_modulation.c` | 增加开环电压算法基本数值与无效输入测试 |
| `MDK-ARM/servo_v1.uvprojx` | 核对并补齐工程源文件/包含目录，使新增模块进入目标构建 |

## 12. 构建验证

最终执行 Keil完整重建，日志结果为：

```text
Program Size: Code=47340 RO-data=1460 RW-data=172 ZI-data=2780
"servo_v1\servo_v1.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: 00:00:03
```

构建验证采用日志内容作为成功依据。Keil命令行进程可能在构建失败时仍返回0，因此不能只用
进程退出码判断。

主机测试脚本已随算法结构更新测试用例，但本机当前命令环境未找到可直接调用的 `cl.exe`
或 `zig.exe`，因此本轮最终证据是 Keil ARM目标全量构建和实机运行结果，不把未执行的主机
测试写成已通过。

## 13. 当前操作方法

### 13.1 安全上电

1. 实验电源先设置较低限流；首次复测建议断开电机，只观察栅极/PWM。
2. 上电或复位后确认 READY = 1、nFAULT = 1、`fault_flags = 0`。
3. 确认 `d_voltage_v = 0.2`、`dc_bus_voltage_v = 24`，不要再填入标幺值。
4. 算法会自动运行，但功率输出保持关闭。
5. 在 Watch 中把 `g_foc_voltage_test.output_request` 写为1。
6. 确认 `output_armed = 1`、`output_enabled = 1` 后观察PWM和电机。

### 13.2 正常关闭

在 Watch 中写：

```text
g_foc_voltage_test.output_request = 0
```

主循环会无条件关闭 MOE，清除 armed/enabled状态，并把斜坡实际值恢复到0，下一次使能时
重新从0 V开始爬升。

### 13.3 故障判断

| 观察值 | 含义 |
| --- | --- |
| `fault_flags & 8` | 功率级 READY/nFAULT异常 |
| `first_fault_status_raw & 0x01` | VCC欠压 |
| `first_fault_status_raw & 0x02` | 过温关断 |
| `first_fault_status_raw & 0x04` | VDS保护 |
| `first_fault_status_raw & 0x08` | 驱动器复位状态 |

发生故障后固件不会因引脚恢复而自动重新打开 MOE，必须先查明原因并重新复位/清故障。

## 14. 当前限制和后续工作

1. **开环模式没有电流闭环。** `current_limit`不会限制本次直接电压给定，实验电源限流仍是
   必要的外部保护。
2. `dc_bus_voltage_v`目前是固定配置24 V，不是ADC实时母线采样。母线变化会造成实际标幺
   比例误差。
3. 电角频率与机械转速的关系还没有结合极对数、编码器零位和方向正式管理。
4. 当前电机“能转”只验证电压开环方向和链路，不等于相序、编码器方向、电角度零偏已经
   满足闭环要求。
5. PA0测量应继续记录最大脉宽和周期抖动；正式电流环接入后必须重新测量。
6. 下一阶段建议先完成电流偏置标定和相电流比例核对，再恢复 `ServoManager`，逐步接入
   D/Q电流闭环，不能直接从当前电压开环跳到高电压运行。

## 15. 今日里程碑总结

今天已经完成从 ADC硬件节拍、开环D/Q电压、连续电角度、FOC调制、TIM1三相PWM、功率级
安全门到实机电机转动的完整电压开环链路。期间定位并修复了工程注册、调度入口、MOE故障
锁存、自举预充电、电压量纲以及上电自动输出等问题。

当前节点可以明确标记为：

> **FOC电压开环完成；电机正常转动；20 kHz周期下ADC完成至FOC输出的软件执行时间小于
> 50 µs。**

