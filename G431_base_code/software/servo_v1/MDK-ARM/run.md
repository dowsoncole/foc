# 上电运行逻辑说明

本文说明当前 `software/servo_v1` 固件从复位到进入主循环后的实际执行路径，以及三相功率级、电流采样和 FOC 的协作关系。说明以 `Core/Src/main.c`、`Modules/Three_Phase_Driver/`、`Modules/Current_Sampling/`、`Modules/Driver_Communication/` 和 `FOC/` 中的当前代码为准。

## 1. 先给结论

当前程序上电后**不是直接进入带位置反馈的完整伺服运行**，而是：

1. 初始化 STM32、时钟、GPIO、I2C3、TIM6 和 TIM1。
2. 通过 I2C 初始化 STSPIN32G4，清除上电锁存故障，并确认 `READY=1`、`nFAULT=1`。
3. 保持 TIM1 的 MOE 清零，功率级栅极输出始终关闭；不再执行单相 MOS 验证波形阶段。
4. 启动两个片内运放和 ADC1 注入采样链，ADC 每个 20 kHz PWM 周期采集 U、V 两相电流。
5. 请求零点校准。校准完成后，主循环把 FOC 控制器置为运行状态。
6. FOC 在 ADC 注入序列完成中断中运行，每次读取 U/V 电流并重新计算三相电压，写入 TIM1 的三个比较寄存器，但 MOE 关闭时这些比较值不会输出到栅极。

因此，当前上电阶段没有任何栅极驱动波形。校准完成后 FOC 仍会计算并更新 PWM 比较值，但由于 MOE 始终为 0，功率级不会实际开关。当前代码还没有编码器输入，电气角度固定为 0，`d_reference` 和 `q_reference` 也固定为 0，因此这不是完整的位置伺服控制。

## 2. 上电时序

### 2.1 MCU 和定时器初始化

`main()` 依次执行：

- `HAL_Init()`；
- `SystemClock_Config()`：HSE 8 MHz，经 PLL 得到 168 MHz 系统时钟；
- `MX_GPIO_Init()`；
- `MX_I2C3_Init()`：用于访问 STSPIN32G4 寄存器；
- `MX_TIM6_Init()`：10 kHz 计数、1 s 更新，用作驱动监控心跳；
- `MX_TIM1_Init()`：三相互补 PWM 和 ADC 触发。

TIM1 的关键配置如下：

| 项目 | 当前值 | 作用 |
| --- | ---: | --- |
| PWM 频率 | 20 kHz | 三相功率 PWM 和电流环周期 |
| 计数方式 | 中央对齐 | 降低开关谐波并保持采样对称 |
| 死区 | 500 ns | 防止同一半桥上下管直通 |
| CH1/2/3 | U/V/W 三相互补输出 | 连接 STSPIN32G4 的三相栅极输入 |
| CH4 | ADC 外部触发 | 在更新事件前约 5.714 us 触发注入采样 |
| MOE | 初始化时清零 | 初始化阶段不会把栅极波形送入功率级 |

TIM1 计数器、比较通道和 CC4 触发会先运行，但 MOE 未置位前，六路栅极输出仍被屏蔽。三相比较值初始为 50% 中性点，便于维持自举电容，同时不向电机施加净电压。

### 2.2 STSPIN32G4 驱动器检查

`DrvMonitor_Init()` 最多尝试 3 次调用 `STSPIN32G4_Init()`。每次初始化会：

1. 通过 I2C 写 `CLEAR` 清除上电锁存故障；
2. 读取 `STATUS`，检查 VDS 保护、过温和 VCC 欠压；
3. 等待 `READY` 为高且 `nFAULT` 为高；
4. 成功后把状态设为 `DRV_STATE_READY`，并启动 TIM6 的 1 s 心跳。

如果初始化最终失败，`Error_Handler()` 关闭中断并永久停机。驱动器状态只用于监控和 FOC 安全门，不会触发 MOE 使能。

## 3. 当前程序对功率级的处理

### 3.1 MOE 始终关闭

`MX_TIM1_Init()` 完成三相 PWM、互补输出和 ADC 触发配置后，TIM1 的 MOE 保持清零。`main()` 随后再次调用 `Tim1Pwm_EmergencyStop()` 作显式保险。

本版本没有调用 `Tim1Pwm_EnableOutputs()`，也移除了 `Tim1PwmVerify_Start()`、`Tim1PwmVerify_Task()` 等验证波形流程。因此：

- TIM1 计数器和比较寄存器可以继续运行；
- ADC 触发和 FOC 计算可以继续运行；
- 六路栅极信号不会被 MOE 放行到 STSPIN32G4；
- 电机不会因本程序上电而获得驱动电压或产生转矩。

故障路径中的 `Tim1Pwm_EmergencyStop()` 仍可调用，但它只会进一步确认 MOE 为 0。

### 3.2 FOC 输出

电流采样校准完成后，主循环调用 `FOC_ControllerStart()`。此后每次 ADC 注入序列完成：

1. `HAL_ADCEx_InjectedConvCpltCallback()` 读取 `ADC1->JDR1/JDR2`；
2. JDR1 作为 U 相电流，JDR2 作为 V 相电流；
3. 回调 `MainFoc_OnAdcConversion()`，进入 `FOC_ControllerStep()`；
4. 先用 `DrvMonitor_IsPowerStageSafeFast()` 快速检查 READY/nFAULT 引脚；
5. 读取电流反馈，执行 d/q PI 和电压限幅；
6. `MainFoc_WriteVoltage()` 调用 `Tim1Pwm_SetPhaseVoltage(alpha, beta)`，更新三相 PWM 比较值。

FOC 参数当前为：采样/电流环周期 20 kHz，电压限幅 0.95，电流限幅 10 A，d/q PI 的 `Kp=0.1`、`Ki=5.0`。

当前 `MainFoc_ReadFeedback()` 中：

- 电气角度固定为 `0.0 rad`；
- `d_reference=0`；
- `q_reference=0`；
- 没有编码器或位置环输入。

所以现阶段 FOC 只具备“根据两相电流反馈调节电压”的框架，不能据此认为电机已经实现了闭环定位或正常转速控制。

### 3.3 PWM 比较值与实际输出

FOC 运行期间，`Tim1Pwm_SetPhaseVoltage()` 仍会更新 CH1/CH2/CH3 比较寄存器。这些寄存器值可用于调试算法输出，但由于 MOE 始终清零，不会转换为实际的栅极波形。

## 4. 电流采样和零点校准

初始化顺序是 `MX_OPAMP1_Init()`、`MX_OPAMP3_Init()`、`MX_ADC1_Init(0)`，然后 `CurrentSampling_Start()`：

1. 启动 OPAMP1、OPAMP3；
2. 启动 ADC1 注入序列中断；
3. 由 TIM1_CH4 硬件触发，不需要在 TIM1 CC 中断中软件启动 ADC；
4. `CurrentSampling_RequestCalibration()` 先等待约 10 ms，再在约 1 ms 窗口内采样并计算 U/V 偏置。

ADC 原始码换算为电流时使用当前配置：7 mΩ 分流电阻、运放增益 6.1、3.3 V 参考、16 倍过采样。W 相电流不直接采样，由 `i_w = -(i_u + i_v)` 的三相平衡关系推导。

当前零点校准在 MOE 关闭的功率级安全状态下进行，不会因为上电流程产生额外的电机电流。

## 5. 主循环和故障关断

主循环重复执行：

1. `CurrentSampling_Service()`：推进校准状态机；
2. 校准完成且 FOC 尚未运行时，调用 `FOC_ControllerStart()`；
3. `DrvMonitor_Task()`：TIM6 每 1 s 置位一次标志，主循环随后通过 I2C 轮询驱动器状态；
4. 若 FOC 运行期间驱动快速安全门报告异常，FOC 停止、复位 PI，并调用 `Tim1Pwm_EmergencyStop()`；MOE 不会被重新使能。

FOC 还有一条更快的保护路径：在每个 ADC 完成中断内先检查 READY/nFAULT；若不安全，`FOC_ControllerStep()` 会停止 FOC、复位 PI 积分器并调用 `Tim1Pwm_EmergencyStop()`。

因此：

- 驱动芯片故障时，硬件保护会先拉低栅极；
- 软件随后清除 MOE，避免故障消失后输出未经确认自动恢复；
- 恢复输出需要重新执行显式启动，不会自动重启。

## 6. 关键引脚和模块对应关系

| 功能 | MCU/模块 | 说明 |
| --- | --- | --- |
| U/V/W 高侧 | TIM1_CH1/2/3 | 三相高侧 PWM |
| U/V/W 低侧 | TIM1_CH1N/2N/3N | 互补 PWM，含 500 ns 死区 |
| U 相采样 | PA2 / ADC1_IN3 / OPAMP1 | 注入 rank 1 |
| V 相采样 | PB1 / ADC1_IN12 / OPAMP3 | 注入 rank 2 |
| 驱动器通信 | I2C3 | STSPIN32G4 寄存器访问，仅在主循环使用 |
| 驱动器状态 | READY、nFAULT GPIO | 快速安全门和 1 s 状态监控 |
| 监控心跳 | TIM6 | 1 s 触发一次后台状态轮询 |

## 7. 当前实现边界

- 没有编码器读取和电气角度估算；
- 没有位置环、速度环和通信协议控制给定；
- FOC 的 d/q 给定固定为 0；
- FOC 目前只计算 PWM 比较值，因 MOE 固定关闭而不会驱动功率级；
- 若以后允许输出，必须增加明确的功率级使能状态机和独立的安全授权，不能仅恢复单个 `Tim1Pwm_EnableOutputs()` 调用。

## 8. 建议的示波器/调试观察点

上电调试时可按以下顺序确认：

1. `TIM1->BDTR.MOE`：整个运行期间保持 0；
2. TIM1_CH1/CH1N、CH2/CH2N、CH3/CH3N：应无有效栅极驱动输出；
3. ADC1 JDR1/JDR2：每个 PWM 周期更新一次；
4. `g_current_sampling.calibration_state`：`SETTLING → COLLECTING → DONE`；
5. `g_foc_controller.running`：校准完成后置 1；
6. `g_drv_monitor_tick` 和 `s_mon.state`：每秒检查一次驱动状态。

