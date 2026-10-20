# 编码器现状分析与测速设计记录

日期：2026-10-20（Asia/Shanghai）

用途：供后续 GPT/开发者回溯结论、代码依据、设计边界和未完成工作。

## 1. 请求、范围与基线

用户要求：分层梳理当前编码器实现，重点解释业务逻辑；评估下一步 M/T 测速；设计供 FOC 使用的流程和接口，保持高内聚、低耦合、独立、易理解和高效；生成详细回溯日志与简洁用户文档。

本文前半部分保留首次分析时的代码基线和设计推导；第 10 节记录随后完成的正式实现。当前代码已经实现持续采集、测速、快照、FOC 读取接口和时长统计，尚未烧录做实机性能验收，也未提交或推送。

- Git 根目录：`C:/Users/penghao/Desktop/Servo_9948`。
- 工程目录：该根下 `G431_base_code/software/servo_v1`。
- 当前分支：`codex/encoder-test`。
- HEAD：`6eb7f611745218eb65b23cc925184012c62771e0`，提交标题 `1019`。
- 工作区存在大量既有未提交修改；本文分析的是读取时的工作区，不是纯 HEAD 内容。不能只检出该 SHA 就认为完全恢复了本次分析基线。
- 文档沿用 `MDK-ARM/docs`，入口增加到工程 README。

关键源码读取时 SHA256：

| 文件（相对 servo_v1） | SHA256 |
|---|---|
| `Core/Src/main.c` | `E665825D0A86AC2A6F7BC5B603222B89017C2ABF4925ECE08949F1381A1F13FA` |
| `Modules/Encoder/Src/mt6835.c` | `498B2B1E8B59A99490DBB0B5E6B143335DFCAACFF809B72B884EB1B54A815647` |
| `Tests/Src/mt6835_verify.c` | `4F8027CCFE2C0659C7ABC11CB996FA44CA3C3BC4F7F2FCDFDF3D7BCFF03C3394` |

这些散列用于识别版本，不是源码备份。后续实施前重新检查工作区。

## 2. 当前分层及调用证据

以下路径相对 `software/servo_v1`；行号为本次检查快照，后续优先按函数名定位。

| 层次 | 代码定位 | 已实现内容 |
|---|---|---|
| L1 板级通信 | `Modules/Encoder/Src/spi1_encoder.c:53` 附近，`MX_SPI1_Encoder_Init` | GPIO、SPI 配置；CAL_EN 拉低；软件 CS |
| L1 传输接口 | 同文件 `MT6835_SPI_TransmitReceive` | 同步 HAL SPI 收发；保留软件 SPI 编译选项 |
| L2 芯片驱动 | `Modules/Encoder/Src/mt6835.c:113` | Burst 读取、原始码解析、CRC 计算 |
| 测试业务 | `Tests/Src/mt6835_verify.c:34/58/107` | Start、单帧检查、验证状态推进 |
| 初始化装配 | `Core/Src/main.c:1082` 附近 | SPI 初始化、上电等待、启动验证 |
| 后台调度 | `Core/Src/main.c:1223` | 每轮调用 `Mt6835Verify_Task`，忽略返回值 |
| FOC 适配 | `Core/Src/main.c` 的 `MainFoc_ReadFeedback` | 读取电流和模式对象中的角度/给定 |
| 实时宿主 | `Core/Src/main.c:619` 的 `MainFoc_OnAdcConversion` | 按控制模式调度算法 |
| ADC 回调 | `Modules/Current_Sampling/Src/current_sampling.c:59` | 更新电流样本；在非校准采集/稳定阶段调用注册回调 |
| FOC 端口 | `FOC/Inc/foc_interface.h`、`FOC/Src/foc_interface.c` | 无 HAL 头依赖，通过 `read_feedback` 获取输入 |
| 故障管理 | `Servo/Src/servo_manager.c` | 将 FOC 角度无效映射为伺服故障 |

### 2.1 板级通信

`MT6835_USE_SOFTWARE_SPI=0`，当前编译配置选择硬件 SPI1。PA4 为 CS，PA5 为 SCK，PA6 为 MISO，PA7 为 MOSI；PC4 为 CAL_EN。SPI 模式 3、8 位数据、MSB first，168 MHz / 16 = 10.5 MHz。

`HAL_SPI_MspInit` 位于 `Core/Src/stm32g4xx_hal_msp.c:118`，负责 SPI1 时钟和 AF5 引脚配置。

GPIO 模拟 SPI 是针对历史 PA6/PA7 交叉接线的备选路径；其 `timeout` 被忽略。不能把此历史描述直接当成当前实板接线已确认。

### 2.2 芯片驱动

`MT6835_ReadAngle` 一次拉低 CS，发送 6 字节：`A0 03 00 00 00 00`；在 `rx[2..5]` 获取 0x003～0x006 内容，再释放 CS。

```text
data24    = (reg003 << 16) | (reg004 << 8) | reg005
angle_raw = (data24 >> 3) & 0x1FFFFF
status    = reg005 & 7
crc_ok    = CalcCrc24(data24) == reg006
```

CRC 算法覆盖 24 位角度与状态，采用多项式 0x07、代码初值 0x00。`HAL_OK` 只表示事务成功；CRC 是否通过必须另看 `crc_ok`，报警必须另看 `status`。

真实实现是 48 个 SCK，纯线传输时间约 `48/10.5 MHz = 4.57 μs`。这不是函数总耗时，GPIO/HAL/CRC/中断抢占都需另计。20 kHz 下仅线传输约占周期 9.14%，不能据此宣称完整链路已满足实时预算。

### 2.3 业务逻辑实际处在测试层

当前没有通用的 `encoder.c` 或速度估算器；逻辑由验证模块承担。

`Mt6835Verify_Start()`：

- 设置 RUNNING，清错误数、样本数、角度范围、状态和 CRC 观测字段。
- 清预热次数及重试期限；不清 `last_frame`，因此重启后首帧前它可能仍是上次结果。

`Verify_SampleAngle()`：

1. 传输失败：`comm_errors++`，返回。
2. 传输成功：保存 `last_frame/status_raw/crc_ok`，包括坏 CRC 帧。
3. 前 8 次成功传输：作为预热，不统计 CRC、角度，也不检查报警。
4. 之后 CRC 失败计数；CRC 成功则累加 `samples`、更新 min/max。
5. 只对 CRC 成功的帧检查芯片报警；有报警置 FAILED。

`Mt6835Verify_Task()`：

- RUNNING 时调用上述单帧流程。
- 通信错误累计 ≥8：FAILED/TIMEOUT，设置 100 ms 后重试。
- CRC 错误累计 ≥4：FAILED/CRC，设置 100 ms 后重试。
- 成功样本 ≥256：PASSED。后续直接返回 HAL_OK，不再采集。
- ALARM 分支提前返回，没有设置相同的 100 ms 重试期限；下一轮通常立即重启。
- 错误阈值是本轮累计值，不是连续错误次数；重试调用 Start 会清本轮统计。

因此测试对象既不是持续反馈源，也不是永久诊断日志。`angle_max-angle_min` 跨圈时可能接近整圈，不能用来判断真实累计位移或速度。

### 2.4 与 FOC 的关系

`MainFoc_ReadFeedback()` 的 CURRENT_OPEN_LOOP 分支写固定 `electrical_angle_rad=0`，用于电流环投运。

NORMAL 分支从 `g_global_mode_control.normal_loop` 取电角度及有效标志。初始 `angle_valid=0`，没有发现编码器采集向此对象发布反馈的实现。

`foc_input_t` 只有两相电流、电角度、D/Q 给定以及三类有效标志，没有速度。`FOC_ControllerStep` 在输入角度无效时锁存 `FOC_FAULT_ANGLE_INVALID` 并关断；Servo Manager 映射并管理该故障。

当前 `MainServo_IsCalibrationDone` 只看电流零偏校准。未来接真实角度时，必须另有方向/电角偏置有效性，不能把电流校准完成作为转子标定完成。

## 3. 已发现的问题与边界

本次只记录，不修改。

| 项目 | 证据/影响 | 后续动作 |
|---|---|---|
| 验证通过即停止 | PASSED 分支不再读角度 | 新增持续采集服务，测试改为消费结果 |
| “非阻塞”注释不准确 | 内部 HAL 同步收发，超时参数 10 ms | 区分分步状态机与异步 I/O；正式 ISR 不直接复用 |
| 超时不适合快环 | HAL 等待依赖 tick；高优先级 ISR 中 tick 可能不能推进 | 用硬件计时的有界路径或异步完成/超时状态机 |
| 报警重试不一致 | ALARM 未设置重试期限 | 统一策略，保留首故障及累计统计 |
| CRC 不等于设备存活证明 | 当前 CRC 初值 0，data24=0 时 CRC=0；全零帧可通过 | 结合硬件、电气诊断和合理性；不要简单禁止合法零角度 |
| 注释陈旧 | 头文件称 4×24 bit，测试注释又称 32 bit；实现 48 bit | 以实现和手册 7.6.9 为准 |
| CRC 描述不严谨 | 测试头注释提“24 bit 的 CRC 冗余”，实际 CRC 是 8 位 | 修正文档，不声称绝对排除恒电平 |
| SPI 注释算术错误 | `spi1_encoder.h` 称 24 不是 8 的整数倍 | 软件片选理由应是控制完整事务和锁存，而非该算术说法 |
| 状态读错误折叠 | `MT6835_ReadStatus` 将底层失败统一返回 HAL_ERROR | 如果需要精确诊断，保留原始错误码 |
| 无时间戳/新鲜度 | `mt6835_angle_t` 没有采样时刻 | 在采集适配边界记录 CS 锁存时刻 |
| 无运行期间失效检测 | 主循环忽略测试返回，验证通过后不复测 | 业务层持续提供状态，控制策略消费状态 |

“全零可通过”是由当前 CRC 代码直接推导，不是本次实板断线实验结论。静止角度不变化本身也不能证明传感器失效。

## 4. 硬件资料核对

本地资料：

- `hardware/芯片资料/MT6835.pdf`：42 页，Rev.1.3，2022-12；核对第 25、26 页。
- `hardware/舵机原理图/主控板.pdf`：3 页，图纸日期 2026-07-15；提取全文，渲染并查看第 2、3 页。

手册第 25 页明确 21 位角度、3 位状态和 CRC 多项式；第 26 页明确 burst 命令为 1010、起始地址 0x003、CS 下降沿锁存。该处未明确 CRC 初值；不能仅按注释宣称初值已由手册确认。

原理图第 2 页 U3 的 A/B/Z 引脚 16/15/14 未接出；第 3 页 A_OUTPUT/B_OUTPUT/Z_OUTPUT 驱动独立晶体管输出级，不是 U3 AB 输入链路。因此经典硬件 M/T 不是当前图纸下的软件即插即用方案。图纸存在 SPI 信号叠加标签，且代码保留过交叉接线方案；当前实板是否改线仍须实物确认。

外部一手参考：

- [MagnTek MT6835 官方手册](https://www.magntek.com.cn/upload/pdf/202407/MT6835_Rev.1.3.pdf)：在线抓取超时，本次具体协议以工程内本地手册核对结果为依据。
- [ST AN4013 定时器概览](https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf)：在线可读取；介绍输入捕获与编码器接口。它支持“硬件计数/捕获”的选型思路，不代表本工程引脚路由或 M/T 软件已经实现。

## 5. M、T、M/T 与 SPI 角度测速的选择

以下为工程设计推导。设有效计数分辨率为 C counts/rev，测量窗口为 Δt，时间基准频率为 f_timer：

```text
M：   rpm = 60 × Δcount / (C × Δt)
T：   rpm = sign × 60 × f_timer / (C_edge × Δticks)
M/T： rpm = 60 × M × f_timer / (C × Δticks)
```

T 法的 C_edge 必须与捕获边沿一致：例如只捕获 A 上升沿时不能直接套用四倍频计数分辨率。M/T 的 M 与 Δticks 必须来自同一对边界事件；不能把任意时间的计数与另一时间的时间戳拼接。反向时需要可靠方向信息。

M 法高速相对量化误差小；T 法低速计时间隔长但新结果到达慢；经典 M/T 同时量计数和对应时间，可覆盖较宽速度范围，但需要计数器、边沿捕获、溢出处理及同步。低速无边沿和停止仍需单独处理，不能永久维持旧速度。高速应避免每个边沿都触发软件 ISR。

SPI 方案每次得到离散角度，不知道两帧之间每个角度计数变化的准确时刻。先做带时间戳的差分，是当前最小且可验证的方案。自适应窗口借鉴计数/测时思路，但应命名为 SPI 窗口速度估算，不能声称等价于硬件 M/T。

初版建议固定短窗口和一阶低通，保留窗口参数；只有实测低速精度不足时再引入自适应窗口，避免一开始增加不必要状态。

### 5.1 跨圈、单位与时间

设 N=2^21，先在整数域计算相邻有效样本差：

```c
int32_t delta = (int32_t)raw_now - (int32_t)raw_prev;
if (delta >  N / 2) delta -= N;
if (delta < -N / 2) delta += N;
/* 恰好半圈无法唯一判断方向，应按异常/不确定处理。 */
```

成立条件：相邻有效样本之间的真实转动严格小于半圈。若最高速度 n_max rpm、间隔 dt 秒，要求 `n_max*dt/60 < 0.5`，并留余量。不能靠取模恢复掉帧期间丢失的整圈。

```text
Δt = (uint32_t)(tick_now - tick_prev) / tick_hz
ω_m = direction × Δcount × (2π/N) / Δt     [rad/s]
rpm = ω_m × 60/(2π)                        [仅显示换算]
```

使用硬件单调时基；32 位无符号减法能处理一次回绕，前提是实际间隔小于完整回绕周期，同时仍满足最大有效间隔约束。拒绝 dt=0、过大间隔、异常跳变及非有限计算结果。

量化示例：1 ms 窗口、21 位分辨率，一计数对应约 0.02861 rpm；这仅是数字量化步长，实际磁场噪声、传感器延迟和时间抖动可能更大。采样时钟分辨率和角度精度应分别评估。

若累计多圈位置使用 int64，先做整数差再转 float，避免运行很久后大整数转 float 丢低位。关机后的多圈信息不能由单圈绝对编码器自动恢复。

### 5.2 窗口与滤波

- 每帧完成跨圈展开；测速用窗口内位移和真实时间差，不能直接对窗口首尾单圈角度取一次最短差。
- 固定长度环形缓冲可 O(1) 更新；每帧不用扫描完整历史。
- 自适应方案可累计到最小位移或最大等待时间，但设最小时间、最大窗口和反转重置。不能用路径长度代替带符号净位移。
- 检测到稳定反向时重建窗口，避免正反位移抵消造成长时间伪零速；方向阈值需由静止噪声确定。
- 一阶低通可用 `y += alpha*(x-y)`，变步长时 `alpha=dt/(tau+dt)`；固定更新周期才预计算固定 alpha。
- 明确窗口引入的平均延迟及低通附加相位滞后，匹配未来速度环带宽。
- 只有通信持续有效、位移在噪声门限内且满足静止观察时间时，才可判近零速；失联不等于零速。
- 角度通道不复用测速低通输出，以免增加电流环相位误差。

## 6. 推荐模块边界与接口草案

以下为首次分析阶段的接口草案；实际落地接口以第 10 节和 `encoder.h` 为准。

| 模块 | 唯一职责 | 依赖 |
|---|---|---|
| `spi1_encoder` | 板级通信、硬件时间采集/事务执行 | HAL/寄存器 |
| `mt6835` | 芯片协议和校验 | 板级收发 |
| `encoder_acquisition` | 周期触发、事务状态、把驱动帧转为通用样本 | 芯片驱动、时基 |
| `encoder` | 有效性、方向、展开位置、发布结果 | 通用样本、速度估算 |
| `encoder_speed` | 给定位置与时间，估算机械速度 | 标准整数/浮点类型 |
| `rotor_feedback` | 极对数、电角偏置、时间对齐、FOC 反馈映射 | 编码器快照、标定参数 |

`encoder` 和 `encoder_speed` 不包含 HAL/MT6835/FOC 头文件；采集适配把 `HAL_StatusTypeDef`、芯片状态映射成通用质量标志。方向只变换一次，电角偏置在 rotor_feedback 维护，避免下层混入电机参数。

```c
/* 草案，非现有可编译 API。config/context 类型在实现阶段定义。 */
typedef struct {
    uint32_t raw_count;          /* 0 .. counts_per_rev-1 */
    uint32_t sample_tick;        /* 对应 CS 锁存时刻 */
    uint32_t sequence;
    uint32_t quality_flags;      /* COMM/CRC/ALARM 等通用标志 */
} encoder_sample_t;

typedef struct {
    float mechanical_angle_rad; /* [0, 2π)，已统一方向 */
    float mechanical_speed_rad_s;
    uint32_t angle_tick;         /* 最近接受的角度时刻 */
    uint32_t speed_tick;         /* 最近速度估计的发布/窗口结束时刻 */
    uint32_t speed_window_ticks; /* 表示速度值的测量区间长度 */
    uint32_t sample_sequence;    /* 最近接受角度的序号 */
    uint32_t fault_flags;        /* 当前异常；历史统计另存 */
    uint8_t angle_valid;
    uint8_t speed_valid;
} encoder_snapshot_t;

bool Encoder_Init(encoder_t *ctx, const encoder_config_t *cfg);
void Encoder_Reset(encoder_t *ctx);
void Encoder_Update(encoder_t *ctx, const encoder_sample_t *sample);
bool Encoder_ReadSnapshot(const encoder_t *ctx, uint32_t now_tick,
                          encoder_snapshot_t *out);
bool RotorFeedback_Read(const rotor_feedback_t *ctx, uint32_t target_tick,
                        rotor_feedback_t_out *out);
```

`Encoder_Init` 校验分辨率、方向 ±1、时基频率、最大速度、时间窗口和有效期。初始化/重置只在生产者停用或明确同步后执行。`Encoder_Update` 必须同时接受失败样本通知，以撤销当前有效性；不得只在成功时更新而掩盖错误。

`Encoder_ReadSnapshot` 返回值表示“是否成功读取一致快照”，不代替各字段有效标志。读取时根据 now_tick 重新判断过期，保证采集任务完全停住时也能发现失效。速度有独立有效期；诊断计数包括累计错误、连续错误、最近成功时间和首故障，避免重试清零全部证据。

业务状态建议：INIT → PRIMING → TRACKING；错误/超时进入 INVALID，恢复到 PRIMING，达到有效样本与窗口要求再 TRACKING。机械角可先有效，速度必须至少有两个时间不同的有效样本并满足窗口条件。

坏帧后不更新原始角度历史。短间隔恢复可用真实时间差计算；超过消歧间隔则重新锚定，撤销多圈连续性与速度有效性。首版默认出现当前数据错误就撤销当前反馈有效性；如果未来允许短时保持或预测，应另设受限模式并测试，不能悄悄把旧值标新。

## 7. 调度、并发与性能

### 7.1 正式采集路径

```text
确定时基触发
  → 检查上次事务已结束
  → 记录 CS 下降沿附近硬件时间戳，发起 SPI
  → 收发完成，释放 CS，解析和校验
  → Encoder_Update（有界计算）
  → 发布完整快照

ADC 注入完成 → MainFoc_OnAdcConversion → FOC_ControllerStep
  → read_feedback → RotorFeedback_Read → 快照复制与新鲜度检查
```

默认考虑异步 SPI/DMA，初始可从 20 kHz 同步角度采集、1 kHz 速度输出的预算方案评估，**这些是设计起点，不是已验证配置**。DMA 只减少 CPU 等待，不减少 48 个 SCK，也有启动/完成中断成本。短事务是否值得 DMA 要测量后决定；可对比有界 LL 轮询，不能直接使用 10 ms HAL 超时路径。

安排 SPI 触发提前于 ADC 反馈消费，或者明确消费上一完整样本；记录实际数据年龄，分析一周期延迟。传感器内部延迟与 CS 时间戳不是同一概念，需要依据配置/实测补偿。

如果 SPI 尚忙，记录 missed sample，不重入同一 CS 事务。验证模块与正式服务不能同时独立读 SPI；可启动自检后移交唯一所有权，或让验证逻辑消费正式采集结果。

### 7.2 快照一致性

不能认为加 `volatile` 就能原子发布整个结构体。首版建议单生产者，采用极短临界区完成快照发布/复制，保存并恢复原中断屏蔽状态；SPI、CRC、窗口运算必须在临界区外。测量临界区耗时对 ADC 的影响。

若用双缓冲，需要保证读取期间生产者不会再次覆盖正在读的槽，并使用正确的内存顺序。不要在高优先级 FOC ISR 自旋等待被其抢占的低优先级写者完成序列锁，否则可能无法退出。

### 7.3 时间对齐与 FOC 适配

当前端口 `foc_port_t.read_feedback` 可保持不变，只改正常模式的应用适配。`rotor_feedback` 的极对数、偏置和方向约定应在输出关闭时标定/加载；配置缺失时 angle_valid=0。

若 MT6835 安装在电机转子轴，可由机械角直接换算电角；若在减速输出轴，必须先评估齿轮比、回差和可观测性，不能默认为它适合换相反馈。本次未确认安装位置。

```text
theta_e = wrap_0_2pi(pole_pairs * theta_m + electrical_offset_rad)
omega_e = pole_pairs * omega_m
```

先保证角度新鲜且延迟满足预算，再考虑有限时域预测：
`theta_m(target)=theta_m(sample)+omega_m*(target-sample)`。
速度必须有效、预测时长受限，不能跨长时间故障预测。初版可关闭预测；速度尚未建立时，足够新鲜且已标定的角度仍可独立有效。

现有读反馈回调没有时间参数，可由适配 context 提供本周期 ADC 时刻，或在入口读取同一硬件时基。Park 使用与电流采样对齐的角度；未来 PWM 施加延迟补偿应明确另一个目标时刻，不能随意混为一谈。

速度环尚未实现。未来速度控制器独立以机械 rad/s 为反馈，输出限幅后的 Iq*；当前电流环不因单纯显示速度而强制增加输入字段。速度失效时速度环禁止积分和输出陈旧给定，由宿主选择停机策略。

### 7.4 测量指标

记录 SPI 实际 SCK、CS 到完成延迟、采样间隔抖动、CRC 耗时、估算器耗时、快照临界区耗时、ADC ISR 总耗时、最大数据年龄和 missed samples。报告最大值和分布，不仅是平均值。

采用 float、预计算角度比例和固定系数、固定容量缓冲、O(1) 更新；不动态分配内存，不在 ISR 打印、发送日志或执行无界循环。CRC 表优化等仅在 profile 证明需要时引入。

## 8. 分阶段实现与验收

| 阶段 | 交付 | 验收要点 |
|---|---|---|
| A 持续采集 | 唯一采集者、时间戳、质量状态、快照 | PASSED 后仍更新；断线立即可观测 |
| B 角度业务 | 方向、跨圈、重锚定、新鲜度 | 正反向跨零无整圈速度尖峰 |
| C 测速 | 独立纯算法、窗口、滤波 | 恒速量纲正确、停转/反转响应可量化 |
| D FOC 适配 | 极对数/偏置、正常模式接入 | 标定未完成禁止有效电角反馈 |
| E 速度环 | 独立速度 PI、Iq 限幅、调度 | 在电流环和角度链路验收后实施 |

纯算法主机测试至少覆盖：

1. 首帧、重复时间戳、重复样本序号；速度不应提前有效。
2. 正反匀速、多次跨圈；正向 `N-10 → 5` 应为 +15，反向应为 -15。
3. 恰好半圈、超最高速度跳变、长时掉帧：报告不确定并重新锚定。
4. 时间戳回绕，例如 `0xFFFFFFF0 → 0x10` 得到 32 ticks。
5. 错 CRC、传输失败、有效 CRC 报警；无效数据不得进入估算。
6. 静止噪声、低速、急停、反向；区分近零、尚未更新和失效。
7. 窗口长度变化、超时、滤波复位；不存在除零、NaN 或无限循环。
8. 大累计位置后微小增量；保持整数差分精度。
9. 生产者被中断、快照过期；读取不得混帧或自旋死锁。
10. FOC 正常模式使用标定后反馈，原固定角电流测试仍按原路径工作。

台架检查：输出先保持关闭做手转/静止采集；核实正方向和安装轴；与可信转速参考比对；覆盖最低/最高目标速度及反转；完成电角标定后才做低能量换相试验。没有实测前不填写“已通过”。

## 9. 待确认信息与后续恢复入口

尚需实际值：编码器安装轴、最高/最低转速、期望速度环带宽、极对数、实板 SPI 接线、硬件计时资源、DMA 分配与中断优先级、静止噪声、允许反馈年龄、是否存在 AB 飞线。

无需为本文档分析阻塞；这些值会影响下一步实现的参数和外设分配，不应伪造默认电机参数。

后续 GPT 先读第 10 节确认当前实现，再结合第 3、5～8 节查看设计理由，并按函数名重查源码。旧的验证模块已经删除，不要恢复“采满后停止”的行为。

工程 README 还引用了 `servo_closed_loop_implementation_plan.md`，本次检查发现目标不存在；未将其当作已确认设计依据，也未在本任务中补写该独立文档。

本次验证方式：代码调用链静态核对、CRC 全零性质推导、手册文本核对、原理图视觉核对、文档链接及差异检查。PDF 解析首次遇到输出编码错误，设置 UTF-8 后成功；PyMuPDF 未安装，改用已有 Poppler 渲染，无需安装新包。临时图片在 `tmp/pdfs/encoder_review`，不属于固件交付。

## 10. 正式实现记录

### 10.1 已完成内容

- 新增 `Modules/Encoder/Inc/encoder.h` 和 `Src/encoder.c`。
- 删除 `Tests/Inc/mt6835_verify.h`、`Tests/Src/mt6835_verify.c`，并从 Keil 工程移除。
- `mt6835.c` 增加无状态的 burst 命令构造和 DMA 响应解码，阻塞读接口复用同一解码逻辑。
- `spi1_encoder.c` 增加 SPI1 DMA 异步事务；DMA1 Channel 1 为 RX、Channel 2 为 TX。
- TIM1 周期边界每 50 μs 启动一次 DMA；DMA IRQ 优先级 2，低于 ADC/FOC 的优先级 1。
- DMA 完成后校验 CRC 和芯片报警，处理单圈跨越，维护 `int64_t` 多圈位置。
- 20 点实际时间窗口计算机械 rad/s，并用 10 ms 时间常数的一阶低通处理。
- 发布机械角、速度、电角度、时间、序号、质量和独立有效标志组成的快照。
- `MainFoc_ReadFeedback()` 正常模式调用 `Encoder_ReadElectricalAngleForFoc()`；只读内存，不发起 SPI。
- 未调用 `Encoder_ConfigureMotor()` 前电角度保持无效，防止未知方向、极对数和零偏进入 Park 变换。
- 主循环调用 `Encoder_Service()`，只处理超过 100 μs 的异常事务恢复。

### 10.2 调度关系

```text
TIM1 update ISR (priority 1)
  Tim1Pwm_DispatchPeriodElapsed()
  Encoder_OnPwmPeriodIsr() -> SPI DMA start

DMA RX completion ISR (priority 2)
  release CS -> decode -> CRC/status -> position/speed -> publish snapshot

ADC JEOS ISR (priority 1)
  current snapshot -> FOC_ControllerStep()
  -> MainFoc_ReadFeedback()
  -> Encoder_ReadElectricalAngleForFoc()
```

DMA 完成处理不能抢占 ADC ISR。同优先级的 TIM1 update 与 ADC ISR 也不会互相抢占；若事件重合，后到者等待先到者结束。快照读取始终使用上一份完整发布结果。

### 10.3 性能证据接口

`encoder_diagnostics_t` 记录：事务总周期、DMA 启动 CPU 周期、完成处理周期、FOC 快照读取周期、样本年龄、完整 ADC/FOC ISR 周期、忙冲突、错误和 50 μs 截止时间超限次数。

DWT CYCCNT 在 `Encoder_Init()` 启用。168 MHz 下：

```text
微秒 = cycles / 168
FOC 周期预算 = 8400 cycles = 50 μs
SPI 理论线传输 = 768 cycles ≈ 4.57 μs
```

`ADC1_2_IRQHandler` 从入口到 HAL 回调结束进行整体测量并调用 `Encoder_RecordFocIsrCycles()`。PA0 测试脉冲仍保留，可用示波器交叉验证。

### 10.4 编译证据与未完成项

2026-10-20 使用 Keil ARM Compiler 6.24 构建 `servo_v1.uvprojx`：0 error、0 warning。链接 map 已确认 `Encoder_Init`、`Encoder_OnPwmPeriodIsr`、`Encoder_ReadElectricalAngleForFoc`、DMA1 Channel 1/2 IRQ 和 SPI DMA 接口进入固件。

尚未完成：目标板烧录、实际方向/极对数/电角零位配置、静止噪声测量、速度参考对比、通信时长实测和 FOC 预算实测。没有这些数据前不能宣称硬件闭环已经验收。速度环本身也未实现；当前只提供速度反馈接口。
