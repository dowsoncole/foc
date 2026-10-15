# G431 舵机驱动固件

本工程面向 STM32G431VBTx + STSPIN32G4 三相栅极驱动器，使用 Keil MDK 和 STM32 HAL。代码按「上层业务调用下层接口、下层不反向调用上层」的方向组织。

**这份文档按模块编排。** 每个模块一节，节内先说明「这个模块自己拥有哪些外设资源、各自起什么作用」，再说明「它对上提供哪些接口、它的上层是谁、应该被谁调用、它又依赖谁」。

外设**不单独成章**——I2C3 只在驱动芯片通讯模块里讲，TIM1 只在三相驱动模块里讲。只有放在所属模块里，才能说清它是为谁服务的。

| 项目 | 内容 |
| --- | --- |
| 主控 MCU | STM32G431VBTx（Cortex-M4F，LQFP100），SYSCLK 168 MHz（HSE 8 MHz） |
| 栅极驱动 | STSPIN32G4，I2C3 从机地址 `0x47` |
| 功率级 PWM | TIM1 中央对齐 20 kHz，三相互补 + 500 ns 死区，PE8~PE13 |
| 监控心跳 | TIM6，1 s |
| 位置反馈 | MT6835 磁编码器（SPI1，PA4~PA7；L1 配置 + L2 驱动 + 通讯验证已完成） |
| 电流采样 | OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组两通道、16 倍过采样，由 TIM1_CH4 触发（**配置层已完成，业务层未实现**） |
| Keil 工程 | `software/servo_v1/MDK-ARM/servo_v1.uvprojx` |
| CubeMX | `software/servo_v1/servo_v1.ioc` |

---

## 1. 本轮代码审查与修复摘要

对全部业务代码做了一轮审查。**已修复 4 处**，另有 8 处只记录未改动（需决策，见第 13 节）。

### 已修复

| # | 文件 | 问题 | 严重度 | 修复方式 |
| --- | --- | --- | --- | --- |
| 1 | `FOC/Src/foc_interface.c` | `FOC_ControllerStep()` 的安全门分支里**无条件调用 `port.disable_output()`**，而进入该分支的原因之一恰恰可能就是 `disable_output == 0`。上层只注册部分回调时，运行到这里就是**对空函数指针解引用 → HardFault** | ⛔ 高（会崩溃） | 调用前加判空。前四个条件的短路求值已保证 `is_power_stage_safe()` 不会在指针为空时被调用 |
| 2 | `Modules/Three_Phase_Driver/Inc/tim1_pwm.h` | `TIM1_PWM_DEADTIME_NS` **32 位整数溢出**：`84U * 1000000000UL` = 8.4e10，超出 `uint32` 上限后回绕，宏求值结果是 **14 而不是 500** | ⚠ 中（当前无实际影响） | 改为 `(TDTS * 1000UL) / (CLOCK_HZ / 1000000UL)`，中间量最大 84000，结果精确 500 |
| 3 | `Core/Src/main.c` | 驱动监控报 FAULT / COMM_LOST 后，**没有任何一方去清 MOE**。芯片内部保护会把栅极拉低（硬件不会损坏），但软件侧 MOE 仍置位、测量状态仍是 RUNNING；**故障条件一消失，输出会在无人复核的情况下自行恢复** | ⚠ 中（安全隐患） | 主循环加 L6 跨层仲裁：`IsRunning() && GetState() != READY` 时调 `Tim1PwmVerify_Stop()` |
| 4 | `Tests/Inc/tim1_pwm_verify.h` | Runbook 写「默认 U/10% 时 CCR1 = 420」，但 `main.c` 默认 `duty = 50`（标度 1000）→ 实际 **CCR1 = 210（5%）**。照 Runbook 核对寄存器会误判代码有 bug | ℹ 低（文档错误） | 改成同时列出 5% 与 10% 两种情况的期望值 |

> **关于 #2：** `TIM1_PWM_DEADTIME_NS` 目前**没有任何代码引用**（实际写入 `BDTR.DTG` 的是 `TIM1_PWM_DEADTIME_TDTS`，那个是对的）。所以溢出**不影响当前固件行为**，500 ns 死区是正确的。但它是个陷阱——谁哪天用这个宏做断言或换算，就会拿到 14 ns。

> **关于 #3：** 这是本轮**唯一改变运行行为**的修复。原来的失效模式是「故障消失后输出自动恢复」，现在改为「故障即关断，恢复必须显式重新 Start」。若更希望自动恢复，可以改回去。

### 注释中文化进度

已把 **9 个业务文件**的注释改为中文：`main.c`、`tim1_pwm.c/h`、`stspin32g4.c/h`、`drv_monitor.c/h`、`tim1_pwm_verify.c/h`、`measurement_manager.h`、`foc_interface.c/h`。

**仍为英文：** `tim6_heartbeat.c`、`i2c3_bus.c/h`、`foc_algorithm.c/h`、`Core/` 下的 `gpio.c`、`stm32g4xx_it.c`、`stm32g4xx_hal_msp.c`。

> 所有翻译都经过「剥离注释后比对代码指纹」校验，确认**代码零改动**（`main.c` 除外，它含 #3 那处刻意修复）。翻译过程中曾手误把一个宏值写错，靠这套校验及时发现并修正。

---

## 2. 分层和依赖方向

```text
L6  Application/Core（main.c）
        初始化编排 · 主循环后台任务 · 跨层仲裁 · 持有全局测量配置
    ↓ 调用
L5  Tests（验证业务）          FOC（纯算法 + 硬件端口中间层）
    ↓ 使用                        ↓ 经 foc_port_t 四个函数指针
L4  Modules/Three-Phase Driver（TIM1 配置 + PWM 业务）
    ↓
L3  Modules/Driver Communication（驱动状态机和监控快照，独占 I2C）
    ↓
L2  Modules/Driver Communication（STSPIN32G4 芯片寄存器接口）
    ↓
L1  外设配置：i2c3_bus / tim6_heartbeat / gpio / tim1_pwm(MX_TIM1_Init)
              adc1_current_sampling(OPAMP1/3 + ADC1) / msp
    ↓
L0  中断向量：stm32g4xx_it.c（只做分发，不含业务）
    ↓
    STM32 HAL / CMSIS / 芯片寄存器
```

**具体约束：**

- `main.c` 可以调用所有下层公开接口，负责初始化顺序和任务调度。
- `gpio.c`、`i2c3_bus.c`、`tim6_heartbeat.c`、`tim1_pwm.c`、`adc1_current_sampling.c` **不得包含 `main.h`，也不得调用 `Error_Handler()`**。初始化失败通过返回值交给 `main.c` 处理。（已核实：这些文件当前均满足此约束）
- `main.h` 只属于应用入口，**不再保存 GPIO 引脚定义**。
- GPIO 定义和 GPIO 初始化接口属于 `Core/Inc/gpio.h` / `Core/Src/gpio.c`。
- `FOC/Src/foc_algorithm.c` **不包含 HAL、GPIO、定时器或器件驱动头文件**；硬件访问只能通过 `foc_port_t` 回调完成。
- 下层不得调用上层；配置层不得调用业务层（HAL 的 MSP 回调除外，它本身属配置层）。

**模块 ↔ 层次 ↔ 目录对照：**

| 模块 | 目录 | 层次 | 拥有的外设 | 状态 |
| --- | --- | --- | --- | --- |
| A 驱动芯片通讯 | `Modules/Driver_Communication/` | L1+L2+L3 | I2C3、TIM6、PE7/14/15 | ✅ 完成 |
| B 三相驱动 | `Modules/Three_Phase_Driver/` | L1+L4 | TIM1、PE8~PE13 | ✅ 完成 |
| C 控制算法 | `FOC/` | L5 | 无（纯算法） | ⚠ 已实现，端口未接线 |
| D 测试验证 | `Tests/` | L5 | 无（借用模块 B） | ✅ **当前固件运行的就是它** |
| E 板级支撑 | `Core/` | L0+L1+L6 | GPIO、中断向量、MSP、时钟 | ✅ 完成 |
| F 电流采样 | `Modules/Current_Sampling/` | L1 | ADC1、OPAMP1、OPAMP3、PA1/PA2/PA3、PB0/PB1/PB2 | ⚠ 配置层完成，**业务层未实现** |
| G 编码器 | `Modules/Encoder/` | L1+L2 | SPI1（PA4~PA7） | ✅ L1 配置 + L2 器件驱动完成 |
| — 通讯协议栈 | *（待建）* | L5 | CAN/UART | ❌ 无源码 |

---

## 3. 目录职责

```text
software/servo_v1/
├── Core/                                    模块 E：板级支撑
│   ├── Inc/main.h                  应用声明和 Error_Handler
│   ├── Inc/gpio.h                  GPIO 引脚定义与 GPIO 初始化声明
│   ├── Inc/stm32g4xx_it.h          中断向量声明
│   ├── Inc/stm32g4xx_hal_conf.h    HAL 裁剪（TICK_INT_PRIORITY=15，
│   │                               USE_HAL_TIM_REGISTER_CALLBACKS=0）
│   └── Src/main.c                  初始化编排、主循环、后台任务、跨层仲裁
│       Src/gpio.c                  三个栅极驱动控制脚
│       Src/stm32g4xx_it.c          L0 中断向量分发
│       Src/stm32g4xx_hal_msp.c     时钟/复用/NVIC 绑定
│       Src/system_stm32g4xx.c      CMSIS 系统初始化
├── Modules/
│   ├── Driver_Communication/                模块 A：驱动芯片通讯
│   │   ├── Inc/i2c3_bus.h          I2C3 配置接口            ← L1 配置
│   │   ├── Inc/tim6_heartbeat.h    TIM6 1 秒心跳配置接口      ← L1 配置
│   │   ├── Inc/stspin32g4.h        芯片寄存器接口            ← L2 业务
│   │   ├── Inc/drv_monitor.h       驱动状态机和监控快照       ← L3 业务
│   │   └── Src/（同名四个 .c）
│   ├── Three_Phase_Driver/                  模块 B：三相驱动
│   │   ├── Inc/tim1_pwm.h          三相 PWM 配置和功率级接口
│   │   └── Src/tim1_pwm.c          ⚠ 配置与业务同居一个文件
│   └── Current_Sampling/                      模块 F：电流采样
│       ├── Inc/adc1_current_sampling.h  OPAMP1/3 + ADC1 注入组配置接口 ← L1
│       └── Src/adc1_current_sampling.c  业务层（偏置标定 + 安培换算）待建
├── FOC/                                     模块 C：控制算法（独立分组，不放 Core）
│   ├── Inc/foc_algorithm.h         参数、输入、输出、算法对象
│   ├── Inc/foc_interface.h         算法控制器和硬件端口
│   └── Src/
├── Encoder/                                 模块 G：磁编码器（本轮新增）
│   ├── Inc/spi1_encoder.h         SPI1 配置接口 ← L1
│   ├── Inc/mt6835.h               MT6835 寄存器接口 ← L2
│   └── Src/（同名两个 .c）
├── Tests/                                   模块 D：产线或调试验证，不属于算法层
│   ├── Inc/tim1_pwm_verify.h · Inc/measurement_manager.h
│   ├── Inc/mt6835_verify.h                  编码器 SPI 通讯验证
│   └── Src/tim1_pwm_verify.c · Src/mt6835_verify.c
├── Drivers/                                 第三方库（勿改，升级时整体替换）
└── MDK-ARM/                                 Keil 工程与编译产物
```

**Keil 工程分组 ↔ 磁盘目录**（分组名与目录已保持一致）：

```text
Application/Startup            → MDK-ARM/startup_stm32g431xx.s
Application/Core               → Core/
Modules/Driver Communication   → Modules/Driver_Communication/
Modules/Three-Phase Driver     → Modules/Three_Phase_Driver/
Modules/Current Sampling       → Modules/Current_Sampling/
Modules/Encoder                → Modules/Encoder/
Modules/FOC                    → FOC/          ← 注意分组名带 Modules/ 前缀
Tests                          → Tests/
Drivers/STM32G4xx_HAL_Driver   → Drivers/STM32G4xx_HAL_Driver/
Drivers/CMSIS · ::CMSIS        → Drivers/CMSIS/
```

Include 路径：`../Core/Inc; ../FOC/Inc; ../Modules/Driver_Communication/Inc; ../Modules/Three_Phase_Driver/Inc; ../Modules/Current_Sampling/Inc; ../Modules/Encoder/Inc; ../Tests/Inc; ../Drivers/STM32G4xx_HAL_Driver/Inc; .../Inc/Legacy; ../Drivers/CMSIS/Device/ST/STM32G4xx/Include; ../Drivers/CMSIS/Include`

每次新增或移动源文件时，必须同时更新 `servo_v1.uvprojx` 和 `servo_v1.cproject.yml`，再在 Keil 工程树中确认分组与文件路径一致。

---

## 4. 模块 A：驱动芯片通讯（`Modules/Driver_Communication/`）

### 4.1 ⚠ 先澄清命名：这个模块不叫「三相通讯」，也不是三相通讯

Keil 分组名曾经是 `Modules/Three-Phase Communication`，容易误读。现已改为 `Modules/Driver Communication`。工程里「通讯」有三种完全不同的含义：

| 名称 | 实际含义 | 实现位置 | 状态 |
| --- | --- | --- | --- |
| **驱动芯片通讯** | I2C3 ↔ STSPIN32G4 寄存器读写（板内控制通路） | **本模块** | ✅ 已实现 |
| 三相功率通路 | U/V/W 栅极 PWM 驱动电机 | 模块 B | ✅ 已实现 |
| 对外协议栈 | CAN / UART 与上位机通讯 | — | ❌ 未实现 |

> 原理图 `hardware/舵机原理图/5_通讯电路.SchDoc` 里画的是第三种（对外通讯），固件侧目前**没有**对应源码。`main.c` 的 TODO 注释明确写着 "protocol stack (CAN/UART)" 待实现。

### 4.2 模块职责

一句话：**MCU 与 STSPIN32G4 栅极驱动芯片之间的全部往来，以及「功率级现在能不能开」这个判定。**

模块内部自上而下三层：

```text
drv_monitor.c     (L3 监控服务)  判定驱动健康状态，独占 I2C 总线
      ↓
stspin32g4.c      (L2 器件驱动)  屏蔽寄存器细节与 LOCK 时序
      ↓
i2c3_bus.c        (L1 外设配置)  配好 I2C3，交出 hi2c3 句柄
tim6_heartbeat.c  (L1 外设配置)  配好 TIM6，交出 htim6 句柄
```

### 4.3 本模块拥有的外设资源（各自起什么作用）

| 外设 | 配置文件 | 作用（为什么这个模块需要它） | 导出给上层的东西 |
| --- | --- | --- | --- |
| **I2C3**<br>PC8/PC9, AF8 | `Src/i2c3_bus.c` | 与 STSPIN32G4 通讯的**唯一物理通道**。读写寄存器、清故障、唤醒、进待机全靠它 | `extern I2C_HandleTypeDef hi2c3`<br>`HAL_StatusTypeDef MX_I2C3_Init(void)` |
| **TIM6**<br>无引脚 | `Src/tim6_heartbeat.c` | 产生 **1 s 心跳**，驱动 L3 的后台轮询节奏。用独立基本定时器而不是 SysTick，是为了让心跳周期与 HAL 时基解耦 | `extern TIM_HandleTypeDef htim6`<br>`HAL_StatusTypeDef MX_TIM6_Init(void)` |
| **GPIO 三个控制脚**<br>PE7/PE14/PE15 | `Core/Src/gpio.c`<br>*（⚠ 归属见 13.B）* | `GD_WAKE`(PE7) 唤醒芯片；`GD_READY`(PE14) 与 `GD_NFAULT`(PE15) 是芯片向 MCU 报告状态的两个开漏输出，**是硬件真正用来门控功率级的信号** | `GD_WAKE_Pin` / `GD_READY_Pin` / `GD_NFAULT_Pin` 及对应 `_GPIO_Port` 宏，定义在 **`Core/Inc/gpio.h`** |

**I2C3 配置细节：** Timing `0x40B285C2`、7 位地址模式、关闭双地址/广播呼叫/NoStretch、模拟滤波使能、数字滤波 0。引脚在 `stm32g4xx_hal_msp.c` 的 `HAL_I2C_MspInit()` 里配为 `AF_OD` + `PULLUP` + `VERY_HIGH` 速度。

> 该接口只初始化 I2C3 和 PC8/PC9，**不知道 STSPIN32G4 的寄存器语义**，器件读写由 `stspin32g4.c` 完成。I2C 总线只允许在初始化阶段和 L3 的低频监控任务中使用，**禁止放进 PWM/电流环 ISR**。

**TIM6 配置细节：** Prescaler `16800-1`、Period `10000-1`、向上计数、自动重载预装载使能 → 168 MHz / 16800 = **10 kHz 计数时钟**，/ 10000 = **1 s 周期**。中断只置位 `g_drv_monitor_tick`，不进行 I2C 传输、不运行状态机、不调用延时函数。

> ⚠ `MX_TIM6_Init()` **只做 `HAL_TIM_Base_Init()`，不启动定时器**。启动动作（`HAL_TIM_Base_Start_IT`）由 L3 的 `DrvMonitor_Init()` 负责——刻意为之：心跳必须和监控逻辑同时开始，不能提前空转。

**控制脚电平语义（业务层必须记牢，两个脚方向相反）：**

| 信号 | 方向 | 有效电平 | 含义 |
| --- | --- | --- | --- |
| `GD_WAKE` | MCU → 驱动器 | 高 | 退出待机（芯片侧有内部下拉） |
| `GD_READY` | 驱动器 → MCU | 高 | 驱动器可工作 |
| `GD_NFAULT` | 驱动器 → MCU | **低有效** | 0 表示故障 |

> `gpio.c` 里 `GD_WAKE` 上电初始化为 `GPIO_PIN_RESET`（低），即**默认不唤醒**——正确的安全初值。
> `GD_READY` / `GD_NFAULT` 配为输入 + `PULLUP`。nFAULT 低有效配上拉是安全的（无故障时读到 1）；但 READY 高有效配上拉意味着**芯片没接或没上电时会读到「就绪」的假象**。当前有 I2C 轮询和 STATUS 寄存器交叉校验兜底，所以不致命（见 13.G）。

### 4.4 L2 器件接口：STSPIN32G4（`stspin32g4.c/h`）

**职责边界：** 只封装寄存器访问和控制脚语义——保护寄存器的 LOCK 解锁时序、命令寄存器的特殊写法、引脚电平与 STATUS 寄存器的一致性等待。

**不做：** 不判断驱动是否健康（L3 的事）、不决定何时开关功率级（L4/L6 的事）、不配置 I2C 外设（L1 的事，只用 `hi2c3`）。

**上层是谁：** 只有 L3 `drv_monitor`。**其他任何模块都不应直接调用本层**——I2C 总线必须由单一持有者串行化。

```c
HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                          uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                           uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_ReadRegisters(uint8_t reg, uint8_t *data,
                                           uint16_t size, uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_WriteRegisters(uint8_t reg, const uint8_t *data,
                                            uint16_t size, uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en,
                                          uint32_t timeout);
uint8_t STSPIN32G4_IsReady(void);
uint8_t STSPIN32G4_IsFault(void);
uint8_t STSPIN32G4_PinsAllowSwitching(void);
```

| 接口 | 说明 |
| --- | --- |
| `ReadRegister` / `WriteRegister` | 转发到对应的 `Registers` 版本（size=1）。写保护寄存器时自动完成 LOCK 解锁，上层无需关心 |
| `ReadRegisters` / `WriteRegisters` | `data` 为空或 `size == 0` 返回 `HAL_ERROR` |
| **`Init`** | 清上电锁存故障 → 读 STATUS 校验无保护置位 → 有界等待引脚与寄存器一致。返回 `HAL_BUSY` 表示引脚尚未稳定，**调用者应重试而非判死** |
| `IsReady` / `IsFault` | 分别读 `GD_READY`（1 = 可用）与 `GD_NFAULT` **原始低有效**电平（0 = 有故障）。**不碰 I2C，任何上下文可调** |
| **`PinsAllowSwitching`** | **恒定耗时安全门**：一次读取 GPIOE 的 `IDR`，同时判定 READY=1 且 nFAULT=1。因两脚同端口，**一次读即原子快照**，不会出现「读到一半引脚翻转」。不访问 I2C，适合实时路径 |
| `ClearFaults` | 向 CLEAR(0x09) 写 `0xFF` |
| `Wake` | 拉高 WAKE → 等 READY 释放 → 拉低 WAKE → **重新上锁 LOCK** → 调 `Init()` 清故障。超时返回 `HAL_TIMEOUT` 并先把 WAKE 拉回低 |
| `EnterStandby` | 见下方两个硬约束 |

**⚠ `EnterStandby()` 的两个硬约束（源码注释重点标注）：**

1. **电流预算：** 待机稳压器最多提供 `IREG3V3,Stby = 6 mA`（典型 5 mA）。STM32G431 跑 168 MHz 远超此值，所以 `stby_reg_en = 1` **只在 MCU 随后立即进入 Stop 模式时才成立**，否则稳压器触发过流保护并复位芯片。
2. **这是序列中最后一次 I2C 访问：** STBY 位锁存后经 `tSTBY`（100 µs）芯片进入低功耗，I2C 接口即不可用，**之后不能再尝试重新上锁**（那次传输必然超时）。重新上锁由 `Wake()` 完成。

**导出的寄存器语义常量（L3 直接使用）：**

```c
STSPIN32G4_I2C_ADDRESS        /* 0x47U << 1 */

/* 寄存器地址：0x01 POWMNG(保护) · 0x02 LOGIC(保护) · 0x07 READY
              0x08 NFAULT(保护) · 0x09 CLEAR(命令) · 0x0A STBY(保护)
              0x0B LOCK · 0x0C RESET(命令,保护) · 0x80 STATUS(只读) */
/* 位定义：POWMNG_* / LOGIC_* / READY_* / NFAULT_* / STATUS_* */
/* 命令值：CLEAR_FAULTS = 0xFF · RESET_CMD = 0xFF · STBY_ENTER = 0x01 */
/* 锁：   LOCK_UNLOCK = 0xA5 · LOCK_LOCK = 0x00 */
STSPIN32G4_PIN_SETTLE_MS      /* 2，写 CLEAR 后等引脚与 STATUS 一致 */
```

> 写保护寄存器需令 `LOCK == ~NLOCK`（写 `0xA5`）。
> `PIN_SETTLE_MS = 2` 的由来：手册 `tWAKE,LDO` 只有 10 µs，但 `HAL_GetTick()` 分辨率是 1 ms，所以放宽到 2 ms。

### 4.5 L3 驱动监控与故障策略（`drv_monitor.c/h`）

**职责边界：** 回答一个问题——**「功率级现在能不能开？」** 它是全工程**唯一**允许发起 I2C 通信的模块，把芯片状态收敛成一份可安全跨上下文读取的快照。

**上层是谁：** L6 `main.c`（初始化 + 主循环轮询 + 跨层仲裁），以及未来的电流环 / 速度环 / 协议栈（只读快照或投递请求）。

#### ⚠ 分层契约（改动前必读）

```text
TIM6 更新中断  : 只置 1 s 心跳标志，绝不访问 I2C
DrvMonitor_Task(): 在主循环运行，独占全部 I2C 通信
电流环/速度环  : 绝不调用本模块的慢路径，只读无撕裂快照
                 或用 IsPowerStageSafeFast()
```

**为什么：** I2C 传输有不可预测的等待时间，放进 20 kHz 电流环会直接破坏 PWM 时序。所以监控走「低频后台任务 + 快照发布」模式。

```c
HAL_StatusTypeDef DrvMonitor_Init(void);
void              DrvMonitor_Task(void);
void              DrvMonitor_PostRequest(drv_request_t request);
void              DrvMonitor_RequestStandby(uint8_t stby_reg_en);
HAL_StatusTypeDef DrvMonitor_GetSnapshot(drv_monitor_t *snapshot);
drv_state_t       DrvMonitor_GetState(void);
uint8_t           DrvMonitor_IsPowerStageSafeFast(void);
```

| 接口 | 说明 | 应被谁调用 |
| --- | --- | --- |
| **`Init`** | 在无实时任务时清除上电锁存故障（**必须**：不写 CLEAR 芯片一直不动作），最多重试 3 次，检查器件状态并启动 TIM6。失败返回 `HAL_ERROR` | `main()` bring-up，一次 |
| **`Task`** | 在主循环中反复调用；没有 1 秒 tick 时立即返回（开销近零），有 tick 时才执行一次状态寄存器轮询和状态机推进 | `main()` 主循环，每轮都调 |
| `PostRequest` | 非阻塞投递，由下一次 1 s 任务消费。**任何上下文可安全调用** | L6 / 未来协议栈（远程故障复位） |
| `RequestStandby` | 带 `STBY_REG_EN` 选择的待机请求，内部转投 `DRV_REQ_STANDBY` | L6 |
| **`GetSnapshot`** | **短临界区内整体拷贝**（`__get_PRIMASK` → `__disable_irq` → 拷贝 → 恢复），**任何上下文含 ISR 均安全**。`snapshot` 为 NULL 返回 `HAL_ERROR` | 电流环 / 速度环 / 遥测 |
| `GetState` | 轻量快速路径，返回 `s_mon.state` | `main()`、模块 D |
| **`IsPowerStageSafeFast`** | 转发到 L2 的 `STSPIN32G4_PinsAllowSwitching()`。**只读两个 GPIO，不访问 I2C，不改快照，执行时间固定** | 20 kHz 电流环（经 `foc_port_t.is_power_stage_safe`） |

监控状态包括：`INIT`、`STANDBY`、`READY`、`FAULT`、`COMM_LOST`。详细寄存器错误通过 `drv_monitor_t` 快照发布。**工作副本 `s_work` 只在后台任务中修改**，发布和读取快照都使用短临界区，因此中断不会读到半更新结构体。

> **不要在中断里直接读 `s_mon`**——`extern drv_monitor_t s_mon` 非 static 仅为调试器 Watch 可见，请一律用 `GetSnapshot()`。

#### 两条互补的故障检测路径

电流环不应每次通过 I2C 查询 STATUS。实时路径只检查两个硬件输入：

```c
if (DrvMonitor_IsPowerStageSafeFast() == 0U)
{
    /* 通过 FOC 端口关闭 MOE，禁止继续输出 */
}
```

| 路径 | 手段 | 频率 | 负责什么 |
| --- | --- | --- | --- |
| 快速路径 | GPIO 读 READY / nFAULT | 每个 PWM 周期（20 kHz） | 快速关断条件 |
| 慢速路径 | I2C 读 STATUS 寄存器 | 1 s | 故障原因、VDS / 热关断 / 欠压诊断、通信重试 |

#### 导出的类型

```c
/* 状态机 —— 上层判定能否使能功率级的唯一依据 */
typedef enum {
  DRV_STATE_INIT = 0,   /* 上电：正在清除锁存故障 */
  DRV_STATE_STANDBY,    /* 驱动休眠，等待唤醒请求 */
  DRV_STATE_READY,      /* 健康，允许使能功率级 */
  DRV_STATE_FAULT,      /* 保护动作中，输出必须保持关闭 */
  DRV_STATE_COMM_LOST   /* I2C 不可达，原因未知 */
} drv_state_t;

/* 上层可投递的请求 */
typedef enum {
  DRV_REQ_NONE = 0,
  DRV_REQ_CLEAR_FAULT,  /* 锁存保护之后重新武装 */
  DRV_REQ_WAKE,         /* 退出待机 */
  DRV_REQ_STANDBY       /* 进入低功耗 */
} drv_request_t;

/* STATUS(0x80) 位域，与 DS13630 Table 25 逐位对应 */
typedef union { uint8_t raw; struct {
  uint8_t vcc_uvlo:1; uint8_t thsd:1; uint8_t vds_p:1;
  uint8_t reset:1; uint8_t rsvd:3; uint8_t lock:1;
} bit; } drv_status_t;

/* 故障标志：latched 记住已消失的故障，避免在两次轮询之间丢失原因 */
typedef struct {
  uint8_t active:1;    /* 当前有任一保护置位 */
  uint8_t vds_p:1;     /* 锁存：VDS 保护（过流） */
  uint8_t thsd:1;      /* 锁存：热关断 */
  uint8_t vcc_uvlo:1;  /* 锁存：VCC 欠压 */
  uint8_t comm:1;      /* 锁存：I2C 通信失败 */
  uint8_t rsvd:3;
} drv_fault_t;

/* 对外发布的快照 */
typedef struct {
  drv_state_t  state;
  drv_status_t status;       /* 最近一次 STATUS 原始内容 */
  drv_fault_t  fault;        /* 实时 + 锁存保护标志 */
  uint8_t      READY;        /* READY 引脚原始电平，高有效 */
  uint8_t      nFAULT;       /* nFAULT 引脚原始电平，低有效 */
  uint8_t      comm_errors;  /* 连续 I2C 失败次数 */
  uint32_t     uptime_s;     /* 监控启动以来的秒数 */
  uint32_t     last_fault_s; /* 最近一次锁存故障时的 uptime */
} drv_monitor_t;
```

**内部可调参数与私有实现（`static`，不对外）：**

```c
#define DRV_I2C_TIMEOUT_MS     10U   /* 400kHz 下一次 1 字节传输约 50 µs */
#define DRV_WAKE_TIMEOUT_MS    50U   /* 拉高 WAKE 后 READY 释放的时间 */
#define DRV_COMM_ERR_LIMIT     3U    /* 连续失败多少次判定 COMM_LOST */
#define DRV_INIT_RETRY         3U    /* 上电清故障尝试次数 */
#define DRV_INIT_RETRY_GAP_MS  10U
```

**状态机判定优先级**（`DrvMonitor_Evaluate()`，自上而下短路）：

```text
comm_errors >= 3                        → COMM_LOST
fault.active                            → FAULT
(非 STANDBY) && (nFAULT==0 || READY==0) → FAULT
(非 STANDBY)                            → READY
(是 STANDBY)                            → 保持 STANDBY
```

> 待机状态下不轮询 I2C（`state != DRV_STATE_STANDBY` 才调 `PollStatus`），因为待机时 I2C 接口已关闭，轮询只会产生虚假通信错误。但**引脚电平仍照常采样**——即使 I2C 挂了，引脚状态依然有效。

**`DrvMonitor_Task()` 完整流程：**

```text
心跳未到 → 立即返回
uptime_s++
采样 READY / nFAULT 引脚电平（不碰 I2C）
ProcessRequest()          消费上层请求（CLEAR_FAULT / WAKE / STANDBY）
若非 STANDBY → PollStatus()   读 STATUS + 故障记账
Evaluate()                推进状态机
临界区内 s_mon = s_work   整体发布，观察者不会看到半更新状态
```

### 4.6 本模块实现的中断回调（重要约定）

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* 定义在 drv_monitor.c，由 TIM6_DAC_IRQHandler 经 HAL_TIM_IRQHandler 调用。
   仅置 g_drv_monitor_tick = 1 后立即返回：无 I2C、无状态机、无 HAL_Delay，
   执行时间几个周期，不干扰电流环与 PWM 时序。 */
```

> ⚠ **STM32G4 的 OPAMP 没有独立的 RCC 时钟门**，所以 `HAL_OPAMP_MspInit()` 只使能 GPIO 时钟；运放本体由 `HAL_OPAMP_Init()` 写自己的 CSR 使能位打开。
>
> 🔑 **全工程唯一性约定：** 工程配置 `USE_HAL_TIM_REGISTER_CALLBACKS = 0`，所以 `HAL_TIM_PeriodElapsedCallback` 是**弱符号覆盖**模式，**全工程只能有一个定义**。它被 TIM6（心跳）和 TIM1_UP（周期边界）两个向量共用，因此唯一定义放在 `drv_monitor.c`，内部按 `htim->Instance == TIM6` 过滤。
>
> **新增模块若需要同名回调，绝不要重复定义**（会链接冲突），应挂到这里的分发逻辑中，或改用其他回调名（例如电流环用 `HAL_TIM_OC_DelayElapsedCallback`）。

---

## 5. 模块 B：三相驱动（`Modules/Three_Phase_Driver/`）

### 5.1 模块职责

一句话：**把「某相高侧占空比」翻译成功率级的栅极信号，并掌管 MOE 这道功率级总闸。**

**不知道电机模型、不做闭环、不判断驱动健康**——这些分别是模块 C、L6、模块 A 的职责。

### 5.2 本模块拥有的外设资源

| 外设 | 配置位置 | 作用 | 导出给上层的东西 |
| --- | --- | --- | --- |
| **TIM1**<br>PE8~PE13 | 本模块 `Src/tim1_pwm.c` 的 `MX_TIM1_Init()` | 产生 **20 kHz 三相互补 PWM**，直接驱动 STSPIN32G4 的六路栅极输入。CH4 额外用作 **ADC 采样触发**（为未来电流环预留） | `extern TIM_HandleTypeDef htim1`<br>`HAL_StatusTypeDef MX_TIM1_Init(void)` |
| 六路栅极 GPIO | `Core/Src/stm32g4xx_hal_msp.c` 的 `HAL_TIM_MspPostInit()` | 把 PE8~PE13 配为推挽复用输出 | 无（HAL 自动回调） |
| TIM1 的 NVIC | `Core/Src/stm32g4xx_hal_msp.c` 的 `HAL_TIM_PWM_MspInit()` | CC 中断优先级 0（电流环采样）、UP 中断优先级 1（周期边界） | 无 |

**TIM1 引脚映射（本模块对硬件的核心约定，改引脚前必看）：**

| MCU 引脚 | TIM1 通道 | 接到 STSPIN32G4 | 电机相 |
| --- | --- | --- | --- |
| PE9 | TIM1_CH1 | INH1（高侧） | A / U |
| PE8 | TIM1_CH1N | INL1（低侧） | A / U |
| PE11 | TIM1_CH2 | INH2 | B / V |
| PE10 | TIM1_CH2N | INL2 | B / V |
| PE13 | TIM1_CH3 | INH3 | C / W |
| PE12 | TIM1_CH3N | INL3 | C / W |

> 依据 STSPIN32G4 手册 "Table 7. MCU to gate driver internal connection"：六路栅极信号**必须**配为推挽复用输出（`AF_PP` + `AF2_TIM1` + `NOPULL`），栅极驱动 IC 只接受这一种 GPIO 配置。

**TIM1 配置细节：** 中央对齐模式 1、Prescaler 0、ARR = 4199、RCR = 1、时钟分频 DIV1 → 168 MHz / (2 × 4200) = **20 kHz 载波**，且每载波周期恰好一次更新事件。死区 `DTG = 84`（tDTS 单位，1 tDTS = 5.952 ns）→ **500 ns**。OSSR/OSSI 关闭、LockLevel OFF、Break/Break2 禁用、AutomaticOutput 禁用。`TRGO = UPDATE`。CH1~3 为 PWM 模式 1、高有效；CH4 为 PWM 模式 2、`Pulse = 4199 - 168 = 4031`（比更新事件提前 1 µs）。

> **为什么 Break 输入禁用：** BKIN/BKIN2 对应栅极驱动的 READY 和 NFAULT，但**本 MCU 上 PE15 根本没有 TIM1 复用功能**。这两个脚改由模块 A 的 `drv_monitor` 读取，保护工作交给芯片自身的 VDS / 热保护。要启用 Break 必须先决定放弃哪个引脚。
>
> **为什么 OSSR/OSSI 关闭：** MOE 清零期间，六个引脚交给栅极驱动自己的输入上下拉电阻，而不是被驱动到某个电平——那个电平对栅极驱动意味着什么，无法从本数据手册读出。打开 OSSR 会把 OC/OCN 驱到空闲态，那是一个明确的「刹车 / 滑行」选择，属于功率级使能逻辑的职责，不该放在初始化里。

### 5.3 【配置部分】L1 接口

```c
extern TIM_HandleTypeDef htim1;
HAL_StatusTypeDef MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
```

`MX_TIM1_Init()` 配置并启动 20 kHz 中央对齐互补 PWM、500 ns 死区和 CH4 采样触发。**它保持 `MOE = 0`，因此初始化不会接通功率级。** `main()` 必须检查返回值。六路栅极 GPIO 由 HAL 在末尾调用 `HAL_TIM_MspPostInit()` 配置。

> 🔑 **`MX_TIM1_Init()` 特意不清 MOE。** 这是配置与业务最重要的一道分界：**配置只把硬件准备好，不接通功率。** 使能功率级是独立的显式业务步骤 `Tim1Pwm_EnableOutputs()`。
>
> 内含两项**防御性检查**，失败直接返回 `HAL_ERROR`：
> - `TIM1_PWM_DTG_IN_RANGE(deadtime)` —— 死区值必须能用 DTG 的 `0x00..0x7F` 段编码
> - `HAL_RCC_GetPCLK2Freq() == TIM1_PWM_TIMER_CLOCK_HZ` —— 时钟树必须是预期的 168 MHz。**换一套时钟树会悄无声息地同时改变载波频率和死区时间，所以这里选择「检查」而不是「假设」**
>
> `HAL_TIM_MspPostInit()` 必须在定时器配置完成**之后**才初始化引脚，否则初始化过程中引脚会产生毛刺，可能误触发栅极驱动。**不要把它合并进 `MspInit`。**

**配置部分导出的常量：**

```c
TIM1_PWM_TIMER_CLOCK_HZ      168000000  /* APB2 不分频，TIM1 时钟 = SYSCLK */
TIM1_PWM_FREQUENCY_HZ            20000
TIM1_PWM_COUNTS_PER_PERIOD        8400  /* 一个载波周期的计数总数 = 2×(ARR+1) */
TIM1_PWM_COUNTER_PERIOD           4199  /* 装入 ARR 的峰值 */
TIM1_PWM_DEADTIME_TDTS              84  /* DTG 编码值，= 500 ns */
TIM1_PWM_DEADTIME_NS               500  /* ← 本轮修复了此宏的整数溢出 */
TIM1_PWM_ADC_TRIGGER_LEAD_NS      1000  /* ADC 触发领先更新事件 1 µs = 168 计数 */
```

> `TIM1_PWM_DEADTIME_TDTS` 的 `0x00..0x7F` 编码段以 tDTS 为单位，**上限是 127 计数**；更长的死区需改用 `0x80..0xBF` 编码（步长 2×tDTS）。
>
> 编译期还有两道保护：`TIM1_PWM_PERIOD_FITS_U16`（16 位比较寄存器必须能容纳整个周期，否则 `#error`）和 `TIM1_PWM_DTG_IN_RANGE`。

### 5.4 【业务部分】L4 功率级接口

```c
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty);
uint16_t          Tim1Pwm_GetPhaseDuty(uint32_t channel);
void              Tim1Pwm_SetNeutral(void);
void              Tim1Pwm_EnableOutputs(void);
void              Tim1Pwm_EmergencyStop(void);
uint8_t           Tim1Pwm_IsOutputEnabled(void);
```

**上层是谁：** 模块 D 测试层（当前唯一使用者）、未来的模块 C FOC（经 `foc_port_t.write_voltage` / `disable_output`）、L6（仲裁与急停）。

| 接口 | 说明 | 应被谁调用 |
| --- | --- | --- |
| **`SetPhaseDuty`** | `channel` 取 `TIM_CHANNEL_1/_2/_3`；`duty` 0~1000（500 = 50%），超范围自动钳位。**先写寄存器再更新影子值**——寄存器才是功率级实际看到的状态，影子值只用于记账，因此与并发更新赛跑的读回永远不会误报引脚行为。**只做一次比较寄存器写入，可由实时路径（ISR）调用**。非法通道返回 `HAL_ERROR` | 未来 FOC 电压输出、模块 D |
| `GetPhaseDuty` | 返回该相最后写入的占空比；非法通道返回 0 | 调试 / 状态上报 |
| **`SetNeutral`** | 三相同时装载 50% 自举电容刷新模式并清零指令占空比。**应在使能输出前调用** | 模块 D、未来 FOC 启动流程 |
| **`EnableOutputs`** | 释放 MOE，六路栅极信号到达功率级。**只能在监控确认 READY 且 nFAULT 未触发后调用** | 模块 D、L6 |
| **`EmergencyStop`** | 清 MOE，所有栅极输出**立即同时**拉到无效电平，不等软件故障处理路径。任何快速故障都应调用它 | L3 故障响应、模块 D、L6 |
| `IsOutputEnabled` | 直接读 `TIM1->BDTR & TIM_BDTR_MOE`，1 = 正在驱动功率级 | 模块 D、L6 |

> ⚠ **`EmergencyStop()` 必须用无条件版本**：`__HAL_TIM_MOE_DISABLE()` 只在所有通道的 `CCxE` 和 `CCxNE` 都已清零时才生效，**在运行中的功率级上它会悄无声息地什么都不做**。代码用的是 `__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY()`。这是个很容易踩的坑。

**业务部分导出的常量：**

```c
TIM1_PWM_DUTY_SCALE         1000  /* 应用层占空比标度 */
TIM1_PWM_DUTY_NEUTRAL        500
TIM1_PWM_COMPARE_FULL_SCALE 4200  /* "高侧常通"对应比较值，使用时钳到 ARR */
TIM1_PWM_COMPARE_NEUTRAL    2100
```

### 5.5 ⚠ 占空比换算：两个致命陷阱

```c
#define TIM1_PWM_DUTY_TO_COMPARE(duty) \
  (((uint32_t)(duty) * TIM1_PWM_COMPARE_FULL_SCALE) / TIM1_PWM_DUTY_SCALE)
```

1. **比较值与占空比成正比，不是反比。** 比较值越大，高侧导通时间越长。**不要**把这个映射取反。
2. **不要按 `TIM1_PWM_COUNTS_PER_PERIOD`(8400) 缩放，必须按比较带 4200 缩放。** 超过 ARR 的比较值**不会扩展占空比范围，而是彻底停止翻转**——计数器永远到不了那个值，输出会一直停在有效电平。若误按整周期缩放，占空比超过 50% 就会全部饱和到接近 100%。

对照表（实测计数器序列，中央对齐，CNT < CCR 时有效，一个载波周期 = 2 × ARR = 8398 计数）：

| CCR | 高侧占空比 | duty |
| --- | --- | --- |
| 0 | 0.0000% | 0 |
| 210 | 5.0000% | 50 ← **当前 `main.c` 默认值** |
| 2100 | 50.0000% | 500 |
| 2520 | 60.0024% | 600 |
| 4199 | 99.9881% | 1000 |
| 4200 | 100.0000%（同样饱和） | — |

整数除法残余误差最大为周期的 0.0119%，所以标度的 50% 点精确落在载波的 50%。

> ⚠ **两个端点都会饱和，都不能当空闲值用：** `duty = 1000` 让低侧整个周期不导通，`duty = 0` 让低侧整个周期导通；两者都会使该半桥的**自举电容无法刷新**，而 `duty = 0` 还会通过低侧对电机**制动**。运行中的相请保持在标度内部，空闲态请用 `SetNeutral()`。

### 5.6 预留但尚未实现的业务接口

```c
/* TIM1 CH4 采样时刻回调 —— 未来的电流环就在这里运行。
   CC4 比较点位于更新事件前 TIM1_PWM_ADC_TRIGGER_LEAD_NS，ADC 应由 OC4REF 触发。
   ADC1 注入组已由模块 F 配置完毕，并且已经挂在 CC4 上（MX_ADC1_Init(0U) +
   CurrentSampling_Start()），硬件会在每个 OC4REF 边沿自动更新 JDR1/JDR2。
   但还没有消费者去读 JDR，所以当前仍是空桩：没有消费者时，以载波频率运行的
   空中断体只花费一次中断进入的开销。
   HAL_TIM_IRQHandler() 会无条件调用该回调，所以现在无需定义；等电流环需要时再加。
   注意：周期边界回调 HAL_TIM_PeriodElapsedCallback 已被 drv_monitor 为 TIM6 心跳
   占用，所以这里必须用 OC 回调（HAL_TIM_OC_DelayElapsedCallback），
   不能用 PeriodElapsed。 */
```

### 5.7 配置与业务在本模块中的混居情况

`tim1_pwm.c` 一个文件里同时住着配置和业务，内部已用 `USER CODE` 块自然分段，可作为将来拆分的现成边界：

| 位置 | 归属 | 内容 |
| --- | --- | --- |
| `USER CODE BEGIN 0` | 混合 | `DTG_IN_RANGE` / `PERIOD_FITS_U16` 编译期检查、`s_phase_duty[3]` 影子值、`s_phase_channel[3]` 通道查表 |
| `USER CODE BEGIN 1` | 业务（私有） | `Tim1Pwm_FindPhase()` 通道→相位映射、`Tim1Pwm_WriteCompare()` 含 ARR 钳位 |
| 同上 | 配置（私有） | `Tim1Pwm_NsToCounts()` 纳秒→计数换算 |
| `MX_TIM1_Init()` | **配置** | 含 `TIM1_Init 0/1/2` 三个子块，末尾调 `HAL_TIM_MspPostInit()` |
| `USER CODE BEGIN 2` | **业务** | 六个 `Tim1Pwm_*` 公开接口 + CH4 回调桩说明 |

> `s_phase_channel[3]` 查表的存在理由：**HAL 的通道标识符是寄存器偏移量而非位序号，`TIM_CHANNEL_1` 的值是零**，所以用 `(TIM_CHANNEL_1 << phase)` 推导后续通道会永远选中通道 1。这是个很隐蔽的坑，已用查表规避。

---

## 6. 模块 C：控制算法（`FOC/`）— ⚠ 已实现但尚未接线

### 6.1 模块职责与拥有的外设

一句话：**电流环算法。纯业务，零硬件依赖**——这是全工程分层最干净的一层。

**拥有的外设：无。** 这是刻意的。换一套硬件（换 MCU、换驱动芯片、换编码器）时本模块一行都不用改。

| 文件 | 职责 | 硬件依赖 |
| --- | --- | --- |
| `Src/foc_algorithm.c` | 纯数学：Clarke / Park 变换、d/q 轴 PI 调节器、反 Park、电压限幅 | **无**（只 `#include <stdint.h>` 与 `<math.h>`） |
| `Src/foc_interface.c` | 控制器生命周期和调度，通过回调把算法与硬件隔离 | 经 `foc_port_t` 间接依赖 |

### 6.2 算法参数结构体

```c
typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_min;
    float integral_max;
} foc_pi_t;

typedef struct {
    float sample_period_s;
    float voltage_limit;
    float current_limit;
    foc_pi_t d_controller;
    foc_pi_t q_controller;
} foc_parameters_t;
```

`sample_period_s` 是电流环实际调用周期，单位秒；`voltage_limit` 限制 d/q 合成电压矢量的幅值；`current_limit` 限制 d/q 参考值；两个 PI 控制器分别维护自己的积分状态和积分上下限。**参数属于算法对象，不放在 `main.h` 或外设驱动中。**

### 6.3 算法输入和输出

```c
typedef struct {
    float phase_a;
    float phase_b;
    float electrical_angle_rad;
    float d_reference;
    float q_reference;
    uint8_t enabled;
} foc_input_t;

typedef struct {
    float d_current;
    float q_current;
    float d_voltage;
    float q_voltage;
    float alpha_voltage;
    float beta_voltage;
    float phase_a_voltage;
    float phase_b_voltage;
    float phase_c_voltage;
    uint8_t saturated;
} foc_output_t;
```

输入的 `phase_a/phase_b` 是两相采样电流，算法先计算 Clarke 变换，再按 `electrical_angle_rad` 做 Park 变换得到 d/q 电流。PI 输出经过电压矢量限幅后执行反 Park 变换并生成三相电压指令。`enabled = 0` 时积分器和输出复位，避免停机后积分项残留。

**算法流程（`FOC_AlgorithmStep`）：**

```text
Clarke:  alpha = ia
         beta  = (ia + 2·ib) × 0.5773502692   （= 1/√3）
Park:    id =  alpha·cosθ + beta·sinθ
         iq = -alpha·sinθ + beta·cosθ          （θ = electrical_angle_rad）
若 enabled == 0 → 复位积分器与输出后立即返回
PI(d):   error = clamp(d_reference, ±current_limit) - id
PI(q):   error = clamp(q_reference, ±current_limit) - iq
限幅:    若 √(vd² + vq²) > voltage_limit → 按比例整体缩放，置 saturated = 1
反 Park: valpha = vd·cosθ - vq·sinθ
         vbeta  = vd·sinθ + vq·cosθ
三相:    va = valpha
         vb = -0.5·valpha + 0.8660254038·vbeta
         vc = -0.5·valpha - 0.8660254038·vbeta
```

> 🔑 **`foc_input_t` 只需要 `phase_a` / `phase_b` 两相电流**——第三相由 `ia + ib + ic = 0` 推出。**这决定了未来的电流采样层只需两路 ADC 通道**，是配置层被算法需求反向约束的唯一例子。
>
> `foc_limit_voltage()` 里 `magnitude <= 0.0f` 时直接返回，避免除零。

### 6.4 纯算法 API

```c
void FOC_AlgorithmInit(foc_algorithm_t *algorithm,
                       const foc_parameters_t *parameters);
void FOC_AlgorithmReset(foc_algorithm_t *algorithm);
void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input);
```

上电或参数变更后调用一次 `FOC_AlgorithmInit()`；停机、故障或切换控制模式时调用 `FOC_AlgorithmReset()`；每个固定采样周期调用一次 `FOC_AlgorithmStep()`，**不要在主循环中调用电流环**。算法函数只处理内存中的结构体，**不能读 GPIO、读 I2C 或写 PWM 寄存器**。结果写入 `algorithm->output`，不返回值。

### 6.5 FOC 中间层端口

```c
typedef struct {
    uint8_t (*is_power_stage_safe)(void *context);
    void (*read_feedback)(void *context, foc_input_t *input);
    void (*write_voltage)(void *context, const foc_output_t *output);
    void (*disable_output)(void *context);
    void *context;
} foc_port_t;
```

**端口是 FOC 与底层的唯一接缝**（依赖倒置）：FOC 层不 `#include` 任何硬件头文件，只调用这四个函数指针。**这正是「业务逻辑不依赖具体外设配置」的标准做法。**

**四个回调应绑定到什么（接线对照表）：**

| 端口回调 | 应绑定到 | 约束 |
| --- | --- | --- |
| `is_power_stage_safe` | `DrvMonitor_IsPowerStageSafeFast()` | **必须只读两个 GPIO、绝不走 I2C**，因为会在 20 kHz 中断里被调用，执行时间必须恒定 |
| `read_feedback` | 电流采样层（⚠ 配置层与启动编排已就绪，注入组正随 `TIM1_CH4` 更新 JDR1/JDR2；但读码与换算未实现）+ 编码器层（❌ 未实现） | 填 ADC 电流（两相，来自 `hadc1` 的 JDR1/JDR2）、编码器角度（MT6835 机械角 × 极对数）、d/q 参考 |
| `write_voltage` | `Tim1Pwm_SetPhaseDuty()` × 3 | 先把三相电压换算为 0~1000 占空比 |
| `disable_output` | `Tim1Pwm_EmergencyStop()` | 清除 TIM1 MOE，安全关断 |

### 6.6 控制器 API 和执行流程

```c
void FOC_ControllerInit(foc_controller_t *controller,
                        const foc_parameters_t *parameters,
                        const foc_port_t *port);
void FOC_ControllerStart(foc_controller_t *controller);
void FOC_ControllerStop(foc_controller_t *controller);
void FOC_ControllerStep(foc_controller_t *controller);
const foc_output_t *FOC_ControllerGetOutput(const foc_controller_t *controller);
```

| 接口 | 行为 | 应被谁调用 |
| --- | --- | --- |
| `ControllerInit` | 清零对象 → 装入端口 → 按参数初始化算法。**任一入参为空时直接返回，且不清零**（避免抹掉调用方已有内容） | 未来的电流环宿主，一次 |
| `ControllerStart` | 只置 `running = 1`。**不触碰硬件**——真正的输出使能由模块 B 的 MOE 决定 | 同上 |
| `ControllerStop` | 先复位算法，再写出零输出并关闭功率级。两个回调都先判空，**允许上层只注册部分端口** | 故障响应、停机流程 |
| `ControllerStep` | 单次电流环迭代，见下 | **必须在 TIM1 采样/比较中断里** |
| `ControllerGetOutput` | 返回最近一次输出的只读指针；控制器为空返回 0 | 遥测、限幅判断 |

推荐执行顺序：

```text
TIM1_CH4 采样时刻（TIM1_CC_IRQHandler，优先级 0，20 kHz）
  └─ FOC_ControllerStep()
       ├─ is_power_stage_safe()       GPIO 快速安全检查，无 I2C
       ├─ read_feedback()             获取 ADC/编码器反馈
       ├─ FOC_AlgorithmStep()         Clarke → Park → PI → 反 Park
       └─ write_voltage()             输出三相 PWM 指令
```

如果安全回调缺失、返回 0，或其他必需回调为空，控制器会复位算法、停止运行并调用 `disable_output()`。因此 **FOC 控制器不会在没有硬件安全门的情况下继续驱动功率级**。

> 🛡 **本轮修复就在这里：** 该分支原先**无条件**调用 `disable_output()`，而进入该分支的原因之一恰恰可能是 `disable_output == 0` → 空指针解引用 → HardFault。现已加判空。前四个条件的短路求值保证了 `is_power_stage_safe()` 不会在指针为空时被调用。

### 6.7 ⚠ 当前状态：端口未接线，未被调用

已核实：`FOC_Controller*` / `FOC_Algorithm*` / `foc_port_t` / `read_feedback` / `write_voltage` 这些符号**只出现在 `FOC/` 目录内部**，`Core/`、`Modules/`、`Tests/` 无一处引用。这一层目前编译进去但未投运。

**接线示例（建议放 L6 或新建 `Modules/FOC_Glue/`，胶水层属业务）：**

```c
static uint8_t foc_is_safe(void *ctx) { return DrvMonitor_IsPowerStageSafeFast(); }
static void    foc_disable(void *ctx) { Tim1Pwm_EmergencyStop(); }

static void foc_read_feedback(void *ctx, foc_input_t *in)
{
    in->phase_a              = /* ← 模块 F：读 hadc1 的 JDR，减偏置后换算成安培 */;
    in->phase_b              = /* ← 同上 */;
    in->electrical_angle_rad = /* ← 编码器层：MT6835 机械角 × 极对数 */;
    in->d_reference          = /* ← 上层速度环输出或直接给定 */;
    in->q_reference          = /* ← 同上 */;
}

static void foc_write_voltage(void *ctx, const foc_output_t *out)
{
    /* alpha/beta 必须先按母线电压归一化；也可把 FOC voltage_limit
       直接定义成归一化标度。驱动层完成零序注入和三相占空比换算。 */
    (void)Tim1Pwm_SetPhaseVoltage(out->alpha_voltage, out->beta_voltage);
}
```

> ⚠ **调用位置的硬性要求：** `FOC_ControllerStep()` 必须在 **20 kHz 的 `TIM1_CC` 中断**里调用（ADC 采样完成后），**不能**放主循环——主循环没有确定性周期，电流环会失效。这也正是模块 A 的分层契约要求「电流环只读快照、不做 I2C」的原因。
>
> **使能前置条件：** 开功率级必须同时满足 `DrvMonitor_GetState() == DRV_STATE_READY` 且 `Tim1Pwm_IsOutputEnabled()`；任一不满足立即 `Tim1Pwm_EmergencyStop()`。

---

## 7. 模块 D：测试验证（`Tests/`）

### 7.1 模块职责与拥有的外设

一句话：**产线或调试验证 MOS 管栅极驱动波形。** 与模块 C 平行、互不依赖，同样调用模块 B 的业务接口。**这是当前固件实际运行的东西。**

**拥有的外设：无自有外设。** 借用模块 B 的 TIM1（经 `Tim1Pwm_*` 业务接口），不直接碰任何寄存器。

| 文件 | 职责 |
| --- | --- |
| `Src/tim1_pwm_verify.c` | 产生单相验证波形 |
| `Inc/measurement_manager.h` | 统一测量配置与运行状态对象（**纯头文件，无 .c**） |
| `Src/mt6835_verify.c` · `Inc/mt6835_verify.h` | MT6835 SPI 通讯验证：角度帧 CRC-8 自洽（链路主判据）+ 传输返回码 + STATUS 报警 |

### 7.2 对上提供的接口

**上层是谁：** L6 `main.c`。

```c
HAL_StatusTypeDef Tim1PwmVerify_Start(measurement_manager_t *manager);
HAL_StatusTypeDef Tim1PwmVerify_Task(measurement_manager_t *manager);
void              Tim1PwmVerify_Stop(measurement_manager_t *manager);
uint8_t           Tim1PwmVerify_IsRunning(const measurement_manager_t *manager);
```

| 接口 | 说明 | 应被谁调用 |
| --- | --- | --- |
| **`Start`** | 装载选中相波形 → **阻塞等待 `startup_delay_ms`（默认 50 ms）自举电容预充电** → 按 `moe_enable` 使能或关断输出 → 校验 MOE 是否真的生效。`manager` 为空或 `module` 不匹配返回 `HAL_ERROR` | `main()`，且仅在 `DrvMonitor_GetState() == DRV_STATE_READY` 时 |
| **`Task`** | **非阻塞的「比对并同步」**：应用待生效的 phase/duty/MOE 修改；输出被关闭时判定测量失败——**这就是清 MOE 结束测试的机制** | `main()` 主循环 |
| **`Stop`** | 急停 + 回中性点 + 清 `moe_applied` + 状态置 IDLE。**任何时候都可安全调用** | L6 / 调试器 / 故障仲裁 |
| `IsRunning` | 1 = 仍在输出波形 | L6 |

> ⚠ `Start()` 内部有一次 `HAL_Delay(startup_delay_ms)` **阻塞延时**，因此**只应在初始化阶段调用**，不要在实时路径里调。

### 7.3 对模块 B 业务接口的使用方式（典型示范）

```c
/* 参数校验：duty 为 0 或 >= TIM1_PWM_DUTY_SCALE 时拒绝启动 */
if ((measurement->duty == 0U) || (measurement->duty >= TIM1_PWM_DUTY_SCALE)) { ... }

/* 三相先清零 */
Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_1, 0U);
Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_2, 0U);
Tim1Pwm_SetPhaseDuty(TIM_CHANNEL_3, 0U);
/* 选中相给目标占空比，其余两相 0% → 低侧导通，相端钳到 PGND */
return Tim1Pwm_SetPhaseDuty(selected_channel, measurement->duty);

Tim1Pwm_EmergencyStop();     /* 关断 */
Tim1Pwm_SetNeutral();        /* 回到 50% 自举刷新 */
Tim1Pwm_EnableOutputs();     /* 使能 */
Tim1Pwm_IsOutputEnabled();   /* 校验使能是否真的生效 */
```

> `duty` 是**原值直接传入** `Tim1Pwm_SetPhaseDuty()`，中间**没有任何换算**。所以 `main.c` 里的 `.duty = 50U` 就是 5%，对应 `CCR1 = 210`。这关系到第 13 节待核对项 A。

### 7.4 统一测量管理对象

**设计意图：** 用一个对象持有「当前选中的测量模块 + 其配置 + 其结果」。未来新增测量项时**加一个 union 成员即可，不要把测试开关和参数散落到 `main.c` 各处**。

```c
typedef enum {
  MEASUREMENT_MODULE_NONE = 0,
  MEASUREMENT_MODULE_MOS_DRIVER      /* 当前唯一实现的测量模块 */
} measurement_module_t;

typedef enum {
  MEASUREMENT_STATE_IDLE = 0, MEASUREMENT_STATE_RUNNING, MEASUREMENT_STATE_FAILED
} measurement_state_t;

typedef enum {
  MOS_DRIVER_PHASE_U = 0, MOS_DRIVER_PHASE_V, MOS_DRIVER_PHASE_W
} mos_driver_phase_t;

typedef struct {
  /* —— 配置项：可在运行途中由调试器或上位机修改 —— */
  mos_driver_phase_t phase;
  uint16_t duty;
  uint8_t  moe_enable;
  uint32_t startup_delay_ms;
  /* —— 运行状态：volatile，由测量模块维护，请勿手改 —— */
  volatile uint8_t moe_applied;
  volatile mos_driver_phase_t phase_applied;
  volatile uint16_t duty_applied;
} mos_driver_measurement_t;

typedef struct {
  measurement_module_t         module;
  volatile measurement_state_t state;
  union { mos_driver_measurement_t mos_driver; } config;
} measurement_manager_t;

extern measurement_manager_t g_measurement_manager;  /* 定义在 main.c */
```

> `*_applied` 三个字段是「已生效值」的影子副本，`Task()` 靠比对 `phase != phase_applied` 判断是否有待应用的修改——这套机制让调试器改字段就能实时生效，无需重新编译。

### 7.5 ⚠ 上电前必读（Runbook）

1. **必须脱开电机、用限流直流母线供电。** 该波形刻意施加非零线电压，用途是探测单个半桥，**不是用来转动电机的**。
2. 上电后先在调试器里确认 `DrvMonitor_GetState() == DRV_STATE_READY`，再做任何事。此时不要接任何承受不了电机保持电流的负载。
3. 探测选中相的栅源电压与其 U/V/W 相端：相端应以 20 kHz、按配置占空比翻转。
4. 确认另外两相相端电压贴近 PGND。
5. 核对寄存器期望值（占空比标度 0..1000，比较值 = duty × 4200 / 1000）：
   - 当前默认 `duty = 50`（5%）→ `TIM1->CCR1 = 210`，`CCR2/3 = 0`
   - 若改为 `duty = 100`（10%）→ `TIM1->CCR1 = 420`
   - `TIM1->ARR` 恒为 4199，运行时 `BDTR.DTG = 84`，`BDTR.MOE = 1`
6. 波形持续输出，直到 MOE 字段被清零或调用 `Tim1PwmVerify_Stop()`。

> 自举电容预充电延时的依据（DS13630 §5.1, §5.2.4）：`MX_TIM1_Init()` 后 TIM1 已在运行且 CCRx=0（三相低侧全导通），低侧 MOSFET 把 OUTx 钳到 PGND，自举电容通过内部二极管充电（`VCC → [内部二极管] → BOOTx → [Cboot] → OUTx ≈ GND`）。STSPIN32G4 内部 VCC buck 从上电到稳定需约 5-10 ms，自举充电再加约 10 ms。**没有这个延时，MOE 首次置起时高侧栅极驱动电压不足，MOSFET 会工作在线性区、Rds(on) 很大，从而触发 VDS 监测保护（DS13630 §5.3）。**

---

## 8. 模块 E：板级支撑（`Core/`）

这个目录装着三类东西：中断向量分发、板级 GPIO/时钟配置、应用入口。它们不属于任何功能模块，但所有模块都依赖它们。

### 8.1 L0 中断向量层（`stm32g4xx_it.c`）

**只做分发，不含任何业务代码。** 业务逻辑通过 HAL 的弱回调进入各模块。

| 中断向量 | 优先级 | 频率 | 派发到 | 当前用途 |
| --- | --- | --- | --- | --- |
| `TIM1_CC_IRQHandler` | **0（最高）** | 20 kHz | `HAL_TIM_OC_DelayElapsedCallback` | **预留**：CH4 采样时刻 → 未来电流环入口 |
| `TIM1_UP_TIM16_IRQHandler` | 1 | 20 kHz | `HAL_TIM_PeriodElapsedCallback` | **预留**：不依赖 ADC 的周期边界。当前仅做一次实例判断后返回 |
| `TIM6_DAC_IRQHandler` | **15（最低）** | 1 s | `HAL_TIM_PeriodElapsedCallback` | 模块 A 的驱动监控心跳 |
| `SysTick_Handler` | 15 | 1 kHz | `HAL_IncTick()` | HAL 时基，`HAL_GetTick()` / `HAL_Delay()` 依赖它 |

**优先级设计意图：**

```text
TIM1_CC (0)   ← 电流环采样时刻，最高，可抢占其他一切
TIM1_UP (1)   ← 周期边界状态机，只被 CC 抢占
TIM6    (15)  ← 心跳，最低，绝不抢占电流环与 PWM/采样中断
SysTick (15)  ← TICK_INT_PRIORITY = 15，同为最低
```

TIM1_CC 与 TIM1_UP 是**唯一允许互相抢占**的一对（CC 抢占 UP）。

> ⚠ **这个优先级方案隐含一条约束：** 电流环（优先级 0）若执行超过 1 ms，SysTick（优先级 15）会被阻塞，`HAL_GetTick()` 停止推进，进而让所有基于 tick 的超时失效（`DrvMonitor` 的 I2C 超时、`Tim1PwmVerify` 的 `HAL_Delay`）。当前 CC 中断体是空的所以无碍，**实现电流环后必须保证其耗时远小于 1 ms**（20 kHz 周期本身就只有 50 µs）。
>
> ⚠ `TIM1_UP` 以 20 kHz 触发但当前无实际工作，每 50 µs 白白花费一次中断进入 + 一次实例比较。这是为电流环预留骨架的刻意代价，实现电流环后应赋予其真实用途或直接关闭该中断。

### 8.2 GPIO 与时钟配置

`MX_GPIO_Init()`（`Core/Src/gpio.c`）开启 GPIOA、GPIOE、GPIOF 时钟并配置模块 A 用到的三个栅极驱动控制脚。引脚宏定义在 **`Core/Inc/gpio.h`**（已从 `main.h` 移出）：

```c
#define GD_WAKE_Pin          GPIO_PIN_7
#define GD_WAKE_GPIO_Port    GPIOE
#define GD_READY_Pin         GPIO_PIN_14
#define GD_READY_GPIO_Port   GPIOE
#define GD_NFAULT_Pin        GPIO_PIN_15
#define GD_NFAULT_GPIO_Port  GPIOE
```

**时钟树**（`main.c` 的 `SystemClock_Config()`）：

```text
HSE 8 MHz ──/2──► 4 MHz VCO 输入 ──×84──► 336 MHz VCO ──/2──► 168 MHz SYSCLK
                                                                    │
                          AHB /1 ──► HCLK 168 MHz ──────────────────┤
                          APB1 /1 ──► PCLK1 168 MHz ──► TIM6 168 MHz
                          APB2 /1 ──► PCLK2 168 MHz ──► TIM1 168 MHz
                          FLASH_LATENCY_4 · VOLTAGE_SCALE1_BOOST
```

> VCO 输入取 4 MHz 是 ST 推荐的最低 PLL 抖动点。
>
> ⚠ **SYSCLK 刻意保持 168 MHz**，因为所有派生时序常量都依赖它：20 kHz TIM1 载波、500 ns 死区、TIM6 的 1 s 心跳（分频 16800）、I2C 时序字 `0x40B285C2`。**改主频会连带破坏上述所有常量**——模块 B 的 `MX_TIM1_Init()` 已加了 `HAL_RCC_GetPCLK2Freq()` 检查来防止这种静默失效。

### 8.3 MSP 层（`stm32g4xx_hal_msp.c`）

**职责：把「外设实例」与「具体硬件」绑定**（时钟使能、引脚复用、NVIC 优先级）。由 HAL 在各 `MX_*_Init()` 内部自动回调，业务层不直接调用。

| 回调 | 触发者 | 服务哪个模块 | 做什么 |
| --- | --- | --- | --- |
| `HAL_MspInit()` | `HAL_Init()` | 全局 | 使能 SYSCFG/PWR 时钟，关闭 UCPD 死电池上拉 |
| `HAL_I2C_MspInit()` | `MX_I2C3_Init()` | A | 使能 GPIOC/I2C3 时钟，配 PC8/PC9 为 `AF_OD` + `PULLUP` + `VERY_HIGH` + `AF8_I2C3` |
| `HAL_I2C_MspDeInit()` | 反初始化 | A | 关时钟，`HAL_GPIO_DeInit(GPIOC, PIN_8\|PIN_9)` |
| `HAL_TIM_Base_MspInit()` | `MX_TIM6_Init()` | A | 使能 TIM6 时钟，设 `TIM6_DAC_IRQn` 优先级 **(15,0)** 并使能 |
| `HAL_TIM_Base_MspDeInit()` | 反初始化 | A | 关时钟与 NVIC |
| `HAL_TIM_PWM_MspInit()` | `MX_TIM1_Init()` | B | 使能 TIM1 时钟，设 `TIM1_CC_IRQn` **(0,0)**、`TIM1_UP_TIM16_IRQn` **(1,0)** 并使能 |
| `HAL_TIM_PWM_MspDeInit()` | 反初始化 | B | 关时钟与两个 NVIC |
| `HAL_TIM_MspPostInit()` | `MX_TIM1_Init()` 末尾 | B | 使能 GPIOE 时钟，配 PE8~PE13 为 `AF_PP` + `NOPULL` + `HIGH` + `AF2_TIM1` |
| `HAL_OPAMP_MspInit()` | `MX_OPAMP1_Init()` / `MX_OPAMP3_Init()` | F | 配 PA1/PA2/PA3 与 PB0/PB1/PB2 为 `GPIO_MODE_ANALOG` + `NOPULL`。⚠ PB2 兼作 BOOT1，必须显式配置 |
| `HAL_OPAMP_MspDeInit()` | 反初始化 | F | `HAL_GPIO_DeInit()` 释放上述六个模拟脚 |
| `HAL_ADC_MspInit()` | `MX_ADC1_Init()` | F | 使能 `ADC12` 时钟。**刻意不配 ADC 中断**：注入组由 `TIM1_CC` ISR（优先级 0）消费，再加一个向量只会平白增加延迟 |
| `HAL_ADC_MspDeInit()` | 反初始化 | F | 关 `ADC12` 时钟 |

### 8.4 L6 应用入口层（`main.c`）

**职责：** 只做三件事——按正确顺序初始化、编排主循环后台任务、持有全局配置对象。**不含任何算法或器件细节。**

```c
HAL_Init();
SystemClock_Config();   /* 必须最先，后续所有时序常量依赖它 */
MX_GPIO_Init();         /* 模块 A 的控制脚 */
if (MX_I2C3_Init()  != HAL_OK) Error_Handler();
if (MX_TIM6_Init()  != HAL_OK) Error_Handler();
if (MX_TIM1_Init()  != HAL_OK) Error_Handler();   /* MOE 仍为 0 */
if (MX_OPAMP1_Init() != HAL_OK) Error_Handler();  /* 模块 F：U 相电流放大 */
if (MX_OPAMP3_Init() != HAL_OK) Error_Handler();  /* 模块 F：W 相电流放大 */
if (MX_ADC1_Init(1U) != HAL_OK) Error_Handler();  /* 软件触发，未挂 TIM1_CH4 */
if (DrvMonitor_Init() != HAL_OK) Error_Handler(); /* 清锁存故障 + 启动心跳 */
```

> **两个运放必须先于 ADC 初始化。** ADC1 采的正是运放的输出脚（PA2 = IN3，PB1 = IN12），运放没起来时那两个节点上没有有效信号，先校准 ADC 只会把噪声当成基线记下来。
>
> **`MX_ADC1_Init(0U)` 传 0 = 由 `TIM1_CH4` 上升沿触发**（非 0 才是软件触发，留给上电偏置标定用）。随后的 `CurrentSampling_Start()` 按「运放先启动、ADC 后武装触发器」的顺序接通信号链。
>
> ⚠ 代价是：`TIM1_CH4` 已在 20 kHz 运行，每个载波周期都会跑一轮 11.43 µs 的过采样转换，而目前还没有任何消费者读这些数据。这个耗时与 1 µs 触发提前量的余量问题见第 9.3 节与第 13 节待核对项 I。

顺序有意义：GPIO/I2C/TIM6 必须先于依赖它们的业务模块。`SystemClock_Config()` 是 `main.c` 内**唯一**由业务文件实现的配置函数（CubeMX 生成，在 `USER CODE` 块外）。

bring-up 与主循环：

```c
/* 只有栅极驱动报告 READY 后才启动测量 */
if (DrvMonitor_GetState() == DRV_STATE_READY) {
    (void)Tim1PwmVerify_Start(&g_measurement_manager);
}

while (1)
{
    DrvMonitor_Task();      /* 心跳未到时立即返回，每轮零成本；全工程 I2C 都在这里 */

    /* 【本轮新增】L6 跨层仲裁：驱动报告非 READY 时关断功率级。
       放在 L6 而不是 L3：监控层不得调用三相驱动层（下层不能依赖上层），
       跨层仲裁本来就是应用入口层的职责。
       只在 IsRunning 为真时触发，每轮最多关断一次；Stop 会把状态置为 IDLE。
       恢复输出必须显式重新 Start，不会自动重启。 */
    if ((Tim1PwmVerify_IsRunning(&g_measurement_manager) != 0U) &&
        (DrvMonitor_GetState() != DRV_STATE_READY)) {
        Tim1PwmVerify_Stop(&g_measurement_manager);
    }

    (void)Tim1PwmVerify_Task(&g_measurement_manager);

    /* TODO: 协议栈（CAN/UART）、参数存储、温度记录、上位机遥测 …… 都属于这一层。 */
}
```

**实时 FOC 不应放进这个循环**，而应由 TIM1 采样/比较中断驱动。主循环中的监控任务没有 tick 时立即返回；有 tick 时才做 I2C 访问。

L6 定义的、供下层使用的对象：

```c
measurement_manager_t g_measurement_manager = {
  .module = MEASUREMENT_MODULE_MOS_DRIVER,
  .state  = MEASUREMENT_STATE_IDLE,
  .config.mos_driver = {
    .phase            = MOS_DRIVER_PHASE_U,
    .duty             = 50U,   /* 标度 0..1000，故 50 表示 5 %。⚠ 见 13.A */
    .moe_enable       = 1U,    /* 预充电延时结束后自动使能波形输出 */
    .startup_delay_ms = 50U,   /* 等 VCC 与栅极驱动自举电容稳定 */
    .moe_applied      = 0U,
    .phase_applied    = MOS_DRIVER_PHASE_U,
    .duty_applied     = 0U
  }
};
```

> 刻意做成全局对象，以便在调试器中可见、且能在测量运行途中修改。

---

## 9. 模块 F：电流采样（`Modules/Current_Sampling/`）— ⚠ 配置层与启动编排已完成，业务层未实现

### 9.1 模块职责与拥有的外设资源

一句话：**用 STM32G431VB 的片内运放把两路相电流采样信号放大，再用 ADC1 注入组数字化，交给电流环。**

本轮实现的是**配置层（L1）+ 启动编排**：把运放和 ADC 准备好、校准好、按正确顺序接通信号链，然后交出句柄。**业务层（L4）尚未实现**——没有偏置标定、没有原始码到安培的换算、没有供 `foc_port_t.read_feedback` 调用的接口。

模块采用第 12 节推荐的「配置/业务分文件」范式：

```text
Modules/Current_Sampling/
├── Inc/adc1_current_sampling.h   ← L1：MX_*_Init() 与 CurrentSampling_Start() 声明
│                                      + 句柄 extern + 硬件常量
└── Src/adc1_current_sampling.c   ← L1 实现
    （业务文件待建：Inc/current_sampling.h + Src/current_sampling.c）
```

| 外设 | 配置位置 | 作用 | 导出给上层的东西 |
| --- | --- | --- | --- |
| **OPAMP1**（片内）<br>PA1 / PA3 / PA2 | `Src/adc1_current_sampling.c` 的 `MX_OPAMP1_Init()` | 独立跟随模式放大一路相电流信号，输出到 PA2 | `extern OPAMP_HandleTypeDef hopamp1`<br>`HAL_StatusTypeDef MX_OPAMP1_Init(void)` |
| **OPAMP3**（片内）<br>PB0 / PB2 / PB1 | 同上，`MX_OPAMP3_Init()` | 独立跟随模式放大另一路，输出到 PB1 | `extern OPAMP_HandleTypeDef hopamp3`<br>`HAL_StatusTypeDef MX_OPAMP3_Init(void)` |
| **ADC1**（注入组）<br>IN3 = PA2<br>IN12 = PB1 | 同上，`MX_ADC1_Init()` | 把两路运放输出数字化，16 倍过采样，由 TIM1_CH4 触发；含上电自校准 | `extern ADC_HandleTypeDef hadc1`<br>`HAL_StatusTypeDef MX_ADC1_Init(uint8_t)` |
| 信号链启动 | 同上，`CurrentSampling_Start()` | 按「运放先启动、ADC 后武装触发器」的顺序接通信号 | `HAL_StatusTypeDef CurrentSampling_Start(void)` |
| 六个模拟脚的 GPIO | `Core/Src/stm32g4xx_hal_msp.c` 的 `HAL_OPAMP_MspInit()` | 配为 `GPIO_MODE_ANALOG` + `NOPULL` | 无（HAL 自动回调） |
| ADC12 时钟 | 同上，`HAL_ADC_MspInit()` | 使能 `ADC12` 内核时钟 | 无 |

**为什么只采两相：** FOC 的 Clarke 变换只需要 `ia` / `ib`，第三相由 `ia + ib + ic = 0` 推出（见第 6.3 节）。这是全工程里**算法需求反向约束配置层**的唯一例子。

**为什么用注入组而不是规则组：** 注入组有独立的 `JDR1`~`JDR4` 数据寄存器，不会被规则组的 DMA 流覆盖；且注入转换可以由 `TIM1_CH4` 精确触发，保证采样点与 PWM 载波同步。电流环要的就是这个确定性。

**为什么两路都走外部引脚而不用运放内部通道：**

| 运放 | ADC1 上有内部通道吗 | 结论 |
| --- | --- | --- |
| OPAMP3 | ❌ 没有。`VOPAMP3` 只挂在 `ADC2_IN18` / `ADC3_IN13` | PB1 = `ADC1_IN12` 是**唯一**可选路径 |
| OPAMP1 | ✅ 有，`ADC_CHANNEL_VOPAMP1` = `ADC1_IN13` | 参考配置取外部 PA2 = IN3，本文件与之一致 |

### 9.2 L1 配置接口

```c
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp3;
extern ADC_HandleTypeDef   hadc1;

extern volatile uint32_t ia_offsets;   /* 零电流偏置，⚠ 当前无代码填充，恒为 0 */
extern volatile uint32_t ib_offsets;

HAL_StatusTypeDef MX_OPAMP1_Init(void);
HAL_StatusTypeDef MX_OPAMP3_Init(void);
HAL_StatusTypeDef MX_ADC1_Init(uint8_t software_trigger_flag);
HAL_StatusTypeDef CurrentSampling_Start(void);
```

四个函数都遵循本工程 L1 层的既定约定：**不含 `main.h`、不调 `Error_Handler()`，失败一律返回 `HAL_ERROR` 交给 `main.c` 处理**。

> 🔑 **这里与用户提供的参考配置有一处刻意差异。** 参考配置返回 `void` 且失败时直接调 `Error_Handler()`，那会让 L1 配置层反向依赖 L6 的入口文件，违反第 2 节的分层约束（`gpio.c` / `i2c3_bus.c` / `tim6_heartbeat.c` / `tim1_pwm.c` 都遵守「返回状态码、不碰 `Error_Handler()`」）。本文件已改写为返回 `HAL_StatusTypeDef`，由 `main.c` 决定如何处置。

**`MX_ADC1_Init()` 的关键配置及其理由：**

| 配置项 | 取值 | 为什么 |
| --- | --- | --- |
| `ClockPrescaler` | `ADC_CLOCK_SYNC_PCLK_DIV4` | ADC12 内核时钟 168 MHz → ADC 时钟 42 MHz（G431 同步模式上限 60 MHz，合规） |
| `InjectedNbrOfConversion` | 2 | rank 1 = `ADC_CHANNEL_3`(PA2)，rank 2 = `ADC_CHANNEL_12`(PB1) |
| `InjectedSamplingTime` | 2.5 cycles | 全表最短档。驱动源是片内运放输出、源阻抗低，短采样足够；⚠ 但与过采样叠加后的总耗时见 9.3 |
| `InjecOversamplingMode` | `ENABLE`，比率 16，右移 `NONE` | 硬件累加 16 次转换后写 JDR 做噪声平均；**不移位**，所以读出的是 16 位累加值 |
| `EOCSelection` | `ADC_EOC_SEQ_CONV` | 整个序列结束才置 EOC，因此读 JDR 时两个通道的数据必然同时有效，不会读到半更新序列 |
| `Overrun` | `ADC_OVR_DATA_PRESERVED` | 溢出时保留旧数据。电流环宁可拿到略旧的一次采样，也不要撕裂的数据 |
| `QueueInjectedContext` | `DISABLE` | 每个 TIM1_CH4 触发点都重新开跑固定序列，适合 20 kHz 等间隔触发 |
| `OversamplingMode` | `DISABLE` | ⚠ 这一项关的是**规则组**过采样，与注入组的 `InjecOversamplingMode` 互不影响，别误读 |
| `DataAlign` | `ADC_DATAALIGN_LEFT` | ⚠ **该赋值被硬件忽略**：过采样使能时 ALIGN 位不起作用，RM0440 §21.4.26 明确「过采样模式下不支持左对齐」。实际是右对齐，见待核对项 K |
| `multimode.Mode` | `ADC_MODE_INDEPENDENT` | 不用 ADC1/ADC2 双模，两相由同一注入序列顺序转换 |

**`software_trigger_flag` 参数的含义：**

| 取值 | 触发源 | 用途 | 当前 `main.c` |
| --- | --- | --- | --- |
| 非 0 | `ADC_INJECTED_SOFTWARE_START` | 上电偏置标定。摘除 TIM1_CH4 触发，标定不受 PWM 时序干扰 | — |
| 0 | `ADC_EXTERNALTRIGINJEC_T1_CC4` 上升沿 | 运行时的电流环，采样点跟随载波 | ✅ **传的是 `0U`** |

> 🔑 边沿参数 `ADC_EXTERNALTRIGINJECCONV_EDGE_RISING` 在两个分支下都赋了值，这是安全的：HAL 在触发源为软件启动时会**跳过** `JEXTEN` 的写入（`stm32g4xx_hal_adc_ex.c` 里 `ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START` 的判断，单通道与多通道两个分支都有），所以软件触发模式下这个赋值根本不落到寄存器。
>
> 🔑 **校准顺序有硬性前提：** `HAL_ADCEx_Calibration_Start()` 要求 ADC 处于关闭状态，所以前面显式调了一次 `HAL_ADC_Stop()`。而 `HAL_ADC_Init()` 已经自动完成「退出深掉电模式 → 使能内部稳压器 → 忙等稳压」，所以此处**不需要**额外延时。
>
> 🔑 **配置与启动分离**，与模块 B 的 `MX_TIM1_Init()` 刻意不清 MOE 是同一条分界原则：`MX_ADC1_Init()` 只配置和校准，`CurrentSampling_Start()` 才启动运放并武装注入触发器。

**`CurrentSampling_Start()` 的顺序为什么重要：** 先 `HAL_OPAMP_Start()` 两路运放，再 `HAL_ADCEx_InjectedStart()`。反过来的话，ADC 会在运放尚未建立输出时就开始转换，把无效电平当成采样值。硬件触发模式下 `HAL_ADCEx_InjectedStart()` 只负责使能 ADC 并武装触发器，后续转换由 `TIM1_CH4` 自动发起，**不需要在 ISR 里手工启动**。

**导出的硬件常量：**

```c
ADC1_CS_ADC_CLOCK_HZ        42000000  /* 168 MHz / 4，同步模式 */
ADC1_CS_CYCLES_PER_CONV             15  /* 采样 2.5 + 逐次逼近 12.5 */
ADC1_CS_OVERSAMPLING_RATIO          16
ADC1_CS_OVERSAMPLED_CONV_NS       5714  /* 单通道一轮过采样 = 5.71 µs */
ADC1_CS_SEQUENCE_CONV_NS         11428  /* 两通道序列 = 11.43 µs */
ADC1_CS_TRIGGER_LEAD_NS           1000  /* 记录模块 B 的 CH4 提前量，仅备注 */
```

### 9.3 时序与数据换算（改参数必看）

**时序：**

```text
ADC 时钟            = 168 MHz / 4                      = 42 MHz
单次转换            = (2.5 + 12.5) cycles / 42 MHz     = 357 ns
单通道过采样 16 倍   = 357 ns x 16                      = 5.71 µs
注入序列两个通道     = 5.71 µs x 2                      = 11.43 µs
PWM 周期（20 kHz）                                      = 50 µs
TIM1_CH4 触发提前量（模块 B）                            = 1 µs
```

⚠ **序列耗时是触发提前量的 11 倍**，这是个必须决策的问题，详见第 13 节待核对项 I。

**数据换算：**

| 量 | 值 | 说明 |
| --- | --- | --- |
| 12 位满量程 | 4095 | `ADC_RESOLUTION_12B` |
| JDR 满量程 | **65520** | 4095 × 16，过采样累加且**未右移**，占满 16 位 |
| 12 位等效码 | `JDR / 16` | 业务层换算的第一步 |
| 数据对齐 | **右对齐** | ⚠ 尽管配置写的是 `ADC_DATAALIGN_LEFT`，过采样使能时该位被硬件忽略（RM0440 §21.4.26）。**若误按左对齐理解会多移一次位，得到相差 16 倍的电流值** |
| 零点 | `ia_offsets` / `ib_offsets` | 功率级关断时采到的运放中点偏置。**当前恒为 0**（业务层未实现） |

> 换算成安培还需要采样电阻阻值与运放增益（由外部电阻网络决定）。这两个参数属于业务层，且相别归属尚未实测确认（待核对项 J），因此**没有写死进配置层代码**。

### 9.4 尚未实现的业务层

配置层交出了句柄、信号链也已接通，注入组正随载波持续更新 `JDR1` / `JDR2`。但**从原始码到可用电流值之间的所有环节都还不存在**：

```text
待实现（建议放 Modules/Current_Sampling/Src/current_sampling.c）
├── 零电流偏置标定      上电、MOE=0 时采若干轮取均值，写入 ia_offsets / ib_offsets
├── 原始码读取           HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1/2)
├── 单位换算             JDR → 减偏置 → /16 → ×(VREF / 4095) → /(Rshunt × Gain) → 安培
└── 供 FOC 消费的接口    填 foc_input_t.phase_a / phase_b（见 6.5 节 read_feedback 行）
```

**上层是谁：** 未来的 FOC 端口胶水层（经 `foc_port_t.read_feedback`），以及 L6 `main.c`（标定编排）。

**读取位置：** 必须在 `TIM1_CC` ISR（优先级 0）里，且要注意 11.43 µs 的序列耗时意味着**中断入口立即读 JDR 拿到的是滞后一个周期的数据**（见待核对项 I）。

> ⚠ `ia_offsets` / `ib_offsets` 目前被链接器当作未使用符号移除（构建日志里可见 `Removing adc1_current_sampling.o(.bss.ia_offsets), (4 bytes)`），这是刻意的——它们是给业务层预留的落点，定义在配置层只是为了让调试器可见、并明确标定结果的归属。实现业务层后会自然被保留。

---

## 10. 跨模块调用关系速查

### 10.1 「我要做 X，该调用谁？」

| 需求 | 调用接口 | 模块 | 层 | 前置条件 |
| --- | --- | --- | --- | --- |
| 判断能否开功率级 | `DrvMonitor_GetState() == DRV_STATE_READY` | A | L3 | `DrvMonitor_Init()` 已执行 |
| 在中断里判断能否开关 | `DrvMonitor_IsPowerStageSafeFast()` | A | L3 | 无（恒定耗时，不碰 I2C） |
| 读驱动详细状态 | `DrvMonitor_GetSnapshot(&snap)` | A | L3 | 任何上下文安全 |
| 清除锁存故障 | `DrvMonitor_PostRequest(DRV_REQ_CLEAR_FAULT)` | A | L3 | 非阻塞，下次 1 s 任务生效 |
| 唤醒 / 进待机 | `PostRequest(DRV_REQ_WAKE)` / `RequestStandby(en)` | A | L3 | 同上 |
| 读写 STSPIN32G4 寄存器 | `STSPIN32G4_Read/WriteRegister()` | A | L2 | **只应由 L3 调用** |
| 设某相占空比 | `Tim1Pwm_SetPhaseDuty(channel, duty)` | B | L4 | `MX_TIM1_Init()` 已执行 |
| 接通功率级 | `Tim1Pwm_SetNeutral()` → `Tim1Pwm_EnableOutputs()` | B | L4 | 状态为 READY |
| 立刻断功率级 | `Tim1Pwm_EmergencyStop()` | B | L4 | 无（任何时候可调） |
| 查 MOE 是否真的生效 | `Tim1Pwm_IsOutputEnabled()` | B | L4 | 无 |
| 跑验证波形 | `Tim1PwmVerify_Start/Task/Stop()` | D | L5 | 脱开电机 + 限流母线 |
| 跑电流环 | `FOC_ControllerStep()` | C | L5 | ⚠ 需先实现四个端口回调；须放 `TIM1_CC` ISR |
| 初始化 I2C / TIM6 / GPIO | `MX_I2C3_Init()` / `MX_TIM6_Init()` / `MX_GPIO_Init()` | A / E | L1 | **只由 `main()` 各调一次** |
| 初始化 TIM1 PWM | `MX_TIM1_Init()` | B | L1/L4 | 同上 |
| 初始化电流采样运放 | `MX_OPAMP1_Init()` / `MX_OPAMP3_Init()` | F | L1 | **只由 `main()` 各调一次，且必须先于 `MX_ADC1_Init()`** |
| 初始化 ADC1 注入组 | `MX_ADC1_Init(software_trigger_flag)` | F | L1 | 同上；`0U` = TIM1_CH4 触发，非 0 = 软件触发（标定用） |
| 接通电流采样信号链 | `CurrentSampling_Start()` | F | L1 | 必须在三个 `MX_*_Init()` 全部成功之后调用一次 |
| 读相电流原始码 | `HAL_ADCEx_InjectedGetValue(&hadc1, rank)` | F | L1 | ⚠ 业务层未实现；须在 `TIM1_CC` ISR 内，且注意 9.3 的时序滞后 |

### 10.2 依赖矩阵（✅ = 允许调用）

| 调用方 ↓ / 被调方 → | L1 配置 | L2 器件 | L3 监控 | L4 三相 | L5 FOC | L5 测试 |
| --- | --- | --- | --- | --- | --- | --- |
| **L2 器件驱动**（A） | ✅ 句柄+宏 | — | ❌ | ❌ | ❌ | ❌ |
| **L3 监控服务**（A） | ✅ 句柄 | ✅ | — | ❌ | ❌ | ❌ |
| **L4 三相驱动**（B） | ✅ 句柄 | ❌ | ❌ | — | ❌ | ❌ |
| **L5 FOC**（C） | ❌ | ❌ | 只读快照 / 快速安全门 | 经端口回调 | — | ❌ |
| **L5 测试**（D） | ❌ | ❌ | 只读状态 | ✅ | ❌ | — |
| **L1 配置**（F 电流采样） | — | ❌ | ❌ | ❌ | ❌ | ❌ |
| **L6 入口**（E） | ✅ 全部 | ❌ 不直接调 | ✅ | ✅ | ✅ | ✅ |

> 模块 F 目前**只有 L1 配置层**：它不调用任何其他层，只被 L6 `main.c` 在初始化阶段调用。将来的业务层（L4）可以读 L1 句柄，但同样不得反向调用 L5/L6。

### 10.3 禁止的调用方向

- ❌ **配置层调用业务层**（HAL 的 MSP 回调除外，它本身属配置层）
- ❌ **L2/L3 调用 L4/L5/L6**（下层不得依赖上层）
- ❌ **电流环/速度环调用 L3 的慢路径**（只能读快照或用 `IsPowerStageSafeFast()`）
- ❌ **任何实时 ISR 里做 I2C、`HAL_Delay()`、无界循环或日志输出**
- ❌ **除 L3 外的模块直接调用 `STSPIN32G4_*`**（I2C 总线必须由单一持有者串行化）
- ❌ **直接读 `s_mon`**（用 `DrvMonitor_GetSnapshot()` 取无撕裂副本）
- ❌ **FOC 层 `#include` 硬件头文件**（必须经 `foc_port_t` 回调）
- ❌ **重复定义 `HAL_TIM_PeriodElapsedCallback`**（已被 `drv_monitor.c` 占用；`USE_HAL_TIM_REGISTER_CALLBACKS = 0` 意味着全工程只能有一个定义）
- ❌ **新增业务模块的头文件从 `main.h` 借用宏**（头文件只暴露业务接口）
- ❌ **在注入序列完成前读 `JDR`**（`EOCSelection = ADC_EOC_SEQ_CONV` 只保证不读到半更新序列，不保证数据已就绪；序列耗时见 9.3）
- ❌ **除 L1 配置层与未来业务层外的模块直接操作 `hadc1` / `hopamp1` / `hopamp3`**（采样外设必须单一持有者）
- ❌ **把过采样后的 JDR 当作 12 位左对齐值处理**（实际是右对齐的 16 位累加值，见待核对项 K）

### 10.4 上下文（执行环境）契约

同一组接口在不同上下文中可用性不同，这是本工程最重要的约束之一。

| 上下文 | 周期 | 允许的调用 | 禁止的调用 |
| --- | --- | --- | --- |
| `TIM1_CC` ISR（优先级 0） | 20 kHz（50 µs） | `Tim1Pwm_SetPhaseDuty()`（单次写，安全）、`DrvMonitor_IsPowerStageSafeFast()`、`DrvMonitor_GetSnapshot()`、`FOC_ControllerStep()` | **任何 I2C**、`HAL_Delay()`、`DrvMonitor_Task()`、长循环、日志 |
| `TIM1_UP` ISR（优先级 1） | 20 kHz | 未来周期边界状态机 | 同上 |
| `TIM6_DAC` ISR（优先级 15） | 1 s | 置 tick 标志 | **任何 I2C**、状态机推进、`HAL_Delay()` |
| 主循环（无优先级） | 无截止 | `DrvMonitor_Task()`（含全部 I2C）、`Tim1PwmVerify_Task()`、`DrvMonitor_PostRequest()`、跨层仲裁 | 长时间阻塞（会拖慢监控心跳） |
| 初始化阶段 | 一次性 | 阻塞式 `STSPIN32G4_*`、`DrvMonitor_Init()`、`Tim1PwmVerify_Start()`（含 50 ms 延时）、`MX_OPAMP1/3_Init()`、`MX_ADC1_Init()`（含阻塞式自校准）、`CurrentSampling_Start()` | —（此时无实时任务，阻塞无害） |

> ⚠ **模块 F 的注入组刻意不占用独立中断向量。** `HAL_ADC_MspInit()` 里没有配置任何 ADC NVIC——注入组由模块 B 已经拥有的 `TIM1_CC` ISR（优先级 0）消费。再加一个向量只会平白增加中断延迟，还会与第 8.1 节的优先级方案冲突。

---

## 11. 编译、烧录与调试

### 编译

1. Keil MDK 打开 `software/servo_v1/MDK-ARM/servo_v1.uvprojx`
2. 目标选 `servo_v1`，**Build Target**（F7）
3. 产物：`MDK-ARM/out/servo_v1/servo_v1/servo_v1.axf`

| 产物 | 用途 |
| --- | --- |
| `servo_v1.axf` | 可执行固件，烧录用 |
| `servo_v1.axf.map` | 链接映射，查 Flash/RAM 占用与符号地址 |
| `servo_v1.htm` | 构建报告，各段大小汇总 |
| `compile_commands.json` | 编译命令数据库，供 clangd 等索引工具使用 |

> `out/` 与 `tmp/` 均为构建生成物，可安全删除后重新编译。

### 烧录 / 调试

Keil 中 **Debug**（Ctrl+F5）下载并调试；调试器配置在 `MDK-ARM/DebugConfig/`，对应芯片 `STM32G431VBTx`。

调试器 Watch 窗口可直接观察这些刻意导出的全局对象：

| 对象 | 模块 / 层 | 观察什么 |
| --- | --- | --- |
| `s_mon` | A / L3 | 驱动状态机、STATUS 寄存器、锁存故障、READY/nFAULT 电平、连续通信失败次数 |
| `htim1` | B / L1 | 定时器句柄；配合 `TIM1->CCR1/2/3`、`ARR`、`BDTR` 看 PWM 与 MOE |
| `hadc1` / `hopamp1` / `hopamp3` | F / L1 | 采样外设句柄；配合 `ADC1->JDR1`、`JDR2` 看注入组原始码，`ADC1->JSQR` 看触发源与序列 |
| `ia_offsets` / `ib_offsets` | F / L1 | 零电流偏置标定值的预留落点。**当前无任何代码填充，恒为 0** |
| `g_measurement_manager` | E / L6 | 测量配置与运行状态，**可在运行途中修改**，`Task()` 会自动同步 |

### 修改外设配置

1. 优先修改 CubeMX 配置（`servo_v1.ioc`）并重新 **Generate Code**
2. ⚠ 重新生成会覆盖 CubeMX 管辖的文件——**自研逻辑必须写在 `USER CODE BEGIN/END` 块内**
3. 被管辖的包括 `Core/` 下各文件，以及 `Modules/Driver_Communication/` 下的 `i2c3_bus.c` / `tim6_heartbeat.c`（见 13.C 的重复生成风险）。`Modules/Current_Sampling/` 下的文件**当前不在 .ioc 里**（ADC1 / OPAMP1 / OPAMP3 未通过 CubeMX 启用），所以它们是纯手写、不受重新生成影响
4. `FOC/`、`Tests/`、`stspin32g4.c`、`drv_monitor.c`、`tim1_pwm.c` 的 `USER CODE BEGIN 2` 段不在管辖范围。`adc1_current_sampling.c/h` 整个文件都不在管辖范围（但保留了同名 `USER CODE` 块，便于将来接入 CubeMX）
5. 检查 L1 返回值和 Keil 分组是否仍一致

**改业务逻辑则完全不需要碰 CubeMX。**

---

## 12. 当前实现进度与下一步

| 模块 | 层 | 内容 | 状态 |
| --- | --- | --- | --- |
| E 板级支撑 | L0·L1·L6 | GPIO / 中断向量 / MSP / 时钟树 / 入口编排 | ✅ 完成 |
| A 驱动芯片通讯 | L1 | I2C3、TIM6 配置 | ✅ 完成 |
| A 驱动芯片通讯 | L2 | STSPIN32G4 寄存器驱动 | ✅ 完成 |
| A 驱动芯片通讯 | L3 | 驱动监控状态机 + 故障锁存 + 无撕裂快照 | ✅ 完成 |
| B 三相驱动 | L1+L4 | 三相 20 kHz 互补 PWM + MOE 控制 | ✅ 完成 |
| D 测试验证 | L5 | MOS 管驱动验证波形 | ✅ 完成，**当前固件运行的就是这个** |
| C 控制算法 | L5 | FOC 电流环算法 + 端口调度 | ⚠ 算法已实现，**端口未接线，未被调用** |
| F 电流采样 | L1 | OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组两通道 + 16 倍过采样 + 上电自校准 | ✅ 完成 |
| F 电流采样 | L1 | 信号链启动编排 `CurrentSampling_Start()`（运放先启动、ADC 后武装触发器） | ✅ 完成 |
| F 电流采样 | L4 | 偏置标定 + 原始码到安培换算 + 供 `read_feedback` 使用 | ❌ 未实现 |
| G 编码器 | L1 | SPI1 配置：PA4 软件 CSN / PA5 SCK / PA6 MISO / PA7 MOSI，模式 3，10.5 MHz | ✅ 完成 |
| G 编码器 | L2 | MT6835 24 bit 帧读写 + 角度帧 CRC-8 + STATUS 解码 | ✅ 完成 |
| D 测试验证 | L5 | MT6835 SPI 通讯验证（`mt6835_verify`，主循环非阻塞推进） | ✅ 完成 |
| — | — | 对外协议栈（CAN/UART） | ❌ 未实现（原理图已有通讯电路） |
| — | — | 速度环 / 位置环 | ❌ 未实现 |
| — | — | 参数存储 / 温度记录 / 遥测 | ❌ 未实现 |

**固件当前行为：** 上电 → 配置外设（MOE=0，功率级未接通）→ 配置两个片内运放与 ADC1 注入组并自校准 → `CurrentSampling_Start()` 启动运放、武装 `TIM1_CH4` 触发 → `DrvMonitor_Init()` 清除栅极驱动锁存故障并启动 1 s 心跳 → 若驱动 READY 则等 50 ms 自举预充电后输出 U 相 5% 验证波形（其余两相低侧导通钳到 PGND）→ 主循环跑驱动监控心跳 + 测量同步 + 故障仲裁。**不会转动电机；注入组虽已随载波持续转换并更新 JDR1/JDR2，但没有任何代码读取或换算这些数据。**

**推荐的下一步实现顺序**（按依赖关系）：

1. ~~**电流采样层（配置部分）**~~ → ✅ **本轮已完成**，见第 9 节。ADC1 + 两通道（算法只需两相）+ TIM1_CH4 触发已就绪
2. **电流采样层（业务部分）** → 填 `Modules/Current_Sampling/` 的业务文件。零电流偏置标定（写入 `ia_offsets` / `ib_offsets`）+ JDR 原始码 → 安培换算，供 `foc_port_t.read_feedback` 消费。**上电前必须先解决第 13 节待核对项 I 的时序余量与 J 的相别归属**
3. ~~**编码器层（配置 + L2）**~~ → ✅ 本轮完成。剩余业务部分：机械角 × 极对数 → 电角度，接入 `foc_port_t.read_feedback`
4. **FOC 端口接线** → 按 6.5 节对照表实现四个回调，把上面两层与模块 B 连起来
5. **电流环上电** → 在 `TIM1_CC` ISR（优先级 0）中调 `FOC_ControllerStep()`，先低压限流空载验证。**务必保证单次迭代远小于 1 ms**（见 8.1 节的 SysTick 阻塞约束）
6. **速度环 / 位置环** → 主循环或独立定时器，输出 `q_reference`
7. **协议栈** → CAN/UART。配置部分：外设初始化；业务部分：帧解析，接入 `DrvMonitor_PostRequest()` 做远程故障复位与参数下发

> 建议新建模块**直接采用「配置/业务分文件」范式**，避免重蹈 `tim1_pwm.c` 混居的覆辙：
> ```text
> Modules/<模块名>/
> ├── Inc/  <外设>_config.h    ← 配置：MX_*_Init() 声明 + 句柄 extern + 硬件常量
> │         <模块>.h           ← 业务：模块公开 API
> └── Src/  <外设>_config.c    ← 配置
>           <模块>.c           ← 业务
> ```
> 模块 A 已经这样做了（`i2c3_bus.c` / `tim6_heartbeat.c` 与 `stspin32g4.c` / `drv_monitor.c` 分开），可作为范例。

---

## 13. 待核对项（需决策）

以下是审查中发现的**不一致或需权衡之处**，未擅自改动：

### A. `main.c` 默认占空比：5% 还是 10%？（建议优先确认）

三处描述原本互相矛盾：

| 位置 | 原文 | 换算 |
| --- | --- | --- |
| `main.c` `.duty = 50U` 的原注释 | 「10 % high-side PWM」 | — |
| 实际标度 | `TIM1_PWM_DUTY_SCALE = 1000` | 50 → **5 %** |
| `tim1_pwm_verify.c` | `duty` **原值直接**传给 `Tim1Pwm_SetPhaseDuty()`，无换算 | 确认是 5 % |
| `tim1_pwm_verify.h` 原 Runbook | 「默认 U/10% 时 `CCR1` 是 420」 | 420 对应 `duty = 100` |

**已做的处理：** 改注释使其与代码一致（都写 5 %，Runbook 同时列出 5% 与 10% 两种期望值），**没有改 `.duty` 的数值**——改占空比会改变功率级实际输出，属于硬件行为决策。

**需要定：** 如果原本设计意图是 10 %，应把 `.duty` 改成 `100U`；如果 5 % 才是想要的（更保守，测试电流更小），那现在就是对的。

### B. `gpio.c` 里配的三个控制脚归属模块 A，却在 `Core/`

PE7/PE14/PE15 是驱动芯片的 WAKE/READY/nFAULT，逻辑上属模块 A，但由 `Core/Src/gpio.c` 配置、宏定义在 `Core/Inc/gpio.h`。这造成「模块 A 必须依赖 Core 才能拿到自己的引脚」。

CubeMX 生成的 GPIO 初始化默认落在 `Core/`，这是常见现象。若要理顺，可把这三个脚移入模块 A（例如新建 `drv_gpio.c` 提供 `MX_DrvGpio_Init()`），但需同步改 CubeMX 生成设置，否则下次 Generate Code 会重复生成。

### C. `i2c3_bus.c` / `tim6_heartbeat.c` 放在模块内，但导出的是通用句柄

这两个文件在 `Modules/Driver_Communication/Src/` 下，导出的却是 `hi2c3` / `htim6` 这样的**通用外设句柄**。日后编码器模块若要用 I2C 或另一个定时器，会出现「Encoder 模块反向依赖 Driver_Communication 模块」的错误耦合。

⚠ **另有一个更现实的风险：** 这两个文件属 CubeMX 管辖（带 `USER CODE` 块），但已被移出 CubeMX 的默认输出位置。**下次 Generate Code 时 CubeMX 会在 `Core/Src/` 重新生成 `i2c.c` / `tim.c`**，产生两套配置同一外设的文件，可能引发重复定义。

两种可选方案，任选其一并在本文档固化：

```text
方案 A（推荐）：移回 Core/，所有通用外设配置集中在此
  Core/Src/  gpio.c · i2c3_bus.c · tim6_heartbeat.c

方案 B：保持现状，但明确约定这两个外设是本模块专属，
        其他模块不得复用，需另配自己的外设实例。
        并且必须在 .ioc 里禁用这两个外设的代码生成，避免重复生成。
```

### D. `FOC/` 目录不在 `Modules/` 下，但 Keil 分组名是 `Modules/FOC`

其他业务模块都在 `Modules/`，FOC 单独放在工程根，而 Keil 分组却带 `Modules/` 前缀。若是为强调「纯算法、零硬件依赖」的分层意图，这样放合理；否则建议移入 `Modules/FOC/` 使目录与分组名一致。

### E. `foc_algorithm.c` 的 PI 输出限幅复用了积分限幅字段

`foc_pi_step()` 返回值用 `foc_clamp(output, integral_min, integral_max)`，即**输出限幅和积分限幅共用同一对边界**。能正常工作（最终还有 `voltage_limit` 做幅值限幅兜底），但把两个不同概念合并了，调参时容易困惑。建议给 `foc_pi_t` 加独立的 `output_min` / `output_max`。

### F. `stspin32g4.c` 直接操作 GPIO（器件驱动层含硬件知识）

L2 直接调 `HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin)` 等。严格说「读哪个引脚」属配置层知识。当前可接受（宏来自 `gpio.h`），但更干净的做法是 L1 提供访问函数：

```c
/* Core/Inc/gpio.h —— 配置层提供，业务层消费 */
static inline uint8_t GD_ReadReady(void) {
  return (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET);
}
static inline uint8_t GD_ReadFaultActive(void) {      /* 返回 1 = 有故障 */
  return (HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin) == GPIO_PIN_RESET);
}
static inline void    GD_SetWake(uint8_t level) {
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```

好处：① `nFAULT` 低有效的取反逻辑收敛到一处，消除读错风险；② 日后改用 EXTI 只需改配置层；③ L2 变成纯寄存器语义层，可脱离具体引脚做单元测试。

### G. `GD_READY` 配为上拉，芯片缺失时会读到「就绪」假象

`gpio.c` 把 READY/nFAULT 都配为 `GPIO_PULLUP`。nFAULT 低有效配上拉是安全的；但 READY 高有效配上拉意味着**芯片没焊或没上电时读到 1（看似就绪）**。当前有 I2C 轮询与 STATUS 寄存器交叉校验兜底，且 `Evaluate()` 也会因 I2C 失败进入 `COMM_LOST`，所以不致命。若要更保守可改 `PULLDOWN`，但**需先查原理图确认有无外部上拉**（以 `hardware/G431_pin_config.txt` 与 `hardware/舵机原理图/` 为准）。

### H. 注释中文化未完成

见第 1 节末。剩余文件：`tim6_heartbeat.c`、`i2c3_bus.c/h`、`foc_algorithm.c/h`、`gpio.c`、`stm32g4xx_it.c`、`stm32g4xx_hal_msp.c`。

> 本轮新增的 `adc1_current_sampling.c/h` 已直接用中文注释，不在此列。

### I. ⚠ 注入序列耗时 11.43 µs，远超 1 µs 的触发提前量（建议优先确认）

模块 B 把 `TIM1_CH4` 的比较点设在更新事件**前 1 µs**（`TIM1_PWM_ADC_TRIGGER_LEAD_NS`），意图是让 ADC 在 PWM 周期边界前完成采样。但模块 F 的注入序列实际耗时是：

```text
单次转换 = 采样 2.5 cycles + 12 位逐次逼近 12.5 cycles = 15 cycles
         15 / 42 MHz                        = 357 ns
过采样 16 倍（单通道）  357 ns x 16         = 5.71 µs
注入序列两个通道        5.71 µs x 2         = 11.43 µs
PWM 周期（20 kHz）                          = 50 µs
```

**11.43 µs 是 1 µs 提前量的 11 倍。** 三个后果需要决策：

1. **采样跨越更新事件。** 从 T-1 µs 触发，到 T+10.43 µs 才结束，数据横跨两个 PWM 周期，不再对应单一的载波时刻。
2. **过采样窗口内可能发生开关动作。** 5.71 µs 的单通道窗口占载波周期的 11.4%，期间若有开关边沿，16 次累加会把开关噪声一起平均进去。这既可能是想要的纹波平均，也可能是污染——取决于采样点落在载波的哪个位置。
3. **在 `TIM1_CC` ISR 里读到的是上一周期的数据。** FOC 电流环要在优先级 0 的 `TIM1_CC` 中断里跑，而序列需要 11.43 µs 才完成；若在中断入口立即读 JDR，拿到的是滞后一个周期的采样值。`EOCSelection = ADC_EOC_SEQ_CONV` 保证不会读到半更新序列，但**保证不了数据的时新性**。

可选方向（择一并在此固化）：

```text
方向 A：降低过采样倍率（如 4 倍）→ 序列缩到 2.86 µs，但噪声平均变弱
方向 B：延长采样时间并改用规则组 + DMA，注入组只留给保护判断
方向 C：保持现状，接受一个周期的滞后（电流环带宽远低于 20 kHz 时可接受）
方向 D：把 CH4 提前量从 1 µs 加大到 > 11.43 µs，让序列在更新事件前跑完
        —— 这会改变模块 B 的 TIM1_PWM_ADC_TRIGGER_LEAD_NS，需同步第 5.2 节
```

> 方向 D 最贴合原设计意图，但会挤压同一周期内的电流环执行时间（50 µs 周期里已用掉 11.43 µs 采样 + 电流环本身），**必须实测确认单次 `FOC_ControllerStep()` 的耗时**（第 8.1 节已有 1 ms 的 SysTick 阻塞约束）。

### J. ⚠ 两路采样各自对应哪一相（U / V / W）尚未实测确认

**已确定的部分**（依据数据手册与 HAL/器件头文件，无需再查）：片内 `OPAMP1` 输出在 PA2 = `ADC1_IN3` = 注入 rank 1；片内 `OPAMP3` 输出在 PB1 = `ADC1_IN12` = 注入 rank 2。

**未确定的部分**：这两路各自采的是电机哪一相。原理图上存在互相矛盾的线索（两个反馈网络标注为「电机U相电流反馈」「电机W相电流反馈」，而两个采样电阻的网络名是 `PDGND1U` / `PDGND1V`），但**原理图不作为判据**——本项目原理图的规范性不足，据此推断容易得出错误结论。

因此：

- 代码注释里**只描述「通道 / 引脚 / rank」这些确定事实**，不断言相别。
- `ia_offsets` / `ib_offsets` 的 `a` / `b` 只是沿用参考配置的命名，**不代表已确认的相别**。
- 业务层做安培换算、以及 FOC 的 `foc_input_t.phase_a / phase_b` 接线之前，**必须先确认这个映射**。接错的后果是 Clarke 变换算出错误的 β 分量，电流环会朝错误方向调节。

**确认方法（实测，一次即可定论）：** 功率级保持关断（`MOE = 0`）先记录两路 JDR 的静态基线；然后只对单一相通一个小电流，看哪一路 JDR 偏离基线。两轮即可确定 rank 1 / rank 2 与相别的对应关系。

### K. 过采样模式下 `DataAlign` 被硬件忽略，但配置里仍写着左对齐

`MX_ADC1_Init()` 里 `hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT`，注释写「启用过采样，该参数被忽略」。这一点已核实成立：RM0440 §21.4.26 明确「过采样模式下不支持左对齐，ROVSE / JOVSE 置位时 ALIGN 位被忽略」。

所以实际读出的是**右对齐的 16 位累加值**（满量程 4095 × 16 = 65520）。**配置本身无错，但读数换算必须按右对齐理解**——若误以为是左对齐，会把数据再移位一次，得到相差 16 倍的电流值。业务层实现换算时务必注意，此处保留左对齐赋值只为与参考配置逐字一致。

### L. `MX_ADC1_Init()` 的 `software_trigger_flag` 参数没有对应的运行时切换接口

当前只能在初始化时一次性决定触发源。若将来需要在运行时切换（例如标定阶段用软件触发、运行阶段用 `TIM1_CH4`），需要重新走一遍注入配置。HAL 支持 `HAL_ADCEx_InjectedConfigChannel()` 重配，但**必须确认重配时注入组未在转换中**，否则会撕裂序列。业务层若需要这个能力，应在 `adc1_current_sampling.c` 里补一个显式的切换函数，而不是让上层直接操作句柄。

---

## 14. 修改和验证规则

1. 修改引脚、时钟、外设初始化时，**优先修改 CubeMX 配置并重新生成**，再检查 L1 返回值和 Keil 分组。
2. 新增业务模块时，**头文件只暴露业务接口，不从 `main.h` 借用宏**。
3. **任何实时 ISR 禁止 I2C、`HAL_Delay()`、无界循环和日志输出。**
4. FOC 算法修改后必须验证 **PI 积分限幅、电压矢量限幅和 `enabled = 0` 复位路径**。
5. 使用 Keil 的 **Build Target** 构建，确认 `0 Error(s), 0 Warning(s)`；再检查工程树中的分组和源文件路径没有出现旧文件名。
6. 移动或重命名源文件时，必须同步四处：磁盘文件、`servo_v1.uvprojx` 的 `FileName` **与** `FilePath`、`servo_v1.cproject.yml`、所有调用方的 `#include`。（历史上重命名 `MOS_Driver` → `Driver_Communication` 时曾漏改调用方与 `FilePath`，导致工程无法构建。）
7. 修改占空比标度或 `TIM1_PWM_DUTY_TO_COMPARE` 时，必须同步核对第 5.5 节的对照表与第 7.5 节 Runbook 的寄存器期望值。
8. 涉及功率级使能/关断的改动，必须验证第 10.4 节的上下文契约未被破坏（尤其「ISR 内不得有 I2C」）。
9. 改动电流采样的过采样倍率、采样时间或通道数时，必须重新核算注入序列总耗时，并核对第 13 节待核对项 I 的时序余量结论；`ADC1_CS_*` 系列常量与第 9.3 节的换算表要同步更新。
10. ⚠ **Keil 命令行构建的退出码不可作为成功依据。** `UV4.exe -b` 即使编译失败也可能返回 0，必须读构建日志确认 `0 Error(s)` 且产物已生成（本轮实测：日志为 `2 Error(s) / Target not created` 时退出码仍是 0）。

---

## 15. 已知问题

| # | 问题 | 原因 | 处理 |
| --- | --- | --- | --- |
| 1 | ~~FOC 安全门空指针调用~~ | `FOC_ControllerStep()` 无条件调 `disable_output` | ✅ **本轮已修复**（加判空） |
| 2 | ~~`TIM1_PWM_DEADTIME_NS` 求值为 14 而非 500~~ | 32 位整数溢出 | ✅ **本轮已修复**（当前无代码引用此宏，属预防性修复） |
| 3 | ~~驱动报故障后无人清 MOE，故障消失输出自动恢复~~ | 缺少跨层仲裁 | ✅ **本轮已修复**（`main.c` 加 L6 仲裁） |
| 4 | ~~Runbook 的 CCR1 期望值与默认 duty 矛盾~~ | 注释未随代码更新 | ✅ **本轮已修复**（Runbook 同时列出两种情况） |
| 5 | Keil 报 `Cannot read project file`，打不开 `.uvprojx` | 文件是 UTF-8 但缺 BOM 头，Keil 解析失败 | 运行上级目录 `fix_uvprojx.py`，它会重新格式化 XML 并补上 UTF-8 BOM |
| 6 | `TIM1_UP` 以 20 kHz 触发但无实际工作 | 为电流环预留骨架 | 刻意代价，源码注释已说明。实现电流环后应赋予其真实用途或关闭 |
| 7 | 下次 CubeMX Generate Code 会重复生成 `i2c.c` / `tim.c` | 配置文件被移出默认输出位置 | 见 13.C |
| 8 | 注入序列耗时 11.43 µs，是 1 µs 触发提前量的 11 倍 | 16 倍过采样把单通道耗时放大 16 倍 | 见 13.I，电流环上线前必须决策 |
| 9 | 两路采样各自对应哪一相未确认 | 尚未做通电实测；原理图线索互相矛盾且不作为判据 | 见 13.J，业务层换算前必须实测确认 |
| 10 | `ia_offsets` / `ib_offsets` 恒为 0，被链接器当未用符号移除 | 偏置标定属业务层，本轮只实现配置层 | 刻意如此，头文件注释已说明；实现业务层后自然消失 |

---

## 16. basecode 之外的相关目录

本工程位于 `Servo_9948/` 下的 `G431_base_code/`，同级还有：

| 目录 / 文件 | 说明 |
| --- | --- |
| `../PythonExample/` | PyQt5 编写的伺服调试上位机「AUUSN Motor Studio」，**纯仿真演示，不连接真实硬件**；自带独立 README |
| `../fix_uvprojx.py` | 修复 Keil 工程文件编码问题的脚本（见第 15 节 #5） |
| `../keil_error.md` | 上述 Keil 报错的现象记录（截图 + 文字） |
| `../问题点记录.docx` | 开发过程中的问题记录文档 |
| `../tmp/pdfs/` | 数据手册页面截图（引脚图、I2C 寄存器表等），开发参考用的临时资料 |
