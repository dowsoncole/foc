#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""README 更新：新增「模块 F 电流采样」章节，并把原 9~15 节顺移为 10~16 节，
同步修正所有受影响的交叉引用与状态表格。

顺序有依赖：先顺移节号，再插入新章节，避免匹配冲突。
所有替换要求唯一匹配，任一处失败即整体退出且不写盘（原子）。
"""
import os, sys

README = r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code\README.md"

with open(README, "rb") as f:
    raw = f.read()
text = raw.decode("utf-8")
pending = []


def rep(old, new, tag=""):
    """登记一次替换。

    ⚠ 唯一性校验推迟到应用阶段（见文件末尾的落盘循环），因为部分替换的
    匹配文本依赖前序替换的结果（例如「第 9.4 节」先被改成「第 10.4 节」，
    之后的规则条目才能匹配到它）。校验仍是强制的：任一处不是恰好 1 次
    匹配就整体退出且不写盘，保持原子性。
    """
    pending.append((old, new, tag))


# ============================================================ 第一步：节号顺移
HEAD_RENUMBER = [
    ("## 9. 跨模块调用关系速查",              "## 10. 跨模块调用关系速查"),
    ("### 9.1 「我要做 X，该调用谁？」",       "### 10.1 「我要做 X，该调用谁？」"),
    ("### 9.2 依赖矩阵（✅ = 允许调用）",      "### 10.2 依赖矩阵（✅ = 允许调用）"),
    ("### 9.3 禁止的调用方向",                 "### 10.3 禁止的调用方向"),
    ("### 9.4 上下文（执行环境）契约",         "### 10.4 上下文（执行环境）契约"),
    ("## 10. 编译、烧录与调试",                "## 11. 编译、烧录与调试"),
    ("## 11. 当前实现进度与下一步",            "## 12. 当前实现进度与下一步"),
    ("## 12. 待核对项（需决策）",              "## 13. 待核对项（需决策）"),
    ("## 13. 修改和验证规则",                  "## 14. 修改和验证规则"),
    ("## 14. 已知问题",                        "## 15. 已知问题"),
    ("## 15. basecode 之外的相关目录",         "## 16. basecode 之外的相关目录"),
]
for old, new in HEAD_RENUMBER:
    rep(old, new, "节号:" + old[:24])

# ====================================================== 第二步：交叉引用修正
XREF = [
    # 待核对项从第 12 节变为第 13 节
    ("另有 8 处只记录未改动（需决策，见第 12 节）",
     "另有 8 处只记录未改动（需决策，见第 13 节）"),
    ("*（⚠ 归属见 12.B）*",            "*（⚠ 归属见 13.B）*"),
    ("所以不致命（见 12.G）",           "所以不致命（见 13.G）"),
    ("这关系到第 12 节待核对项 A。",     "这关系到第 13 节待核对项 A。"),
    ("/* 标度 0..1000，故 50 表示 5 %。⚠ 见 12.A */",
     "/* 标度 0..1000，故 50 表示 5 %。⚠ 见 13.A */"),
    ("（见 12.C 的重复生成风险）",       "（见 13.C 的重复生成风险）"),
    ("| 配置文件被移出默认输出位置 | 见 12.C |",
     "| 配置文件被移出默认输出位置 | 见 13.C |"),
    # 跨模块速查从第 9 节变为第 10 节
    ("必须验证第 9.4 节的上下文契约未被破坏",
     "必须验证第 10.4 节的上下文契约未被破坏"),
    # 已知问题从第 14 节变为第 15 节
    ("修复 Keil 工程文件编码问题的脚本（见第 14 节 #5）",
     "修复 Keil 工程文件编码问题的脚本（见第 15 节 #5）"),
]
for old, new in XREF:
    rep(old, new, "引用:" + old[:20])

# ================================================== 第三步：概述与对照表状态
rep("| 电流采样 | **未实现**（TIM1_CH4 已预留 ADC 触发） |",
    "| 电流采样 | OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组两通道、16 倍过采样，"
    "由 TIM1_CH4 触发（**配置层已完成，业务层未实现**） |",
    "概述表")

rep("| — 电流采样 | *（待建）* | — | ADC1 | ❌ 空 Keil 分组 |",
    "| F 电流采样 | `Modules/Current_Sampling/` | L1 | ADC1、OPAMP1、OPAMP3、"
    "PA1/PA2/PA3、PB0/PB1/PB2 | ⚠ 配置层完成，**业务层未实现** |",
    "模块对照表")

# ------------------------------------------------------- 第 2 节：分层图与约束
rep("L1  外设配置：i2c3_bus / tim6_heartbeat / gpio / tim1_pwm(MX_TIM1_Init) / msp",
    "L1  外设配置：i2c3_bus / tim6_heartbeat / gpio / tim1_pwm(MX_TIM1_Init)\n"
    "              adc1_current_sampling(OPAMP1/3 + ADC1) / msp",
    "分层图 L1")

rep("- `gpio.c`、`i2c3_bus.c`、`tim6_heartbeat.c`、`tim1_pwm.c` **不得包含 `main.h`，"
    "也不得调用 `Error_Handler()`**。初始化失败通过返回值交给 `main.c` 处理。"
    "（已核实：这些文件当前均满足此约束）",
    "- `gpio.c`、`i2c3_bus.c`、`tim6_heartbeat.c`、`tim1_pwm.c`、"
    "`adc1_current_sampling.c` **不得包含 `main.h`，也不得调用 `Error_Handler()`**。"
    "初始化失败通过返回值交给 `main.c` 处理。（已核实：这些文件当前均满足此约束）",
    "L1 约束")

# ---------------------------------------------------------- 第 3 节：目录树
rep("""│   └── Three_Phase_Driver/                  模块 B：三相驱动
│       ├── Inc/tim1_pwm.h          三相 PWM 配置和功率级接口
│       └── Src/tim1_pwm.c          ⚠ 配置与业务同居一个文件""",
    """│   ├── Three_Phase_Driver/                  模块 B：三相驱动
│   │   ├── Inc/tim1_pwm.h          三相 PWM 配置和功率级接口
│   │   └── Src/tim1_pwm.c          ⚠ 配置与业务同居一个文件
│   └── Current_Sampling/                      模块 F：电流采样
│       ├── Inc/adc1_current_sampling.h  OPAMP1/3 + ADC1 注入组配置接口 ← L1
│       └── Src/adc1_current_sampling.c  业务层（偏置标定 + 安培换算）待建""",
    "目录树")

rep("Modules/Current Sampling       → 空分组（待建）",
    "Modules/Current Sampling       → Modules/Current_Sampling/",
    "Keil 分组表")

rep("Include 路径：`../Core/Inc; ../FOC/Inc; ../Modules/Driver_Communication/Inc; "
    "../Modules/Three_Phase_Driver/Inc; ../Tests/Inc;",
    "Include 路径：`../Core/Inc; ../FOC/Inc; ../Modules/Driver_Communication/Inc; "
    "../Modules/Three_Phase_Driver/Inc; ../Modules/Current_Sampling/Inc; ../Tests/Inc;",
    "Include 路径")

# ------------------------------------------- 第 5.6 节：模块 B 的 ADC 预留说明
rep("""   CC4 比较点位于更新事件前 TIM1_PWM_ADC_TRIGGER_LEAD_NS，ADC 应由 OC4REF 触发。
   当前故意为空桩：ADC 尚未配置，没有消费者，而以载波频率运行的空中断体""",
    """   CC4 比较点位于更新事件前 TIM1_PWM_ADC_TRIGGER_LEAD_NS，ADC 应由 OC4REF 触发。
   ADC1 注入组已由模块 F 配置完毕（当前为软件触发，尚未挂到 CC4），但还没有
   消费者去读 JDR。当前故意为空桩：没有消费者，而以载波频率运行的空中断体""",
    "5.6 ADC 预留")

# ------------------------------------------- 第 6.5 / 6.7 节：FOC 端口接线
rep("| `read_feedback` | 电流采样层（❌ 未实现）+ 编码器层（❌ 未实现） | "
    "填 ADC 电流（两相）、编码器角度（MT6835 机械角 × 极对数）、d/q 参考 |",
    "| `read_feedback` | 电流采样层（⚠ 配置层已就绪，读码与换算未实现）"
    "+ 编码器层（❌ 未实现） | "
    "填 ADC 电流（两相，来自 `hadc1` 的 JDR1/JDR2）、编码器角度"
    "（MT6835 机械角 × 极对数）、d/q 参考 |",
    "6.5 read_feedback")

rep("    in->phase_a              = /* ← 电流采样层：两相 ADC 电流 */;",
    "    in->phase_a              = /* ← 模块 F：读 hadc1 的 JDR，减偏置后换算成安培 */;",
    "6.7 接线示例")

# ------------------------------------------------------- 第 8.3 节：MSP 回调表
rep("| `HAL_TIM_MspPostInit()` | `MX_TIM1_Init()` 末尾 | B | 使能 GPIOE 时钟，"
    "配 PE8~PE13 为 `AF_PP` + `NOPULL` + `HIGH` + `AF2_TIM1` |",
    "| `HAL_TIM_MspPostInit()` | `MX_TIM1_Init()` 末尾 | B | 使能 GPIOE 时钟，"
    "配 PE8~PE13 为 `AF_PP` + `NOPULL` + `HIGH` + `AF2_TIM1` |\n"
    "| `HAL_OPAMP_MspInit()` | `MX_OPAMP1_Init()` / `MX_OPAMP3_Init()` | F | "
    "配 PA1/PA2/PA3 与 PB0/PB1/PB2 为 `GPIO_MODE_ANALOG` + `NOPULL`。"
    "⚠ PB2 兼作 BOOT1，必须显式配置 |\n"
    "| `HAL_OPAMP_MspDeInit()` | 反初始化 | F | "
    "`HAL_GPIO_DeInit()` 释放上述六个模拟脚 |\n"
    "| `HAL_ADC_MspInit()` | `MX_ADC1_Init()` | F | 使能 `ADC12` 时钟。"
    "**刻意不配 ADC 中断**：注入组由 `TIM1_CC` ISR（优先级 0）消费，"
    "再加一个向量只会平白增加延迟 |\n"
    "| `HAL_ADC_MspDeInit()` | 反初始化 | F | 关 `ADC12` 时钟 |",
    "8.3 MSP 表")

rep("> 🔑 **全工程唯一性约定：** 工程配置 `USE_HAL_TIM_REGISTER_CALLBACKS = 0`",
    "> ⚠ **STM32G4 的 OPAMP 没有独立的 RCC 时钟门**，所以 `HAL_OPAMP_MspInit()` "
    "只使能 GPIO 时钟；运放本体由 `HAL_OPAMP_Init()` 写自己的 CSR 使能位打开。\n"
    ">\n"
    "> 🔑 **全工程唯一性约定：** 工程配置 `USE_HAL_TIM_REGISTER_CALLBACKS = 0`",
    "8.3 OPAMP 时钟说明")

# --------------------------------------------------- 第 8.4 节：main 初始化序列
rep("""if (MX_TIM1_Init()  != HAL_OK) Error_Handler();   /* MOE 仍为 0 */
if (DrvMonitor_Init() != HAL_OK) Error_Handler(); /* 清锁存故障 + 启动心跳 */
```""",
    """if (MX_TIM1_Init()  != HAL_OK) Error_Handler();   /* MOE 仍为 0 */
if (MX_OPAMP1_Init() != HAL_OK) Error_Handler();  /* 模块 F：U 相电流放大 */
if (MX_OPAMP3_Init() != HAL_OK) Error_Handler();  /* 模块 F：W 相电流放大 */
if (MX_ADC1_Init(1U) != HAL_OK) Error_Handler();  /* 软件触发，未挂 TIM1_CH4 */
if (DrvMonitor_Init() != HAL_OK) Error_Handler(); /* 清锁存故障 + 启动心跳 */
```

> **两个运放必须先于 ADC 初始化。** ADC1 采的正是运放的输出脚"
    "（PA2 = IN3，PB1 = IN12），运放没起来时那两个节点上没有有效信号，"
    "先校准 ADC 只会把噪声当成基线记下来。\n"
    ">\n"
    "> `MX_ADC1_Init(1U)` 传软件触发也是刻意的：当前固件跑的是模块 D 的栅极"
    "验证波形，TIM1_CH4 已在 20 kHz 运行，若让注入组跟随它触发，每个载波周期"
    "都会多做一轮 11.43 µs 的过采样转换，而目前没有任何消费者读这些数据。"
    "电流环上线时改传 `0`，见第 9 节与第 13 节待核对项 I。""",
    "8.4 main 序列")

# ------------------------------------------------------- 第 10 节：调试 Watch 表
rep("""调试器 Watch 窗口可直接观察三个刻意导出的全局对象：

| 对象 | 模块 / 层 | 观察什么 |
| --- | --- | --- |
| `s_mon` | A / L3 | 驱动状态机、STATUS 寄存器、锁存故障、READY/nFAULT 电平、连续通信失败次数 |
| `htim1` | B / L1 | 定时器句柄；配合 `TIM1->CCR1/2/3`、`ARR`、`BDTR` 看 PWM 与 MOE |
| `g_measurement_manager` | E / L6 | 测量配置与运行状态，**可在运行途中修改**，`Task()` 会自动同步 |""",
    """调试器 Watch 窗口可直接观察这些刻意导出的全局对象：

| 对象 | 模块 / 层 | 观察什么 |
| --- | --- | --- |
| `s_mon` | A / L3 | 驱动状态机、STATUS 寄存器、锁存故障、READY/nFAULT 电平、连续通信失败次数 |
| `htim1` | B / L1 | 定时器句柄；配合 `TIM1->CCR1/2/3`、`ARR`、`BDTR` 看 PWM 与 MOE |
| `hadc1` / `hopamp1` / `hopamp3` | F / L1 | 采样外设句柄；配合 `ADC1->JDR1`、`JDR2` 看注入组原始码，`ADC1->JSQR` 看触发源与序列 |
| `ia_offsets` / `ib_offsets` | F / L1 | 零电流偏置标定值的预留落点。**当前无任何代码填充，恒为 0** |
| `g_measurement_manager` | E / L6 | 测量配置与运行状态，**可在运行途中修改**，`Task()` 会自动同步 |""",
    "10 Watch 表")

# ------------------------------------------- 第 11 节：CubeMX 管辖范围
rep("3. 被管辖的包括 `Core/` 下各文件，以及 `Modules/Driver_Communication/` 下的 "
    "`i2c3_bus.c` / `tim6_heartbeat.c`（见 13.C 的重复生成风险）",
    "3. 被管辖的包括 `Core/` 下各文件，以及 `Modules/Driver_Communication/` 下的 "
    "`i2c3_bus.c` / `tim6_heartbeat.c`（见 13.C 的重复生成风险）。"
    "`Modules/Current_Sampling/` 下的文件**当前不在 .ioc 里**（ADC1 / OPAMP1 / OPAMP3 "
    "未通过 CubeMX 启用），所以它们是纯手写、不受重新生成影响",
    "11 CubeMX 管辖")

rep("4. `FOC/`、`Tests/`、`stspin32g4.c`、`drv_monitor.c`、`tim1_pwm.c` 的 "
    "`USER CODE BEGIN 2` 段不在管辖范围",
    "4. `FOC/`、`Tests/`、`stspin32g4.c`、`drv_monitor.c`、`tim1_pwm.c` 的 "
    "`USER CODE BEGIN 2` 段不在管辖范围。`adc1_current_sampling.c/h` 整个文件"
    "都不在管辖范围（但保留了同名 `USER CODE` 块，便于将来接入 CubeMX）",
    "11 管辖例外")

# --------------------------------------------- 第 12 节：进度表与下一步清单
rep("| — | — | 电流采样（ADC） | ❌ 未实现（空 Keil 分组，TIM1_CH4 触发已预留） |",
    "| F 电流采样 | L1 | OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组两通道 + "
    "16 倍过采样 + 上电自校准 | ✅ 完成 |\n"
    "| F 电流采样 | L4 | 偏置标定 + 原始码到安培换算 + 供 `read_feedback` 使用 | "
    "❌ 未实现 |",
    "12 进度表")

rep("""**固件当前行为：** 上电 → 配置外设（MOE=0，功率级未接通）→ `DrvMonitor_Init()` 清除栅极驱动锁存故障并启动 1 s 心跳 → 若驱动 READY 则等 50 ms 自举预充电后输出 U 相 5% 验证波形（其余两相低侧导通钳到 PGND）→ 主循环跑驱动监控心跳 + 测量同步 + 故障仲裁。**不会转动电机。**""",
    """**固件当前行为：** 上电 → 配置外设（MOE=0，功率级未接通）→ 配置两个电流放大运放与 ADC1 注入组（软件触发，只校准不启动转换）→ `DrvMonitor_Init()` 清除栅极驱动锁存故障并启动 1 s 心跳 → 若驱动 READY 则等 50 ms 自举预充电后输出 U 相 5% 验证波形（其余两相低侧导通钳到 PGND）→ 主循环跑驱动监控心跳 + 测量同步 + 故障仲裁。**不会转动电机，也不会产生任何电流采样数据。**""",
    "12 当前行为")

rep("""1. **电流采样层** → 填 `Modules/Current Sampling`。配置部分：ADC1 + **两通道**（算法只需两相）+ 由 OC4REF 触发；业务部分：原始码 → 安培换算
2. **编码器层** → 填 `Modules/Encoder`。配置部分：SPI 或 I2C；业务部分：MT6835 机械角 × 极对数 → 电角度""",
    """1. ~~**电流采样层（配置部分）**~~ → ✅ **本轮已完成**，见第 9 节。ADC1 + 两通道（算法只需两相）+ TIM1_CH4 触发已就绪
2. **电流采样层（业务部分）** → 填 `Modules/Current_Sampling/` 的业务文件。零电流偏置标定（写入 `ia_offsets` / `ib_offsets`）+ JDR 原始码 → 安培换算，供 `foc_port_t.read_feedback` 消费。**上电前必须先解决第 13 节待核对项 I 的时序余量与 J 的相别归属**
3. **编码器层** → 填 `Modules/Encoder`。配置部分：SPI 或 I2C；业务部分：MT6835 机械角 × 极对数 → 电角度""",
    "12 下一步 1")

rep("""3. **FOC 端口接线** → 按 6.5 节对照表实现四个回调，把上面两层与模块 B 连起来
4. **电流环上电** → 在 `TIM1_CC` ISR（优先级 0）中调 `FOC_ControllerStep()`，先低压限流空载验证。**务必保证单次迭代远小于 1 ms**（见 8.1 节的 SysTick 阻塞约束）
5. **速度环 / 位置环** → 主循环或独立定时器，输出 `q_reference`
6. **协议栈** → CAN/UART。配置部分：外设初始化；业务部分：帧解析，接入 `DrvMonitor_PostRequest()` 做远程故障复位与参数下发""",
    """4. **FOC 端口接线** → 按 6.5 节对照表实现四个回调，把上面两层与模块 B 连起来
5. **电流环上电** → 在 `TIM1_CC` ISR（优先级 0）中调 `FOC_ControllerStep()`，先低压限流空载验证。**务必保证单次迭代远小于 1 ms**（见 8.1 节的 SysTick 阻塞约束）
6. **速度环 / 位置环** → 主循环或独立定时器，输出 `q_reference`
7. **协议栈** → CAN/UART。配置部分：外设初始化；业务部分：帧解析，接入 `DrvMonitor_PostRequest()` 做远程故障复位与参数下发""",
    "12 下一步 2")

# ------------------------------------- 第 13 节：待核对项新增 I / J
rep("""### H. 注释中文化未完成

见第 1 节末。剩余文件：`tim6_heartbeat.c`、`i2c3_bus.c/h`、`foc_algorithm.c/h`、`gpio.c`、`stm32g4xx_it.c`、`stm32g4xx_hal_msp.c`。""",
    """### H. 注释中文化未完成

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

### J. ⚠ 两路采样各自对应哪一相尚未确认

原理图 `hardware/舵机原理图/主控板.pdf` 的网表里存在**互相矛盾**的相别线索：

| 线索 | 内容 | 指向 |
| --- | --- | --- |
| 两个反馈网络的标注文字 | 「电机U相电流反馈」「电机W相电流反馈」 | U 与 W |
| 两个采样电阻的网络名 | `PDGND1U` / `PDGND1V` | U 与 V |

**两条线索不一致**，且无法从网表文本可靠判定 `ADC1_IN3`（rank 1）与 `ADC1_IN12`（rank 2）各自对应哪一相。因此：

- 代码注释里**只描述「通道 / 引脚 / rank」这些确定事实**，不断言相别。
- `ia_offsets` / `ib_offsets` 的 `a` / `b` 只是沿用参考配置的命名，**不代表已确认的相别**。
- 业务层做安培换算、以及 FOC 的 `foc_input_t.phase_a / phase_b` 接线之前，**必须先确认这个映射**。接错的后果是 Clarke 变换算出错误的 β 分量，电流环会朝错误方向调节。

**确认方法：** 对照原理图 PDF 中「相电流采集」页的实际连线（网表文本提取不出引脚级连接关系），或在硬件上给单相通小电流、看哪一路 JDR 变化。

### K. 过采样模式下 `DataAlign` 被硬件忽略，但配置里仍写着左对齐

`MX_ADC1_Init()` 里 `hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT`，注释写「启用过采样，该参数被忽略」。这一点已核实成立：RM0440 §21.4.26 明确「过采样模式下不支持左对齐，ROVSE / JOVSE 置位时 ALIGN 位被忽略」。

所以实际读出的是**右对齐的 16 位累加值**（满量程 4095 × 16 = 65520）。**配置本身无错，但读数换算必须按右对齐理解**——若误以为是左对齐，会把数据再移位一次，得到相差 16 倍的电流值。业务层实现换算时务必注意，此处保留左对齐赋值只为与参考配置逐字一致。

### L. `MX_ADC1_Init()` 的 `software_trigger_flag` 参数没有对应的运行时切换接口

当前只能在初始化时一次性决定触发源。若将来需要在运行时切换（例如标定阶段用软件触发、运行阶段用 `TIM1_CH4`），需要重新走一遍注入配置。HAL 支持 `HAL_ADCEx_InjectedConfigChannel()` 重配，但**必须确认重配时注入组未在转换中**，否则会撕裂序列。业务层若需要这个能力，应在 `adc1_current_sampling.c` 里补一个显式的切换函数，而不是让上层直接操作句柄。""",
    "13 待核对项")

# --------------------------------------------- 第 14 节：修改验证规则补一条
rep("8. 涉及功率级使能/关断的改动，必须验证第 10.4 节的上下文契约未被破坏（尤其「ISR 内不得有 I2C」）。",
    """8. 涉及功率级使能/关断的改动，必须验证第 10.4 节的上下文契约未被破坏（尤其「ISR 内不得有 I2C」）。
9. 改动电流采样的过采样倍率、采样时间或通道数时，必须重新核算注入序列总耗时，并核对第 13 节待核对项 I 的时序余量结论；`ADC1_CS_*` 系列常量与第 9.3 节的换算表要同步更新。
10. ⚠ **Keil 命令行构建的退出码不可作为成功依据。** `UV4.exe -b` 即使编译失败也可能返回 0，必须读构建日志确认 `0 Error(s)` 且产物已生成（本轮实测：日志为 `2 Error(s) / Target not created` 时退出码仍是 0）。""",
    "14 验证规则")

# ------------------------------------------------- 第 15 节：已知问题表补条目
rep("| 7 | 下次 CubeMX Generate Code 会重复生成 `i2c.c` / `tim.c` | 配置文件被移出默认输出位置 | 见 13.C |",
    """| 7 | 下次 CubeMX Generate Code 会重复生成 `i2c.c` / `tim.c` | 配置文件被移出默认输出位置 | 见 13.C |
| 8 | 注入序列耗时 11.43 µs，是 1 µs 触发提前量的 11 倍 | 16 倍过采样把单通道耗时放大 16 倍 | 见 13.I，电流环上线前必须决策 |
| 9 | 两路采样各自对应哪一相未确认 | 原理图反馈网络标注（U/W）与采样电阻网络名（U/V）矛盾 | 见 13.J，业务层换算前必须确认 |
| 10 | `ia_offsets` / `ib_offsets` 恒为 0，被链接器当未用符号移除 | 偏置标定属业务层，本轮只实现配置层 | 刻意如此，头文件注释已说明；实现业务层后自然消失 |""",
    "15 已知问题")

# ============================================================ 最后校验并落盘
for old, new, tag in pending:
    n = text.count(old)
    if n != 1:
        print("FAIL[%s]: 匹配 %d 次（需恰好 1 次）\n---\n%s\n---" % (tag, n, old[:300]))
        sys.exit(1)
    text = text.replace(old, new, 1)

with open(README, "wb") as f:
    f.write(text.encode("utf-8"))
print("OK README.md %d -> %d bytes，共 %d 处替换" % (len(raw), len(text.encode("utf-8")), len(pending)))
