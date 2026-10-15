#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修正与补全：

1. 撤回代码注释里两处无法验证的断言：
   - 「已与原理图出现的两组运放网络对应」
   - 「原理图在 PA2 节点上挂了后级 RC 网络」
   改为只陈述能从配置本身与 HAL/器件头文件推出的事实。
2. 用准确内容重写 README 待核对项 J。
3. 插入 README 第 9 节「模块 F：电流采样」。
4. 补齐第 10 节速查表中模块 F 的条目。

所有替换要求唯一匹配，任一处失败即整体退出且不写盘。
"""
import os, sys

ROOT = r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code"
SW = os.path.join(ROOT, "software", "servo_v1")
README = os.path.join(ROOT, "README.md")


def patch(path, pairs):
    with open(path, "rb") as f:
        raw = f.read()
    bom = b""
    body = raw
    if raw.startswith(b"\xef\xbb\xbf"):
        bom = b"\xef\xbb\xbf"
        body = raw[3:]
    crlf = b"\r\n" in body
    text = body.decode("utf-8")
    for old, new in pairs:
        if crlf:
            old = old.replace("\n", "\r\n")
            new = new.replace("\n", "\r\n")
        n = text.count(old)
        if n != 1:
            print("FAIL %s: 匹配 %d 次（需恰好 1 次）\n---\n%s\n---" % (
                os.path.basename(path), n, old[:400]))
            sys.exit(1)
        text = text.replace(old, new)
    out = bom + text.encode("utf-8")
    with open(path, "wb") as f:
        f.write(out)
    print("OK %-34s %6d -> %6d bytes  BOM=%-5s CRLF=%s" % (
        os.path.basename(path), len(raw), len(out), bool(bom), crlf))


# ============================================================ 1. 头文件注释块
HDR = os.path.join(SW, "Modules", "Current_Sampling", "Inc", "adc1_current_sampling.h")

NEW_BANNER = """/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc1_current_sampling.h
  * @brief   相电流采样的 L1 外设配置：OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组
  *          两通道采样，由 TIM1_CH4 触发。
  *
  *          引脚映射（由 STM32G431 的固定复用关系决定，已对照 HAL 头文件核实：
  *          OPAMP_NONINVERTINGINPUT_IO0 / OPAMP_INVERTINGINPUT_IO0 的引脚表）
  *            PA1  -> OPAMP1 VINP0        相电流采样输入
  *            PA3  -> OPAMP1 VINM0        反馈网络
  *            PA2  -> OPAMP1 VOUT         同时是 ADC1_IN3   -> 注入 rank 1
  *            PB0  -> OPAMP3 VINP0        相电流采样输入
  *            PB2  -> OPAMP3 VINM0        反馈网络
  *            PB1  -> OPAMP3 VOUT         同时是 ADC1_IN12  -> 注入 rank 2
  *
  *          ⚠⚠ 上述引脚与板级网络的连线**尚未核实**，上电实测前必须确认，
  *          见 README 待核对项 J。原理图 PDF 转出的网表文本不可用于判定引脚
  *          归属（其中器件引脚表出现两个不同封装的脚号交错混排，且晶振网络
  *          被归到与栅极驱动同一个器件号下，明显失真）。此外原理图上的网络
  *          标号是 OPAMP1_* / OPAMP2_*，而本文件按参考配置用的是 OPAMP1 /
  *          OPAMP3，这个命名差异也需要一并对齐。
  *
  *          ⚠ 两路各自对应哪一相（U / V / W）同样未确认：原理图里两个反馈
  *          网络标注为「电机U相电流反馈」「电机W相电流反馈」，但两个采样电阻
  *          的网络名是 PDGND1U / PDGND1V（U 与 V），两者不一致。在确认之前，
  *          本文件只描述「通道 / 引脚 / rank」这些确定事实，不断言相别。
  *          ia_offsets / ib_offsets 的 a / b 也只是沿用参考配置的命名。
  *
  *          两路运放输出都经外部引脚回到 ADC，不走内部通道：
  *            OPAMP3 —— ADC1 上根本没有它的内部通道。VOPAMP3 只挂在
  *                      ADC2_IN18 / ADC3_IN13（已核实 LL_ADC_CHANNEL_VOPAMP3_*
  *                      的定义），本工程只用 ADC1，所以 PB1 = ADC1_IN12 是
  *                      唯一可选路径。
  *            OPAMP1 —— ADC1 上有 ADC_CHANNEL_VOPAMP1（= ADC1_IN13），但参考
  *                      配置取的是外部 PA2 = IN3，本文件与之一致。
  *
  *          只采两相：FOC 算法的 Clarke 变换只需 ia / ib，第三相由
  *          ia + ib + ic = 0 推出。这是算法需求反向约束配置层的唯一例子。
  ******************************************************************************
  */
/* USER CODE END Header */"""

with open(HDR, "rb") as f:
    hdr_raw = f.read()
hdr_text = hdr_raw.decode("utf-8")
END = "/* USER CODE END Header */"
cut = hdr_text.index(END) + len(END)
hdr_new = NEW_BANNER + hdr_text[cut:]
with open(HDR, "wb") as f:
    f.write(hdr_new.encode("utf-8"))
print("OK %-34s %6d -> %6d bytes（重写文件头注释块）" % (
    os.path.basename(HDR), len(hdr_raw), len(hdr_new.encode("utf-8"))))

# ============================================================ 2. 实现文件注释
patch(os.path.join(SW, "Modules", "Current_Sampling", "Src", "adc1_current_sampling.c"), [
    ("""  /* 独立跟随模式：同相输入 IO0 = PA1，反相输入 IO0 = PA3。
     增益由原理图上的外部电阻网络决定，运放不引入内部增益。
     InternalOutput = DISABLE：输出经 PA2 引脚外发再回到 ADC1_IN3。
     ADC1 上确实有 OPAMP1 的内部通道（ADC_CHANNEL_VOPAMP1 = IN13），但原理图
     在 PA2 节点上挂了后级 RC 网络，走内部通道会绕过它，采到的就不是滤波后
     的信号，所以取外部路径。 */""",
     """  /* 独立（STANDALONE）模式：同相输入 IO0 = PA1，反相输入 IO0 = PA3。
     两个输入都走外部引脚，所以增益完全由板级外部电阻网络决定，运放本身
     不引入内部增益（内部增益只在 PGA 模式下才有）。
     InternalOutput = DISABLE：输出经 PA2 引脚外发，再由 ADC1_IN3 采样。
     ADC1 上其实有 OPAMP1 的内部通道（ADC_CHANNEL_VOPAMP1 = IN13），但参考
     配置取的是外部路径，本文件与之一致。⚠ 该引脚与板级网络的连线尚未核实，
     见头文件说明与 README 待核对项 J。 */"""),
    ("""     ⚠ PB2 兼作 BOOT1：必须显式配为模拟模式，否则残留的启动模式配置
     会干扰运放输入。该配置由 MSP 层的 HAL_OPAMP_MspInit() 完成。""",
     """     ⚠ PB2 兼作 BOOT1：必须显式配为模拟模式，否则残留的启动模式配置
     会干扰运放输入。该配置由 MSP 层的 HAL_OPAMP_MspInit() 完成。
     注意这会改变 PB2 的 BOOT1 行为——本工程从 Flash 启动、经 SWD 下载，
     正常不依赖 PB2 电平，但上电前请一并确认（见 README 待核对项 J）。"""),
])

# ============================================================ 3. README 待核对项 J
OLD_J = """### J. ⚠ 两路采样各自对应哪一相尚未确认

原理图 `hardware/舵机原理图/主控板.pdf` 的网表里存在**互相矛盾**的相别线索：

| 线索 | 内容 | 指向 |
| --- | --- | --- |
| 两个反馈网络的标注文字 | 「电机U相电流反馈」「电机W相电流反馈」 | U 与 W |
| 两个采样电阻的网络名 | `PDGND1U` / `PDGND1V` | U 与 V |

**两条线索不一致**，且无法从网表文本可靠判定 `ADC1_IN3`（rank 1）与 `ADC1_IN12`（rank 2）各自对应哪一相。因此：

- 代码注释里**只描述「通道 / 引脚 / rank」这些确定事实**，不断言相别。
- `ia_offsets` / `ib_offsets` 的 `a` / `b` 只是沿用参考配置的命名，**不代表已确认的相别**。
- 业务层做安培换算、以及 FOC 的 `foc_input_t.phase_a / phase_b` 接线之前，**必须先确认这个映射**。接错的后果是 Clarke 变换算出错误的 β 分量，电流环会朝错误方向调节。

**确认方法：** 对照原理图 PDF 中「相电流采集」页的实际连线（网表文本提取不出引脚级连接关系），或在硬件上给单相通小电流、看哪一路 JDR 变化。"""

NEW_J = """### J. ⚠⚠ 电流采样的硬件通路尚未核实（模块 F 上电实测前的头号阻塞项）

本轮按用户提供的参考配置实现了 STM32 内部 `OPAMP1` / `OPAMP3` + `ADC1` 注入组。**配置本身自洽**（引脚、ADC 通道、内部通道可用性都已对照 HAL/器件头文件核实），但**它与本板的实际连线是否吻合，尚未得到证实**。

**为什么没能核实：** 原理图 `hardware/舵机原理图/主控板.pdf` 转出的网表文本（`主控板.pdf.txt`）**不足以判定引脚归属**，已发现三处失真：

| 失真现象 | 证据 |
| --- | --- |
| 两个不同封装的脚号交错混排 | 同一份引脚表里，`GLS1(29)…EPAD(65)`（STSPIN32G4，VFQFPN64）与 `PA8(44)…PB9(60)`（MCU）互相穿插 |
| 脚号与 LQFP100 实际不符 | 文本把 `PC13` 标为 3 脚、`PA1` 标为 14 脚；STM32G431VB 的 LQFP100 实际是 `PC13` = 7 脚、`PA1` = 18 脚 |
| 晶振被归到功率器件名下 | `NLOSC0IN0` 的成员是 `PIX101 / PIC5602 / PIU406`，即晶振 X1 接在「U4」的 6/7 脚；而 `U4` 同时带着 `VM / SW / VCC / REGIN / EPAD` 这些 STSPIN32G4 专有引脚名。晶振不可能接栅极驱动芯片 |

> 正因如此，本轮**不采纳**任何「这些运放属于 STSPIN32G4」或「属于 STM32」的结论——两个方向都无法从该文本证实。曾一度据脚号推断运放属栅极驱动芯片，后被晶振网络这条反证推翻。

**仍需核对的两个具体问题：**

1. **命名差异。** 原理图上的网络标号是 `OPAMP1_INP/INN/OUT` 与 `OPAMP2_INP/INN/OUT`，而参考配置用的是 `OPAMP1` 与 `OPAMP3`。若板上第二组放大真的走 MCU 的 `OPAMP2`（`PA7`/`PA5`/`PA6`），那么本文件的 `OPAMP3` 配置与 `ADC1_IN12` 通道都是错的（`PA6` 对应 `ADC2_IN4`，不在 ADC1 上）。也可能 `OPAMP2_*` 只是设计者对「板上第二组运放」的编号，或指 STSPIN32G4 的内部运放——需看原理图原件才能定论。
2. **相别归属。** 两个反馈网络的标注文字是「电机**U**相电流反馈」「电机**W**相电流反馈」，而两个采样电阻（`R28` / `R29`，0R020）的网络名是 `PDGND1**U**` / `PDGND1**V**`，**U/W 与 U/V 不一致**。所以 `ADC1_IN3`（rank 1）与 `ADC1_IN12`（rank 2）各自对应哪一相无法确定。

**后果与当前处理：**

- 代码注释里**只描述「通道 / 引脚 / rank」这些确定事实**，不断言相别；`ia_offsets` / `ib_offsets` 的 `a` / `b` 也只是沿用参考配置的命名。
- 相别接错的后果是 Clarke 变换算出错误的 β 分量，电流环会朝错误方向调节。业务层做安培换算、以及 FOC 的 `foc_input_t.phase_a / phase_b` 接线之前**必须先确认**。
- 模块 F 的代码**可以编译、可以初始化**，但在核实前**不建议上电实测**：若那些引脚已被其他网络占用，把运放输出同时挂上去可能损坏器件。

**核实方法（按成本从低到高）：**

1. 直接看原理图 PDF 原件的「相电流采集」页，确认两组运放符号属于哪个器件、输出走向哪个 MCU 引脚；
2. 万用表量 `R28` / `R29` 高侧与 `PA1` / `PB0` 之间是否导通；
3. 上电后（功率级保持关断、`MOE = 0`）量 `PA2` / `PB1` 的静态电平：应约等于运放中点偏置；若为 0 V 或 3.3 V，说明运放没在工作；
4. 给单相通一个小电流，看哪一路 `JDR` 变化——这一步同时解决相别归属问题。"""

# ============================================================ 4. README 第 9 节
SECTION_9 = """## 9. 模块 F：电流采样（`Modules/Current_Sampling/`）— ⚠ 配置层已完成，业务层未实现

### 9.1 模块职责与拥有的外设资源

一句话：**把两路相电流采样信号放大并数字化，交给电流环。**

本轮只实现了**配置层（L1）**：把运放和 ADC 准备好、校准好，然后交出句柄。**业务层（L4）尚未实现**——没有偏置标定、没有原始码到安培的换算、没有供 `foc_port_t.read_feedback` 调用的接口。

模块直接采用了第 12 节推荐的「配置/业务分文件」范式，避免重蹈 `tim1_pwm.c` 混居的覆辙：

```text
Modules/Current_Sampling/
├── Inc/adc1_current_sampling.h   ← L1 配置：MX_*_Init() 声明 + 句柄 extern + 硬件常量
└── Src/adc1_current_sampling.c   ← L1 配置实现
    （业务文件待建：Inc/current_sampling.h + Src/current_sampling.c）
```

| 外设 | 配置位置 | 作用 | 导出给上层的东西 |
| --- | --- | --- | --- |
| **OPAMP1**<br>PA1 / PA3 / PA2 | 本模块 `Src/adc1_current_sampling.c` 的 `MX_OPAMP1_Init()` | 独立跟随模式放大一路相电流采样信号，输出到 PA2 | `extern OPAMP_HandleTypeDef hopamp1`<br>`HAL_StatusTypeDef MX_OPAMP1_Init(void)` |
| **OPAMP3**<br>PB0 / PB2 / PB1 | 同上，`MX_OPAMP3_Init()` | 独立跟随模式放大另一路，输出到 PB1 | `extern OPAMP_HandleTypeDef hopamp3`<br>`HAL_StatusTypeDef MX_OPAMP3_Init(void)` |
| **ADC1**（注入组）<br>IN3 = PA2<br>IN12 = PB1 | 同上，`MX_ADC1_Init()` | 把两路运放输出数字化，16 倍过采样，可由 TIM1_CH4 触发 | `extern ADC_HandleTypeDef hadc1`<br>`HAL_StatusTypeDef MX_ADC1_Init(uint8_t)` |
| 六个模拟脚的 GPIO | `Core/Src/stm32g4xx_hal_msp.c` 的 `HAL_OPAMP_MspInit()` | 配为 `GPIO_MODE_ANALOG` + `NOPULL` | 无（HAL 自动回调） |
| ADC12 时钟 | 同上，`HAL_ADC_MspInit()` | 使能 `ADC12` 内核时钟 | 无 |

> ⚠⚠ **上电实测前必读：** 本模块与板级实际连线的吻合性**尚未核实**，且原理图上第二组运放的网络标号是 `OPAMP2_*` 而本文件用的是 `OPAMP3`。详见第 13 节待核对项 J（本轮新增，列为头号阻塞项）。代码可以编译、可以初始化，但核实前不建议上电——若那些引脚已被其他网络占用，运放输出同时挂上去可能损坏器件。

**为什么只采两相：** FOC 的 Clarke 变换只需要 `ia` / `ib`，第三相由 `ia + ib + ic = 0` 推出（见第 6.3 节）。这是全工程里**算法需求反向约束配置层**的唯一例子。

**为什么用注入组而不是规则组：** 注入组有独立的 `JDR1`~`JDR4` 数据寄存器，不会被规则组的 DMA 流覆盖；且注入转换可以由 `TIM1_CH4` 精确触发，保证采样点与 PWM 载波同步。电流环要的就是这个确定性。

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
```

三个函数都遵循本工程 L1 层的既定约定：**不含 `main.h`、不调 `Error_Handler()`，失败一律返回 `HAL_ERROR` 交给 `main.c` 处理**。

> 🔑 **这里与用户提供的参考配置有一处刻意差异。** 参考配置返回 `void` 且失败时直接调 `Error_Handler()`，那会让 L1 配置层反向依赖 L6 的入口文件，违反第 2 节的分层约束（`gpio.c` / `i2c3_bus.c` / `tim6_heartbeat.c` / `tim1_pwm.c` 都遵守「返回状态码、不碰 `Error_Handler()`」）。本文件已改写为返回 `HAL_StatusTypeDef`，由 `main.c` 决定如何处置。

**`MX_ADC1_Init()` 的关键配置及其理由：**

| 配置项 | 取值 | 为什么 |
| --- | --- | --- |
| `ClockPrescaler` | `ADC_CLOCK_SYNC_PCLK_DIV4` | ADC12 内核时钟 168 MHz → ADC 时钟 42 MHz（G431 同步模式上限 60 MHz，合规） |
| `InjectedNbrOfConversion` | 2 | rank 1 = `ADC_CHANNEL_3`(PA2)，rank 2 = `ADC_CHANNEL_12`(PB1) |
| `InjectedSamplingTime` | 2.5 cycles | 全表最短档。运放输出阻抗低，短采样足够；⚠ 但与过采样叠加后的总耗时见 9.3 |
| `InjecOversamplingMode` | `ENABLE`，比率 16，右移 `NONE` | 硬件累加 16 次转换后写 JDR 做噪声平均；**不移位**，所以读出的是 16 位累加值 |
| `EOCSelection` | `ADC_EOC_SEQ_CONV` | 整个序列结束才置 EOC，因此读 JDR 时两个通道的数据必然同时有效，不会读到半更新序列 |
| `Overrun` | `ADC_OVR_DATA_PRESERVED` | 溢出时保留旧数据。电流环宁可拿到略旧的一次采样，也不要撕裂的数据 |
| `QueueInjectedContext` | `DISABLE` | 每个 TIM1_CH4 触发点都重新开跑固定序列，适合 20 kHz 等间隔触发 |
| `OversamplingMode` | `DISABLE` | ⚠ 这一项关的是**规则组**过采样，与注入组的 `InjecOversamplingMode` 互不影响，别误读 |
| `DataAlign` | `ADC_DATAALIGN_LEFT` | ⚠ **该赋值被硬件忽略**：过采样使能时 ALIGN 位不起作用，RM0440 §21.4.26 明确「过采样模式下不支持左对齐」。实际是右对齐，见待核对项 K |
| `multimode.Mode` | `ADC_MODE_INDEPENDENT` | 不用 ADC1/ADC2 双模，两相由同一注入序列顺序转换 |

**`software_trigger_flag` 参数的含义：**

| 取值 | 触发源 | 用途 |
| --- | --- | --- |
| 非 0 | `ADC_INJECTED_SOFTWARE_START` | 上电偏置标定。摘除 TIM1_CH4 触发，标定不受 PWM 时序干扰。**当前 `main.c` 传的就是这个** |
| 0 | `ADC_EXTERNALTRIGINJEC_T1_CC4` 上升沿 | 运行时的电流环，采样点跟随载波 |

> 🔑 边沿参数 `ADC_EXTERNALTRIGINJECCONV_EDGE_RISING` 在两个分支下都赋了值，这是安全的：HAL 在触发源为软件启动时会**跳过** `JEXTEN` 的写入（`stm32g4xx_hal_adc_ex.c` 里 `ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START` 的判断，单通道与多通道两个分支都有），所以软件触发模式下这个赋值根本不落到寄存器。
>
> 🔑 **校准顺序有硬性前提：** `HAL_ADCEx_Calibration_Start()` 要求 ADC 处于关闭状态，所以前面显式调了一次 `HAL_ADC_Stop()`。而 `HAL_ADC_Init()` 已经自动完成「退出深掉电模式 → 使能内部稳压器 → 忙等稳压」，所以此处**不需要**额外延时。
>
> 🔑 **配置只把硬件准备好，不接通信号通路**——这与模块 B 的 `MX_TIM1_Init()` 刻意不清 MOE 是同一条分界原则。这里不启动运放、不启动注入转换；启动属于未来的业务层，且必须在 `DrvMonitor_GetState() == DRV_STATE_READY` 且 MOE 置位之后。

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

> 换算成安培还需要采样电阻阻值（原理图标注 0R020 = 20 mΩ）与运放增益（由外部电阻网络决定）。⚠ 在待核对项 J 确认硬件通路之前，这些参数**不应写死进代码**——本轮核实过的外部电阻有 `81.6K` / `6.8K` / `1K` / `5K1` 与 `221`（220 pF）电容，但它们的实际拓扑无法从失真的网表文本还原。

### 9.4 尚未实现的业务层

配置层交出了句柄，但**从原始码到可用电流值之间的所有环节都还不存在**：

```text
待实现（建议放 Modules/Current_Sampling/Src/current_sampling.c）
├── 零电流偏置标定      上电、MOE=0 时软件触发采若干轮，取均值写入 ia_offsets / ib_offsets
├── 启动 / 停止运放      HAL_OPAMP_Start() / HAL_OPAMP_Stop()
├── 启动注入转换         HAL_ADCEx_InjectedStart()（硬件触发模式下不产生 ADC 中断）
├── 原始码读取           HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1/2)
├── 单位换算             JDR → 减偏置 → /16 → ×(VREF / 4095) → /(Rshunt × Gain) → 安培
└── 供 FOC 消费的接口    填 foc_input_t.phase_a / phase_b（见 6.5 节 read_feedback 行）
```

**上层是谁：** 未来的 FOC 端口胶水层（经 `foc_port_t.read_feedback`），以及 L6 `main.c`（初始化与标定编排）。

**读取位置：** 必须在 `TIM1_CC` ISR（优先级 0）里，且要注意 11.43 µs 的序列耗时意味着**中断入口立即读 JDR 拿到的是滞后一个周期的数据**（见待核对项 I）。

> ⚠ `ia_offsets` / `ib_offsets` 目前被链接器当作未使用符号移除（构建日志里可见 `Removing adc1_current_sampling.o(.bss.ia_offsets), (4 bytes)`），这是刻意的——它们是给业务层预留的落点，定义在配置层只是为了让调试器可见、并明确标定结果的归属。实现业务层后会自然被保留。

---

"""

# ============================================================ 5. 第 10 节速查表
OLD_101 = "| 初始化 TIM1 PWM | `MX_TIM1_Init()` | B | L1/L4 | 同上 |"
NEW_101 = """| 初始化 TIM1 PWM | `MX_TIM1_Init()` | B | L1/L4 | 同上 |
| 初始化电流采样运放 | `MX_OPAMP1_Init()` / `MX_OPAMP3_Init()` | F | L1 | **只由 `main()` 各调一次，且必须先于 `MX_ADC1_Init()`** |
| 初始化 ADC1 注入组 | `MX_ADC1_Init(software_trigger_flag)` | F | L1 | 同上；⚠ 上电实测前先解决第 13 节待核对项 J |
| 读相电流原始码 | `HAL_ADCEx_InjectedGetValue(&hadc1, rank)` | F | L1 | ⚠ 业务层未实现；须在 `TIM1_CC` ISR 内，且注意 9.3 的时序滞后 |"""

OLD_102 = "| **L6 入口**（E） | ✅ 全部 | ❌ 不直接调 | ✅ | ✅ | ✅ | ✅ |"
NEW_102 = """| **L1 配置**（F 电流采样） | — | ❌ | ❌ | ❌ | ❌ | ❌ |
| **L6 入口**（E） | ✅ 全部 | ❌ 不直接调 | ✅ | ✅ | ✅ | ✅ |

> 模块 F 目前**只有 L1 配置层**：它不调用任何其他层，只被 L6 `main.c` 在初始化阶段调用一次。将来的业务层（L4）可以读 L1 句柄，但同样不得反向调用 L5/L6。"""

OLD_103 = "- ❌ **新增业务模块的头文件从 `main.h` 借用宏**（头文件只暴露业务接口）"
NEW_103 = """- ❌ **新增业务模块的头文件从 `main.h` 借用宏**（头文件只暴露业务接口）
- ❌ **在注入序列完成前读 `JDR`**（`EOCSelection = ADC_EOC_SEQ_CONV` 只保证不读到半更新序列，不保证数据已就绪；序列耗时见 9.3）
- ❌ **除 L1 配置层与未来业务层外的模块直接操作 `hadc1` / `hopamp1` / `hopamp3`**（采样外设必须单一持有者）
- ❌ **把过采样后的 JDR 当作 12 位左对齐值处理**（实际是右对齐的 16 位累加值，见待核对项 K）"""

OLD_104 = "| 初始化阶段 | 一次性 | 阻塞式 `STSPIN32G4_*`、`DrvMonitor_Init()`、`Tim1PwmVerify_Start()`（含 50 ms 延时） | —（此时无实时任务，阻塞无害） |"
NEW_104 = """| 初始化阶段 | 一次性 | 阻塞式 `STSPIN32G4_*`、`DrvMonitor_Init()`、`Tim1PwmVerify_Start()`（含 50 ms 延时）、`MX_OPAMP1/3_Init()`、`MX_ADC1_Init()`（含阻塞式自校准） | —（此时无实时任务，阻塞无害） |

> ⚠ **模块 F 的注入组刻意不占用独立中断向量。** `HAL_ADC_MspInit()` 里没有配置任何 ADC NVIC——注入组由模块 B 已经拥有的 `TIM1_CC` ISR（优先级 0）消费。再加一个向量只会平白增加中断延迟，还会与第 8.1 节的优先级方案冲突。"""

patch(README, [
    (OLD_J, NEW_J),
    ("## 10. 跨模块调用关系速查", SECTION_9 + "## 10. 跨模块调用关系速查"),
    (OLD_101, NEW_101),
    (OLD_102, NEW_102),
    (OLD_103, NEW_103),
    (OLD_104, NEW_104),
])

print("修正与补全完成")
