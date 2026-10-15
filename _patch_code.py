#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""落实两项决策：
  1. rank 1 = U 相，rank 2 = V 相（相别归属已确认）；
  2. CH4 提前量 = 注入序列耗时的一半 = 11428/2 = 5714 ns。

数值核算：
  NsToCounts(5714) = 5714 * 168 / 1000 = 959 计数
  CH4 Pulse        = ARR(4199) - 959   = 3240        （安全落在 0..4199）
  959 计数 * 5.952 ns = 5708 ns ≈ 5.71 µs（整数除法截断 6 ns，可忽略）
  注入序列 11.43 µs，正好以更新事件为中心：T-5.71 µs 触发、T+5.71 µs 结束

同时补一道编译期防下溢检查：CH4 比较值是无符号减法 ARR - NsToCounts(LEAD)，
提前量若 >= 一个载波周期（50 µs）会回绕成巨大值，比较寄存器永远匹配不上，
ADC 触发彻底失效且不报错。

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
                os.path.basename(path), n, old[:260]))
            sys.exit(1)
        text = text.replace(old, new)
    out = bom + text.encode("utf-8")
    with open(path, "wb") as f:
        f.write(out)
    print("OK %-30s %6d -> %6d  BOM=%-5s CRLF=%s" % (
        os.path.basename(path), len(raw), len(out), bool(bom), crlf))


# ============================================ 1. tim1_pwm.h：提前量 + 防下溢
patch(os.path.join(SW, "Modules", "Three_Phase_Driver", "Inc", "tim1_pwm.h"), [
    ("""/* ADC 采样相对 20 kHz 更新事件提前 1 µs；一个计数是 5.952 ns，
   即 ADC 触发提前 168 个计数。 */
#define TIM1_PWM_ADC_TRIGGER_LEAD_NS 1000U""",
     """/* ADC 采样相对 20 kHz 更新事件提前的时间；一个计数是 5.952 ns。

   取值 = 模块 F 注入序列总耗时的一半（11.43 µs / 2 = 5.714 µs），
   即 959 个计数，CH4 比较值 = 4199 - 959 = 3240。这样注入序列正好以更新
   事件为中心：T - 5.71 µs 触发、T + 5.71 µs 结束，两个通道的采样点分别
   落在更新事件两侧对称位置。

   ⚠ 本值与模块 F 的 ADC1_CS_SEQUENCE_CONV_NS 强耦合。改动过采样倍率、
   采样时间或注入通道数时，必须同步这里（见 README 第 14 节规则 9）。 */
#define TIM1_PWM_ADC_TRIGGER_LEAD_NS 5714U

/* 防下溢：CH4 的比较值由 ARR - NsToCounts(LEAD) 得出，是无符号减法。
   提前量若达到或超过一个载波周期（50 µs = 8400 计数），减法会回绕成一个
   远大于 ARR 的值，比较寄存器将永远匹配不上，ADC 触发彻底失效且不会报错。
   这是一个静默失效模式，所以在编译期就挡住。 */
#define TIM1_PWM_ADC_LEAD_FITS_PERIOD \\
  (TIM1_PWM_ADC_TRIGGER_LEAD_NS < (1000000UL / TIM1_PWM_FREQUENCY_HZ))

#if (TIM1_PWM_ADC_LEAD_FITS_PERIOD == 0)
#error "TIM1_PWM_ADC_TRIGGER_LEAD_NS 必须小于一个载波周期，否则 CH4 比较值会下溢回绕，ADC 触发静默失效"
#endif"""),
])

# ==================================== 2. adc1_current_sampling.h：相别 + 提前量
patch(os.path.join(SW, "Modules", "Current_Sampling", "Inc", "adc1_current_sampling.h"), [
    ("""  *            PA1  -> OPAMP1 VINP0        相电流采样输入
  *            PA3  -> OPAMP1 VINM0        反馈网络
  *            PA2  -> OPAMP1 VOUT         同时是 ADC1_IN3   -> 注入 rank 1
  *            PB0  -> OPAMP3 VINP0        相电流采样输入
  *            PB2  -> OPAMP3 VINM0        反馈网络
  *            PB1  -> OPAMP3 VOUT         同时是 ADC1_IN12  -> 注入 rank 2""",
     """  *            PA1  -> OPAMP1 VINP0        U 相电流采样输入
  *            PA3  -> OPAMP1 VINM0        U 相反馈网络
  *            PA2  -> OPAMP1 VOUT         同时是 ADC1_IN3   -> 注入 rank 1 = U 相
  *            PB0  -> OPAMP3 VINP0        V 相电流采样输入
  *            PB2  -> OPAMP3 VINM0        V 相反馈网络
  *            PB1  -> OPAMP3 VOUT         同时是 ADC1_IN12  -> 注入 rank 2 = V 相"""),
    ("""  *          只采两相：FOC 算法的 Clarke 变换只需 ia / ib，第三相由
  *          ia + ib + ic = 0 推出。这是算法需求反向约束配置层的唯一例子。
  *
  *          ⚠ 两路各自对应哪一相（U / V / W）尚未实测确认，见 README 第 13 节
  *          待核对项 J。在确认之前本文件只描述「通道 / 引脚 / rank」这些确定
  *          事实，不断言相别；ia_offsets / ib_offsets 的 a / b 也只是沿用参考
  *          配置的命名。""",
     """  *          只采两相：FOC 算法的 Clarke 变换只需 ia / ib，第三相由
  *          ia + ib + ic = 0 推出。这是算法需求反向约束配置层的唯一例子。
  *
  *          相别映射（已确认）：rank 1 = U 相 = foc_input_t.phase_a = ia，
  *                            rank 2 = V 相 = foc_input_t.phase_b = ib。
  *          这与标准 Clarke 变换的 alpha = ia、beta = (ia + 2*ib)/sqrt(3) 一致，
  *          W 相由 -(ia + ib) 推出，无需采样。ia_offsets / ib_offsets 的
  *          a / b 即对应 U / V 两相。"""),
    ("""/* 注入序列共两个通道，顺序转换，全部就绪需要约 11.43 µs。
   ⚠ 与 TIM1_CH4 触发点的时序关系见 README 第 13 节待核对项 I。 */
#define ADC1_CS_SEQUENCE_CONV_NS    (ADC1_CS_OVERSAMPLED_CONV_NS * 2UL)

/* 触发提前量：TIM1_CH4 比较点比更新事件早 1 µs（模块 B 的
   TIM1_PWM_ADC_TRIGGER_LEAD_NS）。此处只做记录，实际值由模块 B 决定。
   ⚠ 该提前量远小于 ADC1_CS_SEQUENCE_CONV_NS，时序余量见 README 待核对项 I。 */
#define ADC1_CS_TRIGGER_LEAD_NS     1000UL""",
     """/* 注入序列共两个通道，顺序转换，全部就绪需要约 11.43 µs。 */
#define ADC1_CS_SEQUENCE_CONV_NS    (ADC1_CS_OVERSAMPLED_CONV_NS * 2UL)

/* 触发提前量：TIM1_CH4 比较点比更新事件早 5.714 µs，即本序列耗时的一半
   （实际写入 CCR4 的值由模块 B 的 TIM1_PWM_ADC_TRIGGER_LEAD_NS 决定，
   此处只做记录，便于两边对照）。

   取一半是让注入序列正好以更新事件为中心：T - 5.71 µs 触发、T + 5.71 µs
   结束，两个通道的采样点分别落在更新事件两侧对称位置。

   ⚠ 改动过采样倍率、采样时间或通道数会改变 ADC1_CS_SEQUENCE_CONV_NS，
   届时必须同步模块 B 的 TIM1_PWM_ADC_TRIGGER_LEAD_NS（见 README 第 14 节
   规则 9）。两边不一致不会报错，只会让采样点偏离载波的预期位置。 */
#define ADC1_CS_TRIGGER_LEAD_NS     (ADC1_CS_SEQUENCE_CONV_NS / 2UL)"""),
    ("""  * 注入序列 rank 1 = ADC_CHANNEL_3（PA2，OPAMP1 输出），
  *            rank 2 = ADC_CHANNEL_12（PB1，OPAMP3 输出）。
  * ⚠ 两路各自对应哪一相尚未实测确认，见 README 第 13 节待核对项 J。""",
     """  * 注入序列 rank 1 = ADC_CHANNEL_3（PA2，OPAMP1 输出）= U 相，
  *            rank 2 = ADC_CHANNEL_12（PB1，OPAMP3 输出）= V 相。"""),
])

# ==================================== 3. adc1_current_sampling.c：相别注释
patch(os.path.join(SW, "Modules", "Current_Sampling", "Src", "adc1_current_sampling.c"), [
    ("""  /* 独立（STANDALONE）模式：同相输入 IO0 = PA1，反相输入 IO0 = PA3。""",
     """  /* U 相电流放大。
     独立（STANDALONE）模式：同相输入 IO0 = PA1，反相输入 IO0 = PA3。"""),
    ("""  /* 独立跟随模式：同相输入 IO0 = PB0，反相输入 IO0 = PB2，输出 PB1。""",
     """  /* V 相电流放大。
     独立跟随模式：同相输入 IO0 = PB0，反相输入 IO0 = PB2，输出 PB1。"""),
    ("""  /* rank 1：ADC_CHANNEL_3 = PA2 = OPAMP1 输出。
     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */""",
     """  /* rank 1：ADC_CHANNEL_3 = PA2 = OPAMP1 输出 = U 相 = foc_input_t.phase_a。 */"""),
    ("""  /* rank 2：ADC_CHANNEL_12 = PB1 = OPAMP3 输出。
     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */""",
     """  /* rank 2：ADC_CHANNEL_12 = PB1 = OPAMP3 输出 = V 相 = foc_input_t.phase_b。
     W 相不采样，由 -(ia + ib) 推出。 */"""),
    ("""  *   单次转换 = 采样 2.5 cycles + 12 位逐次逼近 12.5 cycles = 15 cycles
  *   -> 357 ns；过采样 16 倍 -> 5.71 µs / 通道，两通道序列 11.43 µs。
  *
  * ⚠ 采样时间 2.5 cycles 是全表最短档，且过采样会把单通道耗时放大 16 倍，
  *   两个通道合计 11.43 µs，已明显超过 TIM1_CH4 相对更新事件 1 µs 的提前量。
  *   时序余量需要实测确认，见 README「待核对项」I。""",
     """  *   单次转换 = 采样 2.5 cycles + 12 位逐次逼近 12.5 cycles = 15 cycles
  *   -> 357 ns；过采样 16 倍 -> 5.71 µs / 通道，两通道序列 11.43 µs。
  *
  * 触发时序：TIM1_CH4 的提前量取本序列耗时的一半（5.714 µs），使序列正好以
  *   更新事件为中心——T - 5.71 µs 触发、T + 5.71 µs 结束。详见 README 9.3。"""),
    ("""     ⚠ 与 TIM1_CH4 触发提前量的时序余量见 README 第 13 节待核对项 I。 */""",
     """     ⚠ 提前量取序列耗时的一半，见 TIM1_PWM_ADC_TRIGGER_LEAD_NS。 */"""),
])

# ==================================== 4. MSP：W phase -> V phase
patch(os.path.join(SW, "Core", "Src", "stm32g4xx_hal_msp.c"), [
    ("""  *         OPAMP1: PA1 (VINP0) / PA3 (VINM0) / PA2 (OUT)   -> U phase
  *         OPAMP3: PB0 (VINP0) / PB2 (VINM0) / PB1 (OUT)   -> W phase""",
     """  *         OPAMP1: PA1 (VINP0) / PA3 (VINM0) / PA2 (OUT)   -> U phase (rank 1)
  *         OPAMP3: PB0 (VINP0) / PB2 (VINM0) / PB1 (OUT)   -> V phase (rank 2)"""),
])

# ==================================== 5. main.c：相别与触发时序注释
patch(os.path.join(SW, "Core", "Src", "main.c"), [
    ("""   * 零电流偏置标定和 JDR 到安培的换算仍属于业务层，接入 FOC 前还需按 README
   * 待核对项确认采样序列时序以及两路通道的实际相别。 */""",
     """   * 相别已确认：rank 1 = U 相 = foc_input_t.phase_a，rank 2 = V 相 =
   * foc_input_t.phase_b，W 相由 -(ia + ib) 推出。
   * CH4 提前量已改为注入序列耗时的一半（5.714 µs），使序列以更新事件为中心。
   *
   * 零电流偏置标定和 JDR 到安培的换算仍属于业务层，尚未实现。 */"""),
])

print("代码修改完成")
