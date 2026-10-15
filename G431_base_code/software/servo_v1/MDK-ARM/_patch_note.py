#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把代码注释中「U 相 / W 相」的断言降级为待核对。

原因：原理图网表文本里存在互相矛盾的相别线索——
  两个反馈网络标注为「电机U相电流反馈」「电机W相电流反馈」（U 与 W）
  两个采样电阻网络名为 PDGND1U / PDGND1V（U 与 V）
且无法从文本提取结果可靠判定 rank 1 / rank 2 各自对应哪一相。
在硬件确认之前，注释只描述通道与引脚这一确定事实，相别标注为待核对。
"""
import os, sys

ROOT = r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code"
CS = os.path.join(ROOT, "software", "servo_v1", "Modules", "Current_Sampling")


def patch(path, pairs):
    with open(path, "rb") as f:
        raw = f.read()
    text = raw.decode("utf-8")
    for old, new in pairs:
        n = text.count(old)
        if n != 1:
            print("FAIL %s: 匹配 %d 次（需恰好 1 次）\n---\n%s\n---" % (
                os.path.basename(path), n, old[:400]))
            sys.exit(1)
        text = text.replace(old, new)
    out = text.encode("utf-8")
    with open(path, "wb") as f:
        f.write(out)
    print("OK %-34s %6d -> %6d bytes" % (os.path.basename(path), len(raw), len(out)))


# ---------------------------------------------------------------- 头文件
patch(os.path.join(CS, "Inc", "adc1_current_sampling.h"), [
    ("""  *          引脚映射（原理图 hardware/舵机原理图/主控板.pdf 核实）
  *            PA1  -> OPAMP1_INP (VINP0)   U 相采样电阻分压输入
  *            PA3  -> OPAMP1_INN (VINM0)   U 相反馈网络
  *            PA2  -> OPAMP1_OUT           同时是 ADC1_IN3
  *            PB0  -> OPAMP3_INP (VINP0)   W 相采样电阻分压输入
  *            PB2  -> OPAMP3_INN (VINM0)   W 相反馈网络
  *            PB1  -> OPAMP3_OUT           同时是 ADC1_IN12""",
     """  *          引脚映射（STM32G431 固定复用，已与原理图出现的两组运放网络对应）
  *            PA1  -> OPAMP1_INP (VINP0)   相电流采样输入
  *            PA3  -> OPAMP1_INN (VINM0)   反馈网络
  *            PA2  -> OPAMP1_OUT           同时是 ADC1_IN3   -> 注入 rank 1
  *            PB0  -> OPAMP3_INP (VINP0)   相电流采样输入
  *            PB2  -> OPAMP3_INN (VINM0)   反馈网络
  *            PB1  -> OPAMP3_OUT           同时是 ADC1_IN12  -> 注入 rank 2
  *
  *          ⚠ 两路各自对应哪一相（U / V / W）尚未确认：原理图里两个反馈网络
  *          标注为「电机U相电流反馈」「电机W相电流反馈」，但两个采样电阻的
  *          网络名是 PDGND1U / PDGND1V（U 与 V），两者不一致。在硬件核对清楚
  *          之前，本文件只描述「通道 / 引脚 / rank」这些确定事实，不断言相别。
  *          见 README 待核对项 J。ia_offsets / ib_offsets 的 a / b 同样只是
  *          沿用参考配置的命名，不代表已确认的相别。"""),
    ("""/* 触发提前量：TIM1_CH4 比较点比更新事件早 1 µs（模块 B 的
   TIM1_PWM_ADC_TRIGGER_LEAD_NS）。此处只做记录，实际值由模块 B 决定。 */""",
     """/* 触发提前量：TIM1_CH4 比较点比更新事件早 1 µs（模块 B 的
   TIM1_PWM_ADC_TRIGGER_LEAD_NS）。此处只做记录，实际值由模块 B 决定。
   ⚠ 该提前量远小于 ADC1_CS_SEQUENCE_CONV_NS，时序余量见 README 待核对项 I。 */"""),
])

# ---------------------------------------------------------------- 实现文件
patch(os.path.join(CS, "Src", "adc1_current_sampling.c"), [
    ("  /* rank 1：ADC_CHANNEL_3 = PA2 = OPAMP1 输出（U 相） */",
     "  /* rank 1：ADC_CHANNEL_3 = PA2 = OPAMP1 输出。\n"
     "     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */"),
    ("  /* rank 2：ADC_CHANNEL_12 = PB1 = OPAMP3 输出（W 相） */",
     "  /* rank 2：ADC_CHANNEL_12 = PB1 = OPAMP3 输出。\n"
     "     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */"),
    ("  *   时序余量需要实测确认，见 README 第 12 节待核对项 I。",
     "  *   时序余量需要实测确认，见 README「待核对项」I。"),
])

# ------------------------------------------------------------------ main.c
patch(os.path.join(ROOT, "software", "servo_v1", "Core", "Src", "main.c"), [
    ("   * 都会多做一轮 11.43 µs 的过采样转换（见第 12 节待核对项 I 的时序余量\n"
     "   * 问题），而目前还没有任何消费者去读这些数据。",
     "   * 都会多做一轮 11.43 µs 的过采样转换（时序余量问题见 README「待核对项」\n"
     "   * I），而目前还没有任何消费者去读这些数据。"),
])

print("相别断言降级完成")
