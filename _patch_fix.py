#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修正防下溢检查宏：改为按计数判据，与 CH4 比较值的实际算法一致。

真实约束：Pulse = ARR - NsToCounts(LEAD) 是无符号减法，要求
  NsToCounts(LEAD) <= ARR (= TIM1_PWM_COUNTER_PERIOD = 4199)
即 LEAD <= 4199 * 1000 / 168 = 24994 ns。
（原先写成与「一个载波周期 50 µs」比较，既单位错配又界限过松。）

中间量最大 24994 * 168 = 4198992，远小于 uint32 上限，无溢出风险。
"""
import os, sys

H = (r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\servo_v1"
     r"\Modules\Three_Phase_Driver\Inc\tim1_pwm.h")

OLD = """/* 防下溢：CH4 的比较值由 ARR - NsToCounts(LEAD) 得出，是无符号减法。
   提前量若达到或超过一个载波周期（50 µs = 8400 计数），减法会回绕成一个
   远大于 ARR 的值，比较寄存器将永远匹配不上，ADC 触发彻底失效且不会报错。
   这是一个静默失效模式，所以在编译期就挡住。 */
#define TIM1_PWM_ADC_LEAD_FITS_PERIOD \\
  (TIM1_PWM_ADC_TRIGGER_LEAD_NS < (1000000UL / TIM1_PWM_FREQUENCY_HZ))

#if (TIM1_PWM_ADC_LEAD_FITS_PERIOD == 0)
#error "TIM1_PWM_ADC_TRIGGER_LEAD_NS 必须小于一个载波周期，否则 CH4 比较值会下溢回绕，ADC 触发静默失效"
#endif"""

NEW = """/* 提前量换算成计数，公式与 tim1_pwm.c 的 Tim1Pwm_NsToCounts() 一致。
   5714 ns * 168 / 1000 = 959 计数。 */
#define TIM1_PWM_ADC_LEAD_COUNTS \\
  ((TIM1_PWM_ADC_TRIGGER_LEAD_NS * (TIM1_PWM_TIMER_CLOCK_HZ / 1000000UL)) / 1000UL)

/* 防下溢：CH4 的比较值由 ARR - NsToCounts(LEAD) 得出，是无符号减法，
   所以提前量的计数值必须不超过 ARR（= TIM1_PWM_COUNTER_PERIOD = 4199，
   即提前量上限 24994 ns）。注意界限是半周期计数 ARR，不是整个载波周期：
   中央对齐下计数器只走到 ARR 就折返。
   一旦越界，减法会回绕成一个远大于 ARR 的值，比较寄存器永远匹配不上，
   ADC 触发彻底失效且不会报错——静默失效，所以在编译期就挡住。 */
#define TIM1_PWM_ADC_LEAD_FITS_PERIOD \\
  (TIM1_PWM_ADC_LEAD_COUNTS <= TIM1_PWM_COUNTER_PERIOD)

#if (TIM1_PWM_ADC_LEAD_FITS_PERIOD == 0)
#error "TIM1_PWM_ADC_TRIGGER_LEAD_NS 过大：其计数值必须 <= TIM1_PWM_COUNTER_PERIOD，否则 CH4 比较值会下溢回绕，ADC 触发静默失效"
#endif"""

with open(H, "rb") as f:
    raw = f.read()
text = raw.decode("utf-8")
n = text.count(OLD)
if n != 1:
    print("FAIL: 匹配 %d 次（需恰好 1 次）" % n)
    sys.exit(1)
text = text.replace(OLD, NEW)
with open(H, "wb") as f:
    f.write(text.encode("utf-8"))
print("OK tim1_pwm.h %d -> %d bytes" % (len(raw), len(text.encode("utf-8"))))

# 校验：5714 ns -> 959 计数 <= 4199，通过；上限 24994 ns -> 4199 计数，仍通过
for lead in (5714, 24994, 25000):
    counts = lead * 168 // 1000
    print("  LEAD=%5d ns -> %4d 计数, ARR=%d, 检查=%s" % (
        lead, counts, 4199, "通过" if counts <= 4199 else "拦截"))
