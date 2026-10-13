HSE = 8_000_000
PLLM, PLLN, PLLR = 2, 84, 2
vco_in = HSE / PLLM
vco = vco_in * PLLN
sysclk = int(vco / PLLR)
print('VCO in  %.3f MHz  (legal 2.66-8, ST recommends 4)' % (vco_in / 1e6))
print('VCO out %.1f MHz  (legal 128-344)' % (vco / 1e6))
print('SYSCLK  %.1f MHz  (G431 max 170)' % (sysclk / 1e6))
assert vco_in == 4_000_000 and vco == 336_000_000 and sysclk == 168_000_000

ARR = (sysclk // 20000 // 2) - 1          # TIM1_PWM_COUNTER_PERIOD
print('ARR     %d  (=0x%04X, was 0x1067=4199)' % (ARR, ARR))
period_counts = 2 * (ARR + 1)
print('carrier %d Hz  (=168 MHz / %d)' % (sysclk / period_counts, period_counts))

full_scale = ARR + 1                       # TIM1_PWM_COMPARE_FULL_SCALE
for duty in (10, 50, 100, 500, 1000):
    ccr = (duty * full_scale) // 1000
    ocref_ns = 2 * ccr * 1e9 / sysclk
    dtg = 84
    dt_ns = dtg * 1e9 / sysclk
    pin_ns = max(0.0, ocref_ns - dt_ns)
    print('duty %4d -> CCR %4d (0x%04X)  OCREF %8.1f ns  -deadtime %5.1f ns  pin %8.1f ns'
          % (duty, ccr, ccr, ocref_ns, dt_ns, pin_ns))

print('dead time      %.1f ns (DTG=84, tDTS=%.3f ns)' % (84 * 1e9 / sysclk, 1e9 / sysclk))
print('ADC trig lead  %.1f ns (168 counts)' % (168 * 1e9 / sysclk))
print('TIM6 tick      %.1f Hz (PSC=16800)' % (sysclk / 16800))
