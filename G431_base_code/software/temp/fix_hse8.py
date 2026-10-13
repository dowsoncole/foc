import re, io, sys

ROOT = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\servo_v1'

def load(p):
    with io.open(p, 'rb') as f:
        raw = f.read()
    assert not raw.startswith(b'\xef\xbb\xbf'), 'unexpected BOM in ' + p
    return raw.decode('utf-8')

def save(p, s):
    with io.open(p, 'wb') as f:
        f.write(s.encode('utf-8'))

def sub_once(text, pattern, repl, path, tag):
    new, n = re.subn(pattern, repl, text)
    if n != 1:
        print('FAIL [%s] %s matched %d times' % (path, tag, n))
        sys.exit(1)
    return new

# ---- 1. main.c : PLL 参数 (CRLF 文件) -------------------------------------
p = ROOT + r'\Core\Src\main.c'
t = load(p)
old_crlf = ('  /* X1 is 12 MHz: 12 MHz / 1 * 28 = 336 MHz VCO, /2 = 168 MHz SYSCLK. */\r\n'
            '  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;\r\n'
            '  RCC_OscInitStruct.PLL.PLLN = 28;\r\n')
new_crlf = ('  /* X1 is 8 MHz: 8 MHz / 2 = 4 MHz VCO input, * 84 = 336 MHz VCO, /2 = 168 MHz\r\n'
            '     SYSCLK. A 4 MHz VCO input is what ST recommends for the lowest PLL jitter,\r\n'
            '     and SYSCLK deliberately stays at 168 MHz so every derived timing constant\r\n'
            '     keeps holding: the 20 kHz TIM1 carrier, the 500 ns dead time, the 10 kHz\r\n'
            '     TIM6 tick (prescaler 16800) and the I2C timing word. */\r\n'
            '  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;\r\n'
            '  RCC_OscInitStruct.PLL.PLLN = 84;\r\n')
if old_crlf not in t:
    print('FAIL main.c PLL block not found verbatim'); sys.exit(1)
t = t.replace(old_crlf, new_crlf, 1)
save(p, t)
print('OK main.c: PLLM DIV1->DIV2, PLLN 28->84')

# ---- 2. stm32g4xx_hal_conf.h : HSE_VALUE / EXTERNAL_CLOCK_VALUE ------------
p = ROOT + r'\Core\Inc\stm32g4xx_hal_conf.h'
t = load(p)
t = sub_once(t,
             r'/\* The board fitted crystal X1 \(TXC AV12000007\) is 12 MHz\. \*/(\r?\n)'
             r'(  #define HSE_VALUE    \()12000000UL(\))',
             r'/* The board fitted crystal X1 is 8 MHz. */\1\g<2>8000000UL\3',
             p, 'HSE_VALUE')
t = sub_once(t,
             r'(#define EXTERNAL_CLOCK_VALUE    \()12000000UL(\))',
             r'\g<1>8000000UL\2',
             p, 'EXTERNAL_CLOCK_VALUE')
save(p, t)
print('OK stm32g4xx_hal_conf.h: HSE_VALUE 12MHz->8MHz, EXTERNAL_CLOCK_VALUE 12MHz->8MHz')

# ---- 3. system_stm32g4xx.c : HSE_VALUE 兜底定义 + 注释 ---------------------
p = ROOT + r'\Core\Src\system_stm32g4xx.c'
t = load(p)
t = sub_once(t,
             r'/\* The board fitted crystal X1 \(TXC AV12000007\) is 12 MHz\. \*/(\r?\n)'
             r'(  #define HSE_VALUE     )12000000U( )',
             r'/* The board fitted crystal X1 is 8 MHz. */\1\g<2>8000000U\3',
             p, 'HSE_VALUE')
t = sub_once(t, r'12 MHz for this board', r'8 MHz for this board', p, 'doc comment')
save(p, t)
print('OK system_stm32g4xx.c: HSE_VALUE 12MHz->8MHz, doc comment updated')

# ---- 4. tim1_pwm.h : 时钟来源注释 -----------------------------------------
p = ROOT + r'\Core\Inc\tim1_pwm.h'
t = load(p)
t = sub_once(t,
             r'SYSCLK = HSE 12 MHz / 1 \* 28 / 2 = 168 MHz',
             r'SYSCLK = HSE 8 MHz / 2 * 84 / 2 = 168 MHz',
             p, 'clock comment')
save(p, t)
print('OK tim1_pwm.h: clock-derivation comment updated')

# ---- 5. servo_v1.ioc : CubeMX 侧同步，避免下次生成代码回退 -----------------
p = ROOT + r'\servo_v1.ioc'
t = load(p)
for pat, rep, tag in [
    (r'^RCC\.HSE_VALUE=12000000$',            'RCC.HSE_VALUE=8000000',      'HSE_VALUE'),
    (r'^RCC\.EXTERNAL_CLOCK_VALUE=12000000$', 'RCC.EXTERNAL_CLOCK_VALUE=8000000', 'EXTERNAL_CLOCK_VALUE'),
    (r'^RCC\.PLLM=RCC_PLLM_DIV1$',            'RCC.PLLM=RCC_PLLM_DIV2',     'PLLM'),
    (r'^RCC\.PLLN=28$',                       'RCC.PLLN=84',                'PLLN'),
    (r'^RCC\.VCOInputFreq_Value=12000000$',   'RCC.VCOInputFreq_Value=4000000', 'VCOInputFreq'),
]:
    t = sub_once(t, pat, rep, p, tag)
save(p, t)
print('OK servo_v1.ioc: HSE/PLLM/PLLN/VCOInput synced')

print('ALL DONE')
