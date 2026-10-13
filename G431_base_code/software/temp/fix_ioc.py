import re, io, sys

p = r'C:\Users\penghao\Desktop\Servo_9948\G431_base_code\software\servo_v1\servo_v1.ioc'
raw = io.open(p, 'rb').read()
assert not raw.startswith(b'\xef\xbb\xbf')
t = raw.decode('utf-8')

pairs = [
    (r'^RCC\.HSE_VALUE=12000000(\r?)$',            r'RCC.HSE_VALUE=8000000\1',            'HSE_VALUE'),
    (r'^RCC\.EXTERNAL_CLOCK_VALUE=12000000(\r?)$',  r'RCC.EXTERNAL_CLOCK_VALUE=8000000\1', 'EXTERNAL_CLOCK_VALUE'),
    (r'^RCC\.PLLM=RCC_PLLM_DIV1(\r?)$',             r'RCC.PLLM=RCC_PLLM_DIV2\1',           'PLLM'),
    (r'^RCC\.PLLN=28(\r?)$',                        r'RCC.PLLN=84\1',                      'PLLN'),
    (r'^RCC\.VCOInputFreq_Value=12000000(\r?)$',    r'RCC.VCOInputFreq_Value=4000000\1',   'VCOInputFreq'),
]
for pat, rep, tag in pairs:
    t, n = re.subn(pat, rep, t, flags=re.M)
    if n != 1:
        print('FAIL %s matched %d' % (tag, n)); sys.exit(1)
    print('OK %s' % tag)

io.open(p, 'wb').write(t.encode('utf-8'))
print('IOC DONE')
