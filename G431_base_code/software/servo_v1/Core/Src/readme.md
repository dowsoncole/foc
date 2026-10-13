![1791791884281](image/readme/1791791884281.png)

1, 根据图片和stspin32431芯片的数据手册，配置mcu引脚和外设。包括i2c, 和ready,nfault,wake.修改要符合hal工程规范，包括文件组织方式，和代码风格。

2, 新增TIM1三相互补PWM配置（tim1_pwm.c/h），以及上电验证序列（tim1_pwm_verify.c/h）。

   | 文件 | 职责 |
   | --- | --- |
   | `Core/Src/tim1_pwm.c` | TIM1 初始化：中央对齐 20 kHz，CH1..CH3 互补输出 + 500 ns 死区，CH4 作为 ADC 采样触发；duty -> compare 映射 |
   | `Core/Inc/tim1_pwm.h` | 定时参数（载波、死区、duty 标度）与 PWM 控制接口 |
   | `Core/Src/tim1_pwm_verify.c` | 验证序列：静态三路占空比图形 + 1 Hz 开环 V/f 慢扫，带看门狗 |
   | `Core/Inc/tim1_pwm_verify.h` | 验证序列的整定值与接口 |

   复用文件：`Core/Src/stm32g4xx_hal_msp.c`（TIM1 时钟 / NVIC / PE8..PE13 复用）、
   `Core/Src/stm32g4xx_it.c`（TIM1_CC、TIM1_UP_TIM16 向量）、`Core/Src/main.c`（调用顺序）。

3,
