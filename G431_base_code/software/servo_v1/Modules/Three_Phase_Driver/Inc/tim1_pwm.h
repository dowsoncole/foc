/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm.h
  * @brief   面向 STSPIN32G4 功率级的 TIM1 三相互补 PWM。
  *
  *          引脚映射（TIM1 通道 → 栅极驱动输入 → 电机相）
  *            PE8  -> TIM1_CH1N   PE9  -> TIM1_CH1    -> GD INL1 / INH1  (A 相)
  *            PE10 -> TIM1_CH2N   PE11 -> TIM1_CH2    -> GD INL2 / INH2  (B 相)
  *            PE12 -> TIM1_CH3N   PE13 -> TIM1_CH3    -> GD INL3 / INH3  (C 相)
  *
  *          引脚映射与 GPIO 配置依据 STSPIN32G4 数据手册
  *          "Table 7. MCU to gate driver internal connection"：六路栅极信号
  *          全部必须是推挽复用输出，栅极驱动 IC 只接受这一种 GPIO 配置。
  *
  *          本头文件同时承担两个角色：
  *            配置线 —— 硬件时序常量 + MX_TIM1_Init()
  *            业务线 —— Tim1Pwm_* 功率级控制接口 + 占空比标度常量
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM1_PWM_H__
#define __TIM1_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* --------------------------------------------------------------------------*/
/* 时序参数（配置线）                                                        */
/* --------------------------------------------------------------------------*/
/* SystemClock_Config() 把 SYSCLK 设为 HSE 8 MHz / 2 * 84 / 2 = 168 MHz，
   APB2 分频为 1，所以 TIM1 的内核时钟就是 168 MHz。 */
#define TIM1_PWM_TIMER_CLOCK_HZ     168000000UL

/* 中央对齐 PWM：计数器按 0 -> ARR -> 0 往复，因此一个载波周期占用
   2 x (ARR + 1) 个计数时钟；重复计数器保证每个载波周期恰好产生一次更新事件。
   这里把「装入 ARR 的峰值」和「一个完整周期的计数总数」分别命名，
   因为比较寄存器是以后者为基准来表达的。 */
#define TIM1_PWM_FREQUENCY_HZ       20000UL
#define TIM1_PWM_COUNTS_PER_PERIOD  (TIM1_PWM_TIMER_CLOCK_HZ / TIM1_PWM_FREQUENCY_HZ)
#define TIM1_PWM_COUNTER_PERIOD     ((TIM1_PWM_COUNTS_PER_PERIOD / 2UL) - 1UL)

/* 主输出与其互补输出之间插入的死区时间。TIM_BDTR_DTG 取值 0x00..0x7F 时
   以 tDTS 为单位（时钟分频 DIV1 -> 1/168 MHz = 5.952 ns），故 84 计数 = 500 ns。
   该编码段的上限是 127 计数；更长的死区需要改用 0x80..0xBF 编码（步长 2 x tDTS）。 */
#define TIM1_PWM_DEADTIME_TDTS      84U

/* 一个 tDTS 是 1 / 168 MHz = 5.952 ns，所以 84 计数 = 500 ns。
   修复：原写法 (TDTS * 1000000000UL) 会让 84e9 超出 uint32 上限并回绕，
   宏求值结果是 14 而不是 500。改成先除以 MHz 再乘 1000，中间量最大只有
   84000，不会溢出，结果精确等于 500 ns。 */
#define TIM1_PWM_DEADTIME_NS        ((TIM1_PWM_DEADTIME_TDTS * 1000UL) / \
                                     (TIM1_PWM_TIMER_CLOCK_HZ / 1000000UL))

/* 应用层占空比标度：表示一个载波周期内半桥「高侧」开关导通的时间占比。
   0 = 高侧从不导通，1000 = 高侧常通，500 = 50 %。
   它能整除 TIM1_PWM_COUNTS_PER_PERIOD，因此一个占空比步长对应 8.4 个比较计数。 */
#define TIM1_PWM_DUTY_SCALE         1000U
#define TIM1_PWM_DUTY_NEUTRAL       (TIM1_PWM_DUTY_SCALE / 2U)

/* ADC 采样相对 20 kHz 更新事件提前 1 µs；一个计数是 5.952 ns，
   即 ADC 触发提前 168 个计数。 */
#define TIM1_PWM_ADC_TRIGGER_LEAD_NS 1000U

/* --------------------------------------------------------------------------*/
/* 寄存器级换算                                                              */
/* --------------------------------------------------------------------------*/
/* 栅极驱动由互补输出驱动：TIM1_CH1..3 承载高侧信号（数据手册
   "MCU to gate driver internal connection" 表：PE9 -> CH1 -> INH1，
   PE8 -> CH1N -> INL1），低侧由栅极驱动 IC 内部对主信号取反得到。

   PWM 模式 1、中央对齐下，主输出的占空比是「计数器低于比较值」的时间占比。
   下面所有常量都由两个事实决定：

     1. 比较值与占空比成正比，不是它的补。比较值越大，高侧导通时间越长。
     2. 超过 ARR 的比较值不再翻转：计数器到不了那个值，输出会一直停在有效
        电平，而不是把占空比范围扩大。因此可用比较带是 0..ARR，
        只有标称计数周期 2 x (ARR + 1) 的一半。若按整周期而不是按比较带来
        缩放占空比，标度的上半段就会饱和：任何大于 50 % 的占空比都会被
        钉在接近 100 %。

   以计数器序列本身为基准（中央对齐，CNT < CCR 时有效，
   一个载波周期 = 2 x ARR = 8398 个计数时钟）实测：
     CCR    0 ->   0.0000 % 高侧      duty   0 ->   0.0000 %
     CCR 2100 ->  50.0000 %           duty 500 ->  50.0000 %
     CCR 2520 ->  60.0024 %           duty 600 ->  60.0024 %
     CCR 4199 ->  99.9881 %           duty1000 ->  99.9881 %
     CCR 4200 -> 100.0000 %（同样饱和）
   整数除法的残余误差最大为周期的 0.0119 %，所以标度的 50 % 点
   精确落在载波的 50 %。 */

/* 表示「高侧常通」的比较值，也就是可用比较带的顶端。
   ARR + 1 会比顶端多一个计数，因此在使用处被钳位到 ARR。 */
#define TIM1_PWM_COMPARE_FULL_SCALE (TIM1_PWM_COUNTER_PERIOD + 1UL)

/* 50 % 点对应的比较值，也是空闲模式装载的值。 */
#define TIM1_PWM_COMPARE_NEUTRAL    TIM1_PWM_DUTY_TO_COMPARE(TIM1_PWM_DUTY_NEUTRAL)

/**
  * @brief  把 0..TIM1_PWM_DUTY_SCALE 的高侧占空比换算成装入通道的比较值。
  *
  * 按比较带缩放：duty 0 装载 0（高侧从不导通），duty TIM1_PWM_DUTY_SCALE
  * 装载 ARR + 1，使用处会把它钳位到 ARR（高侧常通）。比较带里的 +1
  * 让 50 % 点恰好落在 (ARR + 1) 的一半，而不是比它少一个计数。
  *
  * 不要把这个映射取反，也不要按 TIM1_PWM_COUNTS_PER_PERIOD 缩放：
  * 比较值与占空比成正比，而任何超过 ARR 的值都会停止翻转而不是扩大范围。
  */
#define TIM1_PWM_DUTY_TO_COMPARE(duty)                                        \
  (((uint32_t)(duty) * TIM1_PWM_COMPARE_FULL_SCALE) / TIM1_PWM_DUTY_SCALE)

/* --------------------------------------------------------------------------*/
/* 导出变量                                                                  */
/* --------------------------------------------------------------------------*/
/* 声明为非 static，一是让句柄出现在调试器 Watch 窗口，二是让中断层能寻址它。
   只有 tim1_pwm.c 会修改它。 */
extern TIM_HandleTypeDef htim1;

/* --------------------------------------------------------------------------*/
/* 导出函数                                                                  */
/* --------------------------------------------------------------------------*/

/**
  * @brief  【配置线】配置并启动 TIM1：通道 1..3 输出 20 kHz 中央对齐互补 PWM，
  *         死区 500 ns；通道 4 作为 ADC 触发，位置比更新事件提前
  *         TIM1_PWM_ADC_TRIGGER_LEAD_NS。
  *
  * 计数器、各通道输出和 CC4 中断都在这里启动，但 MOE 保持清零，
  * 因此没有任何栅极信号到达功率级。使能功率级是一个独立的显式步骤：
  * Tim1Pwm_EnableOutputs()。
  *
  * 只能在系统时钟配好之后由 main() 调用一次。
  * @retval HAL_OK；死区无法编码或时钟树与预期不符时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_TIM1_Init(void);

/**
  * @brief  【配置线】六路栅极输出的 GPIO 配置，实现在 MSP 层
  *         （Core/Src/stm32g4xx_hal_msp.c）。由 MX_TIM1_Init() 末尾自动调用。
  * @note   必须在定时器配置完成之后才初始化引脚，否则初始化过程中引脚会产生
  *         毛刺，可能误触发栅极驱动。
  */
void     HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
  * @brief  【业务线】设置某一相的高侧占空比（0..TIM1_PWM_DUTY_SCALE，500 = 50 %）。
  *         超出标度的值会被钳位。比较寄存器以单次写入完成，因此中断服务程序
  *         可以在定时器运行时调用本函数。
  * @param  channel TIM_CHANNEL_1 / _2 / _3 之一。
  * @retval HAL_OK；channel 不是 PWM 相时返回 HAL_ERROR。
  * @note   两个端点都会饱和。duty 1000 让低侧整个周期都不导通，duty 0 让低侧
  *         整个周期都导通；两者都会使该半桥的自举电容无法刷新，而 duty 0 还会
  *         通过低侧对电机制动。运行中的相请保持在标度内部，空闲态请用中性点
  *         模式，绝不要把任一端点当作空闲值使用。
  */
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty);

/**
  * @brief  根据归一化 α/β 电压矢量更新三相 PWM（对称 SVPWM）。
  *
  *         alpha/beta 使用无量纲标度：电压矢量的三个相量由
  *         [alpha, -0.5*alpha + sqrt(3)/2*beta,
  *          -0.5*alpha - sqrt(3)/2*beta] 构成。函数用零序分量把三相
  *         指令平移到 0..1000，并在必要时等比例缩放，保证比较值始终
  *         落在有效的 PWM 比较带内。
  *
  *         这是 TIM1 驱动层的实时安全入口：只写三个比较寄存器，不访问
  *         I2C/GPIO，不改变 MOE，也不依赖 MCSDK 的 PWMC_Handle_t。
  *         FOC 层应通过 foc_port_t.write_voltage 调用它。
  * @param  alpha  归一化 α 轴电压指令
  * @param  beta   归一化 β 轴电压指令
  * @retval HAL_OK；输入为 NaN/无穷值时返回 HAL_ERROR 且不更新输出
  */
HAL_StatusTypeDef Tim1Pwm_SetPhaseVoltage(float alpha, float beta);

/**
  * @brief  【业务线】读回某一相最后写入的占空比。
  * @retval 0..TIM1_PWM_DUTY_SCALE 内的占空比；channel 非法时返回 0。
  */
uint16_t Tim1Pwm_GetPhaseDuty(uint32_t channel);

/**
  * @brief  【业务线】三相同时装载 50 % 的自举电容刷新模式，并清零指令占空比。
  *
  * 这是功率级的安全空闲态：电机上没有净电压，且每个半桥的两个开关都在动作，
  * 低侧在每个载波周期里仍导通一半时间，栅极驱动的自举电容得以保持充电。
  * 应在使能输出之前调用。
  */
void     Tim1Pwm_SetNeutral(void);

/**
  * @brief  【业务线】释放 MOE，让六路栅极信号到达功率级。
  * @note   调用者必须已经确认驱动无故障且可以开关，参见 DrvMonitor_GetState()。
  *         更实时的判定可用 DrvMonitor_IsPowerStageSafeFast()（只读引脚，不走 I2C）。
  */
void     Tim1Pwm_EnableOutputs(void);

/**
  * @brief  【业务线】清除 MOE：所有栅极输出立即被拉到无效电平，
  *         不等待软件故障处理路径。
  */
void     Tim1Pwm_EmergencyStop(void);

/**
  * @brief  【业务线】查询 MOE 状态。
  * @retval 1 表示栅极信号正在驱动功率级，否则为 0。
  */
uint8_t  Tim1Pwm_IsOutputEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_PWM_H__ */
