/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc1_current_sampling.h
  * @brief   相电流采样的 L1 外设配置：片内 OPAMP1 / OPAMP3 跟随放大 +
  *          ADC1 注入组两通道采样，由 TIM1_CH4 触发。
  *
  *          引脚映射（STM32G431VB 片内运放的固定复用关系，依据数据手册与
  *          stm32g4xx_hal_opamp.h 中 OPAMP_NONINVERTINGINPUT_IO0 /
  *          OPAMP_INVERTINGINPUT_IO0 的引脚表）：
  *            PA1  -> OPAMP1 VINP0        U 相电流采样输入
  *            PA3  -> OPAMP1 VINM0        U 相反馈网络
  *            PA2  -> OPAMP1 VOUT         同时是 ADC1_IN3   -> 注入 rank 1 = U 相
  *            PB0  -> OPAMP3 VINP0        V 相电流采样输入
  *            PB2  -> OPAMP3 VINM0        V 相反馈网络
  *            PB1  -> OPAMP3 VOUT         同时是 ADC1_IN12  -> 注入 rank 2 = V 相
  *
  *          两路运放输出都经外部引脚回到 ADC，不走内部通道：
  *            OPAMP3 —— ADC1 上没有它的内部通道。VOPAMP3 只挂在
  *                      ADC2_IN18 / ADC3_IN13（见 LL_ADC_CHANNEL_VOPAMP3_*
  *                      的定义），本工程只用 ADC1，所以 PB1 = ADC1_IN12
  *                      是唯一可选路径。
  *            OPAMP1 —— ADC1 上有 ADC_CHANNEL_VOPAMP1（= ADC1_IN13），但参考
  *                      配置取的是外部 PA2 = IN3，本文件与之一致。
  *
  *          只采两相：FOC 算法的 Clarke 变换只需 ia / ib，第三相由
  *          ia + ib + ic = 0 推出。这是算法需求反向约束配置层的唯一例子。
  *
  *          相别映射（已确认）：rank 1 = U 相 = foc_input_t.phase_a = ia，
  *                            rank 2 = V 相 = foc_input_t.phase_b = ib。
  *          这与标准 Clarke 变换的 alpha = ia、beta = (ia + 2*ib)/sqrt(3) 一致，
  *          W 相由 -(ia + ib) 推出，无需采样。ia_offsets / ib_offsets 的
  *          a / b 即对应 U / V 两相。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADC1_CURRENT_SAMPLING_H__
#define __ADC1_CURRENT_SAMPLING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* --------------------------------------------------------------------------*/
/* 硬件常量                                                                  */
/* --------------------------------------------------------------------------*/

/* ADC1 走同步时钟：ADC12 内核时钟 168 MHz（RCC.ADC12Freq_Value），
   ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4 -> ADC 时钟 42 MHz。
   G431 同步模式下 ADC 时钟上限 60 MHz，42 MHz 合规。 */
#define ADC1_CS_ADC_CLOCK_HZ        42000000UL

/* 单次转换耗时：采样时间 2.5 cycles + 12 位逐次逼近 12.5 cycles = 15 cycles。
   15 / 42 MHz = 357 ns。 */
#define ADC1_CS_CYCLES_PER_CONV     15UL

/* 注入组过采样率 16，且 RightBitShift = NONE，所以硬件把 16 次转换结果
   直接累加后交给 JDR：满量程 4095 x 16 = 65520，占满 16 位。
   ⚠ 读出的原始码是累加值，换算成 12 位等效码必须先除以 16。 */
#define ADC1_CS_OVERSAMPLING_RATIO  16UL

/* 单通道完成一轮过采样所需时间：16 x 357 ns = 5.71 µs。 */
#define ADC1_CS_OVERSAMPLED_CONV_NS 5714UL

/* 注入序列共两个通道，顺序转换，全部就绪需要约 11.43 µs。 */
#define ADC1_CS_SEQUENCE_CONV_NS    (ADC1_CS_OVERSAMPLED_CONV_NS * 2UL)

/* 触发提前量：TIM1_CH4 比较点比更新事件早 5.714 µs，即本序列耗时的一半
   （实际写入 CCR4 的值由模块 B 的 TIM1_PWM_ADC_TRIGGER_LEAD_NS 决定，
   此处只做记录，便于两边对照）。

   取一半是让注入序列正好以更新事件为中心：T - 5.71 µs 触发、T + 5.71 µs
   结束，两个通道的采样点分别落在更新事件两侧对称位置。

   ⚠ 改动过采样倍率、采样时间或通道数会改变 ADC1_CS_SEQUENCE_CONV_NS，
   届时必须同步模块 B 的 TIM1_PWM_ADC_TRIGGER_LEAD_NS（见 README 第 14 节
   规则 9）。两边不一致不会报错，只会让采样点偏离载波的预期位置。 */
#define ADC1_CS_TRIGGER_LEAD_NS     (ADC1_CS_SEQUENCE_CONV_NS / 2UL)

/* --------------------------------------------------------------------------*/
/* 导出变量                                                                  */
/* --------------------------------------------------------------------------*/

/* 声明为非 static，一是让句柄出现在调试器 Watch 窗口，二是让中断层能寻址它。
   只有 adc1_current_sampling.c 会修改它们。 */
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp3;
extern ADC_HandleTypeDef   hadc1;

/* 零电流偏置标定值的预留落点。上电时功率级未接通，注入组的读数就是
   运放中点偏置；把它记下来，运行时的原始码减去它才是真正的相电流。
   ⚠ 当前无任何代码填充这两个变量——填充逻辑属于未来的业务层
   （偏置标定 + 原始码到安培的换算），配置层只负责把外设准备好。
   定义在此处是为了让调试器能观察，并明确标定结果的归属。 */
extern volatile uint32_t ia_offsets;
extern volatile uint32_t ib_offsets;

/* --------------------------------------------------------------------------*/
/* 导出函数                                                                  */
/* --------------------------------------------------------------------------*/

/**
  * @brief  【配置线】配置 OPAMP1 为独立跟随模式，放大一路相电流采样信号。
  *
  * 同相与反相输入都选 IO0（PA1 / PA3）。两个输入都走外部引脚，所以增益完全
  * 由板级外部电阻网络决定，运放本身不引入内部增益（内部增益只在 PGA 模式下
  * 才有）。不使用定时器控制的 Mux 切换，采用出厂 trimming。
  *
   * 只初始化外设，不启动运放。启动由 CurrentSampling_Start() 完成，
   * 保持配置与信号链启动的分界。
  *
  * 引脚的模拟模式配置实现在 MSP 层（Core/Src/stm32g4xx_hal_msp.c 的
  * HAL_OPAMP_MspInit），由 HAL_OPAMP_Init() 自动回调。
  *
  * 只能在系统时钟配好之后由 main() 调用一次。
  * @retval HAL_OK；HAL_OPAMP_Init 失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_OPAMP1_Init(void);

/**
  * @brief  【配置线】配置 OPAMP3 为独立跟随模式，放大另一路相电流采样信号。
  *
  * 同相与反相输入都选 IO0（PB0 / PB2）。⚠ PB2 兼作 BOOT1，必须显式配为
  * 模拟模式，否则残留的启动模式配置会干扰运放输入；这也会改变 PB2 的 BOOT1
  * 行为，本工程从 Flash 启动、经 SWD 下载，不依赖 PB2 电平。
  *
  * 其余约定与 MX_OPAMP1_Init() 相同。
  * @retval HAL_OK；HAL_OPAMP_Init 失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_OPAMP3_Init(void);

/**
  * @brief  【配置线】配置 ADC1 注入组：两个通道、16 倍过采样、由 TIM1_CH4
  *         或软件触发；随后执行一次上电自校准。
  *
  * 注入序列 rank 1 = ADC_CHANNEL_3（PA2，OPAMP1 输出）= U 相，
  *            rank 2 = ADC_CHANNEL_12（PB1，OPAMP3 输出）= V 相。
  *
  * 关键配置及其理由：
  *   - DataAlign 设为左对齐，但注入组过采样已使能，硬件会忽略 ALIGN 位
  *     并强制右对齐（RM0440 §21.4.26「过采样模式下不支持左对齐」）。
  *     保留该赋值只为与参考配置逐字一致，实际读出的是右对齐的 16 位累加值。
  *   - OversamplingMode = DISABLE 针对的是「规则组」过采样；注入组过采样由
  *     InjecOversamplingMode 单独使能，两者互不影响。
  *   - EOCSelection = ADC_EOC_SEQ_CONV：整个注入序列结束才置 EOC，
  *     因此读到 JDR 时两个通道的数据一定同时有效，不会读到半更新序列。
  *   - Overrun = ADC_OVR_DATA_PRESERVED：溢出时保留旧数据而不是覆盖，
  *     电流环宁可拿到略旧的一次采样，也不要拿到撕裂的数据。
  *   - QueueInjectedContext = DISABLE + 注入不连续模式关闭：每个 TIM1_CH4
  *     触发点都重新开跑固定序列，适合 20 kHz 的等间隔触发。
  *
  * 校准在 HAL_ADC_Stop() 之后执行——校准的硬性前提是 ADC 处于关闭状态
  * （HAL_ADCEx_Calibration_Start 内部会再次确认）。HAL_ADC_Init() 已自动
  * 完成退出深掉电模式、使能内部稳压器并等待其稳定，所以此处无需额外延时。
  *
  * @param  software_trigger_flag 非 0 = 注入组由软件触发（用于上电偏置标定，
  *         此时 TIM1_CH4 的触发被摘除，标定不受 PWM 时序干扰）；
  *         0 = 由 ADC_EXTERNALTRIGINJEC_T1_CC4 上升沿触发（运行时的电流环）。
  * @retval HAL_OK；任一步 HAL 调用失败时返回 HAL_ERROR。
   * @note   本函数只配置与校准，不启动注入转换。启动由
   *         CurrentSampling_Start() 独立完成。
  */
HAL_StatusTypeDef MX_ADC1_Init(uint8_t software_trigger_flag);

/**
  * @brief  启动两路电流采样信号链。
  *
  * 先启动 OPAMP1/OPAMP3，再使能 ADC1 注入组。若 ADC 初始化时选择了
  * TIM1_CH4 外部触发，本函数只负责把 ADC 置于可接受外部触发的状态；
  * 后续转换由 TIM1_CH4 自动启动。若选择软件触发，则本函数启动一次
  * 注入序列。
  *
  * @retval HAL_OK；任一步启动失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef CurrentSampling_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC1_CURRENT_SAMPLING_H__ */
