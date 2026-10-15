/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc1_current_sampling.h
  * @brief   相电流采样的 L1 外设配置：OPAMP1 / OPAMP3 跟随放大 + ADC1 注入组
  *          两通道采样，由 TIM1_CH4 触发。
  *
  *          引脚映射（STM32G431 固定复用，已与原理图出现的两组运放网络对应）
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
  *          沿用参考配置的命名，不代表已确认的相别。
  *
  *          两路运放输出都经外部引脚回到 ADC，不走内部通道，理由各不相同：
  *            OPAMP3 —— ADC1 上根本没有它的内部通道。VOPAMP3 只挂在
  *                      ADC2_IN18 / ADC3_IN13，本工程只用 ADC1，
  *                      所以 PB1 = ADC1_IN12 是唯一可选路径。
  *            OPAMP1 —— ADC1 上有 ADC_CHANNEL_VOPAMP1（= ADC1_IN13），但原理图
  *                      在 PA2 节点上挂了后级 RC 网络，走内部通道会绕过它，
  *                      采到的就不是滤波后的信号，所以同样取外部 PA2 = IN3。
  *
  *          只采两相：FOC 算法的 Clarke 变换只需 ia / ib，第三相由
  *          ia + ib + ic = 0 推出。这是算法需求反向约束配置层的唯一例子。
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

/* 注入序列共两个通道，顺序转换，全部就绪需要约 11.43 µs。
   ⚠ 与 TIM1_CH4 触发点的时序关系见 README 第 12 节待核对项 I。 */
#define ADC1_CS_SEQUENCE_CONV_NS    (ADC1_CS_OVERSAMPLED_CONV_NS * 2UL)

/* 触发提前量：TIM1_CH4 比较点比更新事件早 1 µs（模块 B 的
   TIM1_PWM_ADC_TRIGGER_LEAD_NS）。此处只做记录，实际值由模块 B 决定。
   ⚠ 该提前量远小于 ADC1_CS_SEQUENCE_CONV_NS，时序余量见 README 待核对项 I。 */
#define ADC1_CS_TRIGGER_LEAD_NS     1000UL

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
  * @brief  【配置线】配置 OPAMP1 为独立跟随模式，服务 U 相电流放大。
  *
  * 同相与反相输入都选 IO0（PA1 / PA3），增益完全由外部电阻网络决定，
  * 运放本身不引入内部增益。不使用定时器控制的 Mux 切换，采用出厂 trimming。
  *
  * 只初始化外设，不启动运放。启动由 MX_ADC1_Init() 之后的显式步骤完成，
  * 保持「配置只把硬件准备好，不接通信号通路」的分界。
  *
  * 引脚的模拟模式配置实现在 MSP 层（Core/Src/stm32g4xx_hal_msp.c 的
  * HAL_OPAMP_MspInit），由 HAL_OPAMP_Init() 自动回调。
  *
  * 只能在系统时钟配好之后由 main() 调用一次。
  * @retval HAL_OK；HAL_OPAMP_Init 失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_OPAMP1_Init(void);

/**
  * @brief  【配置线】配置 OPAMP3 为独立跟随模式，服务 W 相电流放大。
  *
  * 同相与反相输入都选 IO0（PB0 / PB2）。⚠ PB2 兼作 BOOT1，必须显式配为
  * 模拟模式，否则残留的启动模式配置会干扰运放输入。
  *
  * 其余约定与 MX_OPAMP1_Init() 相同。
  * @retval HAL_OK；HAL_OPAMP_Init 失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_OPAMP3_Init(void);

/**
  * @brief  【配置线】配置 ADC1 注入组：两个通道、16 倍过采样、由 TIM1_CH4
  *         或软件触发；随后执行一次上电自校准。
  *
  * 注入序列 rank 1 = ADC_CHANNEL_3（PA2，OPAMP1 输出，U 相），
  *            rank 2 = ADC_CHANNEL_12（PB1，OPAMP3 输出，W 相）。
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
  * @note   本函数只配置与校准，不启动注入转换。启动是独立的显式步骤，
  *         由未来的业务层在确认功率级安全后调用。
  */
HAL_StatusTypeDef MX_ADC1_Init(uint8_t software_trigger_flag);

#ifdef __cplusplus
}
#endif

#endif /* __ADC1_CURRENT_SAMPLING_H__ */
