/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc1_current_sampling.c
  * @brief   相电流采样的 L1 外设配置：OPAMP1 / OPAMP3 + ADC1 注入组两通道。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "adc1_current_sampling.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;
ADC_HandleTypeDef   hadc1;

/* 零电流偏置标定值的预留落点。上电时功率级尚未接通（MOE = 0），此时注入组
   读到的就是运放中点偏置；运行时的原始码减去它才是真正的相电流。
   填充逻辑属于未来的业务层（偏置标定 + 原始码到安培的换算），配置层不写它。 */
volatile uint32_t ia_offsets;
volatile uint32_t ib_offsets;

/* OPAMP1 init function */
HAL_StatusTypeDef MX_OPAMP1_Init(void)
{
  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* 独立跟随模式：同相输入 IO0 = PA1，反相输入 IO0 = PA3。
     增益由原理图上的外部电阻网络决定，运放不引入内部增益。
     InternalOutput = DISABLE：输出经 PA2 引脚外发再回到 ADC1_IN3。
     ADC1 上确实有 OPAMP1 的内部通道（ADC_CHANNEL_VOPAMP1 = IN13），但原理图
     在 PA2 节点上挂了后级 RC 网络，走内部通道会绕过它，采到的就不是滤波后
     的信号，所以取外部路径。 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  return HAL_OK;
}

/* OPAMP3 init function */
HAL_StatusTypeDef MX_OPAMP3_Init(void)
{
  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* 独立跟随模式：同相输入 IO0 = PB0，反相输入 IO0 = PB2，输出 PB1。
     ⚠ PB2 兼作 BOOT1：必须显式配为模拟模式，否则残留的启动模式配置
     会干扰运放输入。该配置由 MSP 层的 HAL_OPAMP_MspInit() 完成。
     InternalOutput = DISABLE 在这里不是可选项而是必须：VOPAMP3 只挂在
     ADC2_IN18 / ADC3_IN13，ADC1 上没有 OPAMP3 的内部通道，只能经 PB1
     = ADC1_IN12 外部采样。 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  return HAL_OK;
}

/**
  * @brief  配置 ADC1 注入组两个通道（rank 1 = 通道 3 / PA2，rank 2 = 通道 12 / PB1），
  *         16 倍过采样，随后执行一次上电自校准。
  *
  * 时钟换算：ADC12 内核时钟 168 MHz，ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4
  *   -> ADC 时钟 42 MHz（G431 同步模式上限 60 MHz，合规）。
  *   单次转换 = 采样 2.5 cycles + 12 位逐次逼近 12.5 cycles = 15 cycles
  *   -> 357 ns；过采样 16 倍 -> 5.71 µs / 通道，两通道序列 11.43 µs。
  *
  * ⚠ 采样时间 2.5 cycles 是全表最短档，且过采样会把单通道耗时放大 16 倍，
  *   两个通道合计 11.43 µs，已明显超过 TIM1_CH4 相对更新事件 1 µs 的提前量。
  *   时序余量需要实测确认，见 README「待核对项」I。
  */
HAL_StatusTypeDef MX_ADC1_Init(uint8_t software_trigger_flag)
{
  ADC_MultiModeTypeDef    multimode       = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_InjOversamplingTypeDef oversampleConfig = {0};

  /* USER CODE BEGIN ADC1_Init 0 */

  /* 过采样倍率 16，且不做右移：硬件把 16 次转换结果直接累加后写入 JDR，
     满量程 4095 x 16 = 65520 占满 16 位。读数换算成 12 位等效码需除以 16。 */
  oversampleConfig.Ratio = ADC_OVERSAMPLING_RATIO_16;
  oversampleConfig.RightBitShift = ADC_RIGHTBITSHIFT_NONE;

  /* USER CODE END ADC1_Init 0 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  /* ⚠ 左对齐在这里被硬件忽略：注入组过采样已使能，RM0440 §21.4.26 明确
     「过采样模式下不支持左对齐」，ALIGN 位不起作用，JDR 里是右对齐的
     16 位累加值。保留该赋值只为与参考配置逐字一致。 */
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  /* 序列级 EOC：整个注入序列跑完才置标志，因此读到 JDR 时两个通道的数据
     必然同时有效，不会读到半更新的序列。 */
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  /* 溢出时保留旧数据而非覆盖：电流环宁可拿到略旧的一次采样，
     也不要拿到撕裂的数据。 */
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  /* 这里关闭的是「规则组」过采样；注入组过采样由下面的
     InjecOversamplingMode 单独使能，两者互不影响。 */
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /** Configure the ADC multi-mode
  */
  /* 独立模式：不使用 ADC1/ADC2 双模，两相由同一个注入序列顺序转换。 */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;

  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 校准的硬性前提是 ADC 处于关闭状态，所以先显式停一次。
     HAL_ADC_Init() 已自动完成退出深掉电模式、使能内部稳压器并等待其稳定，
     此处无需额外延时。 */
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  /* 关闭注入上下文队列：每个 TIM1_CH4 触发点都重新开跑固定的两通道序列，
     适合 20 kHz 的等间隔触发。 */
  sConfigInjected.QueueInjectedContext = DISABLE;
  /* 2.5 cycles 是采样时间最短档。运放输出阻抗低、且外部已挂 RC 网络，
     短采样时间足够；过采样 16 倍本身也提供了额外的噪声平均。 */
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;

  /* 软件触发用于上电偏置标定：此时 TIM1_CH4 的触发被摘除，标定不受 PWM
     时序干扰。运行时改为 T1_CC4 上升沿，让采样点跟随载波。 */
  if (software_trigger_flag != 0U)
  {
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  }
  else
  {
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  }
  /* ⚠ 该边沿参数只在外部触发分支下真正写入 JSQR：HAL 在触发源为软件启动时
     会跳过 JEXTEN 写入（见 stm32g4xx_hal_adc_ex.c 的
     ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START 判断），
     因此软件触发模式下这个赋值不会生效，无需按分支区分。 */
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling = oversampleConfig;

  /* rank 1：ADC_CHANNEL_3 = PA2 = OPAMP1 输出。
     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;

  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* rank 2：ADC_CHANNEL_12 = PB1 = OPAMP3 输出。
     ⚠ 对应哪一相尚未确认，见头文件说明与 README 待核对项 J。 */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;

  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  return HAL_OK;
}

/**
  * @brief  启动电流采样信号链。
  *
  * MX_*_Init() 只完成配置和 ADC 校准；校准结束后 ADC 仍处于停止状态，
  * OPAMP 也尚未使能。把启动集中在本模块，避免 main 或实时中断直接操作
  * 外设寄存器，也保证运放先于 ADC 建立模拟信号。
  */
HAL_StatusTypeDef CurrentSampling_Start(void)
{
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 外部触发模式下，该调用只负责使能 ADC 并武装注入触发器；
     软件触发模式下，HAL 会同时启动一次注入序列。 */
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
