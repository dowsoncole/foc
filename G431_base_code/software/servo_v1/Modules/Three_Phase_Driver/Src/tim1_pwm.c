/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm.c
  * @brief   TIM1 三相互补 PWM 的初始化与占空比控制。
  *
  *          配置摘要
  *            计数器    ：中央对齐模式 1，ARR = 4199，RCR = 1
  *                        -> 168 MHz / (2 x 4200) = 20 kHz 载波，
  *                           每个载波周期恰好产生一次更新事件
  *            通道 1..3 ：PWM 模式 1，主输出（INH）+ 互补输出（INL），
  *                        死区 500 ns，由 TIM_BDTR 配置
  *            通道 4    ：PWM 模式 2，经 TRGO2 的 OC4REF 上升沿触发 ADC，
  *                        CCR4 与三相 duty 一起逐周期更新
  *            MOE       ：保持清零，栅极输出在显式调用
  *                        Tim1Pwm_EnableOutputs() 之前一直处于无效状态
  *
  *          半桥空闲模式
  *            功率级关闭期间，三相装载的是中性点比较值，对应稳定的 50 % 占空比。
  *            这样低侧开关在每个周期里都有足够长的导通时间来刷新栅极驱动的
  *            自举电容，而同一半桥的两个开关永远不会同时导通。
  *
  *            比较值与高侧占空比成正比，而 ARR 是可用比较带的顶端：超过 ARR 的
  *            比较值不会扩大占空比范围，而是让通道停止翻转。因此两个被钳位的
  *            端点分别意味着高侧「常通」和低侧「常通」。一个不需要被驱动的相，
  *            绝不能装载比较带顶端的值（那会让高侧整个周期都导通）；应当使用
  *            中性点模式，它让两个开关都保持动作。
  *            整数验证接口与浮点 FOC 接口都按高侧占空比映射，后者直接量化 CCR。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim1_pwm.h"
#include "pwm_sampling.h"
#include "stm32g4xx_ll_tim.h"

#include <float.h>
#include <math.h>

/* USER CODE BEGIN 0 */

/* 死区时间是「编码值」而不是普通计数：TIM_BDTR_DTG 取值 0x00..0x7F 时以 tDTS
   为单位，所以该值必须落在这个第一段编码范围内。 */
#define TIM1_PWM_DTG_IN_RANGE(dtg)  ((dtg) <= 0x7FU)

/* 编译期检查：16 位比较寄存器必须能容纳一个完整的 PWM 周期，
   否则占空比映射会被截断。 */
#define TIM1_PWM_PERIOD_FITS_U16    (TIM1_PWM_COUNTS_PER_PERIOD <= 0xFFFFUL)

#if !TIM1_PWM_PERIOD_FITS_U16
#error "TIM1_PWM_FREQUENCY_HZ is too low for a 16-bit compare register"
#endif

static void Tim1Pwm_WriteCompare(uint32_t channel, uint32_t compare);
static uint32_t Tim1Pwm_NsToCounts(uint32_t nanoseconds);
static int32_t  Tim1Pwm_FindPhase(uint32_t channel);

/* 每一相的指令占空比，与已装载的比较值保持同步，这样 Tim1Pwm_SetNeutral()
   和验证模块都能恢复到一个已知状态，而不必反读定时器。
   线程上下文和中断上下文都会写它，所以声明为 volatile。 */
static volatile uint16_t s_phase_duty[3] = {TIM1_PWM_DUTY_NEUTRAL,
                                            TIM1_PWM_DUTY_NEUTRAL,
                                            TIM1_PWM_DUTY_NEUTRAL};

/* HAL 的通道标识符是「寄存器偏移量」，不是「位序号」：TIM_CHANNEL_1 的值是
   零，所以用 (TIM_CHANNEL_1 << phase) 推导后续通道会永远选中通道 1。
   这里改用查表。 */
static const uint32_t s_phase_channel[3] = {TIM_CHANNEL_1,
                                            TIM_CHANNEL_2,
                                            TIM_CHANNEL_3};

static pwm_sampling_parameters_t s_sampling_parameters;
static uint8_t s_sampling_configured;
static uint8_t s_previous_sample_valid;
static uint32_t s_previous_sample_trigger;
static uint32_t s_previous_sample_sequence;
volatile tim1_pwm_sampling_status_t g_tim1_pwm_sampling;

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* --------------------------------------------------------------------------*/
/* 内部辅助函数                                                              */
/* --------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/**
  * @brief  把 PWM 通道标识符映射成相位表下标。
  * @retval TIM_CHANNEL_1..3 返回 0..2，其他任何通道返回 -1。
  */
static int32_t Tim1Pwm_FindPhase(uint32_t channel)
{
  int32_t phase = -1;

  switch (channel)
  {
    case TIM_CHANNEL_1:
      phase = 0;
      break;

    case TIM_CHANNEL_2:
      phase = 1;
      break;

    case TIM_CHANNEL_3:
      phase = 2;
      break;

    default:
      phase = -1;
      break;
  }

  return phase;
}

/**
  * @brief  装载一个比较寄存器。
  *
  * 占空比映射是按 ARR + 1 缩放的，所以 duty 0 有可能落到比较带之外一个计数的
  * 位置。钳位到 ARR 让这种情况有确定且单调的语义：ARR 是仍能翻转的最大值，
  * 含义是「高侧常通」。而一个留在 ARR 之上的值会让通道彻底停止翻转并把输出
  * 钉死——这正是本钳位要防止的失效模式。
  */
static void Tim1Pwm_WriteCompare(uint32_t channel, uint32_t compare)
{
  if (compare > TIM1_PWM_COUNTER_PERIOD)
  {
    compare = TIM1_PWM_COUNTER_PERIOD;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint16_t)compare);
}

/**
  * @brief  把以纳秒为单位的时间换算成定时器计数，向下取整。
  */
static uint32_t Tim1Pwm_NsToCounts(uint32_t nanoseconds)
{
  return ((nanoseconds * (TIM1_PWM_TIMER_CLOCK_HZ / 1000000UL)) / 1000UL);
}

/* FOC 直接换算 CCR，避免先量化成 1000 级再损失定时器分辨率。
   保留原有 SetPhaseDuty 的整数标度，供独立 PWM 验证业务使用。 */
static uint32_t Tim1Pwm_FloatDutyCompare(float duty)
{
  uint32_t compare = (uint32_t)(duty * (float)TIM1_PWM_COMPARE_FULL_SCALE + 0.5f);

  /* duty=1 显式允许 CCR=ARR+1，表示静态高侧；只有具有合适采样和
     栅极供电条件的调用方才可请求。正常 FOC 使用板级采样上限。 */
  if (compare > TIM1_PWM_COMPARE_FULL_SCALE) compare = TIM1_PWM_COMPARE_FULL_SCALE;
  return compare;
}

/* 连续三次读取处理峰/谷折返点，执行次数有界。返回距离谷值的时间计数。 */
static uint32_t Tim1Pwm_CyclePosition(void)
{
  uint32_t direction = TIM1->CR1 & TIM_CR1_DIR;
  uint32_t count = TIM1->CNT;
  uint32_t final_direction = TIM1->CR1 & TIM_CR1_DIR;
  if (direction != final_direction) count = TIM1->CNT;
  return (final_direction != 0U) ? 2U * TIM1_PWM_COUNTER_PERIOD - count : count;
}

HAL_StatusTypeDef Tim1Pwm_SetPhaseVoltage(float phase_a_duty,
                                          float phase_b_duty,
                                          float phase_c_duty)
{
  uint32_t compare[3], interrupt_mask, before, after, commit_guard;
  pwm_sampling_plan_t plan;
  if (!isfinite(phase_a_duty) || !isfinite(phase_b_duty) ||
      !isfinite(phase_c_duty) ||
      (phase_a_duty < 0.0f) || (phase_a_duty > 1.0f) ||
      (phase_b_duty < 0.0f) || (phase_b_duty > 1.0f) ||
      (phase_c_duty < 0.0f) || (phase_c_duty > 1.0f))
  {
    return HAL_ERROR;
  }

  compare[0] = Tim1Pwm_FloatDutyCompare(phase_a_duty);
  compare[1] = Tim1Pwm_FloatDutyCompare(phase_b_duty);
  compare[2] = Tim1Pwm_FloatDutyCompare(phase_c_duty);
  if ((s_sampling_configured == 0U) ||
      (PwmSampling_Plan(&s_sampling_parameters, compare[0], compare[1], compare[2],
                          &plan) == 0U))
  {
    g_tim1_pwm_sampling.window_rejections++;
    return HAL_ERROR;
  }

  /* CCR1..4 一起预装载，在 CNT=0 的同一个 UEV 生效。短临界区中暂禁更新，
     防止新三相 PWM 配到旧触发点。跨过截止点则拒绝，并保持硬件关断。
     不产生 UG（UG 会打乱载波/重复计数器相位）。 */
  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  before = Tim1Pwm_CyclePosition();
  commit_guard = Tim1Pwm_NsToCounts(TIM1_PWM_COMMIT_GUARD_NS) + 1U;
  if (((TIM1->CR1 & TIM_CR1_UDIS) != 0U) ||
      ((2U * TIM1_PWM_COUNTER_PERIOD - before) <= commit_guard))
  {
    g_tim1_pwm_sampling.deadline_rejections++;
    if (interrupt_mask == 0U) __enable_irq();
    return HAL_ERROR;
  }
  SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
  TIM1->CCR1 = compare[0];
  TIM1->CCR2 = compare[1];
  TIM1->CCR3 = compare[2];
  TIM1->CCR4 = plan.trigger;
  after = Tim1Pwm_CyclePosition();
  CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
  if ((after < before) ||
      ((2U * TIM1_PWM_COUNTER_PERIOD - after) <= commit_guard))
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    g_tim1_pwm_sampling.deadline_rejections++;
    if (interrupt_mask == 0U) __enable_irq();
    return HAL_ERROR;
  }
  if (interrupt_mask == 0U) __enable_irq();
  s_phase_duty[0] = (uint16_t)(phase_a_duty * (float)TIM1_PWM_DUTY_SCALE + 0.5f);
  s_phase_duty[1] = (uint16_t)(phase_b_duty * (float)TIM1_PWM_DUTY_SCALE + 0.5f);
  s_phase_duty[2] = (uint16_t)(phase_c_duty * (float)TIM1_PWM_DUTY_SCALE + 0.5f);
  g_tim1_pwm_sampling.planned_trigger = plan.trigger;
  g_tim1_pwm_sampling.planned_sequence_end = plan.sequence_end;
  g_tim1_pwm_sampling.low_window_start = plan.low_window_start;
  g_tim1_pwm_sampling.low_window_end = plan.low_window_end;
  return HAL_OK;
}

static pwm_sampling_parameters_t Tim1Pwm_SamplingParameters(uint32_t sequence_ns)
{
  pwm_sampling_parameters_t parameters;
  parameters.peak = TIM1_PWM_COUNTER_PERIOD;
  parameters.sequence_ticks = Tim1Pwm_NsToCounts(sequence_ns) + 1U;
  parameters.guard_ticks = Tim1Pwm_NsToCounts(TIM1_PWM_DEADTIME_NS +
                                              TIM1_PWM_CURRENT_SETTLE_NS) + 1U;
  return parameters;
}

HAL_StatusTypeDef Tim1Pwm_GetCurrentSamplingDutyBounds(uint32_t sequence_ns,
                                                       float *minimum,
                                                       float *maximum)
{
  uint32_t peak = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t upper, min_compare;
  pwm_sampling_parameters_t parameters;

  if ((minimum == NULL) || (maximum == NULL) || (peak == 0U) ||
      (peak != TIM1_PWM_COUNTER_PERIOD) ||
      (sequence_ns == 0U) || (sequence_ns >= (1000000000UL / TIM1_PWM_FREQUENCY_HZ)))
  {
    return HAL_ERROR;
  }
  parameters = Tim1Pwm_SamplingParameters(sequence_ns);
  if (PwmSampling_UniformLimit(&parameters, &upper) == 0U) return HAL_ERROR;
  min_compare = (Tim1Pwm_NsToCounts(TIM1_PWM_FOC_MIN_PULSE_NS) + 1U) / 2U + 1U;
  if (upper <= min_compare) return HAL_ERROR;
  *minimum = (float)min_compare / (float)TIM1_PWM_COMPARE_FULL_SCALE;
  *maximum = (float)upper / (float)TIM1_PWM_COMPARE_FULL_SCALE;
  return HAL_OK;
}

HAL_StatusTypeDef Tim1Pwm_ConfigureCurrentSampling(uint32_t sequence_ns,
                                                   float *minimum, float *maximum)
{
  pwm_sampling_plan_t plan;
  pwm_sampling_parameters_t parameters;
  /* 调用方保证 ADC 尚未启动；这里额外拒绝任何仍使能功率级的配置请求。 */
  if (Tim1Pwm_IsOutputEnabled() != 0U) return HAL_ERROR;
  if (Tim1Pwm_GetCurrentSamplingDutyBounds(sequence_ns, minimum, maximum) != HAL_OK)
    return HAL_ERROR;
  parameters = Tim1Pwm_SamplingParameters(sequence_ns);
  if (PwmSampling_Plan(&parameters, TIM1->CCR1, TIM1->CCR2, TIM1->CCR3,
                        &plan) == 0U) return HAL_ERROR;
  s_sampling_parameters = parameters;
  TIM1->CCR4 = plan.trigger;
  s_sampling_configured = 1U;
  s_previous_sample_valid = 0U;
  g_tim1_pwm_sampling.planned_trigger = plan.trigger;
  g_tim1_pwm_sampling.planned_sequence_end = plan.sequence_end;
  g_tim1_pwm_sampling.low_window_start = plan.low_window_start;
  g_tim1_pwm_sampling.low_window_end = plan.low_window_end;
  return HAL_OK;
}

float Tim1Pwm_OnCurrentSample(uint32_t sample_sequence)
{
  uint32_t trigger = TIM1->CCR4;
  int32_t ticks = (int32_t)(2U * TIM1_PWM_COUNTER_PERIOD);
  /* 校准阶段暂停 FOC 回调；恢复后的首个控制周期用名义周期重新起算。 */
  if ((s_previous_sample_valid != 0U) &&
      ((sample_sequence - s_previous_sample_sequence) == 1U))
    ticks += (int32_t)trigger - (int32_t)s_previous_sample_trigger;
  s_previous_sample_trigger = trigger;
  s_previous_sample_sequence = sample_sequence;
  s_previous_sample_valid = 1U;
  g_tim1_pwm_sampling.sampled_trigger = trigger;
  g_tim1_pwm_sampling.sample_period_s = (float)ticks / (float)TIM1_PWM_TIMER_CLOCK_HZ;
  return g_tim1_pwm_sampling.sample_period_s;
}

/* USER CODE END 1 */

/* TIM1 初始化函数 */
HAL_StatusTypeDef MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sOcConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  uint32_t compare;
  uint32_t phase;

  /* USER CODE BEGIN TIM1_Init 0 */

  /* 死区常量在这里从宏取出，使其取值范围检查成为被编译的代码，
     而不只是一句注释。 */
  const uint32_t deadtime = (uint32_t)TIM1_PWM_DEADTIME_TDTS;

  if (TIM1_PWM_DTG_IN_RANGE(deadtime) == 0UL)
  {
    /* 无法用 TIM_BDTR_DTG 的 tDTS 计数段编码。 */
    return HAL_ERROR;
  }

  /* 所有时序都基于 SystemClock_Config() 里的 168 MHz PLL 配置，其中 APB2
     不分频，所以 TIM1 的内核时钟等于 PCLK2。换一套时钟树会悄无声息地同时
     改变载波频率和死区时间，因此这里选择「检查」而不是「假设」。 */
  if (HAL_RCC_GetPCLK2Freq() != TIM1_PWM_TIMER_CLOCK_HZ)
  {
    return HAL_ERROR;
  }

  /* USER CODE END TIM1_Init 0 */

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = TIM1_PWM_COUNTER_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  /* OC4REF 上升沿只在上数比较点产生一次；直接 CC4 事件与 REF 边沿不同。 */
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 刹车与死区。两个刹车输入都保持禁用：BKIN 和 BKIN2 对应栅极驱动的 READY
     和 NFAULT 引脚（见数据手册 "MCU to gate driver internal connection" 表），
     而在本 MCU 上 PE15 根本没有任何 TIM1 复用功能。这两个引脚改由
     drv_monitor 读取，保护工作交给驱动芯片自身的 VDS / 热保护。
     要启用刹车输入，必须先明确决定放弃哪个引脚。

     OSSR/OSSI 关闭：MOE 清零期间，六个引脚交给栅极驱动自己的输入上下拉电阻，
     而不是被驱动到某个电平——因为那个电平对栅极驱动意味着什么，无法从本数据
     手册中读出来。驱动芯片的设计可以容忍这种输入配置。若打开 OSSR，则会把
     OC/OCN 驱动到其空闲态，那是一个明确的「刹车 / 滑行」选择，属于功率级
     使能逻辑的职责，不该放在这里。 */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = deadtime;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0U;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0U;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 通道 1..3：PWM 模式 1，输出高有效，互补输出同样高有效。因此比较值越大，
     主输出的导通时间越长，而主输出对应的正是半桥的高侧开关。 */
  sOcConfig.OCMode = TIM_OCMODE_PWM1;
  sOcConfig.Pulse = 0U;
  sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOcConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sOcConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sOcConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sOcConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  for (phase = 0U; phase < 3U; phase++)
  {
    /* Pulse 马上会被覆盖；给一个废弃值只是为了让循环体对每个通道都完全一致。 */
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sOcConfig,
                                  s_phase_channel[phase]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  /* 通道 4 通过 TRGO2 的 OC4REF 触发 ADC；初值用于启动，随后与三相 duty
     同步预装载。RCR=1 在启动前写入，周期更新在谷值，而非峰值。 */
  sOcConfig.OCMode = TIM_OCMODE_PWM2;
  sOcConfig.Pulse = (uint32_t)TIM1_PWM_COUNTER_PERIOD -
                    Tim1Pwm_NsToCounts(TIM1_PWM_ADC_TRIGGER_LEAD_NS);
  sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOcConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sOcConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  /* CCR1..4 必须一起在谷值更新，不能让新触发点提前作用于旧 PWM。 */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);
  s_sampling_configured = 0U;
  s_previous_sample_valid = 0U;

  /* 外设时钟和 NVIC 优先级已由 HAL_TIM_PWM_MspInit() 设置好，
     那个函数是上面 HAL_TIM_PWM_Init() 调用的。此刻只剩下引脚配置，
     位置正是 CubeMX 生成 HAL_TIM_MspPostInit() 的地方。 */
  HAL_TIM_MspPostInit(&htim1);

  /* 带着更新中断开始计数：电流环会需要这个周期边界。
     六路输出仍被 MOE 挡住。 */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 使能每一相的主输出与互补输出。 */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                 LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                 LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

  /* ADC 由硬件 REF 边沿触发，电流环由 JEOS 调度，无需空的 CC4 ISR。 */
  LL_TIM_DisableIT_CC4(TIM1);

  /* 中性点 50 % 模式：每个半桥的两个开关都被驱动，栅极驱动的自举电容得以
     保持电荷，而电机上看不到净电压。 */
  for (phase = 0U; phase < 3U; phase++)
  {
    s_phase_duty[phase] = TIM1_PWM_DUTY_NEUTRAL;
    compare = TIM1_PWM_DUTY_TO_COMPARE(TIM1_PWM_DUTY_NEUTRAL);
    Tim1Pwm_WriteCompare(s_phase_channel[phase], compare);
  }

  /* MOE 保持清零：功率级由 Tim1Pwm_EnableOutputs() 使能，
     时机是在驱动被报告为健康之后。 */
  return HAL_OK;
}

/* USER CODE BEGIN 2 */

/**
  * @brief  设置某一相的高侧占空比（0..TIM1_PWM_DUTY_SCALE，500 = 50 %）。
  * @note   两个端点都会饱和。duty 1000 让低侧整个周期都不导通，duty 0 让低侧
  *         整个周期都导通；两者都会使该半桥的自举电容无法刷新，而 duty 0
  *         还会通过低侧对电机制动。运行中的相请保持在标度内部，空闲态请用
  *         中性点模式，绝不要把任一端点当作空闲值使用。
  */
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty)
{
  int32_t phase = Tim1Pwm_FindPhase(channel);

  if (phase < 0)
  {
    return HAL_ERROR;
  }

  if (duty > TIM1_PWM_DUTY_SCALE)
  {
    duty = TIM1_PWM_DUTY_SCALE;
  }

  /* 先写寄存器：那才是功率级实际看到的状态。影子值只用于记账，
     所以一次与并发更新赛跑的读回，永远不会误报引脚的实际行为。 */
  Tim1Pwm_WriteCompare(channel, TIM1_PWM_DUTY_TO_COMPARE(duty));
  s_phase_duty[phase] = duty;

  return HAL_OK;
}



/**
  * @brief  读回某一相最后写入的占空比。
  */
uint16_t Tim1Pwm_GetPhaseDuty(uint32_t channel)
{
  int32_t  phase = Tim1Pwm_FindPhase(channel);
  uint16_t duty  = 0U;

  if (phase >= 0)
  {
    duty = s_phase_duty[phase];
  }

  return duty;
}

/**
  * @brief  装载相位对齐的中性点模式（三相均为 50 %）。
  *
  * 这是功率级的安全空闲态：电机上没有净电压，且每个半桥的两个开关都在动作，
  * 所以低侧在每个载波周期里仍导通一半时间，栅极驱动的自举电容保持充电。
  */
void Tim1Pwm_SetNeutral(void)
{
  uint32_t phase;

  for (phase = 0U; phase < 3U; phase++)
  {
    s_phase_duty[phase] = TIM1_PWM_DUTY_NEUTRAL;
  }

  /* 三相收到相同的比较值，因此它们同步翻转，电机看到的是一个零矢量。 */
  for (phase = 0U; phase < 3U; phase++)
  {
    Tim1Pwm_WriteCompare(s_phase_channel[phase], TIM1_PWM_COMPARE_NEUTRAL);
  }
}

void Tim1Pwm_EnableCalibrationLowSide(void)
{
  uint32_t phase;

  /* 先确保 MOE 打开时看到的是已经准备好的三相中性点，而不是刚写入的
     duty=0。复位/驱动器唤醒阶段如果同时改变 MOE 和 CCR，输出级可能在
     输入滤波和自举电容尚未稳定时产生一个非对称脉冲，表现为复位电流尖峰。 */
  Tim1Pwm_SetNeutral();
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_Delay(TIM1_PWM_MOE_SETTLE_MS);

  /* 只有 MOE 稳定后才切到下管校准：duty=0 对应高侧关闭、互补低侧导通，
     死区由 TIM1 硬件保证。该函数只允许在启动上下文调用，不能放入 ISR。 */
  for (phase = 0U; phase < 3U; phase++)
  {
    s_phase_duty[phase] = 0U;
    Tim1Pwm_WriteCompare(s_phase_channel[phase], 0U);
  }
}

/**
  * @brief  释放 MOE，让六路栅极信号到达功率级。
  */
void Tim1Pwm_EnableOutputs(void)
{
  __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
  * @brief  清除 MOE：所有栅极输出立即被拉到无效电平。
  * @note   必须用「无条件」版本：__HAL_TIM_MOE_DISABLE() 只在所有通道的
  *         CCxE 和 CCxNE 位都已清零时才生效，所以在运行中的功率级上
  *         它会悄无声息地什么都不做。
  */
void Tim1Pwm_EmergencyStop(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

/**
  * @brief  查询 MOE 状态。
  */
uint8_t Tim1Pwm_IsOutputEnabled(void)
{
  return ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
}

/**
  * @brief  TIM1 通道 4 的采样时刻回调。
  *
  * 未来的电流环就运行在这里：CC4 的比较点被放在更新事件之前
  * TIM1_PWM_ADC_TRIGGER_LEAD_NS，而 ADC 应由 OC4REF 触发。
  * 回调名由 HAL 固定，所以周期边界回调仍然是
  * HAL_TIM_PeriodElapsedCallback，而那个回调已被驱动监控模块
  * 用于 TIM6 心跳。
  *
  * 刻意保留为空桩：ADC 还没配置，采样时刻暂时没有消费者，而以载波频率
  * 运行的空中断体只花费一次中断进入的开销。HAL_TIM_IRQHandler() 会无条件
  * 调用该回调，所以现在无需在此定义——等电流环需要时再加。
  */

/* USER CODE END 2 */
