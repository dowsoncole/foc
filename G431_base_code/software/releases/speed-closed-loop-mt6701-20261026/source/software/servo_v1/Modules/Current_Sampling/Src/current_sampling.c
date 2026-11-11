/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    current_sampling.c
  * @brief   相电流采样业务层：快照、零点校准和原始码换算。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "current_sampling.h"
#include "adc1_current_sampling.h"
#include "stm32g4xx_hal.h"
#include "control_timing.h"

#define CURRENT_SAMPLING_CAL_SETTLE_MS   10UL
#define CURRENT_SAMPLING_CAL_WINDOW_MS   20UL
#define CURRENT_SAMPLING_CAL_TIMEOUT_MS  50UL

current_sampling_t g_current_sampling =
{
  /* 实物确认值：两相低侧采样电阻均为 7 mOhm。原理图中的阻值标注
     与实物不一致，板级参数以实物确认值为准。 */
  .shunt_resistor_uohm = 7000UL,
  .opamp_gain = 6.1f,
  .vref_mv = 3300UL,
  .adc_full_scale = 4095UL,
  .oversampling_ratio = ADC1_CS_OVERSAMPLING_RATIO,
  .sample_rate_hz = CONTROL_FREQUENCY_HZ,
  .offset_u = 0U,
  .offset_v = 0U,
  /* 板上低侧分流采样的原始正方向与 FOC 定义（电流流入电机相端为正）
     相反。实机表现为正 Id 指令下 Id 持续为负，构成正反馈；只在采样
     换算边界翻转，保持 ADC rank、Clarke/Park 和 PWM 相序定义不变。 */
  .polarity_u = -1,
  .polarity_v = -1,
  .raw_u = 0U,
  .raw_v = 0U,
  .sample_sequence = 0UL,
  .calibration_state = CURRENT_SAMPLING_CAL_IDLE,
  .calibration_sample_count = 0UL,
  .calibration_target_samples = 0UL,
  .calibration_deadline_ms = 0UL,
  .calibration_sum_u = 0ULL,
  .calibration_sum_v = 0ULL,
  .calibration_min_u = UINT16_MAX,
  .calibration_max_u = 0U,
  .calibration_min_v = UINT16_MAX,
  .calibration_max_v = 0U
};

static current_sampling_conversion_callback_t s_conversion_callback;
static void *s_conversion_context;
static void CurrentSampling_PublishRawSample(void);

static uint32_t CurrentSampling_Elapsed(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1UL : 0UL;
}

/** ADC 注入序列完成：此时 JDR1/JDR2 已同时更新，立即发布快照并调度控制环。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc == NULL) || (hadc->Instance != ADC1))
  {
    return;
  }

  g_current_sampling.raw_u = (uint16_t)ADC1->JDR1;
  g_current_sampling.raw_v = (uint16_t)ADC1->JDR2;
  CurrentSampling_PublishRawSample();
}

void CurrentSampling_InjectedFastIrq(void)
{
  /* Copy before W1C. Never clear JEOS again after the long FOC callback:
     a new conversion may have completed while that callback was running. */
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  uint16_t raw_u = (uint16_t)ADC1->JDR1;
  uint16_t raw_v = (uint16_t)ADC1->JDR2;
  ADC1->ISR = ADC_FLAG_JEOC | ADC_FLAG_JEOS;
  if (!mask) __enable_irq();
  g_current_sampling.raw_u = raw_u;
  g_current_sampling.raw_v = raw_v;
  SET_BIT(hadc1.State, HAL_ADC_STATE_INJ_EOC);
  CurrentSampling_PublishRawSample();
}

static void CurrentSampling_PublishRawSample(void)
{
  g_current_sampling.sample_sequence++;

  if (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_COLLECTING &&
      g_current_sampling.calibration_sample_count <
      g_current_sampling.calibration_target_samples)
  {
    g_current_sampling.calibration_sum_u += g_current_sampling.raw_u;
    g_current_sampling.calibration_sum_v += g_current_sampling.raw_v;
    if (g_current_sampling.raw_u < g_current_sampling.calibration_min_u)
      g_current_sampling.calibration_min_u = g_current_sampling.raw_u;
    if (g_current_sampling.raw_u > g_current_sampling.calibration_max_u)
      g_current_sampling.calibration_max_u = g_current_sampling.raw_u;
    if (g_current_sampling.raw_v < g_current_sampling.calibration_min_v)
      g_current_sampling.calibration_min_v = g_current_sampling.raw_v;
    if (g_current_sampling.raw_v > g_current_sampling.calibration_max_v)
      g_current_sampling.calibration_max_v = g_current_sampling.raw_v;
    g_current_sampling.calibration_sample_count++;
  }

  if ((s_conversion_callback != NULL) &&
      (g_current_sampling.calibration_state != CURRENT_SAMPLING_CAL_COLLECTING) &&
      (g_current_sampling.calibration_state != CURRENT_SAMPLING_CAL_SETTLING))
  {
    /* PA0 is owned by the outer ADC IRQ: do not shorten its pulse to this
       callback or clear it before CCR/diagnostic processing has returned. */
    s_conversion_callback(s_conversion_context);
  }
}

void CurrentSampling_SetConversionCallback(
    current_sampling_conversion_callback_t callback, void *context)
{
  s_conversion_callback = callback;
  s_conversion_context = context;
}

uint8_t CurrentSampling_LoadBoardCalibration(void)
{
  uint32_t mask = __get_PRIMASK();
  uint8_t loaded = 0U;
  __disable_irq();
  /* Match the analog configuration in the accepted 16 kHz measurement.
     The requested 10 kHz carrier reuses the same raw offsets and scale;
     it is not a new zero-offset measurement. Reject other cadences or a
     software sample-rate setting inconsistent with the carrier. */
  if (((TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
      (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_IDLE) &&
      (g_current_sampling.shunt_resistor_uohm == 7000UL) &&
      (g_current_sampling.opamp_gain == 6.1f) &&
      (g_current_sampling.vref_mv == 3300UL) &&
      (g_current_sampling.adc_full_scale == 4095UL) &&
      (g_current_sampling.oversampling_ratio == 1UL) &&
      (g_current_sampling.sample_rate_hz == CONTROL_FREQUENCY_HZ) &&
      ((CONTROL_FREQUENCY_HZ == 16000UL) || (CONTROL_FREQUENCY_HZ == 10000UL)) &&
      (g_current_sampling.polarity_u == -1) &&
      (g_current_sampling.polarity_v == -1) &&
      (CURRENT_SAMPLING_BOARD_OFFSET_U < 4095U) &&
      (CURRENT_SAMPLING_BOARD_OFFSET_U > 0U) &&
      (CURRENT_SAMPLING_BOARD_OFFSET_V < 4095U) &&
      (CURRENT_SAMPLING_BOARD_OFFSET_V > 0U))
  {
    g_current_sampling.offset_u = CURRENT_SAMPLING_BOARD_OFFSET_U;
    g_current_sampling.offset_v = CURRENT_SAMPLING_BOARD_OFFSET_V;
    /* No new measurement: leave sample count/sums unchanged at their boot
       values. DONE denotes usable offsets, not samples collected this boot. */
    g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_DONE;
    loaded = 1U;
  }
  if (!mask) __enable_irq();
  return loaded;
}

void CurrentSampling_RequestCalibration(void)
{
  if ((g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_SETTLING) ||
      (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_COLLECTING))
  {
    return;
  }

  g_current_sampling.calibration_sum_u = 0ULL;
  g_current_sampling.calibration_sum_v = 0ULL;
  g_current_sampling.calibration_min_u = UINT16_MAX;
  g_current_sampling.calibration_max_u = 0U;
  g_current_sampling.calibration_min_v = UINT16_MAX;
  g_current_sampling.calibration_max_v = 0U;
  g_current_sampling.calibration_sample_count = 0UL;
  g_current_sampling.calibration_target_samples =
      (g_current_sampling.sample_rate_hz * CURRENT_SAMPLING_CAL_WINDOW_MS) /
      1000UL;
  if (g_current_sampling.calibration_target_samples == 0UL)
  {
    g_current_sampling.calibration_target_samples = 1UL;
  }
  g_current_sampling.calibration_deadline_ms =
      HAL_GetTick() + CURRENT_SAMPLING_CAL_SETTLE_MS;
  g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_SETTLING;
}

void CurrentSampling_Service(void)
{
  uint32_t now = HAL_GetTick();

  if (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_SETTLING)
  {
    if (CurrentSampling_Elapsed(now,
                                g_current_sampling.calibration_deadline_ms) != 0UL)
    {
      /* deadline 是采样故障超时，不是正常结束条件。正常校准必须完整采满
         target，禁止再用“只收到一个样本也算成功”的不确定 offset。 */
      g_current_sampling.calibration_deadline_ms = now +
          CURRENT_SAMPLING_CAL_TIMEOUT_MS;
      g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_COLLECTING;
    }
  }
  else if (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_COLLECTING)
  {
    if (g_current_sampling.calibration_sample_count >=
        g_current_sampling.calibration_target_samples)
    {
      g_current_sampling.offset_u = (uint16_t)
          (g_current_sampling.calibration_sum_u /
           g_current_sampling.calibration_target_samples);
      g_current_sampling.offset_v = (uint16_t)
          (g_current_sampling.calibration_sum_v /
           g_current_sampling.calibration_target_samples);
      g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_DONE;
    }
    else if (CurrentSampling_Elapsed(now,
                  g_current_sampling.calibration_deadline_ms) != 0UL)
    {
      g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_ERROR;
    }
  }
}

current_sampling_cal_state_t CurrentSampling_GetCalibrationState(void)
{
  return g_current_sampling.calibration_state;
}

uint16_t CurrentSampling_GetRaw(current_sampling_phase_t phase)
{
  return (phase == CURRENT_SAMPLING_PHASE_V) ?
      g_current_sampling.raw_v : g_current_sampling.raw_u;
}

float CurrentSampling_GetCurrentA(current_sampling_phase_t phase)
{
  int32_t raw_delta;
  uint16_t raw = CurrentSampling_GetRaw(phase);
  uint16_t offset = (phase == CURRENT_SAMPLING_PHASE_V) ?
      g_current_sampling.offset_v : g_current_sampling.offset_u;

  /* raw 和 offset 都是同一格式的右对齐 ADC 码，先在原始码空间相减。 */
  raw_delta = (int32_t)raw - (int32_t)offset;
  raw_delta *= (phase == CURRENT_SAMPLING_PHASE_V) ?
      (int32_t)g_current_sampling.polarity_v :
      (int32_t)g_current_sampling.polarity_u;
  /* JDR contains unshifted sums; use the selected hardware ratio exactly once. */
  return ((float)raw_delta * (float)g_current_sampling.vref_mv) /
         ((float)g_current_sampling.oversampling_ratio *
          (float)g_current_sampling.adc_full_scale *
          1000.0f * ((float)g_current_sampling.shunt_resistor_uohm / 1000000.0f) *
          g_current_sampling.opamp_gain);
}

static int16_t CurrentSampling_ClampS16(int32_t value)
{
  if (value > 32767)
  {
    return 32767;
  }
  if (value < -32768)
  {
    return -32768;
  }
  return (int16_t)value;
}

static int32_t CurrentSampling_GetSignedDelta(current_sampling_phase_t phase)
{
  int32_t delta = (phase == CURRENT_SAMPLING_PHASE_V) ?
      (int32_t)g_current_sampling.raw_v - (int32_t)g_current_sampling.offset_v :
      (int32_t)g_current_sampling.raw_u - (int32_t)g_current_sampling.offset_u;
  delta *= (phase == CURRENT_SAMPLING_PHASE_V) ?
      (int32_t)g_current_sampling.polarity_v :
      (int32_t)g_current_sampling.polarity_u;
  return delta;
}

void CurrentSampling_ReadFeedback_s16(int16_t *phase_u_s16,
                                      int16_t *phase_v_s16,
                                      int16_t *phase_w_s16)
{
  int32_t phase_u = CurrentSampling_GetSignedDelta(CURRENT_SAMPLING_PHASE_U);
  int32_t phase_v = CurrentSampling_GetSignedDelta(CURRENT_SAMPLING_PHASE_V);
  int32_t phase_w = -(phase_u + phase_v);

  if (phase_u_s16 != NULL) *phase_u_s16 = CurrentSampling_ClampS16(phase_u);
  if (phase_v_s16 != NULL) *phase_v_s16 = CurrentSampling_ClampS16(phase_v);
  if (phase_w_s16 != NULL) *phase_w_s16 = CurrentSampling_ClampS16(phase_w);
}

void CurrentSampling_ReadFeedback_A(float *phase_u_a,
                                    float *phase_v_a,
                                    float *phase_w_a)
{
  /* Same arithmetic/order as GetCurrentA, with one common calibration read
     and denominator calculation for the simultaneous U/V sample. */
  float denominator = (float)g_current_sampling.oversampling_ratio *
      (float)g_current_sampling.adc_full_scale * 1000.0f *
      ((float)g_current_sampling.shunt_resistor_uohm / 1000000.0f) *
      g_current_sampling.opamp_gain;
  float vref = (float)g_current_sampling.vref_mv;
  float phase_u = ((float)CurrentSampling_GetSignedDelta(CURRENT_SAMPLING_PHASE_U) * vref) / denominator;
  float phase_v = ((float)CurrentSampling_GetSignedDelta(CURRENT_SAMPLING_PHASE_V) * vref) / denominator;
  float phase_w = -(phase_u + phase_v);

  if (phase_u_a != NULL) *phase_u_a = phase_u;
  if (phase_v_a != NULL) *phase_v_a = phase_v;
  if (phase_w_a != NULL) *phase_w_a = phase_w;
}
