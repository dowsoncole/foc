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

#define CURRENT_SAMPLING_CAL_SETTLE_MS  10UL
#define CURRENT_SAMPLING_CAL_WINDOW_MS   1UL

current_sampling_t g_current_sampling =
{
  .shunt_resistor_uohm = 7000UL,
  .opamp_gain = 6.1f,
  .vref_mv = 3300UL,
  .adc_full_scale = 4095UL,
  .oversampling_ratio = ADC1_CS_OVERSAMPLING_RATIO,
  .sample_rate_hz = 20000UL,
  .offset_u = 0U,
  .offset_v = 0U,
  .polarity_u = 1,
  .polarity_v = 1,
  .raw_u = 0U,
  .raw_v = 0U,
  .sample_sequence = 0UL,
  .calibration_state = CURRENT_SAMPLING_CAL_IDLE,
  .calibration_sample_count = 0UL,
  .calibration_target_samples = 0UL,
  .calibration_deadline_ms = 0UL,
  .calibration_sum_u = 0ULL,
  .calibration_sum_v = 0ULL
};

static current_sampling_conversion_callback_t s_conversion_callback;
static void *s_conversion_context;

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
  g_current_sampling.sample_sequence++;

  if (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_COLLECTING &&
      g_current_sampling.calibration_sample_count <
      g_current_sampling.calibration_target_samples)
  {
    g_current_sampling.calibration_sum_u += g_current_sampling.raw_u;
    g_current_sampling.calibration_sum_v += g_current_sampling.raw_v;
    g_current_sampling.calibration_sample_count++;
  }

  if ((s_conversion_callback != NULL) &&
      (g_current_sampling.calibration_state != CURRENT_SAMPLING_CAL_COLLECTING) &&
      (g_current_sampling.calibration_state != CURRENT_SAMPLING_CAL_SETTLING))
  {
    s_conversion_callback(s_conversion_context);
  }
}

void CurrentSampling_SetConversionCallback(
    current_sampling_conversion_callback_t callback, void *context)
{
  s_conversion_callback = callback;
  s_conversion_context = context;
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
      g_current_sampling.calibration_deadline_ms = now +
          CURRENT_SAMPLING_CAL_WINDOW_MS;
      g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_COLLECTING;
    }
  }
  else if (g_current_sampling.calibration_state == CURRENT_SAMPLING_CAL_COLLECTING)
  {
    if ((g_current_sampling.calibration_sample_count >=
         g_current_sampling.calibration_target_samples) ||
        (CurrentSampling_Elapsed(now,
                                 g_current_sampling.calibration_deadline_ms) != 0UL))
    {
      if (g_current_sampling.calibration_sample_count != 0UL)
      {
        g_current_sampling.offset_u = (uint16_t)
            (g_current_sampling.calibration_sum_u /
             g_current_sampling.calibration_sample_count);
        g_current_sampling.offset_v = (uint16_t)
            (g_current_sampling.calibration_sum_v /
             g_current_sampling.calibration_sample_count);
        g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_DONE;
      }
      else
      {
        g_current_sampling.calibration_state = CURRENT_SAMPLING_CAL_ERROR;
      }
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

  raw_delta = (int32_t)raw - (int32_t)offset;
  raw_delta *= (phase == CURRENT_SAMPLING_PHASE_V) ?
      (int32_t)g_current_sampling.polarity_v :
      (int32_t)g_current_sampling.polarity_u;
  return ((float)raw_delta * (float)g_current_sampling.vref_mv) /
         ((float)g_current_sampling.oversampling_ratio *
          (float)g_current_sampling.adc_full_scale *
          1000.0f * ((float)g_current_sampling.shunt_resistor_uohm / 1000000.0f) *
          g_current_sampling.opamp_gain);
}

void CurrentSampling_ReadFeedback(float *phase_u_a, float *phase_v_a)
{
  if (phase_u_a != NULL)
  {
    *phase_u_a = CurrentSampling_GetCurrentA(CURRENT_SAMPLING_PHASE_U);
  }
  if (phase_v_a != NULL)
  {
    *phase_v_a = CurrentSampling_GetCurrentA(CURRENT_SAMPLING_PHASE_V);
  }
}
