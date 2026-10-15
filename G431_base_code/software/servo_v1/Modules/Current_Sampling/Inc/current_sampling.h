/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    current_sampling.h
  * @brief   与 ADC/OPAMP 配置解耦的相电流采样业务接口。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CURRENT_SAMPLING_H__
#define __CURRENT_SAMPLING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  CURRENT_SAMPLING_PHASE_U = 0,
  CURRENT_SAMPLING_PHASE_V = 1
} current_sampling_phase_t;

typedef enum
{
  CURRENT_SAMPLING_CAL_IDLE = 0,
  CURRENT_SAMPLING_CAL_SETTLING,
  CURRENT_SAMPLING_CAL_COLLECTING,
  CURRENT_SAMPLING_CAL_DONE,
  CURRENT_SAMPLING_CAL_ERROR
} current_sampling_cal_state_t;

typedef struct
{
  /* 配置参数：初始化后由应用/板级配置设定，运行过程中通常不变。 */
  uint32_t shunt_resistor_uohm;
  float    opamp_gain;
  uint32_t vref_mv;
  uint32_t adc_full_scale;
  uint32_t oversampling_ratio;
  uint32_t sample_rate_hz;

  uint16_t offset_u;
  uint16_t offset_v;
  /* 传感器正方向：+1 表示 raw > offset 为流入电机，-1 用于板级反相。 */
  int8_t polarity_u;
  int8_t polarity_v;

  /* 运行参数：由 ADC 中断和校准状态机实时更新。 */
  volatile uint16_t raw_u;
  volatile uint16_t raw_v;
  volatile uint32_t sample_sequence;
  volatile current_sampling_cal_state_t calibration_state;
  volatile uint32_t calibration_sample_count;
  uint32_t calibration_target_samples;
  uint32_t calibration_deadline_ms;
  volatile uint64_t calibration_sum_u;
  volatile uint64_t calibration_sum_v;
} current_sampling_t;

typedef void (*current_sampling_conversion_callback_t)(void *context);

extern current_sampling_t g_current_sampling;

/** 请求一次异步零点校准；调用者必须先让三相下管处于导通状态。 */
void CurrentSampling_RequestCalibration(void);

/** 在主循环或应用任务中周期调用，不阻塞 FOC。 */
void CurrentSampling_Service(void);

current_sampling_cal_state_t CurrentSampling_GetCalibrationState(void);

/** 返回指定相的 16 倍过采样原始累加值，范围约 0..65520。 */
uint16_t CurrentSampling_GetRaw(current_sampling_phase_t phase);

/** 返回减去零点后的实际电流，单位 A。 */
float CurrentSampling_GetCurrentA(current_sampling_phase_t phase);

/** 给 FOC 使用的三相标定 S16 电流反馈，范围 -32768..32767。 */
void CurrentSampling_ReadFeedback_s16(int16_t *phase_u_s16,
                                      int16_t *phase_v_s16,
                                      int16_t *phase_w_s16);

/** 给需要物理量的上层使用的三相电流反馈，单位 A。 */
void CurrentSampling_ReadFeedback_A(float *phase_u_a,
                                    float *phase_v_a,
                                    float *phase_w_a);

/** 注册 ADC 注入序列完成后的实时回调。回调运行在 ADC1_2_IRQHandler 中。 */
void CurrentSampling_SetConversionCallback(
    current_sampling_conversion_callback_t callback, void *context);

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_SAMPLING_H__ */
