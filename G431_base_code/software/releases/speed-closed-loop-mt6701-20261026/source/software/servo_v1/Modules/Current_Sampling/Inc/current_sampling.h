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

/* Replacement PCB, MT6701 SSI active, MOE=0, 10 kHz, 2026-10-26.
   Two independent 1024-sample windows: U=1892.974/1892.964,
   V=1891.796/1891.876 ADC counts. Evidence:
   control_analysis/results/mt6701_speed_scan_20261026/off_adc_noise*.bin.
   Analog-chain or PCB changes require a new OFF calibration. */
#define CURRENT_SAMPLING_BOARD_OFFSET_U 1893U
#define CURRENT_SAMPLING_BOARD_OFFSET_V 1892U

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
  /* 最近一次零点采集窗口的原始码范围，供 Watch 判断模拟链噪声/开关干扰。
     offset 只有在完整采满 target 后才会更新。 */
  volatile uint16_t calibration_min_u;
  volatile uint16_t calibration_max_u;
  volatile uint16_t calibration_min_v;
  volatile uint16_t calibration_max_v;
} current_sampling_t;

typedef void (*current_sampling_conversion_callback_t)(void *context);

extern current_sampling_t g_current_sampling;
/* Normal external-trigger JEOS path: copy JDR1/2, clear old completion flags,
   then dispatch FOC. Other/error IRQ sources still go through the HAL. */
void CurrentSampling_InjectedFastIrq(void);

/** Load saved offsets at startup with MOE off and calibration still IDLE.
    Returns 0 without changing state if the recorded configuration mismatches.
    Does not collect samples, enable output, or clear a calibration error. */
uint8_t CurrentSampling_LoadBoardCalibration(void);

/** 请求一次异步零点校准；调用者必须确保三相电流为零，推荐保持 MOE 关闭。 */
void CurrentSampling_RequestCalibration(void);

/** 在主循环或应用任务中周期调用，不阻塞 FOC。 */
void CurrentSampling_Service(void);

current_sampling_cal_state_t CurrentSampling_GetCalibrationState(void);

/** 返回指定相的右对齐 12 位 ADC 原始码，范围 0..4095。 */
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
