#ifndef WAVEFORM_GENERATOR_H
#define WAVEFORM_GENERATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WAVEFORM_MAX_CHANNELS      4U
#define WAVEFORM_MAX_FRAME_LENGTH 32U

typedef enum
{
  WAVEFORM_OUTPUT_DISABLED = 0,
  WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT,
  WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ,
  WAVEFORM_OUTPUT_CUSTOM,
  WAVEFORM_OUTPUT_MODE_COUNT
} waveform_output_mode_t;

typedef enum
{
  WAVEFORM_CHANNEL_NONE = 0,
  WAVEFORM_CHANNEL_PHASE_U_CURRENT_A = 1,
  WAVEFORM_CHANNEL_PHASE_V_CURRENT_A,
  WAVEFORM_CHANNEL_PHASE_W_CURRENT_A,
  WAVEFORM_CHANNEL_D_CURRENT_REFERENCE_A,
  WAVEFORM_CHANNEL_D_CURRENT_FEEDBACK_A,
  WAVEFORM_CHANNEL_Q_CURRENT_REFERENCE_A,
  WAVEFORM_CHANNEL_Q_CURRENT_FEEDBACK_A,
  WAVEFORM_CHANNEL_D_VOLTAGE_OUTPUT_PU,
  WAVEFORM_CHANNEL_Q_VOLTAGE_OUTPUT_PU,
  WAVEFORM_CHANNEL_PHASE_U_DUTY,
  WAVEFORM_CHANNEL_PHASE_V_DUTY,
  WAVEFORM_CHANNEL_PHASE_W_DUTY,
  WAVEFORM_CHANNEL_ELECTRICAL_ANGLE_RAD,
  WAVEFORM_CHANNEL_DC_BUS_VOLTAGE_V,
  WAVEFORM_CHANNEL_COUNT
} waveform_channel_t;

/* ISR 发布完整业务快照，波形层再按模式选择最多四个通道。 */
typedef struct
{
  float phase_u_current_a;
  float phase_v_current_a;
  float phase_w_current_a;
  float d_current_reference_a;
  float d_current_feedback_a;
  float q_current_reference_a;
  float q_current_feedback_a;
  float d_voltage_output_pu;
  float q_voltage_output_pu;
  float phase_u_duty;
  float phase_v_duty;
  float phase_w_duty;
  float electrical_angle_rad;
  float dc_bus_voltage_v;
} waveform_source_snapshot_t;

typedef uint8_t (*waveform_transport_send_t)(void *context,
                                              const uint8_t *data,
                                              uint16_t length);

typedef struct
{
  waveform_transport_send_t send;
  void *context;
} waveform_transport_t;

typedef struct
{
  uint32_t source_rate_hz;
  uint32_t output_rate_hz;
  waveform_output_mode_t initial_mode;
} waveform_generator_config_t;

/* 全局波形管理对象：requested/custom 是命令，active/统计量是模块状态。 */
typedef struct
{
  volatile uint8_t enabled;
  volatile waveform_output_mode_t requested_mode;
  volatile waveform_output_mode_t active_mode;
  volatile uint8_t custom_channel_count;
  volatile waveform_channel_t custom_channels[WAVEFORM_MAX_CHANNELS];
  volatile uint32_t selection_revision;
  volatile uint32_t applied_selection_revision;
  volatile uint8_t active_channel_count;
  volatile waveform_channel_t active_channels[WAVEFORM_MAX_CHANNELS];
  volatile uint8_t queued_samples;
  volatile uint32_t published_samples;
  volatile uint32_t dropped_samples;
  volatile uint32_t sent_frames;
  volatile uint32_t transport_errors;
  volatile uint32_t selection_errors;
  waveform_generator_config_t config;
} waveform_generator_manager_t;

extern waveform_generator_manager_t g_waveform_generator;

uint8_t WaveformGenerator_Init(const waveform_transport_t *transport,
                               const waveform_generator_config_t *config);
uint8_t WaveformGenerator_SelectMode(waveform_output_mode_t mode);
uint8_t WaveformGenerator_ConfigureCustom(const waveform_channel_t *channels,
                                          uint8_t count);
void WaveformGenerator_PublishFromIsr(
    uint32_t source_sequence,
    const waveform_source_snapshot_t *source);
void WaveformGenerator_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* WAVEFORM_GENERATOR_H */
