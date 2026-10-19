#ifndef WAVEFORM_GENERATOR_H
#define WAVEFORM_GENERATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WAVEFORM_MAX_FRAME_LENGTH  24U

typedef enum
{
  WAVEFORM_SIGNAL_THREE_PHASE_CURRENT_A = 1,
  WAVEFORM_SIGNAL_USER_DEFINED = 0x80
} waveform_signal_group_t;

/* 测试层只依赖此发送端口，不依赖 UART/CAN/RS485 或 HAL。发送在主循环调用。 */
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
  waveform_signal_group_t signal_group;
} waveform_generator_config_t;

typedef struct
{
  volatile uint8_t enabled;
  volatile uint8_t queued_samples;
  volatile uint32_t published_samples;
  volatile uint32_t dropped_samples;
  volatile uint32_t sent_frames;
  volatile uint32_t transport_errors;
  waveform_generator_config_t config;
} waveform_generator_status_t;

extern waveform_generator_status_t g_waveform_generator;

uint8_t WaveformGenerator_Init(const waveform_transport_t *transport,
                               const waveform_generator_config_t *config);

/* 由实时中断发布三个同步变量；只做降采样和内存入队，不访问通信外设。 */
void WaveformGenerator_Publish3FromIsr(uint32_t source_sequence,
                                       float value_0,
                                       float value_1,
                                       float value_2);

/* 主循环每次最多发送一帧。当前帧把三个 A 值编码成有符号 mA。 */
void WaveformGenerator_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* WAVEFORM_GENERATOR_H */
