#include "waveform_generator.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define WAVEFORM_QUEUE_LENGTH  32U
#define WAVEFORM_FRAME_LENGTH  24U
#define WAVEFORM_FRAME_VERSION 1U

typedef struct
{
  uint32_t source_sequence;
  float value[3];
} waveform_sample3_t;

static waveform_transport_t s_transport;
static volatile waveform_sample3_t s_queue[WAVEFORM_QUEUE_LENGTH];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static volatile uint32_t s_decimation_count;
static uint32_t s_decimation;
static uint16_t s_frame_sequence;

waveform_generator_status_t g_waveform_generator;

static uint8_t WaveformGenerator_Crc8(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0U;
  uint32_t i;

  for (i = 0U; i < length; ++i)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x80U) != 0U) ?
          (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static void WaveformGenerator_PutU16(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)(value & 0xFFU);
  destination[1] = (uint8_t)(value >> 8U);
}

static void WaveformGenerator_PutU32(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)(value & 0xFFUL);
  destination[1] = (uint8_t)((value >> 8U) & 0xFFUL);
  destination[2] = (uint8_t)((value >> 16U) & 0xFFUL);
  destination[3] = (uint8_t)(value >> 24U);
}

static int32_t WaveformGenerator_ToMilliUnits(float value)
{
  float scaled = value * 1000.0f;
  if (!isfinite(value)) return 0;
  if (scaled > 2147483520.0f) return INT32_MAX;
  if (scaled < -2147483520.0f) return INT32_MIN;
  return (int32_t)scaled;
}

uint8_t WaveformGenerator_Init(const waveform_transport_t *transport,
                               const waveform_generator_config_t *config)
{
  if ((transport == NULL) || (transport->send == NULL) || (config == NULL) ||
      (config->source_rate_hz == 0UL) || (config->output_rate_hz == 0UL) ||
      (config->output_rate_hz > config->source_rate_hz))
  {
    return 0U;
  }

  s_transport = *transport;
  g_waveform_generator.config = *config;
  s_decimation = config->source_rate_hz / config->output_rate_hz;
  if (s_decimation == 0UL) s_decimation = 1UL;
  s_head = 0U;
  s_tail = 0U;
  s_decimation_count = 0UL;
  s_frame_sequence = 0U;
  g_waveform_generator.enabled = 1U;
  g_waveform_generator.queued_samples = 0U;
  g_waveform_generator.published_samples = 0UL;
  g_waveform_generator.dropped_samples = 0UL;
  g_waveform_generator.sent_frames = 0UL;
  g_waveform_generator.transport_errors = 0UL;
  return 1U;
}

void WaveformGenerator_Publish3FromIsr(uint32_t source_sequence,
                                       float value_0,
                                       float value_1,
                                       float value_2)
{
  uint8_t next_head;

  if (g_waveform_generator.enabled == 0U) return;
  g_waveform_generator.published_samples++;
  if (++s_decimation_count < s_decimation) return;
  s_decimation_count = 0UL;

  next_head = (uint8_t)((s_head + 1U) % WAVEFORM_QUEUE_LENGTH);
  if (next_head == s_tail)
  {
    g_waveform_generator.dropped_samples++;
    return;
  }

  s_queue[s_head].source_sequence = source_sequence;
  s_queue[s_head].value[0] = value_0;
  s_queue[s_head].value[1] = value_1;
  s_queue[s_head].value[2] = value_2;
  s_head = next_head;
  g_waveform_generator.queued_samples++;
}

void WaveformGenerator_Task(void)
{
  uint8_t frame[WAVEFORM_FRAME_LENGTH];
  waveform_sample3_t sample;
  uint8_t tail;
  uint8_t channel;

  if ((g_waveform_generator.enabled == 0U) || (s_tail == s_head)) return;

  tail = s_tail;
  sample.source_sequence = s_queue[tail].source_sequence;
  sample.value[0] = s_queue[tail].value[0];
  sample.value[1] = s_queue[tail].value[1];
  sample.value[2] = s_queue[tail].value[2];

  frame[0] = 0xA5U;
  frame[1] = 0x5AU;
  frame[2] = WAVEFORM_FRAME_VERSION;
  frame[3] = (uint8_t)g_waveform_generator.config.signal_group;
  frame[4] = 3U;
  WaveformGenerator_PutU16(&frame[5], s_frame_sequence);
  WaveformGenerator_PutU32(&frame[7], sample.source_sequence);
  for (channel = 0U; channel < 3U; ++channel)
  {
    WaveformGenerator_PutU32(&frame[11U + 4U * channel],
        (uint32_t)WaveformGenerator_ToMilliUnits(sample.value[channel]));
  }
  frame[23] = WaveformGenerator_Crc8(frame, 23U);

  if (s_transport.send(s_transport.context, frame, sizeof(frame)) != 0U)
  {
    s_frame_sequence++;
    s_tail = (uint8_t)((tail + 1U) % WAVEFORM_QUEUE_LENGTH);
    if (g_waveform_generator.queued_samples != 0U)
      g_waveform_generator.queued_samples--;
    g_waveform_generator.sent_frames++;
  }
  else
  {
    g_waveform_generator.transport_errors++;
  }
}
