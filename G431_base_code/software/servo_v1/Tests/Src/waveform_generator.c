#include "waveform_generator.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define WAVEFORM_QUEUE_LENGTH  32U
#define WAVEFORM_FRAME_VERSION 2U

typedef struct
{
  uint32_t source_sequence;
  waveform_output_mode_t mode;
  uint8_t channel_count;
  waveform_channel_t channels[WAVEFORM_MAX_CHANNELS];
  float value[WAVEFORM_MAX_CHANNELS];
} waveform_sample_t;

static waveform_transport_t s_transport;
static volatile waveform_sample_t s_queue[WAVEFORM_QUEUE_LENGTH];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static volatile uint32_t s_decimation_count;
static uint32_t s_decimation;
static uint16_t s_frame_sequence;

waveform_generator_manager_t g_waveform_generator;

static uint8_t WaveformGenerator_ChannelIsValid(waveform_channel_t channel)
{
  return ((channel > WAVEFORM_CHANNEL_NONE) &&
          (channel < WAVEFORM_CHANNEL_COUNT)) ? 1U : 0U;
}

static uint8_t WaveformGenerator_ApplySelection(void)
{
  waveform_output_mode_t mode = g_waveform_generator.requested_mode;
  waveform_channel_t channels[WAVEFORM_MAX_CHANNELS];
  uint8_t count = 0U;
  uint8_t i;

  if (mode == WAVEFORM_OUTPUT_DISABLED)
  {
    count = 0U;
  }
  else if (mode == WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT)
  {
    channels[0] = WAVEFORM_CHANNEL_PHASE_U_CURRENT_A;
    channels[1] = WAVEFORM_CHANNEL_PHASE_V_CURRENT_A;
    channels[2] = WAVEFORM_CHANNEL_PHASE_W_CURRENT_A;
    count = 3U;
  }
  else if (mode == WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ)
  {
    channels[0] = WAVEFORM_CHANNEL_D_CURRENT_REFERENCE_A;
    channels[1] = WAVEFORM_CHANNEL_D_CURRENT_FEEDBACK_A;
    channels[2] = WAVEFORM_CHANNEL_Q_CURRENT_REFERENCE_A;
    channels[3] = WAVEFORM_CHANNEL_Q_CURRENT_FEEDBACK_A;
    count = 4U;
  }
  else if (mode == WAVEFORM_OUTPUT_CUSTOM)
  {
    count = g_waveform_generator.custom_channel_count;
    if ((count == 0U) || (count > WAVEFORM_MAX_CHANNELS))
    {
      g_waveform_generator.selection_errors++;
      return 0U;
    }
    for (i = 0U; i < count; ++i)
    {
      channels[i] = g_waveform_generator.custom_channels[i];
      if (WaveformGenerator_ChannelIsValid(channels[i]) == 0U)
      {
        g_waveform_generator.selection_errors++;
        return 0U;
      }
    }
  }
  else
  {
    g_waveform_generator.selection_errors++;
    return 0U;
  }

  for (i = 0U; i < WAVEFORM_MAX_CHANNELS; ++i)
  {
    g_waveform_generator.active_channels[i] =
        (i < count) ? channels[i] : WAVEFORM_CHANNEL_NONE;
  }
  g_waveform_generator.active_channel_count = count;
  g_waveform_generator.active_mode = mode;
  g_waveform_generator.applied_selection_revision =
      g_waveform_generator.selection_revision;
  return 1U;
}

static float WaveformGenerator_ReadChannel(
    const waveform_source_snapshot_t *source,
    waveform_channel_t channel)
{
  switch (channel)
  {
    case WAVEFORM_CHANNEL_PHASE_U_CURRENT_A: return source->phase_u_current_a;
    case WAVEFORM_CHANNEL_PHASE_V_CURRENT_A: return source->phase_v_current_a;
    case WAVEFORM_CHANNEL_PHASE_W_CURRENT_A: return source->phase_w_current_a;
    case WAVEFORM_CHANNEL_D_CURRENT_REFERENCE_A: return source->d_current_reference_a;
    case WAVEFORM_CHANNEL_D_CURRENT_FEEDBACK_A: return source->d_current_feedback_a;
    case WAVEFORM_CHANNEL_Q_CURRENT_REFERENCE_A: return source->q_current_reference_a;
    case WAVEFORM_CHANNEL_Q_CURRENT_FEEDBACK_A: return source->q_current_feedback_a;
    case WAVEFORM_CHANNEL_D_VOLTAGE_OUTPUT_PU: return source->d_voltage_output_pu;
    case WAVEFORM_CHANNEL_Q_VOLTAGE_OUTPUT_PU: return source->q_voltage_output_pu;
    case WAVEFORM_CHANNEL_PHASE_U_DUTY: return source->phase_u_duty;
    case WAVEFORM_CHANNEL_PHASE_V_DUTY: return source->phase_v_duty;
    case WAVEFORM_CHANNEL_PHASE_W_DUTY: return source->phase_w_duty;
    case WAVEFORM_CHANNEL_ELECTRICAL_ANGLE_RAD: return source->electrical_angle_rad;
    case WAVEFORM_CHANNEL_DC_BUS_VOLTAGE_V: return source->dc_bus_voltage_v;
    default: return 0.0f;
  }
}

static uint8_t WaveformGenerator_Crc8(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0U;
  uint32_t i;
  for (i = 0U; i < length; ++i)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
      crc = ((crc & 0x80U) != 0U) ?
          (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
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
  uint8_t i;
  if ((transport == NULL) || (transport->send == NULL) || (config == NULL) ||
      (config->source_rate_hz == 0UL) || (config->output_rate_hz == 0UL) ||
      (config->output_rate_hz > config->source_rate_hz) ||
      (config->initial_mode >= WAVEFORM_OUTPUT_MODE_COUNT)) return 0U;

  s_transport = *transport;
  g_waveform_generator.config = *config;
  s_decimation = config->source_rate_hz / config->output_rate_hz;
  if (s_decimation == 0UL) s_decimation = 1UL;
  s_head = 0U;
  s_tail = 0U;
  s_decimation_count = 0UL;
  s_frame_sequence = 0U;
  g_waveform_generator.enabled = 1U;
  g_waveform_generator.requested_mode = config->initial_mode;
  g_waveform_generator.active_mode = WAVEFORM_OUTPUT_DISABLED;
  g_waveform_generator.custom_channel_count = 0U;
  g_waveform_generator.selection_revision = 1UL;
  g_waveform_generator.applied_selection_revision = 0UL;
  g_waveform_generator.active_channel_count = 0U;
  for (i = 0U; i < WAVEFORM_MAX_CHANNELS; ++i)
  {
    g_waveform_generator.custom_channels[i] = WAVEFORM_CHANNEL_NONE;
    g_waveform_generator.active_channels[i] = WAVEFORM_CHANNEL_NONE;
  }
  g_waveform_generator.queued_samples = 0U;
  g_waveform_generator.published_samples = 0UL;
  g_waveform_generator.dropped_samples = 0UL;
  g_waveform_generator.sent_frames = 0UL;
  g_waveform_generator.transport_errors = 0UL;
  g_waveform_generator.selection_errors = 0UL;
  return WaveformGenerator_ApplySelection();
}

uint8_t WaveformGenerator_SelectMode(waveform_output_mode_t mode)
{
  if (mode >= WAVEFORM_OUTPUT_MODE_COUNT) return 0U;
  g_waveform_generator.requested_mode = mode;
  g_waveform_generator.selection_revision++;
  return 1U;
}

uint8_t WaveformGenerator_ConfigureCustom(const waveform_channel_t *channels,
                                          uint8_t count)
{
  uint8_t i;
  if ((channels == NULL) || (count == 0U) ||
      (count > WAVEFORM_MAX_CHANNELS)) return 0U;
  for (i = 0U; i < count; ++i)
    if (WaveformGenerator_ChannelIsValid(channels[i]) == 0U) return 0U;
  for (i = 0U; i < WAVEFORM_MAX_CHANNELS; ++i)
    g_waveform_generator.custom_channels[i] =
        (i < count) ? channels[i] : WAVEFORM_CHANNEL_NONE;
  g_waveform_generator.custom_channel_count = count;
  g_waveform_generator.requested_mode = WAVEFORM_OUTPUT_CUSTOM;
  g_waveform_generator.selection_revision++;
  return 1U;
}

void WaveformGenerator_PublishFromIsr(
    uint32_t source_sequence,
    const waveform_source_snapshot_t *source)
{
  uint8_t next_head;
  uint8_t channel;
  if ((source == NULL) || (g_waveform_generator.enabled == 0U)) return;
  if ((g_waveform_generator.requested_mode != g_waveform_generator.active_mode) ||
      (g_waveform_generator.selection_revision !=
       g_waveform_generator.applied_selection_revision))
    if (WaveformGenerator_ApplySelection() == 0U) return;
  if (g_waveform_generator.active_mode == WAVEFORM_OUTPUT_DISABLED) return;

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
  s_queue[s_head].mode = g_waveform_generator.active_mode;
  s_queue[s_head].channel_count = g_waveform_generator.active_channel_count;
  for (channel = 0U; channel < WAVEFORM_MAX_CHANNELS; ++channel)
  {
    waveform_channel_t selected = g_waveform_generator.active_channels[channel];
    s_queue[s_head].channels[channel] = selected;
    s_queue[s_head].value[channel] =
        (channel < g_waveform_generator.active_channel_count) ?
        WaveformGenerator_ReadChannel(source, selected) : 0.0f;
  }
  s_head = next_head;
  g_waveform_generator.queued_samples++;
}

void WaveformGenerator_Task(void)
{
  uint8_t frame[WAVEFORM_MAX_FRAME_LENGTH];
  waveform_sample_t sample;
  uint8_t tail;
  uint8_t channel;
  uint8_t value_offset;
  uint16_t frame_length;
  if ((g_waveform_generator.enabled == 0U) || (s_tail == s_head)) return;

  tail = s_tail;
  sample.source_sequence = s_queue[tail].source_sequence;
  sample.mode = s_queue[tail].mode;
  sample.channel_count = s_queue[tail].channel_count;
  for (channel = 0U; channel < WAVEFORM_MAX_CHANNELS; ++channel)
  {
    sample.channels[channel] = s_queue[tail].channels[channel];
    sample.value[channel] = s_queue[tail].value[channel];
  }

  frame[0] = 0xA5U;
  frame[1] = 0x5AU;
  frame[2] = WAVEFORM_FRAME_VERSION;
  frame[3] = (uint8_t)sample.mode;
  frame[4] = sample.channel_count;
  WaveformGenerator_PutU16(&frame[5], s_frame_sequence);
  WaveformGenerator_PutU32(&frame[7], sample.source_sequence);
  for (channel = 0U; channel < sample.channel_count; ++channel)
    frame[11U + channel] = (uint8_t)sample.channels[channel];
  value_offset = (uint8_t)(11U + sample.channel_count);
  for (channel = 0U; channel < sample.channel_count; ++channel)
    WaveformGenerator_PutU32(&frame[value_offset + 4U * channel],
        (uint32_t)WaveformGenerator_ToMilliUnits(sample.value[channel]));
  frame_length = (uint16_t)(12U + 5U * sample.channel_count);
  frame[frame_length - 1U] = WaveformGenerator_Crc8(frame, frame_length - 1U);

  if (s_transport.send(s_transport.context, frame, frame_length) != 0U)
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
