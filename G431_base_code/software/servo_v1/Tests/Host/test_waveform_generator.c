#include "waveform_generator.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_WAVEFORM(condition) do { if (!(condition)) { \
  fprintf(stderr, "WAVEFORM FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static uint8_t captured[WAVEFORM_MAX_FRAME_LENGTH];
static uint16_t captured_length;

static uint8_t fake_send(void *context, const uint8_t *data, uint16_t length)
{
  (void)context;
  CHECK_WAVEFORM(length <= sizeof(captured));
  memcpy(captured, data, length);
  captured_length = length;
  return 1U;
}

static uint8_t test_crc8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;
  uint16_t i;
  for (i = 0U; i < length; ++i)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
  }
  return crc;
}

void TestWaveformGenerator(void)
{
  waveform_transport_t transport = { fake_send, NULL };
  waveform_generator_config_t config =
      { 20000UL, 20000UL, WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ };
  waveform_source_snapshot_t source = {0};
  waveform_channel_t custom[3] =
  {
    WAVEFORM_CHANNEL_PHASE_U_CURRENT_A,
    WAVEFORM_CHANNEL_D_VOLTAGE_OUTPUT_PU,
    WAVEFORM_CHANNEL_ELECTRICAL_ANGLE_RAD
  };

  source.phase_u_current_a = -1.25f;
  source.d_current_reference_a = 2.0f;
  source.d_current_feedback_a = 1.5f;
  source.q_current_reference_a = -0.5f;
  source.q_current_feedback_a = -0.25f;
  source.d_voltage_output_pu = 0.125f;
  source.electrical_angle_rad = 3.0f;

  CHECK_WAVEFORM(WaveformGenerator_Init(&transport, &config) != 0U);
  WaveformGenerator_PublishFromIsr(123UL, &source);
  WaveformGenerator_Task();
  CHECK_WAVEFORM(captured_length == 32U);
  CHECK_WAVEFORM(captured[2] == 2U);
  CHECK_WAVEFORM(captured[3] == WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ);
  CHECK_WAVEFORM(captured[4] == 4U);
  CHECK_WAVEFORM(captured[11] == WAVEFORM_CHANNEL_D_CURRENT_REFERENCE_A);
  CHECK_WAVEFORM(captured[12] == WAVEFORM_CHANNEL_D_CURRENT_FEEDBACK_A);
  CHECK_WAVEFORM(captured[13] == WAVEFORM_CHANNEL_Q_CURRENT_REFERENCE_A);
  CHECK_WAVEFORM(captured[14] == WAVEFORM_CHANNEL_Q_CURRENT_FEEDBACK_A);
  CHECK_WAVEFORM(test_crc8(captured, 31U) == captured[31]);

  CHECK_WAVEFORM(WaveformGenerator_ConfigureCustom(custom, 3U) != 0U);
  WaveformGenerator_PublishFromIsr(124UL, &source);
  WaveformGenerator_Task();
  CHECK_WAVEFORM(captured_length == 27U);
  CHECK_WAVEFORM(captured[3] == WAVEFORM_OUTPUT_CUSTOM);
  CHECK_WAVEFORM(captured[4] == 3U);
  CHECK_WAVEFORM(captured[11] == WAVEFORM_CHANNEL_PHASE_U_CURRENT_A);
  CHECK_WAVEFORM(captured[12] == WAVEFORM_CHANNEL_D_VOLTAGE_OUTPUT_PU);
  CHECK_WAVEFORM(captured[13] == WAVEFORM_CHANNEL_ELECTRICAL_ANGLE_RAD);
  CHECK_WAVEFORM(test_crc8(captured, 26U) == captured[26]);

  CHECK_WAVEFORM(WaveformGenerator_SelectMode(WAVEFORM_OUTPUT_DISABLED) != 0U);
  captured_length = 0U;
  WaveformGenerator_PublishFromIsr(125UL, &source);
  WaveformGenerator_Task();
  CHECK_WAVEFORM(captured_length == 0U);
}
