#include "encoder_calibration.h"
#include <math.h>
#include <string.h>

encoder_calibration_t g_encoder_calibration = {
  .requested = {ENCODER_CALIBRATION_DEFAULT_VOLTAGE,
                ENCODER_CALIBRATION_DEFAULT_SECONDS, 0.0f}
};
/* Port table belongs to the caller and must outlive the module (main's frame
   lives forever). Keep only a pointer on this 32-KiB device. */
static const encoder_calibration_port_t *s_ports;
static const encoder_calibration_port_t no_ports = {0};
#define ports (*s_ports)

static uint8_t config_valid(encoder_calibration_config_t c) {
  float magnitude = hypotf(c.d_voltage_v, c.q_voltage_v);
  return isfinite(magnitude) && magnitude > 0.0f &&
      magnitude <= ENCODER_CALIBRATION_MAX_VOLTAGE &&
      isfinite(c.duration_s) && c.duration_s >= 0.05f &&
      c.duration_s <= ENCODER_CALIBRATION_MAX_SECONDS;
}
static void fail(encoder_calibration_fault_t reason) {
  g_encoder_calibration.fault = reason;
  if (ports.stop) ports.stop();
  g_encoder_calibration.state = ENCODER_CAL_FAILED;
}
uint8_t EncoderCalibration_Busy(void) {
  uint32_t s = g_encoder_calibration.state;
  return s == ENCODER_CAL_QUEUED || s == ENCODER_CAL_RUNNING ||
         s == ENCODER_CAL_CAPTURED;
}
void EncoderCalibration_Init(const encoder_calibration_port_t *port,
                              int32_t direction, uint32_t poles) {
  memset(&g_encoder_calibration, 0, sizeof(g_encoder_calibration));
  s_ports = port ? port : &no_ports;
  g_encoder_calibration.requested = (encoder_calibration_config_t){
      ENCODER_CALIBRATION_DEFAULT_VOLTAGE, ENCODER_CALIBRATION_DEFAULT_SECONDS, 0.0f};
  g_encoder_calibration.direction = direction;
  g_encoder_calibration.pole_pairs = poles;
}
uint8_t EncoderCalibration_Load(uint8_t off) {
  encoder_calibration_t *s = &g_encoder_calibration;
  float offset;
  if (!off || s->state != ENCODER_CAL_IDLE || s->command || s->valid ||
      !ports.load || !ports.apply ||
      !ports.load(s->direction, s->pole_pairs, &offset)) return 0;
  if (!isfinite(offset) || offset < 0 || offset >= 6.28318530718f ||
      !ports.apply(s->direction, s->pole_pairs, offset)) return 0;
  s->offset_rad = offset;
  s->valid = s->loaded_from_storage = 1U;
  return 1;
}
uint8_t EncoderCalibration_Begin(int32_t direction, uint32_t poles) {
  encoder_calibration_t *s = &g_encoder_calibration;
  if (s->state == ENCODER_CAL_RUNNING || s->state == ENCODER_CAL_CAPTURED ||
      (direction != 1 && direction != -1) || poles == 0 || poles > 64)
    return 0;
  encoder_calibration_config_t c = s->state == ENCODER_CAL_QUEUED ?
      s->active : s->requested;
  if (!config_valid(c)) return 0;
  s->active = c;
  s->direction = direction;
  s->pole_pairs = poles;
  s->fault = ENCODER_CAL_OK;
  s->state = ENCODER_CAL_RUNNING;
  return 1;
}
uint8_t EncoderCalibration_Capture(uint32_t raw) {
  encoder_calibration_t *s = &g_encoder_calibration;
  if (s->state != ENCODER_CAL_RUNNING || raw >= 2097152UL) return 0;
  float offset = atan2f(s->active.q_voltage_v, s->active.d_voltage_v) -
      (float)s->direction * (float)s->pole_pairs *
      (float)raw * (6.28318530718f / 2097152.0f);
  offset = fmodf(offset, 6.28318530718f);
  if (offset < 0) offset += 6.28318530718f;
  s->candidate_offset_rad = offset;
  /* Cortex-M4 normal-memory ordering; state publishes the completed candidate. */
  s->state = ENCODER_CAL_CAPTURED;
  return 1;
}
void EncoderCalibration_Service(uint8_t off) {
  encoder_calibration_t *s = &g_encoder_calibration;
  uint32_t command = s->command;
  s->command = 0;
  if (command == 2U) { fail(ENCODER_CAL_CANCELLED); return; }
  if (command) {
    if (command != 1U || !config_valid(s->requested)) {
      if (!EncoderCalibration_Busy()) fail(ENCODER_CAL_BAD_CONFIG);
      return;
    }
    if (!off || EncoderCalibration_Busy()) return;
    s->active = s->requested;
    s->fault = ENCODER_CAL_OK;
    s->queued_ms = ports.now_ms ? ports.now_ms() : 0;
    s->state = ENCODER_CAL_QUEUED;
    if (!ports.start || !ports.start()) { fail(ENCODER_CAL_BUSY); return; }
  }
  if (EncoderCalibration_Busy() && ports.backend_fault && ports.backend_fault()) {
    fail(ENCODER_CAL_BACKEND_FAULT); return;
  }
  if (s->state == ENCODER_CAL_QUEUED && ports.now_ms &&
      (uint32_t)(ports.now_ms() - s->queued_ms) > 200U) {
    fail(ENCODER_CAL_START_TIMEOUT); return;
  }
  if (s->state != ENCODER_CAL_CAPTURED || !off) return;
  float offset = s->candidate_offset_rad;
  uint8_t saved = ports.save ?
      ports.save(s->direction, s->pole_pairs, offset, &s->active) : 0U;
  if (saved == 2U) return;
  if (saved != 1U) {
    fail(ENCODER_CAL_SAVE_FAILED); return;
  }
  s->save_count++;
  if (!ports.apply || !ports.apply(s->direction, s->pole_pairs, offset)) {
    fail(ENCODER_CAL_APPLY_FAILED); return;
  }
  s->offset_rad = offset;
  s->valid = 1;
  s->loaded_from_storage = 0;
  s->state = ENCODER_CAL_DONE;
}
