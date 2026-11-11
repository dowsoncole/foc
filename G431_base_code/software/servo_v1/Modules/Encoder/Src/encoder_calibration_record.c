#include "encoder_calibration_store.h"
#include <math.h>
#include <stddef.h>

/* Explicit byte CRC32; independent of MCU CRC peripheral configuration. */
uint32_t EncoderCalibrationStore_Checksum(const encoder_calibration_record_t *r) {
  const uint8_t *p = (const uint8_t *)r;
  uint32_t crc = UINT32_MAX;
  for (uint32_t i = 0; i < offsetof(encoder_calibration_record_t, checksum); i++) {
    crc ^= p[i];
    for (uint32_t bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
  }
  return ~crc;
}
uint8_t EncoderCalibrationStore_Valid(const encoder_calibration_record_t *r) {
  return r && r->magic == ENCODER_CAL_STORE_MAGIC && r->version == 1U &&
      r->commit == ENCODER_CAL_STORE_COMMIT && r->reserved == 0U &&
      (r->direction == 1 || r->direction == -1) &&
      r->pole_pairs > 0 && r->pole_pairs <= 64 &&
      isfinite(r->offset_rad) && r->offset_rad >= 0 && r->offset_rad < 6.28318530718f &&
      isfinite(r->d_voltage_v) && r->d_voltage_v > 0 && r->d_voltage_v <= 2.8f &&
      isfinite(r->duration_s) && r->duration_s >= .01f && r->duration_s <= 10.0f &&
      r->checksum == EncoderCalibrationStore_Checksum(r);
}
