#ifndef ENCODER_CALIBRATION_STORE_H
#define ENCODER_CALIBRATION_STORE_H
#include <stdint.h>

/* Last two 2-KiB pages of the 128-KiB STM32G431VB, reserved by the linker.
   Alternating pages retain the previous valid record through power loss. */
#define ENCODER_CAL_STORE_PAGE0 0x0801F000UL
#define ENCODER_CAL_STORE_PAGE1 0x0801F800UL
#define ENCODER_CAL_STORE_MAGIC 0x31435A45UL
#define ENCODER_CAL_STORE_COMMIT UINT64_C(0x434F4D4D49544544)
typedef struct {
  uint32_t magic, version, sequence;
  int32_t direction;
  uint32_t pole_pairs;
  float offset_rad, d_voltage_v, duration_s;
  uint32_t checksum, reserved;
  uint64_t commit;
} encoder_calibration_record_t;
uint32_t EncoderCalibrationStore_Checksum(const encoder_calibration_record_t *r);
uint8_t EncoderCalibrationStore_Valid(const encoder_calibration_record_t *r);
uint8_t EncoderCalibrationStore_Load(int32_t direction, uint32_t poles,
                                    float *offset);
uint8_t EncoderCalibrationStore_Save(int32_t direction, uint32_t poles,
                                    float offset, float voltage, float seconds);
#endif
