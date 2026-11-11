#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>

#define ENCODER_CALIBRATION_DEFAULT_VOLTAGE 1.5f
#define ENCODER_CALIBRATION_DEFAULT_SECONDS 3.0f
#define ENCODER_CALIBRATION_MAX_VOLTAGE 2.8f
/* DWT elapsed time remains unambiguous below one 168 MHz counter wrap. */
#define ENCODER_CALIBRATION_MAX_SECONDS 10.0f

typedef struct { float d_voltage_v, duration_s, q_voltage_v; } encoder_calibration_config_t;
typedef enum {
  ENCODER_CAL_IDLE, ENCODER_CAL_QUEUED, ENCODER_CAL_RUNNING,
  ENCODER_CAL_CAPTURED, ENCODER_CAL_DONE, ENCODER_CAL_FAILED
} encoder_calibration_state_t;
typedef enum {
  ENCODER_CAL_OK, ENCODER_CAL_BAD_CONFIG, ENCODER_CAL_BUSY,
  ENCODER_CAL_BACKEND_FAULT, ENCODER_CAL_CANCELLED,
  ENCODER_CAL_SAVE_FAILED, ENCODER_CAL_APPLY_FAILED,
  ENCODER_CAL_START_TIMEOUT
} encoder_calibration_fault_t;

/* Business ports: the module knows neither the speed test nor STM32/HAL. */
typedef struct {
  uint8_t (*start)(void);
  void (*stop)(void);
  uint32_t (*backend_fault)(void);
  uint32_t (*now_ms)(void);
  uint8_t (*load)(int32_t direction, uint32_t poles, float *offset);
  /* save: 0=failed, 1=verified, 2=transport busy; retry while safely OFF. */
  uint8_t (*save)(int32_t direction, uint32_t poles, float offset,
                  const encoder_calibration_config_t *recipe);
  uint8_t (*apply)(int32_t direction, uint32_t poles, float offset);
} encoder_calibration_port_t;

typedef struct {
  /* Watch/API: 1=start requested recipe, 2=cancel. Set recipe before command. */
  volatile uint32_t command;
  volatile uint8_t state; /* encoder_calibration_state_t */
  volatile uint8_t fault; /* encoder_calibration_fault_t */
  int8_t direction;
  uint8_t pole_pairs;
  encoder_calibration_config_t requested, active;
  volatile float candidate_offset_rad;
  float offset_rad;
  volatile uint8_t valid, loaded_from_storage;
  volatile uint16_t save_count;
  uint32_t queued_ms;
} encoder_calibration_t;
extern encoder_calibration_t g_encoder_calibration;

/* Port table must remain alive for all subsequent calls. */
void EncoderCalibration_Init(const encoder_calibration_port_t *port,
                              int32_t direction, uint32_t poles);
/* Foreground only; load/apply is performed once, with output OFF. */
uint8_t EncoderCalibration_Load(uint8_t output_off);
void EncoderCalibration_Service(uint8_t output_off);
/* Voltage executor latches one immutable recipe while OFF, before enabling PWM. */
uint8_t EncoderCalibration_Begin(int32_t direction, uint32_t poles);
/* ADC executor captures the last energized raw angle; never writes Flash. */
uint8_t EncoderCalibration_Capture(uint32_t raw_angle);
uint8_t EncoderCalibration_Busy(void);

#endif
