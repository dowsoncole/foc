#ifndef USER_CONTROL_H
#define USER_CONTROL_H
#include "foc_algorithm.h"

typedef enum {
  USER_MODE_VOLTAGE = 0, USER_MODE_ENCODER_CALIBRATION,
  USER_MODE_OPEN_CURRENT, USER_MODE_TORQUE, USER_MODE_SPEED, USER_MODE_POSITION,
  USER_MODE_COUNT
} user_control_mode_t;
typedef enum {
  USER_IDLE = 0, USER_STARTING, USER_RUNNING, USER_DONE, USER_FAULT
} user_control_state_t;
typedef enum {
  USER_OK = 0, USER_BAD_PARAMETER, USER_NOT_READY, USER_BACKEND_FAULT
} user_control_fault_t;

/* Write parameters/mode first, then enable=1. enable=0 stops output.
   Status is read-only. Mode changes stop before starting the new mode.
   A fault requires enable=0 then 1; underlying protection is never cleared here. */
typedef struct {
  uint32_t enable, mode;
  struct { float ud_v, uq_v, electrical_hz; } voltage;
  struct { float ud_v, uq_v, duration_s; } calibration;
  struct { float id_a, iq_a, electrical_hz; } open_current;
  struct { float id_a, iq_a; } torque;
  struct {
    float target_rpm, acceleration_rpm_s, deceleration_rpm_s;
    float forward_limit_rpm, reverse_limit_rpm; /* Both limits positive. */
  } speed;
  /* Zero is the output position at this position-mode enable. Targets remain
     absolute within that session: 720 -> two turns, then 180 -> half turn. */
  struct { float target_deg; } position;
  struct {
    uint32_t state, fault, active_mode;
    float actual_rpm, actual_position_deg, id_a, iq_a, zero_electrical_rad;
    uint32_t calibration_valid;
  } status;
} user_control_t;
extern volatile user_control_t g_user_control;

uint8_t UserControl_Validate(uint32_t mode);
float UserControl_SpeedRamp(float applied_rpm, float target_rpm, float dt);
/* Called only for active voltage/open-current/torque modes from ADC. */
uint8_t UserControl_FastReference(foc_input_t *input, float dt,
                                 float encoder_angle, uint8_t encoder_valid);
void UserControl_ResetAngle(void);
#endif
