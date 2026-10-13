/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    measurement_manager.h
  * @brief   Common configuration and run-time state for board measurements.
  *
  * A single object owns the selected measurement module, its configuration and
  * its result. Add future measurements as another union member instead of
  * scattering test switches and parameters through main.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MEASUREMENT_MANAGER_H__
#define __MEASUREMENT_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  MEASUREMENT_MODULE_NONE = 0,
  MEASUREMENT_MODULE_MOS_DRIVER
} measurement_module_t;

typedef enum
{
  MEASUREMENT_STATE_IDLE = 0,
  MEASUREMENT_STATE_RUNNING,
  MEASUREMENT_STATE_PASSED,
  MEASUREMENT_STATE_FAILED,
  MEASUREMENT_STATE_TIMEOUT
} measurement_state_t;

typedef enum
{
  MOS_DRIVER_PHASE_U = 0,
  MOS_DRIVER_PHASE_V,
  MOS_DRIVER_PHASE_W
} mos_driver_phase_t;

typedef struct
{
  mos_driver_phase_t phase;
  uint16_t duty;
  uint8_t moe_enable;
  /* Reserved for a future result timeout; keep 0 for continuous measurement. */
  uint32_t duration_ms;
  volatile uint32_t deadline_ms;
  volatile uint8_t moe_applied;
  volatile mos_driver_phase_t phase_applied;
  volatile uint16_t duty_applied;
} mos_driver_measurement_t;

typedef struct
{
  measurement_module_t module;
  volatile measurement_state_t state;

  union
  {
    mos_driver_measurement_t mos_driver;
  } config;
} measurement_manager_t;

extern measurement_manager_t g_measurement_manager;

#ifdef __cplusplus
}
#endif

#endif /* __MEASUREMENT_MANAGER_H__ */
