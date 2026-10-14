/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    measurement_manager.h
  * @brief   整板测量的统一配置与运行状态。
  *
  *          由单个对象持有「当前选中的测量模块 + 它的配置 + 它的结果」。
  *          未来新增测量项时，请再加一个 union 成员，而不是把测试开关和
  *          参数散落到 main.c 的各个角落。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MEASUREMENT_MANAGER_H__
#define __MEASUREMENT_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 当前选中的测量模块 */
typedef enum
{
  MEASUREMENT_MODULE_NONE = 0,
  MEASUREMENT_MODULE_MOS_DRIVER
} measurement_module_t;

/* 测量运行状态 */
typedef enum
{
  MEASUREMENT_STATE_IDLE = 0,
  MEASUREMENT_STATE_RUNNING,
  MEASUREMENT_STATE_FAILED
} measurement_state_t;

/* MOS 驱动测量的被测相 */
typedef enum
{
  MOS_DRIVER_PHASE_U = 0,
  MOS_DRIVER_PHASE_V,
  MOS_DRIVER_PHASE_W
} mos_driver_phase_t;

typedef struct
{
  /* —— 配置项：可在运行途中由调试器或上位机修改 —— */
  mos_driver_phase_t phase;
  uint16_t duty;
  uint8_t moe_enable;
  uint32_t startup_delay_ms;
  /* —— 运行状态：volatile，由测量模块维护，请勿手改 —— */
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
