#ifndef FOC_CONTROL_MANAGER_H
#define FOC_CONTROL_MANAGER_H

#include <stdint.h>

#include "foc_interface.h"

/* 三种模式均编译进固件，通过 requested_mode 在运行时请求切换。 */
typedef enum
{
  FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP = 0,
  /* 真实电流闭环 + 虚拟角度；沿用项目约定名称“电流开环”。 */
  FOC_CONTROL_MODE_CURRENT_OPEN_LOOP,
  FOC_CONTROL_MODE_NORMAL,
  FOC_CONTROL_MODE_COUNT
} foc_control_mode_t;

typedef struct
{
  float d_current_reference_a;
  float q_current_reference_a;
  float electrical_angle_rad;
  uint8_t angle_valid;
  uint8_t reference_valid;
} foc_normal_loop_t;

typedef struct
{
  volatile uint8_t running;
  volatile uint8_t output_request;
  volatile uint8_t output_armed;
  volatile uint8_t precharge_in_progress;
  volatile uint8_t power_stage_safe;
  volatile uint8_t output_enabled;
  volatile foc_fault_flags_t fault_flags;
  volatile uint32_t power_stage_pin_levels;
  volatile uint32_t first_unsafe_pin_levels;
  volatile uint32_t unsafe_sample_count;
  volatile uint32_t ready_low_count;
  volatile uint32_t nfault_low_count;
  volatile uint8_t fault_status_capture_pending;
  volatile uint8_t fault_status_valid;
  volatile uint8_t first_fault_status_raw;
} foc_control_runtime_t;

/* 唯一的控制模式管理对象：请求/当前模式、三种模式数据和公共运行状态
   全部集中管理。切换请求只能写 requested_mode，active_mode 由服务函数更新。 */
typedef struct
{
  volatile foc_control_mode_t requested_mode;
  volatile foc_control_mode_t active_mode;
  volatile uint32_t switch_count;
  foc_voltage_open_loop_t voltage_open_loop;
  foc_current_test_t current_open_loop;
  foc_normal_loop_t normal_loop;
  foc_control_runtime_t runtime;
} foc_control_manager_t;

#endif /* FOC_CONTROL_MANAGER_H */
