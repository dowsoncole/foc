#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <stdint.h>

#include "foc_interface.h"

typedef enum
{
  SERVO_STATE_INIT = 0,
  SERVO_STATE_CALIBRATING,
  SERVO_STATE_READY,
  SERVO_STATE_STARTING,
  SERVO_STATE_RUNNING,
  SERVO_STATE_FAULT
} servo_state_t;

typedef uint32_t servo_fault_mask_t;

#define SERVO_FAULT_NONE               0UL
#define SERVO_FAULT_CURRENT_INVALID    (1UL << 0)
#define SERVO_FAULT_ANGLE_INVALID      (1UL << 1)
#define SERVO_FAULT_REFERENCE_INVALID  (1UL << 2)
#define SERVO_FAULT_POWER_STAGE        (1UL << 3)
#define SERVO_FAULT_FOC_CONFIG         (1UL << 4)
#define SERVO_FAULT_CALIBRATION        (1UL << 5)
#define SERVO_FAULT_OUTPUT_DISABLED    (1UL << 6)
#define SERVO_FAULT_FOC_ALGORITHM      (1UL << 7)
#define SERVO_FAULT_PWM_WRITE          (1UL << 8)

typedef struct
{
  uint8_t (*is_calibration_done)(void *context);
  uint8_t (*is_calibration_error)(void *context);
  uint8_t (*is_power_stage_safe)(void *context);
  uint8_t (*is_output_enabled)(void *context);
  void (*prepare_output)(void *context);
  void (*enable_output)(void *context);
  void (*disable_output)(void *context);
  /* 后台上报接口：可绑定 CAN/UART/日志，不会在 ISR 中调用。 */
  void (*notify_fault)(void *context,
                       servo_fault_mask_t active_faults,
                       servo_fault_mask_t latched_faults);
  void *context;
} servo_manager_port_t;

typedef struct
{
  foc_controller_t *foc;
  servo_manager_port_t port;
  volatile uint8_t servo_on_request;
  volatile uint8_t fault_reset_request;
  volatile servo_fault_mask_t pending_faults;
  servo_state_t state;
  servo_fault_mask_t active_faults;
  servo_fault_mask_t latched_faults;
  uint32_t fault_sequence;
  uint32_t reported_fault_sequence;
} servo_manager_t;

void ServoManager_Init(servo_manager_t *manager,
                       foc_controller_t *foc,
                       const servo_manager_port_t *port);
void ServoManager_RequestOn(servo_manager_t *manager);
void ServoManager_RequestOff(servo_manager_t *manager);
void ServoManager_RequestFaultReset(servo_manager_t *manager);

/* 调用者必须先完成本地快速关断；本函数只发布故障事件。 */
void ServoManager_RaiseFaultFromIsr(servo_manager_t *manager,
                                    servo_fault_mask_t fault);

/* 由主循环/应用任务周期调用，禁止在实时 ISR 中调用。 */
void ServoManager_Service(servo_manager_t *manager);

servo_state_t ServoManager_GetState(const servo_manager_t *manager);
servo_fault_mask_t ServoManager_GetActiveFaults(const servo_manager_t *manager);
servo_fault_mask_t ServoManager_GetLatchedFaults(const servo_manager_t *manager);

#endif
