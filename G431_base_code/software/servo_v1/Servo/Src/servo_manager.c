#include "servo_manager.h"

#include <string.h>

static uint8_t ServoManager_PortReady(const servo_manager_t *manager)
{
  return ((manager->port.is_calibration_done != 0) &&
          (manager->port.is_calibration_error != 0) &&
          (manager->port.is_power_stage_safe != 0) &&
          (manager->port.is_output_enabled != 0) &&
          (manager->port.prepare_output != 0) &&
          (manager->port.enable_output != 0) &&
          (manager->port.disable_output != 0)) ? 1U : 0U;
}

static servo_fault_mask_t ServoManager_MapFocFaults(foc_fault_flags_t faults)
{
  servo_fault_mask_t mapped = SERVO_FAULT_NONE;

  if ((faults & FOC_FAULT_CURRENT_INVALID) != 0U)
    mapped |= SERVO_FAULT_CURRENT_INVALID;
  if ((faults & FOC_FAULT_ANGLE_INVALID) != 0U)
    mapped |= SERVO_FAULT_ANGLE_INVALID;
  if ((faults & FOC_FAULT_REFERENCE_INVALID) != 0U)
    mapped |= SERVO_FAULT_REFERENCE_INVALID;
  if ((faults & FOC_FAULT_POWER_STAGE) != 0U)
    mapped |= SERVO_FAULT_POWER_STAGE;
  if ((faults & FOC_FAULT_PORT_CONFIG) != 0U)
    mapped |= SERVO_FAULT_FOC_CONFIG;
  if ((faults & FOC_FAULT_ALGORITHM) != 0U)
    mapped |= SERVO_FAULT_FOC_ALGORITHM;
  if ((faults & FOC_FAULT_PWM_WRITE) != 0U)
    mapped |= SERVO_FAULT_PWM_WRITE;

  return mapped;
}

static void ServoManager_StopOutput(servo_manager_t *manager)
{
  if (manager->port.disable_output != 0)
  {
    manager->port.disable_output(manager->port.context);
  }
  if (manager->foc != 0)
  {
    FOC_ControllerStop(manager->foc);
  }
}

static void ServoManager_EnterFault(servo_manager_t *manager,
                                    servo_fault_mask_t faults)
{
  servo_fault_mask_t new_faults = faults & ~manager->latched_faults;

  ServoManager_StopOutput(manager);
  manager->active_faults |= faults;
  manager->latched_faults |= faults;
  manager->state = SERVO_STATE_FAULT;
  if (new_faults != SERVO_FAULT_NONE)
  {
    manager->fault_sequence++;
  }
}

static void ServoManager_NotifyFaultChange(servo_manager_t *manager)
{
  if ((manager->fault_sequence != manager->reported_fault_sequence) &&
      (manager->port.notify_fault != 0))
  {
    manager->port.notify_fault(manager->port.context,
                               manager->active_faults,
                               manager->latched_faults);
    manager->reported_fault_sequence = manager->fault_sequence;
  }
}

void ServoManager_Init(servo_manager_t *manager,
                       foc_controller_t *foc,
                       const servo_manager_port_t *port)
{
  if ((manager == 0) || (foc == 0) || (port == 0))
  {
    return;
  }

  (void)memset(manager, 0, sizeof(*manager));
  manager->foc = foc;
  manager->port = *port;
  manager->state = SERVO_STATE_INIT;

  if (manager->port.disable_output != 0)
  {
    manager->port.disable_output(manager->port.context);
  }
}

void ServoManager_RequestOn(servo_manager_t *manager)
{
  if (manager != 0)
  {
    manager->servo_on_request = 1U;
  }
}

void ServoManager_RequestOff(servo_manager_t *manager)
{
  if (manager != 0)
  {
    manager->servo_on_request = 0U;
  }
}

void ServoManager_RequestFaultReset(servo_manager_t *manager)
{
  if (manager != 0)
  {
    manager->fault_reset_request = 1U;
  }
}

void ServoManager_RaiseFaultFromIsr(servo_manager_t *manager,
                                    servo_fault_mask_t fault)
{
  if (manager != 0)
  {
    manager->pending_faults |= fault;
  }
}

void ServoManager_Service(servo_manager_t *manager)
{
  servo_fault_mask_t detected_faults;
  uint8_t calibration_done;
  uint8_t calibration_error;
  uint8_t power_stage_safe;

  if ((manager == 0) || (manager->foc == 0))
  {
    return;
  }

  if (ServoManager_PortReady(manager) == 0U)
  {
    ServoManager_EnterFault(manager, SERVO_FAULT_FOC_CONFIG);
    ServoManager_NotifyFaultChange(manager);
    return;
  }

  calibration_done = manager->port.is_calibration_done(manager->port.context);
  calibration_error = manager->port.is_calibration_error(manager->port.context);
  power_stage_safe = manager->port.is_power_stage_safe(manager->port.context);

  detected_faults = manager->pending_faults;
  manager->pending_faults = SERVO_FAULT_NONE;
  detected_faults |= ServoManager_MapFocFaults(
      FOC_ControllerGetFaults(manager->foc));
  if (calibration_error != 0U)
  {
    detected_faults |= SERVO_FAULT_CALIBRATION;
  }
  if ((manager->servo_on_request != 0U) &&
      (power_stage_safe == 0U))
  {
    detected_faults |= SERVO_FAULT_POWER_STAGE;
  }

  if (detected_faults != SERVO_FAULT_NONE)
  {
    ServoManager_EnterFault(manager, detected_faults);
  }

  switch (manager->state)
  {
    case SERVO_STATE_INIT:
      ServoManager_StopOutput(manager);
      manager->state = SERVO_STATE_CALIBRATING;
      break;

    case SERVO_STATE_CALIBRATING:
      if (calibration_done != 0U)
      {
        manager->state = SERVO_STATE_READY;
      }
      break;

    case SERVO_STATE_READY:
      if ((manager->servo_on_request != 0U) &&
          (calibration_done != 0U) &&
          (power_stage_safe != 0U) &&
          (manager->latched_faults == SERVO_FAULT_NONE))
      {
        manager->port.prepare_output(manager->port.context);
        FOC_ControllerStart(manager->foc);
        manager->state = SERVO_STATE_STARTING;
      }
      break;

    case SERVO_STATE_STARTING:
      if (manager->servo_on_request == 0U)
      {
        ServoManager_StopOutput(manager);
        manager->state = SERVO_STATE_READY;
      }
      else if ((manager->foc->feedback_ready != 0U) &&
               (manager->foc->running != 0U) &&
               (power_stage_safe != 0U))
      {
        manager->port.enable_output(manager->port.context);
        if (manager->port.is_output_enabled(manager->port.context) != 0U)
        {
          manager->state = SERVO_STATE_RUNNING;
        }
      }
      break;

    case SERVO_STATE_RUNNING:
      if (manager->servo_on_request == 0U)
      {
        ServoManager_StopOutput(manager);
        manager->state = SERVO_STATE_READY;
      }
      else if (manager->port.is_output_enabled(manager->port.context) == 0U)
      {
        ServoManager_EnterFault(manager, SERVO_FAULT_OUTPUT_DISABLED);
      }
      break;

    case SERVO_STATE_FAULT:
      ServoManager_StopOutput(manager);
      if (manager->fault_reset_request != 0U)
      {
        manager->fault_reset_request = 0U;
        /* 故障复位要求先 Servo OFF，且所有恢复条件已满足。 */
        if ((manager->servo_on_request == 0U) &&
            (calibration_done != 0U) &&
            (calibration_error == 0U) &&
            (power_stage_safe != 0U))
        {
          FOC_ControllerClearFaults(manager->foc);
          manager->active_faults = SERVO_FAULT_NONE;
          manager->latched_faults = SERVO_FAULT_NONE;
          manager->state = SERVO_STATE_READY;
          /* 让通信层收到一次 active=0/latched=0 的清除事件。 */
          manager->fault_sequence++;
        }
      }
      break;

    default:
      ServoManager_EnterFault(manager, SERVO_FAULT_FOC_CONFIG);
      break;
  }

  ServoManager_NotifyFaultChange(manager);
}

servo_state_t ServoManager_GetState(const servo_manager_t *manager)
{
  return (manager != 0) ? manager->state : SERVO_STATE_FAULT;
}

servo_fault_mask_t ServoManager_GetActiveFaults(const servo_manager_t *manager)
{
  return (manager != 0) ? manager->active_faults : SERVO_FAULT_FOC_CONFIG;
}

servo_fault_mask_t ServoManager_GetLatchedFaults(const servo_manager_t *manager)
{
  return (manager != 0) ? manager->latched_faults : SERVO_FAULT_FOC_CONFIG;
}
