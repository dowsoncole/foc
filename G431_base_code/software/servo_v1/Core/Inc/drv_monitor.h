/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drv_monitor.h
  * @brief   STSPIN32G4 gate driver supervision: status snapshot, latched fault
  *          map and the background state machine.
  *
  * Layering contract
  *   - TIM6 update ISR      : raises the 1 s tick flag only, no I2C access
  *   - DrvMonitor_Task()    : runs in the main loop, owns all I2C traffic
  *   - current/speed loops  : never call into this module, they read a
  *                            tear-free copy via DrvMonitor_GetSnapshot()
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* --------------------------------------------------------------------------*/
/* Status register (0x80) mapped bit-exact to DS13630 Table 25               */
/* --------------------------------------------------------------------------*/
typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t vcc_uvlo : 1;   /* bit 0: VCC below undervoltage threshold    */
    uint8_t thsd     : 1;   /* bit 1: device in thermal shutdown          */
    uint8_t vds_p    : 1;   /* bit 2: VDS protection triggered            */
    uint8_t reset    : 1;   /* bit 3: registers reset to default          */
    uint8_t rsvd     : 3;   /* bits 4..6                                  */
    uint8_t lock     : 1;   /* bit 7: protected registers locked          */
  } bit;
} drv_status_t;

/* Hardware condition flags, one per protection source. `latched` remembers a
   condition that already went away so the reason is not lost between polls. */
typedef struct
{
  uint8_t active   : 1;     /* any protection currently asserted           */
  uint8_t vds_p    : 1;     /* latched VDS protection (overcurrent)        */
  uint8_t thsd     : 1;     /* latched thermal shutdown                    */
  uint8_t vcc_uvlo : 1;     /* latched VCC undervoltage                    */
  uint8_t comm     : 1;     /* latched I2C communication failure           */
  uint8_t rsvd     : 3;
} drv_fault_t;

/* Supervision state machine */
typedef enum
{
  DRV_STATE_INIT = 0,       /* power-up: clearing the latched fault          */
  DRV_STATE_STANDBY,        /* driver asleep, awaiting wake request          */
  DRV_STATE_READY,          /* healthy, power stage may be enabled           */
  DRV_STATE_FAULT,          /* protection active, outputs must stay off      */
  DRV_STATE_COMM_LOST       /* I2C unreachable, reason unknown               */
} drv_state_t;

/* Requests the upper layer may post; consumed by the next 1 s task run */
typedef enum
{
  DRV_REQ_NONE = 0,
  DRV_REQ_CLEAR_FAULT,      /* re-arm after a latched protection             */
  DRV_REQ_WAKE,             /* leave standby                                 */
  DRV_REQ_STANDBY           /* enter low consumption mode                    */
} drv_request_t;

/* Published snapshot. The task mutates a private working copy and publishes it
   to s_mon inside a short critical section, so s_mon is never observed in a
   half-updated state. DrvMonitor_GetSnapshot() copies out under the same
   protection and is therefore safe to call from an ISR. */
typedef struct
{
  drv_state_t    state;
  drv_status_t   status;       /* last raw STATUS register content          */
  drv_fault_t    fault;        /* live + latched protection flags           */
  uint8_t        READY;        /* raw READY pin level, active high:
                                  1 = device usable                          */
  uint8_t        nFAULT;       /* raw nFAULT pin level, active low:
                                  0 = fault asserted, 1 = no fault           */
  uint8_t        comm_errors;  /* consecutive I2C failures                  */
  uint32_t       uptime_s;     /* seconds since the monitor started         */
  uint32_t       last_fault_s; /* uptime when the last fault was latched    */
} drv_monitor_t;

/* Published snapshot, exposed (non-static) purely so it shows up in the
   debugger Watch window. Do not read it directly from an ISR: use
   DrvMonitor_GetSnapshot() for a tear-free copy. */
extern drv_monitor_t s_mon;

void              DrvMonitor_Init(void);
void              DrvMonitor_Task(void);
void              DrvMonitor_PostRequest(drv_request_t request);

/**
  * @brief  Post a standby request carrying the STBY_REG_EN choice.
  * @param  stby_reg_en 1 to keep the embedded MCU supplied through the driver
  *                     standby regulator (only valid if the MCU then enters
  *                     Stop mode - see STSPIN32G4_EnterStandby), 0 otherwise.
  */
void              DrvMonitor_RequestStandby(uint8_t stby_reg_en);

/**
  * @brief  Copy the published snapshot under a short critical section.
  * @retval HAL_OK, or HAL_ERROR if snapshot is NULL. Safe from any context.
  */
HAL_StatusTypeDef DrvMonitor_GetSnapshot(drv_monitor_t *snapshot);

drv_state_t       DrvMonitor_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MONITOR_H__ */
