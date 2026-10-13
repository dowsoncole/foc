/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drv_monitor.c
  * @brief   STSPIN32G4 supervision running as a 1 s background task.
  *
  * Everything that touches the I2C bus lives here, in the main loop layer.
  * The TIM6 update ISR only sets g_drv_monitor_tick; it performs no transfer,
  * so no blocking HAL_I2C call can ever run in interrupt context.
  ******************************************************************************
  */
/* USER CODE END Header */

#include <string.h>

#include "drv_monitor.h"
#include "stspin32g4.h"
#include "tim.h"

/* --------------------------------------------------------------------------*/
/* Tunables                                                                  */
/* --------------------------------------------------------------------------*/
#define DRV_I2C_TIMEOUT_MS     10U   /* one 1-byte transfer is ~50 us @400kHz */
#define DRV_WAKE_TIMEOUT_MS    50U   /* READY release after WAKE asserted     */
#define DRV_COMM_ERR_LIMIT     3U    /* consecutive failures before COMM_LOST */
#define DRV_INIT_RETRY         3U    /* power-up fault clear attempts         */
#define DRV_INIT_RETRY_GAP_MS  10U

/* --------------------------------------------------------------------------*/
/* Module state                                                              */
/* --------------------------------------------------------------------------*/
/* Set by the TIM6 ISR, cleared by DrvMonitor_Task(). volatile: written in
   interrupt context, read in thread context. */
volatile uint8_t g_drv_monitor_tick = 0U;

drv_monitor_t               s_mon;
static volatile drv_request_t s_request = DRV_REQ_NONE;

/* --------------------------------------------------------------------------*/
/* Internal helpers                                                          */
/* --------------------------------------------------------------------------*/

/**
  * @brief  Latch the protection conditions reported by the STATUS register.
  *
  * VDS_P and RESET are latched in hardware and need an explicit CLEAR write.
  * THSD and VCC_UVLO recover on their own once the cause is removed, but they
  * are latched here too so a 1 s poll cannot miss a shorter event.
  */
static void DrvMonitor_LatchFaults(void)
{
  if (s_mon.status.bit.vds_p != 0U)
  {
    s_mon.fault.vds_p = 1U;
  }

  if (s_mon.status.bit.thsd != 0U)
  {
    s_mon.fault.thsd = 1U;
  }

  if (s_mon.status.bit.vcc_uvlo != 0U)
  {
    s_mon.fault.vcc_uvlo = 1U;
  }

  s_mon.fault.active = (uint8_t)((s_mon.fault.vds_p != 0U) ||
                                 (s_mon.fault.thsd != 0U) ||
                                 (s_mon.fault.vcc_uvlo != 0U));

  if ((s_mon.fault.active != 0U) && (s_mon.state != DRV_STATE_FAULT))
  {
    s_mon.last_fault_s = s_mon.uptime_s;
  }
}

/**
  * @brief  Derive the supervision state from latched faults and pin levels.
  */
static void DrvMonitor_Evaluate(void)
{
  if (s_mon.comm_errors >= DRV_COMM_ERR_LIMIT)
  {
    s_mon.state = DRV_STATE_COMM_LOST;
  }
  else if (s_mon.fault.active != 0U)
  {
    s_mon.state = DRV_STATE_FAULT;
  }
  else if ((s_mon.state != DRV_STATE_STANDBY) &&
           ((s_mon.nFAULT == 0U) || (s_mon.READY == 0U)))
  {
    /* READY is released high only when the driver is usable; nFAULT is
       asserted low. Both fields store the raw pin levels. */
    s_mon.state = DRV_STATE_FAULT;
  }
  else if (s_mon.state != DRV_STATE_STANDBY)
  {
    s_mon.state = DRV_STATE_READY;
  }
  else
  {
    /* In standby the I2C interface is unavailable by design, so a missing
       READY level is expected and must not be treated as a fault. */
  }
}

/**
  * @brief  One STATUS read plus fault bookkeeping.
  * @retval HAL_OK on a successful transfer
  */
static HAL_StatusTypeDef DrvMonitor_PollStatus(void)
{
  HAL_StatusTypeDef ret;

  ret = STSPIN32G4_ReadRegister(STSPIN32G4_REG_STATUS, &s_mon.status.raw,
                               DRV_I2C_TIMEOUT_MS);

  if (ret == HAL_OK)
  {
    s_mon.comm_errors = 0U;
    s_mon.fault.comm  = 0U;
    DrvMonitor_LatchFaults();
  }
  else
  {
    s_mon.comm_errors++;
    s_mon.fault.comm = 1U;

    if (s_mon.comm_errors >= DRV_COMM_ERR_LIMIT)
    {
      s_mon.last_fault_s = s_mon.uptime_s;
    }
  }

  return ret;
}

/**
  * @brief  Service a request posted by the upper layer.
  *
  * Requests are consumed one per task run so a single 1 s slot stays bounded.
  */
static void DrvMonitor_ProcessRequest(void)
{
  drv_request_t request = s_request;

  s_request = DRV_REQ_NONE;

  switch (request)
  {
    case DRV_REQ_CLEAR_FAULT:
      /* Writing all bits of CLEAR releases the latched protections. A cause
         that is still present (e.g. THSD) is re-latched by the next poll. */
      if (STSPIN32G4_ClearFaults(DRV_I2C_TIMEOUT_MS) == HAL_OK)
      {
        s_mon.fault.vds_p    = 0U;
        s_mon.fault.thsd     = 0U;
        s_mon.fault.vcc_uvlo = 0U;
        s_mon.fault.active   = 0U;
        s_mon.comm_errors    = 0U;
      }
      break;

    case DRV_REQ_WAKE:
      /* Asserts WAKE, waits for READY, then clears the power-up fault. */
      if (STSPIN32G4_Wake(DRV_WAKE_TIMEOUT_MS) == HAL_OK)
      {
        s_mon.comm_errors = 0U;
        s_mon.fault.comm  = 0U;
      }
      else
      {
        s_mon.comm_errors = DRV_COMM_ERR_LIMIT;
        s_mon.fault.comm  = 1U;
      }
      break;

    case DRV_REQ_STANDBY:
      /* Keep the MCU powered; enable the driver standby regulator only when
         the MCU is going to enter Stop mode and its current is below the
         regulator limit. */
      if (STSPIN32G4_EnterStandby(0U, DRV_I2C_TIMEOUT_MS) == HAL_OK)
      {
        s_mon.state       = DRV_STATE_STANDBY;
        s_mon.comm_errors = 0U;
        s_mon.fault.comm  = 0U;
      }
      break;

    case DRV_REQ_NONE:
    default:
      break;
  }
}

/* --------------------------------------------------------------------------*/
/* Interrupt layer                                                           */
/* --------------------------------------------------------------------------*/

/**
  * @brief  TIM update callback, invoked from TIM6_DAC_IRQHandler.
  *
  * Deliberately minimal: raise the tick flag and return. No I2C access, no
  * state machine work, no HAL_Delay. Execution time is a few cycles, so this
  * ISR cannot disturb the current-loop or PWM timing.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    g_drv_monitor_tick = 1U;
  }
}

/* --------------------------------------------------------------------------*/
/* Public API                                                                */
/* --------------------------------------------------------------------------*/

/**
  * @brief  Start supervision and clear the power-up latched fault.
  *
  * Called once from main() before any PWM output is enabled. Blocking retries
  * are acceptable here because no real-time task is running yet: the VCC buck
  * is still in soft-start, so the first attempt may legitimately fail.
  *
  * The caller must keep the power stage disabled unless DrvMonitor_GetState()
  * returns DRV_STATE_READY.
  */
void DrvMonitor_Init(void)
{
  uint8_t  attempt;
  uint32_t tick_start;

  (void)memset(&s_mon, 0, sizeof(s_mon));
  s_mon.state  = DRV_STATE_INIT;
  s_request    = DRV_REQ_NONE;

  for (attempt = 0U; attempt < DRV_INIT_RETRY; attempt++)
  {
    if (STSPIN32G4_Init(DRV_I2C_TIMEOUT_MS) == HAL_OK)
    {
      s_mon.comm_errors = 0U;
      s_mon.state       = DRV_STATE_READY;
      break;
    }

    /* Space the retries out without a blocking HAL_Delay dependency */
    tick_start = HAL_GetTick();
    while ((HAL_GetTick() - tick_start) < DRV_INIT_RETRY_GAP_MS)
    {
    }
  }

  if (s_mon.state != DRV_STATE_READY)
  {
    s_mon.comm_errors = DRV_COMM_ERR_LIMIT;
    s_mon.fault.comm  = 1U;
    s_mon.state       = DRV_STATE_COMM_LOST;
  }

  /* Publish the pin levels and register content captured during Init */
  s_mon.READY  = STSPIN32G4_IsReady();
  s_mon.nFAULT = STSPIN32G4_IsFault();

  if ((s_mon.state == DRV_STATE_READY) &&
      ((s_mon.READY == 0U) || (s_mon.nFAULT == 0U)))
  {
    s_mon.state = DRV_STATE_FAULT;
  }

  /* Start the 1 s heartbeat */
  g_drv_monitor_tick = 0U;
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Background task, called from the main loop.
  *
  * Returns immediately unless the TIM6 heartbeat has expired, so it is safe to
  * call on every loop iteration. Worst-case occupancy is one I2C transfer
  * (~50 us) plus the state machine, far below the 1 s period.
  */
void DrvMonitor_Task(void)
{
  if (g_drv_monitor_tick == 0U)
  {
    return;
  }

  g_drv_monitor_tick = 0U;
  s_mon.uptime_s++;

  /* Pin levels are free to sample and stay valid even when I2C is down */
  s_mon.READY  = STSPIN32G4_IsReady();
  s_mon.nFAULT = STSPIN32G4_IsFault();

  DrvMonitor_ProcessRequest();

  /* In standby the I2C interface is switched off, so polling would only
     generate spurious communication errors. */
  if (s_mon.state != DRV_STATE_STANDBY)
  {
    (void)DrvMonitor_PollStatus();
  }

  DrvMonitor_Evaluate();
}

/**
  * @brief  Post a request for the next task run. Safe from any context.
  */
void DrvMonitor_PostRequest(drv_request_t request)
{
  s_request = request;
}

/**
  * @brief  Read-only access to the published snapshot.
  *
  * The struct is written only by DrvMonitor_Task() in the main loop. A caller
  * in interrupt context should copy what it needs rather than hold the pointer
  * across a task run.
  */
const volatile drv_monitor_t *DrvMonitor_Get(void)
{
  return &s_mon;
}

/**
  * @brief  Fast path for the power stage enable decision.
  */
drv_state_t DrvMonitor_GetState(void)
{
  return s_mon.state;
}
