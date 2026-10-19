/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drv_monitor.c
  * @brief   以 1 s 后台任务形式运行的 STSPIN32G4 监管逻辑（L3 监控服务层）。
  *
  *          所有会碰 I2C 总线的代码都住在这里，也就是主循环层。
  *          TIM6 更新中断只置 g_drv_monitor_tick 标志，不做任何传输，
  *          因此阻塞式的 HAL_I2C 调用永远不可能在中断上下文里运行。
  ******************************************************************************
  */
/* USER CODE END Header */

#include <string.h>

#include "drv_monitor.h"
#include "stspin32g4.h"
#include "tim6_heartbeat.h"
#include "tim1_pwm.h"

/* --------------------------------------------------------------------------*/
/* 可调参数                                                                  */
/* --------------------------------------------------------------------------*/
#define DRV_I2C_TIMEOUT_MS     10U   /* 400kHz 下一次 1 字节传输约 50 µs */
#define DRV_WAKE_TIMEOUT_MS    50U   /* 拉高 WAKE 后 READY 释放的时间 */
#define DRV_COMM_ERR_LIMIT     3U    /* 连续失败多少次判定为 COMM_LOST */
#define DRV_INIT_RETRY         3U    /* 上电清故障的尝试次数 */
#define DRV_INIT_RETRY_GAP_MS  10U

/* --------------------------------------------------------------------------*/
/* 模块状态                                                                  */
/* --------------------------------------------------------------------------*/
/* 由 TIM6 中断置位，由 DrvMonitor_Task() 清零。声明为 volatile：
   在中断上下文里写，在线程上下文里读。 */
volatile uint8_t g_drv_monitor_tick = 0U;

drv_monitor_t                 s_mon;
static drv_monitor_t          s_work;
static volatile drv_request_t s_request = DRV_REQ_NONE;
static volatile uint8_t       s_standby_reg_enable = 0U;

/* --------------------------------------------------------------------------*/
/* 内部辅助函数                                                              */
/* --------------------------------------------------------------------------*/

/**
  * @brief  锁存 STATUS 寄存器报告的保护状态。
  *
  * VDS_P 和 RESET 在硬件里就是锁存的，需要显式写 CLEAR 才能释放。
  * THSD 和 VCC_UVLO 在成因消失后会自行恢复，但这里同样把它们锁存，
  * 这样 1 s 一次的轮询就不会漏掉一个更短的事件。
  */
static void DrvMonitor_LatchFaults(void)
{
  if (s_work.status.bit.vds_p != 0U)
  {
    s_work.fault.vds_p = 1U;
  }

  if (s_work.status.bit.thsd != 0U)
  {
    s_work.fault.thsd = 1U;
  }

  if (s_work.status.bit.vcc_uvlo != 0U)
  {
    s_work.fault.vcc_uvlo = 1U;
  }

  s_work.fault.active = (uint8_t)((s_work.fault.vds_p != 0U) ||
                                  (s_work.fault.thsd != 0U) ||
                                  (s_work.fault.vcc_uvlo != 0U));

  if ((s_work.fault.active != 0U) && (s_work.state != DRV_STATE_FAULT))
  {
    s_work.last_fault_s = s_work.uptime_s;
  }
}

/**
  * @brief  由锁存故障和引脚电平推导出监管状态。
  */
static void DrvMonitor_Evaluate(void)
{
  if (s_work.comm_errors >= DRV_COMM_ERR_LIMIT)
  {
    s_work.state = DRV_STATE_COMM_LOST;
  }
  else if (s_work.fault.active != 0U)
  {
    s_work.state = DRV_STATE_FAULT;
  }
  else if ((s_work.state != DRV_STATE_STANDBY) &&
           ((s_work.nFAULT == 0U) || (s_work.READY == 0U)))
  {
    /* READY 只在驱动可用时才释放为高；nFAULT 是拉低表示置位。
       这两个字段存的都是引脚原始电平。 */
    s_work.state = DRV_STATE_FAULT;
  }
  else if (s_work.state != DRV_STATE_STANDBY)
  {
    s_work.state = DRV_STATE_READY;
  }
  else
  {
    /* 待机状态下 I2C 接口按设计就是不可用的，所以 READY 电平缺失属于预期，
       绝不能当成故障处理。 */
  }
}

/**
  * @brief  读一次 STATUS，并完成故障记账。
  * @retval 传输成功返回 HAL_OK
  */
static HAL_StatusTypeDef DrvMonitor_PollStatus(void)
{
  HAL_StatusTypeDef ret;

  ret = STSPIN32G4_ReadRegister(STSPIN32G4_REG_STATUS, &s_work.status.raw,
                               DRV_I2C_TIMEOUT_MS);

  if (ret == HAL_OK)
  {
    s_work.comm_errors = 0U;
    s_work.fault.comm  = 0U;
    DrvMonitor_LatchFaults();
  }
  else
  {
    s_work.comm_errors++;
    s_work.fault.comm = 1U;

    if (s_work.comm_errors >= DRV_COMM_ERR_LIMIT)
    {
      s_work.last_fault_s = s_work.uptime_s;
    }
  }

  return ret;
}

/**
  * @brief  处理上层投递的请求。
  *
  * 每次任务运行只消费一个请求，这样单个 1 s 时间片的耗时有上界。
  */
static void DrvMonitor_ProcessRequest(void)
{
  drv_request_t request = s_request;

  s_request = DRV_REQ_NONE;

  switch (request)
  {
    case DRV_REQ_CLEAR_FAULT:
      /* 把 CLEAR 的所有位都写 1 可释放已锁存的保护。仍然存在的成因
         （例如 THSD）会在下一次轮询时被重新锁存。 */
      if (STSPIN32G4_ClearFaults(DRV_I2C_TIMEOUT_MS) == HAL_OK)
      {
        s_work.fault.vds_p    = 0U;
        s_work.fault.thsd     = 0U;
        s_work.fault.vcc_uvlo = 0U;
        s_work.fault.active   = 0U;
        s_work.comm_errors    = 0U;
      }
      break;

    case DRV_REQ_WAKE:
      /* 拉高 WAKE，等待 READY，然后清除上电锁存故障。 */
      if (STSPIN32G4_Wake(DRV_WAKE_TIMEOUT_MS) == HAL_OK)
      {
        s_work.comm_errors = 0U;
        s_work.fault.comm  = 0U;
      }
      else
      {
        s_work.comm_errors = DRV_COMM_ERR_LIMIT;
        s_work.fault.comm  = 1U;
      }
      break;

    case DRV_REQ_STANDBY:
      /* 保持 MCU 供电；只有在 MCU 即将进入 Stop 模式、且其电流低于稳压器
         上限时，才使能驱动侧的待机稳压器。 */
      if (STSPIN32G4_EnterStandby(s_standby_reg_enable,
                                  DRV_I2C_TIMEOUT_MS) == HAL_OK)
      {
        s_work.state       = DRV_STATE_STANDBY;
        s_work.comm_errors = 0U;
        s_work.fault.comm  = 0U;
      }
      break;

    case DRV_REQ_NONE:
    default:
      break;
  }
}

/* --------------------------------------------------------------------------*/
/* 中断层                                                                    */
/* --------------------------------------------------------------------------*/

/**
  * @brief  定时器更新回调，由 TIM6_DAC_IRQHandler 调用。
  *
  * 刻意做到最小：置心跳标志然后返回。不访问 I2C，不推进状态机，不用
  * HAL_Delay。执行时间只有几个周期，所以这个中断不可能干扰电流环或
  * PWM 时序。
  *
  * @note   HAL_TIM_PeriodElapsedCallback 被 TIM6 与 TIM1_UP 两个向量共用，
  *         因此它的唯一定义就在本文件，内部按 htim->Instance 过滤。
  *         新增模块若需要同名回调，不要重复定义（会链接冲突），
  *         应挂到这里的分发逻辑中，或改用其他回调名。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    g_drv_monitor_tick = 1U;
  }
  else if (htim->Instance == TIM1)
  {
    Tim1Pwm_DispatchPeriodElapsed();
  }
}

/* --------------------------------------------------------------------------*/
/* 公开 API                                                                  */
/* --------------------------------------------------------------------------*/

/**
  * @brief  启动监管并清除上电锁存故障。
  *
  * 由 main() 在任何 PWM 输出使能之前调用一次。此处允许阻塞重试，因为还没有
  * 实时任务在运行：VCC buck 仍在软启动，第一次尝试失败是合理的。
  *
  * 调用者必须保持功率级关闭，除非 DrvMonitor_GetState() 返回 DRV_STATE_READY。
  */
HAL_StatusTypeDef DrvMonitor_Init(void)
{
  uint8_t  attempt;
  uint32_t tick_start;

  (void)memset(&s_work, 0, sizeof(s_work));
  (void)memset(&s_mon, 0, sizeof(s_mon));
  s_work.state          = DRV_STATE_INIT;
  s_request             = DRV_REQ_NONE;
  s_standby_reg_enable  = 0U;

  for (attempt = 0U; attempt < DRV_INIT_RETRY; attempt++)
  {
    if (STSPIN32G4_Init(DRV_I2C_TIMEOUT_MS) == HAL_OK)
    {
      s_work.comm_errors = 0U;
      s_work.state       = DRV_STATE_READY;
      break;
    }

    /* 把重试拉开间隔，但不依赖阻塞式的 HAL_Delay */
    tick_start = HAL_GetTick();
    while ((HAL_GetTick() - tick_start) < DRV_INIT_RETRY_GAP_MS)
    {
    }
  }

  if (s_work.state != DRV_STATE_READY)
  {
    s_work.comm_errors = DRV_COMM_ERR_LIMIT;
    s_work.fault.comm  = 1U;
    s_work.state       = DRV_STATE_COMM_LOST;
  }

  /* 发布 Init 期间采集到的引脚电平与寄存器内容 */
  s_work.READY  = STSPIN32G4_IsReady();
  s_work.nFAULT = STSPIN32G4_IsFault();

  if ((s_work.state == DRV_STATE_READY) &&
      ((s_work.READY == 0U) || (s_work.nFAULT == 0U)))
  {
    s_work.state = DRV_STATE_FAULT;
  }

  s_mon = s_work;

  /* 启动 1 s 心跳 */
  g_drv_monitor_tick = 0U;
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  后台任务，由主循环调用。
  *
  * TIM6 心跳未到时立即返回，所以每轮循环都调用是安全的。最坏情况占用时间是
  * 一次 I2C 传输（约 50 µs）加状态机推进，远低于 1 s 周期。
  */
void DrvMonitor_Task(void)
{
  if (g_drv_monitor_tick == 0U)
  {
    return;
  }

  g_drv_monitor_tick = 0U;
  s_work.uptime_s++;

  /* 引脚电平可以随便采样，而且即使 I2C 挂了它们依然有效 */
  s_work.READY  = STSPIN32G4_IsReady();
  s_work.nFAULT = STSPIN32G4_IsFault();

  DrvMonitor_ProcessRequest();

  /* 待机状态下 I2C 接口已被关闭，继续轮询只会产生虚假的通信错误。 */
  if (s_work.state != DRV_STATE_STANDBY)
  {
    (void)DrvMonitor_PollStatus();
  }

  DrvMonitor_Evaluate();

  {
    /* 在短临界区内把工作副本整体发布出去，保证任何观察者都不会看到
       写了一半的 s_mon。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_mon = s_work;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
}

/**
  * @brief  投递一个请求，供下一次任务运行处理。任何上下文调用都安全。
  */
void DrvMonitor_PostRequest(drv_request_t request)
{
  s_request = request;
}

void DrvMonitor_RequestStandby(uint8_t stby_reg_en)
{
  s_standby_reg_enable = (stby_reg_en != 0U) ? 1U : 0U;
  DrvMonitor_PostRequest(DRV_REQ_STANDBY);
}

HAL_StatusTypeDef DrvMonitor_GetSnapshot(drv_monitor_t *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = s_mon;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return HAL_OK;
}

/**
  * @brief  功率级使能判定的快速路径。
  */
drv_state_t DrvMonitor_GetState(void)
{
  return s_mon.state;
}

uint8_t DrvMonitor_IsPowerStageSafeFast(void)
{
  return STSPIN32G4_PinsAllowSwitching();
}
