/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    drv_monitor.h
  * @brief   STSPIN32G4 栅极驱动监管：状态快照、锁存故障表与后台状态机
  *          （L3 监控服务层，纯业务线）。
  *
  *          职责边界：回答一个问题——「功率级现在能不能开？」
  *          本层是全工程唯一允许发起 I2C 通信的模块，把芯片状态收敛成一个
  *          可安全跨上下文读取的快照。本层自身不初始化任何外设，
  *          只使用 L1 交出的 htim6 句柄。
  *
  *          分层契约
  *            - TIM6 更新中断    ：只置 1 s 心跳标志，绝不访问 I2C
  *            - DrvMonitor_Task()：在主循环运行，独占全部 I2C 通信
  *            - 电流环 / 速度环  ：绝不调用本模块，只通过
  *                                 DrvMonitor_GetSnapshot() 读取无撕裂副本
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* --------------------------------------------------------------------------*/
/* 状态寄存器 (0x80)，与 DS13630 Table 25 逐位对应                            */
/* --------------------------------------------------------------------------*/
typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t vcc_uvlo : 1;   /* bit 0: VCC 低于欠压阈值                   */
    uint8_t thsd     : 1;   /* bit 1: 器件处于热关断                     */
    uint8_t vds_p    : 1;   /* bit 2: VDS 保护已触发                     */
    uint8_t reset    : 1;   /* bit 3: 寄存器已复位为默认值               */
    uint8_t rsvd     : 3;   /* bits 4..6                                 */
    uint8_t lock     : 1;   /* bit 7: 受保护寄存器已上锁                 */
  } bit;
} drv_status_t;

/* 硬件状态标志，每个保护源一位。latched（锁存）用于记住一个已经消失的状态，
   这样故障原因不会在两次轮询之间丢失。 */
typedef struct
{
  uint8_t active   : 1;     /* 当前有任一保护处于置位状态                */
  uint8_t vds_p    : 1;     /* 锁存：VDS 保护（过流）                    */
  uint8_t thsd     : 1;     /* 锁存：热关断                              */
  uint8_t vcc_uvlo : 1;     /* 锁存：VCC 欠压                            */
  uint8_t comm     : 1;     /* 锁存：I2C 通信失败                        */
  uint8_t rsvd     : 3;
} drv_fault_t;

/* 监管状态机 */
typedef enum
{
  DRV_STATE_INIT = 0,       /* 上电：正在清除锁存故障                    */
  DRV_STATE_STANDBY,        /* 驱动处于休眠，等待唤醒请求                */
  DRV_STATE_READY,          /* 健康，允许使能功率级                      */
  DRV_STATE_FAULT,          /* 保护动作中，输出必须保持关闭              */
  DRV_STATE_COMM_LOST       /* I2C 不可达，原因未知                      */
} drv_state_t;

/* 上层可以投递的请求；由下一次 1 s 任务运行消费 */
typedef enum
{
  DRV_REQ_NONE = 0,
  DRV_REQ_CLEAR_FAULT,      /* 锁存保护之后重新武装                      */
  DRV_REQ_WAKE,             /* 退出待机                                  */
  DRV_REQ_STANDBY           /* 进入低功耗模式                            */
} drv_request_t;

/* 对外发布的快照。后台任务修改的是一份私有工作副本，然后在一段短临界区内
   把它发布到 s_mon，因此 s_mon 永远不会被观察到处于半更新状态。
   DrvMonitor_GetSnapshot() 在同样的保护下拷贝出去，所以从中断里调用是安全的。 */
typedef struct
{
  drv_state_t    state;
  drv_status_t   status;       /* 最近一次读到的 STATUS 寄存器原始内容    */
  drv_fault_t    fault;        /* 实时 + 锁存的保护标志                   */
  uint8_t        READY;        /* READY 引脚原始电平，高有效：
                                  1 = 器件可用                           */
  uint8_t        nFAULT;       /* nFAULT 引脚原始电平，低有效：
                                  0 = 故障置位，1 = 无故障               */
  uint8_t        comm_errors;  /* 连续 I2C 失败次数                       */
  uint32_t       uptime_s;     /* 监控启动以来的秒数                      */
  uint32_t       last_fault_s; /* 最近一次锁存故障时的 uptime             */
} drv_monitor_t;

/* 对外发布的快照。声明为非 static 纯粹是为了让它出现在调试器 Watch 窗口里。
   不要在中断里直接读它：请用 DrvMonitor_GetSnapshot() 取无撕裂副本。 */
extern drv_monitor_t s_mon;

/* --------------------------------------------------------------------------*/
/* 对上（L6 应用入口层 / 未来的电流环与协议栈）提供的接口                    */
/* --------------------------------------------------------------------------*/

/**
  * @brief  启动监管并清除上电锁存故障。
  *
  * 由 main() 在任何 PWM 输出使能之前调用一次。此处允许阻塞重试，因为还没有
  * 实时任务在运行：VCC buck 仍处于软启动阶段，第一次尝试失败是正常的。
  *
  * 调用者必须保持功率级关闭，除非 DrvMonitor_GetState() 返回 DRV_STATE_READY。
  * @retval HAL_OK；启动 TIM6 心跳失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef DrvMonitor_Init(void);

/**
  * @brief  后台任务，由主循环调用。
  *
  * TIM6 心跳未到时立即返回，因此每轮循环都调用是安全的。最坏情况占用时间是
  * 一次 I2C 传输（约 50 µs）加状态机推进，远低于 1 s 周期。
  * @note   全工程的所有 I2C 通信都发生在这里，绝不在中断里。
  */
void              DrvMonitor_Task(void);

/**
  * @brief  投递一个请求，由下一次任务运行消费。任何上下文调用都安全。
  */
void              DrvMonitor_PostRequest(drv_request_t request);

/**
  * @brief  投递一个携带 STBY_REG_EN 选择的待机请求。
  * @param  stby_reg_en 1 表示通过驱动的待机稳压器继续给内嵌 MCU 供电
  *                     （只有在 MCU 随后进入 Stop 模式时才成立，
  *                     参见 STSPIN32G4_EnterStandby），0 表示不使能。
  * @note   待机稳压器最多提供 6 mA，不足以支撑 168 MHz 运行下的 MCU。
  */
void              DrvMonitor_RequestStandby(uint8_t stby_reg_en);

/**
  * @brief  在一段短临界区内拷贝出已发布的快照。
  * @retval HAL_OK；snapshot 为 NULL 时返回 HAL_ERROR。任何上下文调用都安全。
  */
HAL_StatusTypeDef DrvMonitor_GetSnapshot(drv_monitor_t *snapshot);

/**
  * @brief  查询当前监管状态，用于「是否允许使能功率级」的判定。
  * @note   这是轻量快速路径，可在主循环与初始化阶段调用。
  */
drv_state_t       DrvMonitor_GetState(void);

/**
  * @brief  电流环用的快速安全门。只从 GPIO 读一次 READY 与 nFAULT，
  *         绝不访问 I2C，也绝不读后台快照。
  * @retval 仅当 READY=1 且 nFAULT=1 时返回 1，否则返回 0。
  * @note   设计给 20 kHz 中断使用：执行时间恒定，不会破坏采样时序。
  *         转发到 L2 的 STSPIN32G4_PinsAllowSwitching()。
  */
uint8_t           DrvMonitor_IsPowerStageSafeFast(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MONITOR_H__ */
