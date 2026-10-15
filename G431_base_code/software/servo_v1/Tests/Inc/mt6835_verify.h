/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6835_verify.h
  * @brief   MT6835 编码器 SPI 通讯验证（L5 测试/验证层，纯业务线）。
  *
  *          验证「MCU 与 MT6835 之间的 SPI 链路是否真的通、读回的数据是否
  *          可信」，重点核对状态寄存器。判定分四级，全部在主循环非阻塞推进：
  *            1. 传输层：角度帧的 24 bit 事务能否收发成功（HAL 返回码）。
  *            2. 数据层：角度帧 CRC-8（多项式 X8+X2+X+1）是否通过，这是
  *               本测试的「链路是否通」主判据：24 bit 的 CRC 冗余足以把
  *               悬空/恒电平/错相位的 MISO 与真实数据流区分开。
  *               ⚠ 不用 USER_ID 写读回环做判据：0x001 是 EEPROM 类寄存器，
  *               实测写命令后读回不反映 RAM 影子值，回环结果无意义。
  *            3. 状态层：STATUS[2:0] 有无报警（过速 / 磁场过弱 / 欠压）。
  *
  *          ⚠ 不用「角度读数是否变化」做判据：磁铁静止时角度本就恒定，
  *          那不能作为失效证据。angle_min / angle_max 只作观测记录——
  *          手转磁铁时两者跨度会显著增大，供人工确认传感器在感受磁场。
  *
  *          本测试不写角度/配置类 EEPROM 寄存器、不触发烧录与自动设零命令，
  *          避免把芯片配置改坏。
  *
  * ---------------------------------------------------------------------------
  * 调试操作手册（Runbook）
  * ---------------------------------------------------------------------------
  *   1. 上电后在调试器里观察 g_mt6835_verify.state：
  *        IDLE     尚未运行测试
  *        RUNNING  正在采样
  *        PASSED   链路通且数据可信
  *        FAILED   某一级判定不过，看 fail_reason 与 comm_errors
  *   2. Watch 窗口观察 g_mt6835_verify.last_frame：
  *        angle_raw 应在 0 ~ 2097151 之间；crc_ok 应为 1；
  *        status.raw 正常无报警时为 0x00。
  *   3. 手转磁铁轴，angle_max - angle_min 应显著增大（> 10000 LSB ≈ 1.7°）；
  *      静止时该跨度应很小。这一步是人工观测，不参与自动判定。
  *   4. 若 FAILED 且 fail_reason = CRC：先看 MT6835_CRC_INIT 取值（手册未给
  *      初值，见 mt6835.h 注释）；再确认 PA6(MISO) 真的接到芯片 DO、SPI 模式 3
  *      与 AF5 复用正确——预热后 CRC 恒败正是链路断的特征。
  *   5. 若 fail_reason = TIMEOUT：SPI 传输从未成功完成，查 SCK/CSN 波形、
  *      时钟使能与引脚复用。
  *   6. 若 fail_reason = ALARM：status_raw 的 bit0 = 过速、bit1 = 磁场过弱
  *      （磁铁缺失/气隙过大）、bit2 = 欠压，按位排查硬件。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MT6835_VERIFY_H__
#define __MT6835_VERIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "mt6835.h"

/* --------------------------------------------------------------------------*/
/* 测试配置常量                                                              */
/* --------------------------------------------------------------------------*/

/* 测试最少采集的角度帧数：达到该帧数且各级判定通过即 PASSED。
   10.5 MHz 下单帧约 14 µs，256 帧在主循环里毫秒级跑完。 */
#define MT6835_VERIFY_MIN_SAMPLES        256U

/* 累计传输失败（含链路回环读写的 HAL 错误）多少次判定链路不通。 */
#define MT6835_VERIFY_COMM_ERR_LIMIT     8U

/* 累计 CRC 不通过多少次判定数据不可信。允许偶发 1~2 次噪声。 */
#define MT6835_VERIFY_CRC_ERR_LIMIT      4U

/* 人工观测参考：手转磁铁时 angle_max - angle_min 的预期量级（≈1.7°）。
   只写进 Runbook 供核对，不参与自动判定。 */
#define MT6835_VERIFY_MOVED_SPAN_LSB     10000UL

/* --------------------------------------------------------------------------*/
/* 导出类型                                                                  */
/* --------------------------------------------------------------------------*/

typedef enum
{
  MT6835_VERIFY_IDLE = 0,
  MT6835_VERIFY_RUNNING,
  MT6835_VERIFY_PASSED,
  MT6835_VERIFY_FAILED
} mt6835_verify_state_t;

typedef enum
{
  MT6835_VERIFY_FAIL_NONE = 0,
  MT6835_VERIFY_FAIL_TIMEOUT,    /* SPI 传输错误/超时次数超限 */
  MT6835_VERIFY_FAIL_CRC,        /* 预热后角度帧 CRC 持续不通过，链路不可信 */
  MT6835_VERIFY_FAIL_ALARM       /* STATUS[2:0] 报警置位 */
} mt6835_verify_fail_t;

typedef struct
{
  /* —— 运行状态：volatile，由测试模块维护，请勿手改 —— */
  volatile mt6835_verify_state_t state;
  volatile mt6835_verify_fail_t  fail_reason;
  volatile uint32_t              samples;       /* 已采集的有效角度帧数 */
  volatile uint32_t              comm_errors;   /* 累计传输失败次数 */
  volatile uint32_t              crc_errors;    /* 累计 CRC 不通过次数 */
  volatile uint32_t              angle_min;     /* 采样期间角度最小值（观测用） */
  volatile uint32_t              angle_max;     /* 采样期间角度最大值（观测用） */
  volatile uint8_t               status_raw;    /* 最近一帧 STATUS[2:0] */
  volatile uint8_t               crc_ok;        /* 最近一帧 CRC 是否通过 */
  mt6835_angle_t                 last_frame;    /* 最近一帧完整数据（调试器用） */
} mt6835_verify_t;

extern mt6835_verify_t g_mt6835_verify;

/* --------------------------------------------------------------------------*/
/* 导出函数                                                                  */
/* --------------------------------------------------------------------------*/

/**
  * @brief  启动通讯验证：清零全部统计量并进入 RUNNING（可重复调用以重跑）。
  * @retval HAL_OK。
  * @note   应在初始化阶段、MT6835_PowerUpWait() 之后调用一次。内部无阻塞延时。
  */
HAL_StatusTypeDef Mt6835Verify_Start(void);

/**
  * @brief  服务测试。由主循环每轮调用。
  *
  * 非阻塞：每轮采集一帧角度数据，累计三级判定。
  * 有效帧数达到 MT6835_VERIFY_MIN_SAMPLES 且各级判定通过即置 PASSED；
  * 任一级判定不过置 FAILED 并记录 fail_reason。
  *
  * @retval 运行中或已通过返回 HAL_OK；已失败返回 HAL_ERROR。
  */
HAL_StatusTypeDef Mt6835Verify_Task(void);

/**
  * @brief  查询测试是否仍在运行。
  * @retval 运行中返回 1，其余返回 0。
  */
uint8_t Mt6835Verify_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* __MT6835_VERIFY_H__ */
