/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stspin32g4.h
  * @brief   STSPIN32G4 的 I2C 寄存器接口（L2 器件驱动层，纯业务线）。
  *
  *          职责边界：只负责「如何正确读写这颗芯片的寄存器」，包括保护寄存器
  *          的 LOCK 解锁时序、命令寄存器的特殊写法，以及 READY/nFAULT 引脚
  *          电平与 STATUS 寄存器的一致性等待。
  *          不做的事：不判断驱动是否健康（那是 L3 监控层的职责）、不决定何时
  *          开关功率级（那是 L4/L6 的职责）、不配置 I2C 外设（那是 L1 的职责，
  *          本层只使用 L1 交出的 hi2c3 句柄）。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __STSPIN32G4_H__
#define __STSPIN32G4_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "i2c3_bus.h"

/* 7 位地址固定为 1000111b (0x47)；HAL 需要的是 8 位形式 */
#define STSPIN32G4_I2C_ADDRESS       (0x47U << 1)

/* 寄存器映射（DS13630 Rev 1, Tables 17-25） */
#define STSPIN32G4_REG_POWMNG        0x01U  /* 受保护 */
#define STSPIN32G4_REG_LOGIC         0x02U  /* 受保护 */
#define STSPIN32G4_REG_READY         0x07U
#define STSPIN32G4_REG_NFAULT        0x08U  /* 受保护 */
#define STSPIN32G4_REG_CLEAR         0x09U  /* 命令寄存器 */
#define STSPIN32G4_REG_STBY          0x0AU  /* 受保护 */
#define STSPIN32G4_REG_LOCK          0x0BU
#define STSPIN32G4_REG_RESET         0x0CU  /* 命令寄存器，受保护 */
#define STSPIN32G4_REG_STATUS        0x80U  /* 只读 */

/* POWMNG (0x01) 各位 */
#define STSPIN32G4_POWMNG_REG3V3_DIS   (1U << 6)
#define STSPIN32G4_POWMNG_VCC_DIS      (1U << 5)
#define STSPIN32G4_POWMNG_STBY_REG_EN  (1U << 4)
#define STSPIN32G4_POWMNG_VCC_VAL_8V   (0U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_10V  (1U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_12V  (2U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_15V  (3U << 1)

/* LOGIC (0x02) 各位 */
#define STSPIN32G4_LOGIC_VDS_P_DEG_6US (0U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_4US (1U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_3US (2U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_2US (3U << 2)
#define STSPIN32G4_LOGIC_DTMIN         (1U << 1)
#define STSPIN32G4_LOGIC_ILOCK         (1U << 0)

/* READY (0x07) 各位 */
#define STSPIN32G4_READY_STBY_RDY      (1U << 3)
#define STSPIN32G4_READY_THSD_RDY      (1U << 1)
#define STSPIN32G4_READY_VCC_UVLO_RDY  (1U << 0)

/* NFAULT (0x08) 各位 */
#define STSPIN32G4_NFAULT_VDS_P_FLT    (1U << 2)
#define STSPIN32G4_NFAULT_THSD_FLT     (1U << 1)
#define STSPIN32G4_NFAULT_VCC_UVLO_FLT (1U << 0)

/* 命令值 */
#define STSPIN32G4_CLEAR_FAULTS        0xFFU
#define STSPIN32G4_RESET_CMD           0xFFU
#define STSPIN32G4_STBY_ENTER          0x01U

/* LOCK (0x0B)：写受保护寄存器需要 LOCK == ~NLOCK，例如 0xA5 */
#define STSPIN32G4_LOCK_UNLOCK         0xA5U
#define STSPIN32G4_LOCK_LOCK           0x00U

/* STATUS (0x80) 各位 */
#define STSPIN32G4_STATUS_LOCK         (1U << 7)
#define STSPIN32G4_STATUS_RESET        (1U << 3)
#define STSPIN32G4_STATUS_VDS_P        (1U << 2)
#define STSPIN32G4_STATUS_THSD         (1U << 1)
#define STSPIN32G4_STATUS_VCC_UVLO     (1U << 0)

/* 写完 CLEAR 之后，等待开漏的 READY/nFAULT 电平与 STATUS 寄存器达成一致。
   手册给的 tWAKE,LDO 只有 10 µs（DS13630），但 HAL_GetTick() 的分辨率是 1 ms，
   所以这里放宽到几毫秒。 */
#define STSPIN32G4_PIN_SETTLE_MS       2U

/* --------------------------------------------------------------------------*/
/* 对上（L3 监控层）提供的接口                                               */
/* --------------------------------------------------------------------------*/

/**
  * @brief  读单个寄存器。
  * @param  reg     寄存器地址
  * @param  value   读回值存放处，不可为 NULL
  * @param  timeout I2C 超时（ms）
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT / HAL_BUSY
  */
HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                          uint32_t timeout);

/**
  * @brief  写单个寄存器。
  * @note   若目标是受保护寄存器，本函数会自动完成 LOCK 解锁时序，
  *         上层无需关心 LOCK 寄存器。
  */
HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                           uint32_t timeout);

/**
  * @brief  从 reg 起连续读 size 个字节。data 为空或 size 为 0 时返回 HAL_ERROR。
  */
HAL_StatusTypeDef STSPIN32G4_ReadRegisters(uint8_t reg, uint8_t *data,
                                           uint16_t size, uint32_t timeout);

/**
  * @brief  从 reg 起连续写 size 个字节。data 为空或 size 为 0 时返回 HAL_ERROR。
  */
HAL_StatusTypeDef STSPIN32G4_WriteRegisters(uint8_t reg, const uint8_t *data,
                                            uint16_t size, uint32_t timeout);

/**
  * @brief  芯片初始化：清除上电锁存故障，配置 POWMNG / LOGIC，
  *         并校验 READY/nFAULT 引脚电平与寄存器视图一致。
  * @retval HAL_OK 表示芯片可用；HAL_ERROR 表示 I2C 失败或仍有保护置位；
  *         HAL_BUSY 表示引脚电平尚未稳定，调用者应当重试而不是硬性判失败。
  * @note   上电后故障是锁存状态，必须写 CLEAR 芯片才会动作，因此本函数
  *         是使能功率级的前置条件。允许阻塞重试（此时尚无实时任务）。
  */
HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout);

/**
  * @brief  读 READY 引脚电平（高有效）。
  * @retval 1 = 芯片可用，0 = 不可用。不访问 I2C，可在任何上下文调用。
  */
uint8_t STSPIN32G4_IsReady(void);

/**
  * @brief  读 nFAULT 引脚的原始低有效电平。
  * @retval 0 = 有故障，1 = 无故障。不访问 I2C，可在任何上下文调用。
  */
uint8_t STSPIN32G4_IsFault(void);

/**
  * @brief  恒定耗时的安全门：只读 READY 与 nFAULT 两个引脚，绝不访问 I2C。
  * @retval 仅当 READY=1 且 nFAULT=1 时返回 1，否则返回 0。
  * @note   设计给 20 kHz 电流环用：一次读取两个引脚所在的同一端口 IDR，
  *         因此两次电平取自同一时刻，不会出现「读到一半引脚翻转」的不一致。
  *         L3 的 DrvMonitor_IsPowerStageSafeFast() 即转发到本函数。
  */
uint8_t STSPIN32G4_PinsAllowSwitching(void);

/**
  * @brief  向 CLEAR(0x09) 写 0xFF，释放已锁存的保护。
  * @note   仍然存在的故障源（例如热关断）会在下一次轮询时被重新锁存。
  */
HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout);

/**
  * @brief  唤醒：拉高 WAKE、等待 READY 释放、重新上锁 LOCK、清除锁存故障。
  * @retval HAL_TIMEOUT 表示 WAKE 后 READY 未释放（会先把 WAKE 拉回低）；
  *         其余失败返回 HAL_ERROR。
  */
HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout);

/**
  * @brief  请求进入低功耗待机模式。
  * @param  stby_reg_en 1 = 使能待机稳压器（待机期间继续给内嵌 MCU 供电），
  *                     0 = 不使能。
  *
  * 警告 —— 电流预算：待机稳压器最多只能提供 IREG3V3,Stby = 6 mA
  * （IREGStby,lim 典型 5 mA）。STM32G431 跑在 168 MHz 时的电流远超此值，
  * 因此传 1 只在「MCU 随后立即进入 Stop 模式」时才成立。否则待机稳压器会
  * 触发过流保护并复位芯片。
  *
  * 本函数是整个序列的最后一次 I2C 访问：STBY 位锁存后经过 tSTBY（100 µs）
  * 芯片即进入低功耗模式，I2C 接口随之不可用，所以之后不能再尝试重新上锁
  * ——那次传输必然超时。重新上锁由 STSPIN32G4_Wake() 完成。
  *
  * 调用者必须已经关闭功率级：解锁受保护寄存器本来就会把所有栅极驱动输出
  * 强制拉低。
  */
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __STSPIN32G4_H__ */
