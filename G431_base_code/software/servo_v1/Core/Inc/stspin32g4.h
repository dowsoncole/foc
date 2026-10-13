/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stspin32g4.h
  * @brief   STSPIN32G4 I2C register interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __STSPIN32G4_H__
#define __STSPIN32G4_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

/* 7-bit address fixed to 1000111b (0x47); HAL expects the 8-bit form */
#define STSPIN32G4_I2C_ADDRESS       (0x47U << 1)

/* Register map (DS13630 Rev 1, Tables 17-25) */
#define STSPIN32G4_REG_POWMNG        0x01U  /* Protected */
#define STSPIN32G4_REG_LOGIC         0x02U  /* Protected */
#define STSPIN32G4_REG_READY         0x07U
#define STSPIN32G4_REG_NFAULT        0x08U  /* Protected */
#define STSPIN32G4_REG_CLEAR         0x09U  /* Command */
#define STSPIN32G4_REG_STBY          0x0AU  /* Protected */
#define STSPIN32G4_REG_LOCK          0x0BU
#define STSPIN32G4_REG_RESET         0x0CU  /* Command, Protected */
#define STSPIN32G4_REG_STATUS        0x80U  /* Read only */

/* POWMNG (0x01) bits */
#define STSPIN32G4_POWMNG_REG3V3_DIS   (1U << 6)
#define STSPIN32G4_POWMNG_VCC_DIS      (1U << 5)
#define STSPIN32G4_POWMNG_STBY_REG_EN  (1U << 4)
#define STSPIN32G4_POWMNG_VCC_VAL_8V   (0U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_10V  (1U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_12V  (2U << 1)
#define STSPIN32G4_POWMNG_VCC_VAL_15V  (3U << 1)

/* LOGIC (0x02) bits */
#define STSPIN32G4_LOGIC_VDS_P_DEG_6US (0U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_4US (1U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_3US (2U << 2)
#define STSPIN32G4_LOGIC_VDS_P_DEG_2US (3U << 2)
#define STSPIN32G4_LOGIC_DTMIN         (1U << 1)
#define STSPIN32G4_LOGIC_ILOCK         (1U << 0)

/* READY (0x07) bits */
#define STSPIN32G4_READY_STBY_RDY      (1U << 3)
#define STSPIN32G4_READY_THSD_RDY      (1U << 1)
#define STSPIN32G4_READY_VCC_UVLO_RDY  (1U << 0)

/* NFAULT (0x08) bits */
#define STSPIN32G4_NFAULT_VDS_P_FLT    (1U << 2)
#define STSPIN32G4_NFAULT_THSD_FLT     (1U << 1)
#define STSPIN32G4_NFAULT_VCC_UVLO_FLT (1U << 0)

/* Command values */
#define STSPIN32G4_CLEAR_FAULTS        0xFFU
#define STSPIN32G4_RESET_CMD           0xFFU
#define STSPIN32G4_STBY_ENTER          0x01U

/* LOCK (0x0B): writing protected registers needs LOCK == ~NLOCK, e.g. 0xA5 */
#define STSPIN32G4_LOCK_UNLOCK         0xA5U
#define STSPIN32G4_LOCK_LOCK           0x00U

/* STATUS (0x80) bits */
#define STSPIN32G4_STATUS_LOCK         (1U << 7)
#define STSPIN32G4_STATUS_RESET        (1U << 3)
#define STSPIN32G4_STATUS_VDS_P        (1U << 2)
#define STSPIN32G4_STATUS_THSD         (1U << 1)
#define STSPIN32G4_STATUS_VCC_UVLO     (1U << 0)

/* Bounded wait for the open-drain READY/nFAULT levels to agree with the
   STATUS register after a CLEAR write. tWAKE,LDO is only 10 us (DS13630),
   but HAL_GetTick() resolution is 1 ms, so allow a few ms. */
#define STSPIN32G4_PIN_SETTLE_MS       2U

HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                          uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                           uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_ReadRegisters(uint8_t reg, uint8_t *data,
                                           uint16_t size, uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_WriteRegisters(uint8_t reg, const uint8_t *data,
                                            uint16_t size, uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout);
uint8_t STSPIN32G4_IsReady(void);
/* Returns the raw active-low nFAULT pin level: 0=fault, 1=no fault. */
uint8_t STSPIN32G4_IsFault(void);
HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __STSPIN32G4_H__ */
