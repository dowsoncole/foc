/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stspin32g4.c
  * @brief   STSPIN32G4 I2C register interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stspin32g4.h"

HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                          uint32_t timeout)
{
  return STSPIN32G4_ReadRegisters(reg, value, 1U, timeout);
}

HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                           uint32_t timeout)
{
  return STSPIN32G4_WriteRegisters(reg, &value, 1U, timeout);
}

HAL_StatusTypeDef STSPIN32G4_ReadRegisters(uint8_t reg, uint8_t *data,
                                           uint16_t size, uint32_t timeout)
{
  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c3, STSPIN32G4_I2C_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, data, size, timeout);
}

HAL_StatusTypeDef STSPIN32G4_WriteRegisters(uint8_t reg, const uint8_t *data,
                                            uint16_t size, uint32_t timeout)
{
  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Write(&hi2c3, STSPIN32G4_I2C_ADDRESS, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, size,
                           timeout);
}

HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout)
{
  uint8_t  status = 0U;
  uint32_t tick_start;

  /* After power-up the fault is latched: clear it to make the device
     operative. The LOCK register is unprotected, so clearing works even
     before unlocking. */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_CLEAR, STSPIN32G4_CLEAR_FAULTS,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (STSPIN32G4_ReadRegister(STSPIN32G4_REG_STATUS, &status,
                              timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((status & (STSPIN32G4_STATUS_VDS_P | STSPIN32G4_STATUS_THSD |
                 STSPIN32G4_STATUS_VCC_UVLO)) != 0U)
  {
    return HAL_ERROR;
  }

  /* The register view alone is not sufficient: READY/nFAULT are the pins the
     hardware actually uses to gate the power stage, so they must agree.
     READY is open-drain and asserted high when the device is usable; nFAULT
     is open-drain and asserted low when a fault is being reported.

     The STATUS read above already spent ~390 us on the bus, which is ample
     settle time, but allow a short bounded wait for marginal cases. A
     transient mismatch makes the caller retry rather than failing hard. */
  tick_start = HAL_GetTick();
  while ((STSPIN32G4_IsReady() == 0U) || (STSPIN32G4_IsFault() == 0U))
  {
    if ((HAL_GetTick() - tick_start) > STSPIN32G4_PIN_SETTLE_MS)
    {
      return HAL_BUSY;
    }
  }

  return HAL_OK;
}

uint8_t STSPIN32G4_IsReady(void)
{
  return (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET)
         ? 1U : 0U;
}

uint8_t STSPIN32G4_IsFault(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin);
}

HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout)
{
  return STSPIN32G4_WriteRegister(STSPIN32G4_REG_CLEAR, STSPIN32G4_CLEAR_FAULTS,
                                  timeout);
}

HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout)
{
  uint32_t tick_start = HAL_GetTick();

  /* WAKE is active high with an internal pull-down on the driver side */
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

  /* READY is released tWAKE,LDO (10 us) after VCC crosses VCC(On) */
  while (STSPIN32G4_IsReady() == 0U)
  {
    if ((HAL_GetTick() - tick_start) > timeout)
    {
      HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);
      return HAL_TIMEOUT;
    }
  }

  /* Deassert WAKE so a later standby request is not aborted by it */
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);

  /* EnterStandby() could not re-lock: the I2C interface is already dead by the
     time STBY takes effect. While LOCK == ~NLOCK the gate driver outputs are
     forced low (DS13630 Table 23), so the device cannot drive the MOSFETs
     until it is locked again. LOCK is unprotected, so this always works once
     the device is powered. If the MCU lost power during standby the registers
     are already back to defaults (LOCK = 0), and this write is a harmless
     no-op. */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_LOCK, STSPIN32G4_LOCK_LOCK,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Clears the power-up / standby-exit latched fault and verifies the pins */
  return STSPIN32G4_Init(timeout);
}

/**
  * @brief  Request entry into low consumption mode.
  * @param  stby_reg_en 1 to enable the standby regulator (keeps the embedded
  *                     MCU supplied during standby), 0 to leave it off.
  *
  * WARNING - current budget: the standby regulator delivers at most
  * IREG3V3,Stby = 6 mA (IREGStby,lim = 5 mA typ). A STM32G431 running at
  * 168 MHz draws far more than that, so passing 1 here is only valid if the
  * MCU itself enters Stop mode right afterwards. Otherwise the standby
  * regulator trips its overcurrent protection and resets the device.
  *
  * This is the LAST I2C access of the sequence. The device switches to low
  * consumption mode tSTBY (100 us) after the STBY bit is latched and the I2C
  * interface becomes unavailable, so the function must not attempt to re-lock
  * afterwards - that transfer would time out. Re-locking is done by
  * STSPIN32G4_Wake().
  *
  * The caller must already have disabled the power stage: unlocking the
  * protected registers forces all gate driver outputs low anyway.
  */
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en, uint32_t timeout)
{
  uint8_t powmng = 0U;

  /* STBY and POWMNG are protected registers: unlock first */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_LOCK, STSPIN32G4_LOCK_UNLOCK,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (stby_reg_en != 0U)
  {
    /* Read-modify-write so the VCC setting and regulator enables are kept */
    if (STSPIN32G4_ReadRegister(STSPIN32G4_REG_POWMNG, &powmng,
                                timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }

    powmng |= STSPIN32G4_POWMNG_STBY_REG_EN;

    if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_POWMNG, powmng,
                                 timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return STSPIN32G4_WriteRegister(STSPIN32G4_REG_STBY, STSPIN32G4_STBY_ENTER,
                                  timeout);
}
