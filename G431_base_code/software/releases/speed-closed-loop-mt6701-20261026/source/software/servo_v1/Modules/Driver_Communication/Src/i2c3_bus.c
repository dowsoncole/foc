/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c3_bus.c
  * @brief   I2C3 bus configuration used to talk to the STSPIN32G4 pre-driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "i2c3_bus.h"

I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2U;
  hi2c3.Init.OwnAddress1 = 0U;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0U;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}
