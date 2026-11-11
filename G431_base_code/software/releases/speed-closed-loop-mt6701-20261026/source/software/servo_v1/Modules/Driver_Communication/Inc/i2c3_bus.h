/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c3_bus.h
  * @brief   I2C3 bus configuration used to talk to the STSPIN32G4 pre-driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __I2C3_BUS_H__
#define __I2C3_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef MX_I2C3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C3_BUS_H__ */
