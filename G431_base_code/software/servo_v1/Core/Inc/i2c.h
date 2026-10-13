/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   I2C3 peripheral configuration.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c3;

void MX_I2C3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
