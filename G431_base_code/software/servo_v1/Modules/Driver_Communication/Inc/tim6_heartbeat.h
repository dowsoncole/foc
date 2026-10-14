/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim6_heartbeat.h
  * @brief   TIM6 basic timer initialization: 1 s heartbeat for the driver
  *          monitor task.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM6_HEARTBEAT_H__
#define __TIM6_HEARTBEAT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim6;

HAL_StatusTypeDef MX_TIM6_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM6_HEARTBEAT_H__ */
