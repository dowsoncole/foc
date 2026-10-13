/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   TIM6 basic timer initialization: 1 s heartbeat for the driver
  *          monitor task.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim6;

void MX_TIM6_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
