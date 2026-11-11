/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim6_heartbeat.c
  * @brief   TIM6 basic timer initialization: 1 s heartbeat for the driver
  *          monitor task.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim6_heartbeat.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim6;

/* TIM6 init function */
HAL_StatusTypeDef MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */

  /**
    * APB1 timer clock is 168 MHz (APB1 prescaler = 1).
    *   Counter clock = 168 MHz / 16800       = 10 kHz
    *   Update period = 10000 counts / 10 kHz = 1 s
    *
    * The update interrupt only raises a flag. All I2C traffic and state
    * machine processing stays in the main loop layer, so this ISR never
    * touches the bus and cannot disturb the current loop.
    */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16800U - 1U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000U - 1U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
  return HAL_OK;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
