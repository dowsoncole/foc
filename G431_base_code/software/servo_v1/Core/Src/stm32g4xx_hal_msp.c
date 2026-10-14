/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "i2c3_bus.h"
/* USER CODE BEGIN Includes */
#include "tim1_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hi2c->Instance == I2C3)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    __HAL_RCC_I2C3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Lowest application priority: the update ISR only raises a flag and must
       never preempt the current loop or the PWM/sampling interrupts. */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Current-loop class timing, split across the two TIM1 vectors:
         TIM1_CC_IRQn          channel 4 compare -> sampling instant
         TIM1_UP_TIM16_IRQn    period boundary    -> state machine
       Both are one step above the TIM6 heartbeat, and they are the only
       interrupts allowed to preempt each other. */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  }
}

/**
  * @brief  TIM1 GPIO configuration, kept in the MSP layer where CubeMX generates
  *         it. The alternatives below are the only pin configuration the
  *         STSPIN32G4 gate driver accepts.
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM1)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**TIM1 GPIO Configuration
    PE8     ------> TIM1_CH1N
    PE9     ------> TIM1_CH1
    PE10    ------> TIM1_CH2N
    PE11    ------> TIM1_CH2
    PE12    ------> TIM1_CH3N
    PE13    ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
