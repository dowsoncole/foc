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
#include "adc1_current_sampling.h"
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

/**
  * @brief  SPI1 clock and GPIO for the MT6835 magnetic encoder.
  *         PA5 = SCK, PA6 = MISO, PA7 = MOSI, all AF5 (DS12589 Table 13).
  *         PA4 (software CSN) is configured by MX_SPI1_Encoder_Init() itself
  *         so the chip select is high before the SPI peripheral comes up.
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* A debugger reset can leave SPI1 enabled with the previous CR1/CR2
       contents.  Reset the peripheral before HAL_SPI_Init() writes the new
       mode so an old baud-rate/SPE setting cannot suppress the first frame. */
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA9 = 485_TX, PA10 = 485_RX, PA8 = transceiver DE/nRE. */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
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


/**
  * @brief  OPAMP GPIO configuration for the phase-current amplifiers.
  *         OPAMP1: PA1 (VINP0) / PA3 (VINM0) / PA2 (OUT)   -> U phase (rank 1)
  *         OPAMP3: PB0 (VINP0) / PB2 (VINM0) / PB1 (OUT)   -> V phase (rank 2)
  *
  *         All six pins must sit in analog mode. PB2 doubles as BOOT1, so it
  *         has to be configured explicitly or the leftover boot-mode setting
  *         will disturb the inverting input.
  *
  *         PA2 and PB1 are also the ADC1_IN3 / ADC1_IN12 inputs: the amplifier
  *         outputs come back through the external pins (not the internal
  *         VOPAMP channels) because the schematic hangs a post-stage RC
  *         network on those nodes.
  *
  *         The STM32G4 OPAMPs have no dedicated RCC gate, so only the GPIO
  *         clocks are enabled here; the amplifier itself is turned on through
  *         its own CSR enable bit inside HAL_OPAMP_Init().
  */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hopamp->Instance == OPAMP1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (hopamp->Instance == OPAMP3)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**OPAMP3 GPIO Configuration
    PB0     ------> OPAMP3_VINP
    PB1     ------> OPAMP3_VOUT
    PB2     ------> OPAMP3_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp)
{
  if (hopamp->Instance == OPAMP1)
  {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  }
  else if (hopamp->Instance == OPAMP3)
  {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  }
}

/**
  * @brief  ADC1 clock enable for the injected current-sampling group.
  *
  *         JEOS is consumed in ADC1_2_IRQHandler after the complete injected
  *         sequence; keep this interrupt below the timer trigger priority.
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* ADC12SEL 复位值为 NONE。即使 SystemClock_Config() 已配置过，
       在 ADC 初始化入口再次明确选择 SYSCLK，保证 ADC 内核确实有时钟；
       仅打开 ADC12 总线时钟不能完成转换。 */
    __HAL_RCC_ADC12_CONFIG(RCC_ADC12CLKSOURCE_SYSCLK);
    __HAL_RCC_ADC12_CLK_ENABLE();
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    __HAL_RCC_ADC12_CLK_DISABLE();
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
