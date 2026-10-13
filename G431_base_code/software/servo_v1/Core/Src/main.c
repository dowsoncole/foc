/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "tim1_pwm.h"
#include "tim1_pwm_verify.h"
#include "stspin32g4.h"
#include "drv_monitor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t a = 100U;
volatile uint8_t b = 10U;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* ------------------------------------------------------------------------
   * Layer 0 - bring-up, runs once before any real-time task starts
   * ------------------------------------------------------------------------
   * DrvMonitor_Init() clears the power-up latched fault (mandatory: the device
   * stays inoperative until CLEAR is written) and starts the 1 s heartbeat.
   * Blocking retries here are harmless because nothing real-time is running.
   */
  DrvMonitor_Init();

  /* ------------------------------------------------------------------------
   * Layer 1 - TIM1 gate-signal verification
   * ------------------------------------------------------------------------
   * MX_TIM1_Init() has already produced the 20 kHz carrier, but MOE is still
   * cleared, so no gate signal can reach the power stage. The sequence below
   * checks the last untested link in the chain (MCU pin -> gate driver -> power
   * stage) with a pattern that is easy to judge on a scope and gentle enough for
   * a motor that is already bolted to the phases.
   *
   * Read the runbook in tim1_pwm_verify.h before enabling it, and keep the DC
   * link current limited (or the supply below the motor's rated voltage) for the
   * first run: the static pattern holds no torque, but a stalled winding still
   * draws winding_resistance-limited current while it does.
   *
   * Set TIM1_PWM_VERIFY_ENABLE to 0 to skip it. While it is enabled, the driver
   * has to be reported healthy first: a faulted driver must never be driven.
   * The timeout only bounds the pattern; it does not replace a current limit.
   */
#define TIM1_PWM_VERIFY_ENABLE       1U
#define TIM1_PWM_VERIFY_TIMEOUT_MS   10000U

  if ((TIM1_PWM_VERIFY_ENABLE != 0U) &&
      (DrvMonitor_GetState() == DRV_STATE_READY))
  {
    if (Tim1PwmVerify_Start(TIM1_PWM_VERIFY_TIMEOUT_MS) != HAL_OK)
    {
      /* The outputs never left the neutral pattern; the driver monitor keeps
         running so the reason can be read from the snapshot. */
      Error_Handler();
    }
  }

  /* TODO: MX_ADC1_Init() + the current loop replace the verification, and the
   * power stage may only be enabled while DrvMonitor_GetState() == DRV_STATE_READY
   * and TIM1->BDTR still has MOE set. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* --------------------------------------------------------------------
     * Layer 3 - background tasks, no deadline, may block for tens of us
     * --------------------------------------------------------------------
     * DrvMonitor_Task() returns immediately unless TIM6 has raised the 1 s
     * tick, so calling it every iteration costs nothing. All I2C traffic of
     * the whole application happens inside this call, never in an ISR.
     */
    DrvMonitor_Task();

    /* --------------------------------------------------------------------
     * Layer 2 - TIM1 verification sequence
     * --------------------------------------------------------------------
     * Non-blocking: it returns between steps, so this layer never holds the
     * loop. Remove both calls once the current loop owns the PWM channels.
     */
#if (TIM1_PWM_VERIFY_ENABLE != 0U)
    (void)Tim1PwmVerify_Task(TIM1_PWM_VERIFY_TIMEOUT_MS);
#endif

    /* TODO: protocol stack (CAN/UART), parameter storage, temperature
     * logging, host telemetry ... all belong to this layer. */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* X1 is 12 MHz: 12 MHz / 1 * 28 = 336 MHz VCO, /2 = 168 MHz SYSCLK. */
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// whatever it is 
// oe
// complex 
// st 
// 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_H-andler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
