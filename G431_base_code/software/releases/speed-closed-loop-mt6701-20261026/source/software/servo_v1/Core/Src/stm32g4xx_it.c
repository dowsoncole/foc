/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim6_heartbeat.h"
#include "tim1_pwm.h"
#include "adc1_current_sampling.h"
#include "current_sampling.h"
#include "encoder.h"
#include "spi1_encoder.h"
#include "timing_probes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void ADC1_2_IRQHandler(void)
{
  /* First software probe operation. This is IRQ entry, not ADC hardware
     conversion completion; latency/exception stacking precede this edge. */
  TimingProbe_AdcBegin();
  uint32_t entry_cycles = DWT->CYCCNT;
  g_tim1_pwm_adc_entry_cycle = entry_cycles;
  /* Keep both the fast and HAL fallback paths inside the same pulse. */
  uint32_t pending = ADC1->ISR & ADC1->IER;
  if (hadc1.Instance == ADC1 && (pending & ADC_IT_JEOS) &&
      !(ADC1->IER & ADC_IT_JEOC) &&
      !(ADC1->ISR & (ADC_FLAG_OVR | ADC_FLAG_JQOVF | ADC_FLAG_AWD1 | ADC_FLAG_AWD2 | ADC_FLAG_AWD3)) &&
      !(pending & ~(ADC_IT_JEOC | ADC_IT_JEOS)) &&
      hadc1.ErrorCode == HAL_ADC_ERROR_NONE &&
      !(hadc1.State & (HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_CONFIG)) &&
      (ADC1->JSQR & ADC_JSQR_JEXTEN) && !(ADC1->CFGR & ADC_CFGR_JAUTO))
    CurrentSampling_InjectedFastIrq();
  else
    HAL_ADC_IRQHandler(&hadc1);
  Encoder_RecordFocIsrCycles(DWT->CYCCNT - entry_cycles);
  /* FOC, CCR preload writes and deadline diagnostics have all returned.
     Only the compiler epilogue / hardware exception return follows. */
  TimingProbe_AdcEnd();
}

void DMA1_Channel1_IRQHandler(void)
{
  SPI1_Encoder_DmaRxIrqHandler();
  /* Final probe operation after DMA cleanup, CRC/angle processing and
     publication have ALL returned. A spurious IRQ must not shorten an
     in-flight transaction. Only the epilogue/exception return follows. */
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}

void DMA1_Channel2_IRQHandler(void)
{
  SPI1_Encoder_DmaTxIrqHandler();
  /* TX errors can terminate the transaction without an RX completion IRQ. */
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}

void SPI1_IRQHandler(void)
{
  SPI1_Encoder_SpiIrqHandler();
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}

/**
  * @brief This function handles TIM6 global interrupt.
  *
  * 1 s driver-monitor heartbeat. This ISR performs no I2C transfer: it only
  * raises a flag that the main-loop task consumes.
  */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

  }

/**
  * @brief This function handles TIM1 capture/compare interrupt.
  *
 * Reserved vector. CC4 interrupt is disabled: OC4REF through TRGO2 triggers
 * the injected ADC in hardware. Only ADC1_2_IRQHandler schedules FOC after JEOS.
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles TIM1 update interrupt.
  *
  * The period boundary, once per PWM period thanks to the repetition counter.
  * TIM1 starts encoder SPI/DMA directly, then dispatches the application
  * PWM-boundary callback. TIM16 sampling is disabled. ADC still schedules FOC
  * from JEOS; this ISR never waits for completion of SPI or the current loop.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  TimingProbe_EncoderBegin();
  if ((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE))
  {
    /* Update-only vector; clear first, retain the shared application callback.
       Do not scan unrelated capture/compare channels at every PWM boundary. */
    TIM1->SR = ~TIM_SR_UIF;
    Encoder_OnPwmPeriodIsr();
    HAL_TIM_PeriodElapsedCallback(&htim1);
  }
  /* No RX IRQ follows a skipped/rejected start. Preserve a live DMA pulse. */
  if (!MT6835_SPI_IsBusy()) TimingProbe_EncoderEnd();
}
/* USER CODE END 1 */
