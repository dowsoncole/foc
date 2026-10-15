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
#include "i2c3_bus.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim6_heartbeat.h"
#include "tim1_pwm.h"
#include "tim1_pwm_verify.h"
#include "measurement_manager.h"
#include "adc1_current_sampling.h"
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

/* 统一的测量配置与运行状态对象：由 L6 应用入口层持有，交给 L5 测试层消费。
   刻意做成全局对象，一是能在调试器 Watch 窗口直接观察，二是能在测量运行途中
   改字段来切换被测相或占空比，无需重新编译烧录。 */
measurement_manager_t g_measurement_manager =
{
  .module = MEASUREMENT_MODULE_MOS_DRIVER,
  .state = MEASUREMENT_STATE_IDLE,
  .config.mos_driver =
  {
    .phase = MOS_DRIVER_PHASE_U,
    .duty = 50U,         /* 选中相的高侧占空比，标度 0..1000，故 50 表示 5 %。 */
    .moe_enable = 1U,    /* 预充电延时结束后自动使能波形输出。 */
    .startup_delay_ms = 50U, /* 等 VCC 与栅极驱动自举电容稳定。 */
    .moe_applied = 0U,
    .phase_applied = MOS_DRIVER_PHASE_U,
    .duty_applied = 0U
  }
};

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
  if (MX_I2C3_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_TIM6_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_TIM1_Init() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */
  /* ------------------------------------------------------------------------
   * Layer 0 上电初始化：在任何实时任务开始之前一次性跑完
   * ------------------------------------------------------------------------
   * DrvMonitor_Init() 会清除上电锁存故障（必须做：不写 CLEAR，芯片一直处于
   * 不动作状态），并启动 1 s 心跳。此处允许阻塞重试，因为此时还没有任何
   * 实时任务在运行，阻塞不会破坏 PWM 或采样时序。
   */
  if (DrvMonitor_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 只有栅极驱动报告 READY 之后才启动测量。
     MOE 仍由 g_measurement_manager.config.mos_driver.moe_enable 控制。 */
  if (DrvMonitor_GetState() == DRV_STATE_READY)
  {
    (void)Tim1PwmVerify_Start(&g_measurement_manager);
  }

  /* ------------------------------------------------------------------------
   * 电流采样外设配置（模块 F：Modules/Current_Sampling/）
   * ------------------------------------------------------------------------
   * 顺序有意义：两个运放必须先于 ADC 配好。ADC1 采的是运放输出脚
   * （PA2 = ADC1_IN3，PB1 = ADC1_IN12），运放没起来时那两个节点上没有有效
   * 信号，先校准 ADC 只会把噪声当成基线记下来。
   *
   * ADC 使用 TIM1_CH4 硬件触发。MX_ADC1_Init() 只配置和校准，
   * CurrentSampling_Start() 负责按“运放先启动、ADC 后启动”的顺序接通信号链。
   * ADC 运行时不需要在 TIM1_CC ISR 里手工发起转换；CC4 的 OC4REF 会自动触发
   * 注入序列，JDR1/JDR2 由硬件更新。
   *
   * 零电流偏置标定和 JDR 到安培的换算仍属于业务层，接入 FOC 前还需按 README
   * 待核对项确认采样序列时序以及两路通道的实际相别。 */
  if (MX_OPAMP1_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_OPAMP3_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_ADC1_Init(0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (CurrentSampling_Start() != HAL_OK)
  {
    Error_Handler();
  }

  /* TODO: 电流环。功率级只能在
   * DrvMonitor_GetState() == DRV_STATE_READY 且 TIM1->BDTR 的 MOE 仍置位时
   * 才允许使能。 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* --------------------------------------------------------------------
     * 后台任务层：无截止时间，可阻塞数十 µs
     * --------------------------------------------------------------------
     * DrvMonitor_Task() 在 TIM6 心跳未到时立即返回，所以每轮都调用的开销
     * 近乎为零。全工程的所有 I2C 通信都发生在这一次调用内部，绝不在中断里。
     */
    DrvMonitor_Task();

    /* ------------------------------------------------------------------
     * L6 跨层仲裁：驱动报告非 READY 时关断功率级
     * ------------------------------------------------------------------
     * 修复：原先 DrvMonitor 检测到 FAULT / COMM_LOST 后只更新自身状态，
     * 没有任何一方去清 MOE。STSPIN32G4 的内部保护（VDS_P / THSD）确实会把
     * 栅极强制拉低，硬件不会损坏；但软件侧 MOE 仍保持置位、测量状态仍是
     * RUNNING，一旦故障条件消失，输出会在无人复核的情况下自行恢复。
     *
     * 这段仲裁放在 L6 而不是 L3：监控层不得调用三相驱动层（下层不能依赖
     * 上层），跨层仲裁本来就是应用入口层的职责。
     *
     * 只在 IsRunning 为真时触发，因此每轮循环最多关断一次；Stop 会把状态置为
     * IDLE，之后不再重复调用。恢复输出必须显式重新 Start，不会自动重启。
     */
    if ((Tim1PwmVerify_IsRunning(&g_measurement_manager) != 0U) &&
        (DrvMonitor_GetState() != DRV_STATE_READY))
    {
      Tim1PwmVerify_Stop(&g_measurement_manager);
    }

    (void)Tim1PwmVerify_Task(&g_measurement_manager);

    /* TODO: 协议栈（CAN/UART）、参数存储、温度记录、上位机遥测 ……
     * 都属于这一层。 */
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
  /* X1 为 8 MHz：8 MHz / 2 = 4 MHz VCO 输入，× 84 = 336 MHz VCO，/ 2 = 168 MHz
     SYSCLK。VCO 输入取 4 MHz 是 ST 推荐的最低 PLL 抖动点。
     SYSCLK 刻意保持 168 MHz，因为所有派生时序常量都依赖它：20 kHz TIM1 载波、
     500 ns 死区、10 kHz TIM6 计数时钟（分频 16800）以及 I2C 时序字。改主频会
     连带让这些常量全部失效。 */
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  * @param  line: assert_param error source line number
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
