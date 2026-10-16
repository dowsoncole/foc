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
#include "adc1_current_sampling.h"
#include "current_sampling.h"
#include "stspin32g4.h"
#include "drv_monitor.h"
#include "foc_interface.h"
#include "waveform_generator.h"
#include "spi1_encoder.h"
#include "mt6835.h"
#include "mt6835_verify.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* OPAMP/ADC 已启动后，给模拟信号链和驱动器状态一个独立稳定时间；
   这段时间 TIM1/CH4/ADC 继续运行，但 MOE 仍保持关闭。 */
#define CURRENT_SAMPLING_MOE_GUARD_MS  20U

/* 三相下管校准会真正释放 MOE。当前硬件已经报告 VDS_P，必须先完成栅极
   极性、死区和母线电流测量，默认保持关闭，避免上电反复触发驱动保护。 */
#define CURRENT_SAMPLING_ENABLE_LOW_SIDE_CALIBRATION  0U

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

/* ADC JEOS 回调使用的唯一 FOC 控制器实例。PID 参数、限幅和 PWM 周期均由
   该实例的 algorithm.parameters/timing 管理，避免在中断中散落全局变量。 */
foc_controller_t g_foc_controller;
UART_HandleTypeDef huart1;

static uint8_t MainFoc_IsPowerStageSafe(void *context)
{
  (void)context;
  return DrvMonitor_IsPowerStageSafeFast();
}

static void MainFoc_ReadFeedback(void *context, foc_input_t *input)
{
  (void)context;
  if (input == NULL)
  {
    return;
  }
  {
    int16_t phase_u_s16;
    int16_t phase_v_s16;
    int16_t phase_w_s16;
    CurrentSampling_ReadFeedback_s16(&phase_u_s16, &phase_v_s16,
                                     &phase_w_s16);
    input->phase_a = phase_u_s16;
    input->phase_b = phase_v_s16;
  }
  /* 当前工程尚未接入编码器；角度和电流给定由上层后续更新。 */
  input->electrical_angle_rad = 0.0f;
  input->d_reference = 0.0f;
  input->q_reference = 0.0f;
}

static void MainFoc_WriteVoltage(void *context, const foc_output_t *output)
{
  (void)context;
  if (output != NULL)
  {
    (void)Tim1Pwm_SetPhaseVoltage(output->phase_a_duty,
                                  output->phase_b_duty,
                                  output->phase_c_duty);
  }
}

static void MainFoc_DisableOutput(void *context)
{
  (void)context;
  Tim1Pwm_EmergencyStop();
}

static void MainFoc_OnAdcConversion(void *context)
{
  WaveformGenerator_SampleTick();
  if (context != NULL)
  {
    FOC_ControllerStep((foc_controller_t *)context);
    ((foc_controller_t *)context)->timing.last_sample_sequence =
        g_current_sampling.sample_sequence;
  }
}

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
  if (WaveformGenerator_PortInit(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (WaveformGenerator_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* 上电先保持硬件关断。MOE 只在零点校准完成、驱动状态安全且 FOC
     即将启动时释放，避免 ADC 偏置采集阶段把电压施加到电机。 */
  Tim1Pwm_EmergencyStop();
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
   * 相别已确认：rank 1 = U 相 = foc_input_t.phase_a，rank 2 = V 相 =
   * foc_input_t.phase_b，W 相由 -(ia + ib) 推出。
   * CH4 提前量已改为注入序列耗时的一半（5.714 µs），使序列以更新事件为中心。
   *
   * 零电流偏置标定和 JDR 到安培的换算仍属于业务层，尚未实现。 */
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

  {
    const foc_parameters_t foc_parameters =
    {
      .sample_period_s = 1.0f / 20000.0f,
      .pwm_period_s = 1.0f / 20000.0f,
      .voltage_limit = 0.95f,
      .current_limit = 10.0f,
      .d_controller = { .kp = 0.1f, .ki = 5.0f,
                        .integral_min = -0.95f, .integral_max = 0.95f },
      .q_controller = { .kp = 0.1f, .ki = 5.0f,
                        .integral_min = -0.95f, .integral_max = 0.95f }
    };
    const foc_port_t foc_port =
    {
      .is_power_stage_safe = MainFoc_IsPowerStageSafe,
      .read_feedback = MainFoc_ReadFeedback,
      .write_voltage = MainFoc_WriteVoltage,
      .disable_output = MainFoc_DisableOutput,
      .context = NULL
    };
    FOC_ControllerInit(&g_foc_controller, &foc_parameters, &foc_port);
    CurrentSampling_SetConversionCallback(MainFoc_OnAdcConversion,
                                           &g_foc_controller);
  }

  if (CurrentSampling_Start() != HAL_OK)
  {
    Error_Handler();
  }

  if (MainFoc_IsPowerStageSafe(NULL) == 0U)
  {
    Error_Handler();
  }
  /* 先等待更长的稳定时间，再进入校准。默认宏为 0U，校准不释放 MOE；
     ADC 完成回调中的 FOC_ControllerStep() 会立即返回。 */
  HAL_Delay(CURRENT_SAMPLING_MOE_GUARD_MS);
  /* 先进入 SETTLING。默认校准保持 MOE=0，TIM1/CH4/ADC 仍然独立运行；
     只有完成硬件测量并确认下管模式安全后，才允许打开下面的实验开关。 */
  CurrentSampling_RequestCalibration();
#if (CURRENT_SAMPLING_ENABLE_LOW_SIDE_CALIBRATION != 0U)
  Tim1Pwm_EnableCalibrationLowSide();
#endif

  /* ------------------------------------------------------------------------
   * 编码器 SPI 配置与通讯验证（Modules/Encoder + Tests）
   * ------------------------------------------------------------------------
   * MT6835 走 SPI1：PA4 = 软件片选、PA5 = SCK、PA6 = MISO、PA7 = MOSI，
   * 模式 3、10.5 MHz。配置层只管外设，器件语义在 mt6835.c。
   *
   * 上电等待必须在任何 SPI 通讯之前：手册 T_PwrUp = 2 ms。
   * 通讯验证测试在此启动，主循环每轮推进，非阻塞。 */
  if (MX_SPI1_Encoder_Init() != HAL_OK)
  {
    Error_Handler();
  }
  MT6835_PowerUpWait();
  (void)Mt6835Verify_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CurrentSampling_Service();
    WaveformGenerator_Task();
    (void)Mt6835Verify_Task();
    /* --------------------------------------------------------------------
     * 后台任务层：无截止时间，可阻塞数十 µs
     * --------------------------------------------------------------------
     * DrvMonitor_Task() 在 TIM6 心跳未到时立即返回，所以每轮都调用的开销
     * 近乎为零。全工程的所有 I2C 通信都发生在这一次调用内部，绝不在中断里。
     */
    DrvMonitor_Task();

    /* MOE 是 TIM1 高级定时器的最终输出门。FOC running 只代表算法允许运行，
       两者必须分开管理：这里由应用层在校准完成且驱动 READY 后释放 MOE。 */
    if ((g_foc_controller.running == 0U) &&
        (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_DONE) &&
        (MainFoc_IsPowerStageSafe(NULL) != 0U))
    {
      Tim1Pwm_SetNeutral();
//      Tim1Pwm_EnableOutputs();
      FOC_ControllerStart(&g_foc_controller);
    }
    else if ((g_foc_controller.running == 0U) &&
             (CurrentSampling_GetCalibrationState() == CURRENT_SAMPLING_CAL_ERROR))
    {
      Tim1Pwm_EmergencyStop();
    }
    else if ((g_foc_controller.running != 0U) &&
             (MainFoc_IsPowerStageSafe(NULL) == 0U))
    {
      /* 驱动故障时先停止算法，再无条件清 MOE；重新启动必须重新校准并由
         上层显式恢复，避免故障消失后自动重新上电。 */
      FOC_ControllerStop(&g_foc_controller);
      Tim1Pwm_EmergencyStop();
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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

  /* ADC12 的复位时钟源为 NONE。只打开 ADC12 总线时钟还不够，必须显式
     选择内核时钟，否则 ADEN/JADSTART 可以置位，但转换永远不会完成，
     表现为 JEOS 不置位、JDR 不更新、采样序号保持 0。ADC12 源为
     SYSCLK=168 MHz，ADC 配置使用异步分频 4 得到 42 MHz。 */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInitStruct.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
