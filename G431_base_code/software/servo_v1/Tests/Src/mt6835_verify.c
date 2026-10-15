/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6835_verify.c
  * @brief   MT6835 编码器 SPI 通讯验证（L5 测试/验证层）实现。
  *
  *          Task() 每轮采集一帧角度数据（连读命令单帧 32 bit 事务），累计
  *          三级判定：传输层看 HAL 返回码，数据层看 CRC-8 自洽（主判据），
  *          状态层看 STATUS[2:0] 报警。
  *
  *          前 MT6835_VERIFY_WARMUP_FRAMES 帧只用于让 SPI 与芯片锁存状态
  *          稳定，其 CRC 失败不计入 crc_errors、其角度也不计入 samples——
  *          避免上电瞬间的毛刺帧把测试直接判死。预热结束后 CRC 仍持续不
  *          通过，即认定 MISO 链路不可信（悬空/恒电平/接反都会表现为 CRC
  *          恒败，而真实数据流不可能恒败）。
  *
  *          ⚠ 曾用 USER_ID(0x001) 写读回环做链路判据，实测不成立：0x001 是
  *          EEPROM 类寄存器，SPI 写命令后读回不反映 RAM 影子值，回环必然
  *          不符，会冤枉好板子。已废弃该判据。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mt6835_verify.h"

mt6835_verify_t g_mt6835_verify;

/* 预热丢弃帧数：吸收 SPI 初始化后头几帧的毛刺/首次锁存建立过程。 */
#define MT6835_VERIFY_WARMUP_FRAMES   8U

static uint32_t s_warmup;   /* 已丢弃的预热帧计数 */
static uint32_t s_retry_deadline;

HAL_StatusTypeDef Mt6835Verify_Start(void)
{
  g_mt6835_verify.state        = MT6835_VERIFY_RUNNING;
  g_mt6835_verify.fail_reason  = MT6835_VERIFY_FAIL_NONE;
  g_mt6835_verify.samples      = 0U;
  g_mt6835_verify.comm_errors  = 0U;
  g_mt6835_verify.crc_errors   = 0U;
  g_mt6835_verify.angle_min    = 0xFFFFFFFFUL;
  g_mt6835_verify.angle_max    = 0U;
  g_mt6835_verify.status_raw   = 0U;
  g_mt6835_verify.crc_ok       = 0U;
  s_warmup = 0U;
  s_retry_deadline = 0U;
  return HAL_OK;
}

static void Verify_Fail(mt6835_verify_fail_t reason)
{
  g_mt6835_verify.fail_reason = reason;
  g_mt6835_verify.state       = MT6835_VERIFY_FAILED;
}

/* 三级判定共用的单帧采集：传输层 -> 数据层 -> 状态层。 */
static void Verify_SampleAngle(void)
{
  mt6835_angle_t frame;

  /* 传输层：HAL 层收发失败直接计数，不参与数据判定。 */
  if (MT6835_ReadAngle(&frame, MT6835_SPI_TIMEOUT_MS) != HAL_OK)
  {
    g_mt6835_verify.comm_errors++;
    return;
  }

  /* 无论判定结果如何，最近一帧都留给人看（调试器 Watch 用）。 */
  g_mt6835_verify.last_frame = frame;
  g_mt6835_verify.status_raw = frame.status.raw;
  g_mt6835_verify.crc_ok     = frame.crc_ok;

  if (s_warmup < MT6835_VERIFY_WARMUP_FRAMES)
  {
    s_warmup++;
    return;   /* 预热帧：CRC 成败与角度值都不计入统计 */
  }

  /* 数据层（主判据）：CRC-8 自洽。 */
  if (frame.crc_ok == 0U)
  {
    g_mt6835_verify.crc_errors++;
  }
  else
  {
    g_mt6835_verify.samples++;
    if (frame.angle_raw < g_mt6835_verify.angle_min)
    {
      g_mt6835_verify.angle_min = frame.angle_raw;
    }
    if (frame.angle_raw > g_mt6835_verify.angle_max)
    {
      g_mt6835_verify.angle_max = frame.angle_raw;
    }
  }

  /* 状态层只接受 CRC 正确的帧。MISO 断线/悬空时常读到 0xFF，
     这会伪造 STATUS=0x07；若 CRC 不对，必须先报告 CRC/链路故障，
     不能把无效帧误判成“过速+弱磁场+欠压”。 */
  if ((frame.crc_ok != 0U) &&
      ((frame.status.raw & MT6835_STATUS_ALARM_MASK) != 0U))
  {
    Verify_Fail(MT6835_VERIFY_FAIL_ALARM);
  }
}

HAL_StatusTypeDef Mt6835Verify_Task(void)
{
  if (g_mt6835_verify.state != MT6835_VERIFY_RUNNING)
  {
    if ((g_mt6835_verify.state == MT6835_VERIFY_FAILED) &&
        ((int32_t)(HAL_GetTick() - s_retry_deadline) >= 0))
    {
      (void)Mt6835Verify_Start();
    }
    else
    {
      return (g_mt6835_verify.state == MT6835_VERIFY_PASSED) ? HAL_OK
                                                             : HAL_ERROR;
    }
  }

  Verify_SampleAngle();

  /* 判定顺序：先看硬失败，再看计数阈值，最后看是否采够帧数。 */
  if (g_mt6835_verify.state != MT6835_VERIFY_RUNNING)
  {
    return HAL_ERROR;
  }
  if (g_mt6835_verify.comm_errors >= MT6835_VERIFY_COMM_ERR_LIMIT)
  {
    Verify_Fail(MT6835_VERIFY_FAIL_TIMEOUT);
    s_retry_deadline = HAL_GetTick() + 100U;
    return HAL_ERROR;
  }
  if (g_mt6835_verify.crc_errors >= MT6835_VERIFY_CRC_ERR_LIMIT)
  {
    Verify_Fail(MT6835_VERIFY_FAIL_CRC);
    s_retry_deadline = HAL_GetTick() + 100U;
    return HAL_ERROR;
  }
  if (g_mt6835_verify.samples >= MT6835_VERIFY_MIN_SAMPLES)
  {
    g_mt6835_verify.state = MT6835_VERIFY_PASSED;
    return HAL_OK;
  }

  return HAL_OK;
}

uint8_t Mt6835Verify_IsRunning(void)
{
  return (g_mt6835_verify.state == MT6835_VERIFY_RUNNING) ? 1U : 0U;
}
