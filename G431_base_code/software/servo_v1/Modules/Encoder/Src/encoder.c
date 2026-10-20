#include "encoder.h"

#include "mt6835.h"
#include "spi1_encoder.h"

#include <math.h>
#include <string.h>

#define ENCODER_TWO_PI                 6.28318530717958647692f
#define ENCODER_RAD_PER_COUNT          (ENCODER_TWO_PI / (float)MT6835_ANGLE_FULL_SCALE)
#define ENCODER_TRANSFER_TIMEOUT_US    100UL
#define ENCODER_SPEED_FILTER_TAU_S     0.010f

typedef struct
{
  int64_t position;
  uint32_t cycle;
} encoder_speed_point_t;

typedef struct
{
  encoder_speed_point_t speed_history[ENCODER_SPEED_WINDOW_SAMPLES];
  uint8_t tx[6];
  uint8_t rx[6];
  uint8_t history_head;
  uint8_t history_count;
  volatile uint8_t initialized;
  volatile uint8_t transfer_busy;
  uint8_t have_previous_angle;
  volatile uint8_t motor_configured;
  volatile int8_t direction;
  volatile uint16_t pole_pairs;
  volatile float electrical_offset_rad;
  uint32_t previous_raw_angle;
  int64_t multi_turn_count;
  float filtered_speed_rad_s;
  volatile uint32_t transfer_start_cycle;
} encoder_context_t;

static encoder_context_t s_encoder;
volatile encoder_snapshot_t g_encoder_snapshot;
volatile encoder_diagnostics_t g_encoder_diagnostics;

static uint32_t Encoder_NowCycles(void)
{
  return DWT->CYCCNT;
}

static uint32_t Encoder_IrqSave(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  return mask;
}

static void Encoder_IrqRestore(uint32_t mask)
{
  if (mask == 0U)
  {
    __enable_irq();
  }
}

static float Encoder_WrapAngle(float angle)
{
  while (angle >= ENCODER_TWO_PI)
  {
    angle -= ENCODER_TWO_PI;
  }
  while (angle < 0.0f)
  {
    angle += ENCODER_TWO_PI;
  }
  return angle;
}

static uint32_t Encoder_MaxAgeCycles(void)
{
  return (SystemCoreClock / ENCODER_CONTROL_RATE_HZ) *
         ENCODER_MAX_SAMPLE_AGE_CYCLES;
}

static uint32_t Encoder_TransferTimeoutCycles(void)
{
  return (SystemCoreClock / 1000000UL) * ENCODER_TRANSFER_TIMEOUT_US;
}

static void Encoder_PublishInvalid(encoder_fault_flags_t fault,
                                   uint8_t status_raw)
{
  uint32_t mask = Encoder_IrqSave();
  g_encoder_snapshot.fault_flags = fault;
  g_encoder_snapshot.status_raw = status_raw;
  g_encoder_snapshot.angle_valid = 0U;
  g_encoder_snapshot.speed_valid = 0U;
  g_encoder_snapshot.electrical_angle_valid = 0U;
  Encoder_IrqRestore(mask);
}

static void Encoder_ResetTracking(void)
{
  encoder_snapshot_t empty_snapshot = {0};

  s_encoder.history_head = 0U;
  s_encoder.history_count = 0U;
  s_encoder.have_previous_angle = 0U;
  s_encoder.previous_raw_angle = 0U;
  s_encoder.multi_turn_count = 0;
  s_encoder.filtered_speed_rad_s = 0.0f;
  (void)memset(s_encoder.speed_history, 0, sizeof(s_encoder.speed_history));
  g_encoder_snapshot = empty_snapshot;
  if (s_encoder.motor_configured == 0U)
  {
    g_encoder_snapshot.fault_flags = ENCODER_FAULT_NOT_CONFIGURED;
  }
}

static void Encoder_UpdateSpeed(int64_t position, uint32_t sample_cycle,
                                encoder_snapshot_t *next)
{
  encoder_speed_point_t oldest;

  if (s_encoder.history_count < ENCODER_SPEED_WINDOW_SAMPLES)
  {
    s_encoder.speed_history[s_encoder.history_head].position = position;
    s_encoder.speed_history[s_encoder.history_head].cycle = sample_cycle;
    s_encoder.history_head = (uint8_t)((s_encoder.history_head + 1U) %
                                       ENCODER_SPEED_WINDOW_SAMPLES);
    s_encoder.history_count++;
    next->mechanical_speed_rad_s = 0.0f;
    next->speed_valid = 0U;
    return;
  }

  oldest = s_encoder.speed_history[s_encoder.history_head];
  s_encoder.speed_history[s_encoder.history_head].position = position;
  s_encoder.speed_history[s_encoder.history_head].cycle = sample_cycle;
  s_encoder.history_head = (uint8_t)((s_encoder.history_head + 1U) %
                                     ENCODER_SPEED_WINDOW_SAMPLES);

  {
    uint32_t delta_cycles = sample_cycle - oldest.cycle;
    if (delta_cycles != 0U)
    {
      float dt = (float)delta_cycles / (float)SystemCoreClock;
      float raw_speed = (float)(position - oldest.position) *
                        ENCODER_RAD_PER_COUNT / dt;
      float alpha = dt / (ENCODER_SPEED_FILTER_TAU_S + dt);
      s_encoder.filtered_speed_rad_s +=
          alpha * (raw_speed - s_encoder.filtered_speed_rad_s);
      next->mechanical_speed_rad_s = s_encoder.filtered_speed_rad_s;
      next->speed_valid = 1U;
      return;
    }
  }

  next->mechanical_speed_rad_s = s_encoder.filtered_speed_rad_s;
  next->speed_valid = 0U;
}

static void Encoder_PublishFrame(const mt6835_angle_t *frame,
                                 uint32_t sample_cycle)
{
  encoder_snapshot_t next;
  uint32_t directed_raw;

  (void)memset(&next, 0, sizeof(next));

  if (s_encoder.direction > 0)
  {
    directed_raw = frame->angle_raw;
  }
  else
  {
    directed_raw = (MT6835_ANGLE_FULL_SCALE - frame->angle_raw) &
                   (MT6835_ANGLE_FULL_SCALE - 1UL);
  }

  if (s_encoder.have_previous_angle == 0U)
  {
    s_encoder.have_previous_angle = 1U;
    s_encoder.previous_raw_angle = frame->angle_raw;
    s_encoder.multi_turn_count = (int64_t)directed_raw;
  }
  else
  {
    int32_t delta = (int32_t)frame->angle_raw -
                    (int32_t)s_encoder.previous_raw_angle;
    if (delta > (int32_t)(MT6835_ANGLE_FULL_SCALE / 2UL))
    {
      delta -= (int32_t)MT6835_ANGLE_FULL_SCALE;
    }
    else if (delta < -(int32_t)(MT6835_ANGLE_FULL_SCALE / 2UL))
    {
      delta += (int32_t)MT6835_ANGLE_FULL_SCALE;
    }
    s_encoder.previous_raw_angle = frame->angle_raw;
    s_encoder.multi_turn_count += (int64_t)delta * (int64_t)s_encoder.direction;
  }

  next.raw_angle = frame->angle_raw;
  next.multi_turn_count = s_encoder.multi_turn_count;
  next.mechanical_angle_rad = (float)directed_raw * ENCODER_RAD_PER_COUNT;
  next.sample_cycle = sample_cycle;
  next.sequence = g_encoder_snapshot.sequence + 1UL;
  next.status_raw = frame->status.raw;
  next.angle_valid = 1U;
  next.fault_flags = (s_encoder.motor_configured != 0U) ?
      ENCODER_FAULT_NONE : ENCODER_FAULT_NOT_CONFIGURED;

  Encoder_UpdateSpeed(s_encoder.multi_turn_count, sample_cycle, &next);

  if (s_encoder.motor_configured != 0U)
  {
    next.electrical_angle_rad = Encoder_WrapAngle(
        (float)s_encoder.pole_pairs * next.mechanical_angle_rad +
        s_encoder.electrical_offset_rad);
    next.electrical_angle_valid = 1U;
  }

  {
    uint32_t mask = Encoder_IrqSave();
    g_encoder_snapshot = next;
    Encoder_IrqRestore(mask);
  }
}

static void Encoder_TransferComplete(HAL_StatusTypeDef status, void *context)
{
  mt6835_angle_t frame;
  uint32_t entry_cycles = Encoder_NowCycles();
  uint32_t transaction_cycles = entry_cycles - s_encoder.transfer_start_cycle;
  (void)context;

  g_encoder_diagnostics.transaction_cycles_latest = transaction_cycles;
  if ((g_encoder_diagnostics.transaction_cycles_min == 0U) ||
      (transaction_cycles < g_encoder_diagnostics.transaction_cycles_min))
  {
    g_encoder_diagnostics.transaction_cycles_min = transaction_cycles;
  }
  if (transaction_cycles > g_encoder_diagnostics.transaction_cycles_max)
  {
    g_encoder_diagnostics.transaction_cycles_max = transaction_cycles;
  }

  if (status != HAL_OK)
  {
    g_encoder_diagnostics.transfer_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_COMMUNICATION, 0U);
  }
  else if (MT6835_DecodeAngleResponse(s_encoder.rx, &frame) != HAL_OK)
  {
    g_encoder_diagnostics.transfer_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_COMMUNICATION, 0U);
  }
  else if (frame.crc_ok == 0U)
  {
    g_encoder_diagnostics.crc_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_CRC, frame.status.raw);
  }
  else if ((frame.status.raw & MT6835_STATUS_ALARM_MASK) != 0U)
  {
    g_encoder_diagnostics.alarm_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_SENSOR_ALARM, frame.status.raw);
  }
  else
  {
    Encoder_PublishFrame(&frame, s_encoder.transfer_start_cycle);
    g_encoder_diagnostics.completed_count++;
  }

  {
    uint32_t elapsed = Encoder_NowCycles() - entry_cycles;
    g_encoder_diagnostics.completion_cycles_latest = elapsed;
    if (elapsed > g_encoder_diagnostics.completion_cycles_max)
    {
      g_encoder_diagnostics.completion_cycles_max = elapsed;
    }
  }
  /* 必须最后释放业务 busy：TIM1 的优先级高于 DMA，若在解析 rx 缓冲前
     清 busy，下一周期可能抢占并用同一缓冲启动新事务。 */
  s_encoder.transfer_busy = 0U;
}

uint8_t Encoder_Init(void)
{
  encoder_diagnostics_t empty_diagnostics = {0};

  (void)memset(&s_encoder, 0, sizeof(s_encoder));
  g_encoder_diagnostics = empty_diagnostics;
  s_encoder.direction = 1;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (MX_SPI1_Encoder_Init() != HAL_OK)
  {
    return 0U;
  }
  MT6835_PowerUpWait();
  MT6835_BuildAngleReadCommand(s_encoder.tx);
  Encoder_ResetTracking();
  s_encoder.initialized = 1U;
  return 1U;
}

uint8_t Encoder_ConfigureMotor(int8_t direction, uint16_t pole_pairs,
                               float electrical_offset_rad)
{
  uint32_t mask;

  if (((direction != 1) && (direction != -1)) || (pole_pairs == 0U) ||
      !isfinite(electrical_offset_rad))
  {
    return 0U;
  }

  mask = Encoder_IrqSave();
  s_encoder.direction = direction;
  s_encoder.pole_pairs = pole_pairs;
  s_encoder.electrical_offset_rad = Encoder_WrapAngle(electrical_offset_rad);
  s_encoder.motor_configured = 1U;
  Encoder_ResetTracking();
  Encoder_IrqRestore(mask);
  return 1U;
}

void Encoder_OnPwmPeriodIsr(void)
{
  HAL_StatusTypeDef status;
  uint32_t entry_cycles;

  if (s_encoder.initialized == 0U)
  {
    return;
  }

  g_encoder_diagnostics.trigger_count++;
  if (s_encoder.transfer_busy != 0U)
  {
    g_encoder_diagnostics.busy_trigger_count++;
    return;
  }

  entry_cycles = Encoder_NowCycles();
  s_encoder.transfer_start_cycle = entry_cycles;
  s_encoder.transfer_busy = 1U;
  status = MT6835_SPI_TransmitReceiveAsync(
      s_encoder.tx, s_encoder.rx, sizeof(s_encoder.tx),
      Encoder_TransferComplete, NULL);
  if (status != HAL_OK)
  {
    s_encoder.transfer_busy = 0U;
    g_encoder_diagnostics.start_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_COMMUNICATION, 0U);
  }

  {
    uint32_t elapsed = Encoder_NowCycles() - entry_cycles;
    g_encoder_diagnostics.start_cycles_latest = elapsed;
    if (elapsed > g_encoder_diagnostics.start_cycles_max)
    {
      g_encoder_diagnostics.start_cycles_max = elapsed;
    }
  }
}

void Encoder_Service(void)
{
  uint32_t mask = Encoder_IrqSave();

  if ((s_encoder.initialized != 0U) && (s_encoder.transfer_busy != 0U) &&
      ((Encoder_NowCycles() - s_encoder.transfer_start_cycle) >
       Encoder_TransferTimeoutCycles()))
  {
    /* 异常路径保持短临界区，防止下一次 TIM1 触发与 abort 交叉。Abort
       回调统一记录通信错误并撤销快照有效性。 */
    (void)MT6835_SPI_AbortAsync();
  }
  Encoder_IrqRestore(mask);
}

uint8_t Encoder_ReadSnapshot(encoder_snapshot_t *snapshot)
{
  uint32_t mask;

  if ((snapshot == NULL) || (s_encoder.initialized == 0U))
  {
    return 0U;
  }

  mask = Encoder_IrqSave();
  *snapshot = g_encoder_snapshot;
  Encoder_IrqRestore(mask);

  if ((snapshot->angle_valid != 0U) &&
      ((Encoder_NowCycles() - snapshot->sample_cycle) > Encoder_MaxAgeCycles()))
  {
    snapshot->fault_flags |= ENCODER_FAULT_STALE;
    snapshot->angle_valid = 0U;
    snapshot->speed_valid = 0U;
    snapshot->electrical_angle_valid = 0U;
  }
  return (snapshot->angle_valid != 0U) ? 1U : 0U;
}

uint8_t Encoder_ReadElectricalAngleForFoc(float *electrical_angle_rad)
{
  encoder_snapshot_t snapshot;
  uint32_t entry_cycles = Encoder_NowCycles();
  uint8_t valid = 0U;

  if (s_encoder.transfer_busy != 0U)
  {
    g_encoder_diagnostics.foc_reads_while_busy++;
  }

  if ((electrical_angle_rad != NULL) &&
      (Encoder_ReadSnapshot(&snapshot) != 0U) &&
      (snapshot.electrical_angle_valid != 0U))
  {
    uint32_t age = entry_cycles - snapshot.sample_cycle;
    *electrical_angle_rad = snapshot.electrical_angle_rad;
    g_encoder_diagnostics.foc_sample_age_cycles_latest = age;
    if (age > g_encoder_diagnostics.foc_sample_age_cycles_max)
    {
      g_encoder_diagnostics.foc_sample_age_cycles_max = age;
    }
    valid = 1U;
  }

  {
    uint32_t elapsed = Encoder_NowCycles() - entry_cycles;
    g_encoder_diagnostics.foc_read_cycles_latest = elapsed;
    if (elapsed > g_encoder_diagnostics.foc_read_cycles_max)
    {
      g_encoder_diagnostics.foc_read_cycles_max = elapsed;
    }
  }
  return valid;
}

void Encoder_GetDiagnostics(encoder_diagnostics_t *diagnostics)
{
  uint32_t mask;
  if (diagnostics == NULL)
  {
    return;
  }
  mask = Encoder_IrqSave();
  *diagnostics = g_encoder_diagnostics;
  Encoder_IrqRestore(mask);
}

void Encoder_RecordFocIsrCycles(uint32_t cycles)
{
  uint32_t period_cycles;

  if (s_encoder.initialized == 0U)
  {
    return;
  }
  g_encoder_diagnostics.foc_isr_cycles_latest = cycles;
  if (cycles > g_encoder_diagnostics.foc_isr_cycles_max)
  {
    g_encoder_diagnostics.foc_isr_cycles_max = cycles;
  }
  period_cycles = SystemCoreClock / ENCODER_CONTROL_RATE_HZ;
  if (cycles >= period_cycles)
  {
    g_encoder_diagnostics.foc_deadline_overrun_count++;
  }
}
