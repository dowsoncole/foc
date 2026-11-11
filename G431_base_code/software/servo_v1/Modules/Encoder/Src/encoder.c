#include "encoder.h"
#include "encoder_frame_guard.h"

#include "mt6835.h"
#include "mt6701_ssi.h"
#include "spi1_encoder.h"
#include "timing_probes.h"

#include <math.h>
#include <string.h>

#define ENCODER_TWO_PI                 6.28318530717958647692f
#define ENCODER_RAD_PER_COUNT          (ENCODER_TWO_PI / (float)MT6835_ANGLE_FULL_SCALE)
#define ENCODER_TRANSFER_TIMEOUT_US    100UL
/* Velocity-loop commissioning: retain the 1 ms angle-difference window,
   but avoid adding a further 10 ms feedback lag. Raw angle validation and
   freshness limits are independent and unchanged. */
#define ENCODER_SPEED_FILTER_TAU_S     0.001f

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
  uint32_t last_speed_cycle;
  volatile uint32_t transfer_start_cycle;
  uint32_t max_counts_per_256_cycles;
  uint32_t previous_trigger_cycle;
  /* 调试镜像：最近一次完成帧的 CRC 计算值与一致性。
     last_computed_crc = 由 rx[2..4] 重算的 CRC-8；
     last_crc_match = 1 表示与传感器返回的 rx[5] 一致。 */
  uint8_t last_computed_crc;
  uint8_t last_crc_match;
} encoder_context_t;

 encoder_context_t s_encoder;
volatile encoder_snapshot_t g_encoder_snapshot;
volatile uint8_t g_encoder_feedback_required = 1U;
volatile encoder_diagnostics_t g_encoder_diagnostics;
volatile encoder_fault_capture_t g_encoder_fault_capture;
/* Inactive legacy telemetry retained for debugger/host layout compatibility. */
volatile encoder_zero_guard_t g_encoder_zero_guard;
volatile encoder_zero_fault_capture_t g_encoder_zero_fault_capture;
/* User-fixed default for capture profiles. Encoder_Init clears diagnostic
   records; main re-submits CAP1 after calibration initialization. Command is
   consumed once, never held/re-written by the loop after a fault. */
volatile encoder_logic_trigger_t g_encoder_logic_trigger = {
  .command = ENCODER_FAULT_TRIGGER_DIAGNOSTIC ? ENCODER_CAPTURE_START_COMMAND : 0U
};
static volatile uint32_t s_feedback_tolerance_cycles = ENCODER_MAX_SAMPLE_AGE_CYCLES;
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
static volatile uint8_t s_logic_capture_started;
#endif

static void Encoder_LogicTrigger(uint32_t reason, uint32_t cycle)
{
  /* Independent of capture profile/state: include failed starts and aborts. */
  if ((reason & ENCODER_FAULT_COMMUNICATION) != 0U)
    TimingProbe_EncoderCommunicationError();
  /* Sensor STATUS alarms are telemetry-only by user configuration. */
  if (reason == ENCODER_FAULT_SENSOR_ALARM) return;
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
  /* CRC evidence is still recorded by Encoder_CaptureFault. Never publish
     this frame or renew the last good sample's timestamp. FOC stops when
     that sample exceeds the configured age; other first faults still latch. */
  if (reason == ENCODER_FAULT_CRC) return;
#endif
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED)
  {
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
    /* First-event commissioning stop for faults other than a dropped CRC frame.
       Interrupt masking also serializes this with the MOE release gate. */
    if (g_encoder_feedback_required)
      TIM1->BDTR &= ~TIM_BDTR_MOE;
#endif
    g_encoder_logic_trigger.reason = reason;
    g_encoder_logic_trigger.cycle = cycle;
    g_encoder_logic_trigger.previous_angle = s_encoder.previous_raw_angle;
    g_encoder_logic_trigger.observed_zero = 0U; /* Legacy field; no register polling. */
    g_encoder_logic_trigger.register_address = 0U;
    for (uint32_t i = 0U; i < 6U; i++)
      g_encoder_logic_trigger.rx[i] = s_encoder.rx[i];
    /* Record the first detection in RAM; GPIOs are reserved for timing. */
    g_encoder_logic_trigger.state = ENCODER_CAPTURE_FIRED;
  }
  if (!mask) __enable_irq();
#else
  (void)reason;
  (void)cycle;
#endif
}

static void Encoder_CaptureFault(uint32_t reason)
{
  encoder_fault_frame_t f={0};
  f.cycle=DWT->CYCCNT;
  Encoder_LogicTrigger(reason, f.cycle);
  f.reason=reason;
  f.previous_angle=s_encoder.previous_raw_angle;
  f.previous_cycle=g_encoder_snapshot.sample_cycle;
  f.spi_status=SPI1->SR;
  for (uint32_t i=0;i<6U;i++) f.rx[i]=s_encoder.rx[i];
#if MT6701_SSI_ENABLE
  f.computed_crc = g_mt6701_ssi.crc_calculated;
  f.status = g_mt6701_ssi.status;
#else
  f.computed_crc=MT6835_CalcCrc24(((uint32_t)f.rx[2]<<16)|
                                ((uint32_t)f.rx[3]<<8)|f.rx[4]);
  f.status=f.rx[4]&MT6835_ANGLE_L_STATUS_MASK;
#endif
  uint32_t n=g_encoder_fault_capture.count;
  if (!n) g_encoder_fault_capture.first=f;
  if (reason==ENCODER_FAULT_SENSOR_ALARM) g_encoder_fault_capture.last_alarm=f;
  g_encoder_fault_capture.last=f;
  g_encoder_fault_capture.recent[n&7U]=f;
  g_encoder_fault_capture.count=n+1U;
}

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
         s_feedback_tolerance_cycles;
}

uint8_t Encoder_ConfigureFeedbackTolerance(uint32_t carrier_periods)
{
  if (carrier_periods == 0U || carrier_periods > ENCODER_TOLERANCE_MAX_CYCLES)
    return 0U;
  uint32_t mask = Encoder_IrqSave();
  if (TIM1->BDTR & TIM_BDTR_MOE)
  {
    Encoder_IrqRestore(mask);
    return 0U;
  }
  s_feedback_tolerance_cycles = carrier_periods;
  Encoder_IrqRestore(mask);
  return 1U;
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
    s_encoder.last_speed_cycle = sample_cycle;
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
      /* Differencing window and filter update interval are NOT the same. */
      float update_dt = (float)(sample_cycle-s_encoder.last_speed_cycle) /
                        (float)SystemCoreClock;
      s_encoder.last_speed_cycle = sample_cycle;
      float alpha = update_dt / (ENCODER_SPEED_FILTER_TAU_S + update_dt);
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

static void Encoder_ResyncAfterBadFrame(void)
{
  uint32_t start = Encoder_NowCycles();
  if (MT6835_SPI_ResyncIdle() == HAL_OK)
    g_encoder_diagnostics.spi_resync_count++;
  else
    g_encoder_diagnostics.spi_resync_refused_count++;
  uint32_t cycles = Encoder_NowCycles() - start;
  g_encoder_diagnostics.spi_resync_cycles_latest = cycles;
  if (cycles > g_encoder_diagnostics.spi_resync_cycles_max)
    g_encoder_diagnostics.spi_resync_cycles_max = cycles;
}

static void Encoder_TransferComplete(HAL_StatusTypeDef status, void *context)
{
  TimingProbe_EncoderDmaCallbackBegin();
  mt6835_angle_t frame;
  uint32_t entry_cycles = Encoder_NowCycles();
  /* PB7 is low for this complete callback, including decode/publication. */
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


#if MT6701_SSI_ENABLE
  g_mt6701_ssi.last_hal_error = hspi1.ErrorCode;
  Mt6701Ssi_ProcessFrame(s_encoder.rx, status);
  /* Keep the common feedback ABI in 21-bit counts; MT6701 resolution remains
     14 bits. Scaling by 128 preserves existing speed, unwrap and step guards. */
  frame.angle_raw = g_mt6701_ssi.angle_raw << 7U;
  frame.status.raw = g_mt6701_ssi.status;
  frame.crc_ok = g_mt6701_ssi.sample_valid;
  s_encoder.last_computed_crc = g_mt6701_ssi.crc_calculated;
  s_encoder.last_crc_match = frame.crc_ok;
#else
  /* 实时刷新本帧 CRC 诊断镜像（不改变控制路径，供 Watch 观察）。 */
  s_encoder.last_computed_crc = MT6835_CalcCrc24(
      ((uint32_t)s_encoder.rx[2] << 16) |
      ((uint32_t)s_encoder.rx[3] << 8) |
      (uint32_t)s_encoder.rx[4]);
  s_encoder.last_crc_match =
      (s_encoder.last_computed_crc == s_encoder.rx[5]) ? 1U : 0U;

#endif
  if (status != HAL_OK)
  {
    Encoder_CaptureFault(ENCODER_FAULT_COMMUNICATION);
    g_encoder_diagnostics.transfer_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_COMMUNICATION, 0U);
  }
#if !MT6701_SSI_ENABLE
  else if (MT6835_DecodeAngleResponse(s_encoder.rx, &frame) != HAL_OK)
  {
    Encoder_CaptureFault(ENCODER_FAULT_COMMUNICATION);
    g_encoder_diagnostics.transfer_error_count++;
    Encoder_PublishInvalid(ENCODER_FAULT_COMMUNICATION, 0U);
  }
#endif
  else if (frame.crc_ok == 0U)
  {
    TimingProbe_EncoderCrcError();
    Encoder_CaptureFault(ENCODER_FAULT_CRC);
    g_encoder_diagnostics.crc_error_count++;
    /* A persistent master-side bad-frame state was recovered by an SPI1 RCC
       reset in an OFF-only hardware comparison. Resync at this idle boundary
       so one CRC failure cannot poison all later transactions. Bad data is
       still rejected; only the next CRC-valid, plausible frame may replace
       the previous sample, within the UNCHANGED freshness budget below. */
    Encoder_ResyncAfterBadFrame();
    /* Never publish corrupt data. A single dropped frame may use the LAST
       validated angle only within the configured age budget. Its original
       timestamp/sequence are preserved, so repeated errors expire normally. */
    if (!g_encoder_snapshot.angle_valid ||
        (Encoder_NowCycles()-g_encoder_snapshot.sample_cycle)>Encoder_MaxAgeCycles())
      Encoder_PublishInvalid(ENCODER_FAULT_CRC, frame.status.raw);
  }
  else if (s_encoder.have_previous_angle &&
           !Encoder_RawStepPlausible(s_encoder.previous_raw_angle, frame.angle_raw,
               s_encoder.transfer_start_cycle - g_encoder_snapshot.sample_cycle,
               s_encoder.max_counts_per_256_cycles))
  {
    /* CRC8 alone can accept a corrupt payload. Never feed a physically
       impossible angle step into Park/current PI or refresh its timestamp. */
    Encoder_CaptureFault(ENCODER_FAULT_IMPLAUSIBLE);
    g_encoder_diagnostics.angle_step_error_count++;
    Encoder_ResyncAfterBadFrame();
    if (!g_encoder_snapshot.angle_valid ||
        (Encoder_NowCycles() - g_encoder_snapshot.sample_cycle) > Encoder_MaxAgeCycles())
      Encoder_PublishInvalid(ENCODER_FAULT_IMPLAUSIBLE, frame.status.raw);
  }
  else
  {
    /* Accept CRC-valid, plausible angles even when STATUS reports an alarm.
       Preserve raw status and alarm evidence without disabling PWM. */
    if ((frame.status.raw & MT6835_STATUS_ALARM_MASK) != 0U)
    {
      Encoder_CaptureFault(ENCODER_FAULT_SENSOR_ALARM);
      g_encoder_diagnostics.alarm_count++;
    }
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
  /* Release ownership only after frame processing, before ending PB7. */
  {
    uint32_t mask=Encoder_IrqSave();
    s_encoder.transfer_busy = 0U;
    Encoder_IrqRestore(mask);
  }
  TimingProbe_EncoderDmaCallbackEnd();
}

uint8_t Encoder_Init(void)
{
  encoder_diagnostics_t empty_diagnostics = {0};
  s_feedback_tolerance_cycles = ENCODER_MAX_SAMPLE_AGE_CYCLES;

  (void)memset((void *)&g_encoder_logic_trigger, 0, sizeof(g_encoder_logic_trigger));
  g_encoder_logic_trigger.version = 1U;
  g_encoder_logic_trigger.diagnostic_mode = ENCODER_FAULT_TRIGGER_DIAGNOSTIC;
  TimingProbe_EncoderEnd();
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  s_logic_capture_started = 0U;
#endif

  (void)memset(&s_encoder, 0, sizeof(s_encoder));
  (void)memset((void *)&g_encoder_zero_guard,0,sizeof(g_encoder_zero_guard));
  (void)memset((void *)&g_encoder_zero_fault_capture,0,
               sizeof(g_encoder_zero_fault_capture));
  g_encoder_diagnostics = empty_diagnostics;
  s_encoder.direction = 1;
  s_encoder.max_counts_per_256_cycles = (uint32_t)(
      ((float)ENCODER_GUARD_COUNTS_PER_REV * (ENCODER_PLAUSIBILITY_MAX_RPM / 60.0f) * 256.0f) /
      (float)SystemCoreClock) + 1U;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (MX_SPI1_Encoder_Init() != HAL_OK)
  {
    return 0U;
  }
#if MT6701_SSI_ENABLE
  memset((void *)&g_mt6701_ssi, 0, sizeof(g_mt6701_ssi));
  g_mt6701_ssi.enabled = 1U;
  g_mt6701_ssi.initialized = 1U;
  /* Zero dummy TX, three bytes; no SSI command phase. */
#else
  MT6835_PowerUpWait();
  MT6835_BuildAngleReadCommand(s_encoder.tx);
#endif
  Encoder_ResetTracking();
  /* TIM1 update now starts SPI directly. Quiesce the formerly used TIM16
     source without disabling the NVIC vector shared with TIM1. No phase
     countdown or phase-minus-one expression exists in the zero-delay path. */
  __HAL_RCC_TIM16_CLK_ENABLE();
  TIM16->CR1 = 0U;
  TIM16->DIER = 0U;
  TIM16->SR = 0U;
  TIM16->CNT = 0U;
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

  /* Transport runs in every initialized PWM slot, independently of output
     enable, control mode and diagnostic arming/fault state. */
  if (s_encoder.initialized == 0U)
  {
    return;
  }

  uint32_t trigger_cycle = Encoder_NowCycles();
  if (s_encoder.previous_trigger_cycle)
  {
    uint32_t gap = trigger_cycle - s_encoder.previous_trigger_cycle;
    g_encoder_diagnostics.trigger_interval_latest = gap;
    if (!g_encoder_diagnostics.trigger_interval_min || gap < g_encoder_diagnostics.trigger_interval_min)
      g_encoder_diagnostics.trigger_interval_min = gap;
    if (gap > g_encoder_diagnostics.trigger_interval_max)
      g_encoder_diagnostics.trigger_interval_max = gap;
  }
  s_encoder.previous_trigger_cycle = trigger_cycle;
  g_encoder_diagnostics.trigger_count++;
  if (s_encoder.transfer_busy != 0U)
  {
    g_encoder_diagnostics.busy_trigger_count++;
    g_encoder_diagnostics.last_busy_cycle = trigger_cycle;
    g_encoder_diagnostics.last_busy_spi_sr = SPI1->SR;
    return;
  }

  entry_cycles = Encoder_NowCycles();
  /* transaction_cycles starts here; PD2 only marks the TIM1 update ISR. */
  s_encoder.transfer_start_cycle = entry_cycles;
  s_encoder.transfer_busy = 1U;
  /* Every carrier slot reads a CRC-protected angle frame; no ZERO_POS polling. */
  status = MT6835_SPI_TransmitReceiveAsync(
      s_encoder.tx, s_encoder.rx,
#if MT6701_SSI_ENABLE
      3U,
#else
      sizeof(s_encoder.tx),
#endif
      Encoder_TransferComplete, NULL);
#if (MT6835_USE_SOFTWARE_SPI != 0U)
  /* Optional synchronous bit-bang backend has no DMA completion IRQ. */
  TimingProbe_EncoderEnd();
#endif
  if (status != HAL_OK)
  {
    /* A rejected start has no callback to end the pulse. */
    TimingProbe_EncoderEnd();
    s_encoder.transfer_busy = 0U;
    Encoder_LogicTrigger(ENCODER_FAULT_COMMUNICATION, Encoder_NowCycles());
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

#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC
  uint32_t command = g_encoder_logic_trigger.command;
  if (command != 0U)
  {
    g_encoder_logic_trigger.command = 0U;
    if (command == ENCODER_CAPTURE_START_COMMAND &&
        g_encoder_logic_trigger.state == ENCODER_CAPTURE_WAITING &&
        !s_logic_capture_started && s_encoder.initialized &&
        !s_encoder.transfer_busy && !(TIM1->BDTR & TIM_BDTR_MOE))
    {
      g_encoder_logic_trigger.state = ENCODER_CAPTURE_ARMED;
      s_logic_capture_started = 1U;
    }
    else g_encoder_logic_trigger.rejected_commands++;
  }
#endif

  if ((s_encoder.initialized != 0U) && (s_encoder.transfer_busy != 0U) &&
      ((Encoder_NowCycles() - s_encoder.transfer_start_cycle) >
       Encoder_TransferTimeoutCycles()))
  {
    /* 异常路径保持短临界区，防止下一次 TIM1 触发与 abort 交叉。Abort
       回调统一记录通信错误并撤销快照有效性。 */
    (void)MT6835_SPI_AbortAsync();
    /* No DMA IRQ return follows a foreground abort. Retain the legacy
       transaction-end hook for host compatibility. */
    if (!s_encoder.transfer_busy) TimingProbe_EncoderEnd();
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

#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
  if (g_encoder_logic_trigger.state == ENCODER_CAPTURE_FIRED)
  {
    snapshot->fault_flags |= g_encoder_logic_trigger.reason;
    snapshot->angle_valid = snapshot->speed_valid = snapshot->electrical_angle_valid = 0U;
  }
#endif

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
