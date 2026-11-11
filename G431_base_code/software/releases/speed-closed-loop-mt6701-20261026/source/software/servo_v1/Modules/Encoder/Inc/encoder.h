#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "control_timing.h"

/* 0: production timing probe. 1: OFF-only fault capture. 2: commissioning
   capture with existing motor interlocks. Both capture modes boot WAIT and
   require CAP1 while OFF (main submits it automatically after boot calibration).
   Mode 2 permits MOE only while ARMED; the first
   non-CRC control fault removes MOE and latches feedback invalid until reset.
   Sensor STATUS alarms are recorded only and do not trigger this latch;
   their angles still must pass CRC and step-plausibility checks.
   Mode 2 records/drops CRC failures; old feedback is usable only until its
   original timestamp exceeds the configured age limit. */
#ifndef ENCODER_FAULT_TRIGGER_DIAGNOSTIC
#define ENCODER_FAULT_TRIGGER_DIAGNOSTIC 0
#endif
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC < 0 || ENCODER_FAULT_TRIGGER_DIAGNOSTIC > 2
#error "Unsupported encoder trigger profile"
#endif
#define ENCODER_CAPTURE_START_COMMAND 0x43415031UL /* CAP1, one shot per boot */
#define ENCODER_CAPTURE_WAITING 0U
#define ENCODER_CAPTURE_ARMED   1U
#define ENCODER_CAPTURE_FIRED   2U
typedef struct {
  uint32_t version, diagnostic_mode, command, state, rejected_commands;
  uint32_t reason, cycle, previous_angle, observed_zero, register_address;
  uint8_t rx[8];
} encoder_logic_trigger_t;
/* Only command is host-writable. Never clear a fired trigger or a guard in
   RAM. Preserve evidence, review the cause, then explicitly reset OFF. */
extern volatile encoder_logic_trigger_t g_encoder_logic_trigger;
/* Set only at the output-OFF mode-configuration boundary. */
extern volatile uint8_t g_encoder_feedback_required;

/* This is an additional commissioning gate, not a replacement for motor
   readiness, calibrated offsets, current limits or the run watchdog. */
static inline uint8_t Encoder_PowerOutputPermitted(void)
{
#if ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 1
  return 0U;
#elif ENCODER_FAULT_TRIGGER_DIAGNOSTIC == 2
  return !g_encoder_feedback_required ||
         g_encoder_logic_trigger.state == ENCODER_CAPTURE_ARMED;
#else
  return 1U;
#endif
}

#define ENCODER_CONTROL_RATE_HZ          CONTROL_FREQUENCY_HZ
#define ENCODER_SPEED_WINDOW_SAMPLES     CONTROL_SAMPLES_PER_MS
#define ENCODER_MAX_SAMPLE_AGE_CYCLES    10U /* Boot default: 1 ms at 10 kHz. */
#define ENCODER_TOLERANCE_MAX_CYCLES    10U
/* Zero intentional delay: start SPI/DMA directly in the TIM1 update ISR.
   CS still follows interrupt latency and transfer setup; this is not a
   hardware-synchronous edge. TIM16 is not used to schedule encoder sampling. */
#define ENCODER_SPI_SAMPLE_PHASE_US      0UL
#if ENCODER_SPI_SAMPLE_PHASE_US != 0
#error "Direct TIM1 update sampling requires a zero software phase"
#endif
/* Generous raw-step envelope above the 8500 rpm command / 9072 rpm trip. */
#define ENCODER_PLAUSIBILITY_MAX_RPM      12000.0f

typedef uint32_t encoder_fault_flags_t;

#define ENCODER_FAULT_NONE               0UL
#define ENCODER_FAULT_COMMUNICATION      (1UL << 0)
#define ENCODER_FAULT_CRC                (1UL << 1)
#define ENCODER_FAULT_SENSOR_ALARM       (1UL << 2)
#define ENCODER_FAULT_STALE              (1UL << 3)
#define ENCODER_FAULT_NOT_CONFIGURED     (1UL << 4)
#define ENCODER_FAULT_IMPLAUSIBLE         (1UL << 5)
#define ENCODER_FAULT_ZERO_CONFIG         (1UL << 6)
/* Legacy compatibility constant only: ZERO_POS is no longer read or checked.
   Zero-guard telemetry remains zero, meaning inactive, not verified. */
#define ENCODER_EXPECTED_SENSOR_ZERO      0U

typedef struct
{
  uint32_t raw_angle;
  int64_t multi_turn_count;
  float mechanical_angle_rad;
  float mechanical_speed_rad_s;
  float electrical_angle_rad;
  uint32_t sample_cycle;
  uint32_t sequence;
  encoder_fault_flags_t fault_flags;
  uint8_t status_raw;
  uint8_t angle_valid;
  uint8_t speed_valid;
  uint8_t electrical_angle_valid;
} encoder_snapshot_t;

typedef struct
{
  uint32_t trigger_count;
  uint32_t completed_count;
  uint32_t busy_trigger_count;
  uint32_t start_error_count;
  uint32_t transfer_error_count;
  uint32_t crc_error_count;
  uint32_t alarm_count;
  uint32_t foc_reads_while_busy;

  uint32_t transaction_cycles_latest;
  uint32_t transaction_cycles_min;
  uint32_t transaction_cycles_max;
  uint32_t start_cycles_latest;
  uint32_t start_cycles_max;
  uint32_t completion_cycles_latest;
  uint32_t completion_cycles_max;
  uint32_t foc_read_cycles_latest;
  uint32_t foc_read_cycles_max;
  uint32_t foc_sample_age_cycles_latest;
  uint32_t foc_sample_age_cycles_max;
  uint32_t foc_isr_cycles_latest;
  uint32_t foc_isr_cycles_max;
  uint32_t foc_deadline_overrun_count;
  uint32_t spi_resync_count;
  uint32_t spi_resync_refused_count;
  uint32_t spi_resync_cycles_latest;
  uint32_t spi_resync_cycles_max;
  uint32_t angle_step_error_count;
  uint32_t trigger_interval_latest, trigger_interval_min, trigger_interval_max;
  uint32_t last_busy_cycle, last_busy_spi_sr;
} encoder_diagnostics_t;

/* 调试器 Watch 只读镜像；业务代码仍使用 ReadSnapshot/GetDiagnostics，禁止
   从外部直接修改。 */
extern volatile encoder_snapshot_t g_encoder_snapshot;
extern volatile encoder_diagnostics_t g_encoder_diagnostics;

typedef struct {
  uint32_t cycle, reason, previous_angle, spi_status, previous_cycle;
  uint8_t rx[6], computed_crc, status;
} encoder_fault_frame_t;
/* Updated only on bad CRC/alarm/transport frames. First frame is frozen;
   the eight-slot ring and last frame retain later evidence without HALT. */
typedef struct {
  uint32_t count;
  encoder_fault_frame_t first, last, recent[8], last_alarm;
} encoder_fault_capture_t;
extern volatile encoder_fault_capture_t g_encoder_fault_capture;

typedef struct {
  uint32_t expected_zero, observed_zero, verified_mask, latched;
  uint32_t checks, read_errors, mismatches, last_check_cycle;
} encoder_zero_guard_t;
extern volatile encoder_zero_guard_t g_encoder_zero_guard;

/* Diagnostic evidence, NOT a claim that the sensor register really changed:
   single-register SPI reads have no CRC. Keep the original guard ABI intact. */
#define ENCODER_ZERO_EVENT_MISMATCH       1U
#define ENCODER_ZERO_EVENT_TRANSPORT      2U
typedef struct {
  uint32_t cycle, transfer_start_cycle, spi_status;
  uint32_t previous_zero, observed_zero, expected_zero;
  uint8_t register_address, hal_status, reason, reserved;
  uint8_t rx[3], reserved2;
} encoder_zero_fault_frame_t;
typedef struct {
  uint32_t sequence, count; /* Writer odd/even sequence; first event is frozen. */
  encoder_zero_fault_frame_t first, last, recent[8];
} encoder_zero_fault_capture_t;
extern volatile encoder_zero_fault_capture_t g_encoder_zero_fault_capture;

/* 初始化 SPI/DMA、时间测量和连续采集状态。初始化后角度采集自动由
   TIM1 周期边界触发，但电角度在 ConfigureMotor 成功前保持无效。 */
uint8_t Encoder_Init(void);

/* Application calls only while OFF before starting a speed run. Range 1..10
   carrier periods. Does not refresh timestamps, clear faults or start SPI.
   The active value is s_feedback_tolerance_cycles (read-only in Watch). */
uint8_t Encoder_ConfigureFeedbackTolerance(uint32_t carrier_periods);

/* 上层在功率输出关闭时配置一次。direction 只能为 +1/-1；pole_pairs 必须
   大于 0；offset 是经过标定的电角零位，单位 rad。配置会重置速度历史。 */
uint8_t Encoder_ConfigureMotor(int8_t direction, uint16_t pole_pairs,
                               float electrical_offset_rad);

/* TIM1 更新中断直接调用：立即启动 SPI/DMA，不等待传输完成，不使用 TIM16 延时。 */
void Encoder_OnPwmPeriodIsr(void);

/* 后台超时恢复。主循环每轮调用，不执行正常角度采样。 */
void Encoder_Service(void);

/* 上层统一读取接口。返回 1 表示快照当前有效；各子字段仍有独立有效位。 */
uint8_t Encoder_ReadSnapshot(encoder_snapshot_t *snapshot);

/* FOC 快路径：只复制已发布电角度并检查新鲜度，不发起 SPI 通信。 */
uint8_t Encoder_ReadElectricalAngleForFoc(float *electrical_angle_rad);

/* 诊断与实测接口。cycles / SystemCoreClock 即秒。 */
void Encoder_GetDiagnostics(encoder_diagnostics_t *diagnostics);
void Encoder_RecordFocIsrCycles(uint32_t cycles);

#endif /* ENCODER_H */
