#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_CONTROL_RATE_HZ          20000UL
#define ENCODER_SPEED_WINDOW_SAMPLES     20U
#define ENCODER_MAX_SAMPLE_AGE_CYCLES    3U

typedef uint32_t encoder_fault_flags_t;

#define ENCODER_FAULT_NONE               0UL
#define ENCODER_FAULT_COMMUNICATION      (1UL << 0)
#define ENCODER_FAULT_CRC                (1UL << 1)
#define ENCODER_FAULT_SENSOR_ALARM       (1UL << 2)
#define ENCODER_FAULT_STALE              (1UL << 3)
#define ENCODER_FAULT_NOT_CONFIGURED     (1UL << 4)

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
} encoder_diagnostics_t;

/* 调试器 Watch 只读镜像；业务代码仍使用 ReadSnapshot/GetDiagnostics，禁止
   从外部直接修改。 */
extern volatile encoder_snapshot_t g_encoder_snapshot;
extern volatile encoder_diagnostics_t g_encoder_diagnostics;

/* 初始化 SPI/DMA、时间测量和连续采集状态。初始化后角度采集自动由
   TIM1 周期边界触发，但电角度在 ConfigureMotor 成功前保持无效。 */
uint8_t Encoder_Init(void);

/* 上层在功率输出关闭时配置一次。direction 只能为 +1/-1；pole_pairs 必须
   大于 0；offset 是经过标定的电角零位，单位 rad。配置会重置速度历史。 */
uint8_t Encoder_ConfigureMotor(int8_t direction, uint16_t pole_pairs,
                               float electrical_offset_rad);

/* TIM1 周期回调入口。只启动 DMA，不等待 SPI 完成。 */
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
