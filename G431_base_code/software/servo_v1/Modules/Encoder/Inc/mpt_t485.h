#ifndef MPT_T485_H
#define MPT_T485_H
#include <stdint.h>
/* USART1 belongs exclusively to the output encoder in this build. */
#ifndef OUTPUT_AXIS_ENABLE
#define OUTPUT_AXIS_ENABLE 1U
#endif
/* --------------------------------------------------------------------------
   [千问工作助理 2026-10-28] 位置环反馈延迟优化
   --------------------------------------------------------------------------
   改动内容：OUTPUT_POSITION_PERIOD_MS 由 5U 改为 1U；新增响应超时宏与
             反馈新鲜度实测统计字段。

   改动依据（原 5 ms 是软件选择，不是硬件极限）：
   1. MPT 数据手册「Absolute Parameters」：Update Rate = 50 kHz，即器件内部
      每 20 us 更新一次单圈绝对角度（数据手册解析件 MPT_datasheet.md:612-613）。
   2. RS485 单次往返线时间 = (1 命令字节 + 11 响应字节) * 10 bit / 2.5 Mbaud
      = 48 us，仅占 1 ms 周期的 4.8%，总线占用率极低。
   3. 原代码中的 2 ms 是「响应超时保护门限」，不是器件实际响应时间，
      不应被当作轮询周期的下限。

   为何必须改：位置环开环相位 = 积分器(-90°) + 反馈延迟 + 速度环滞后。
   5 ms 轮询时总延迟约 8 ms，在 20 Hz 处引入 57.6° 滞后，叠加速度环实测
   -21.1°（balanced_matlab_measured_evaluation.json:247,268），相位裕度仅
   11.3°，远低于验收要求 >=45°，可达交越频率被压到约 10.6 Hz。
   降到 1 ms 后总延迟约 1.5 ms，20 Hz 处相位裕度升到约 50.9°，
   20 Hz 交越在架构上可行（详见调试日志第 4.4、8 节推导）。

   安全性：反馈过期门限沿用「3 个周期 + 2 ms」结构，周期缩短后门限随之
   收紧到 5 ms，仍为轮询周期的 5 倍、线时间的 100 倍以上，不会重蹈高速
   工况下新鲜度预算过紧导致必然跳闸的问题。原有停机保护全部保留。
   -------------------------------------------------------------------------- */
#define OUTPUT_POSITION_PERIOD_MS 1U
/* [千问工作助理 2026-10-28] 器件响应超时门限，与轮询周期解耦。
   线时间 48 us，2 ms 门限已有约 40 倍余量，保持不变。 */
#define OUTPUT_POSITION_RESPONSE_TIMEOUT_MS 2U
#define OUTPUT_POSITION_MAX_AGE_MS (3U * OUTPUT_POSITION_PERIOD_MS + 2U)
#define MPT_COUNTS_PER_TURN 131072UL
#define MPT_FRAME_LENGTH 11U
typedef struct {
  uint32_t initialized, enabled, requests, good_frames, crc_errors;
  uint32_t format_errors, status_errors, timeouts, uart_errors;
  uint32_t consecutive_good, valid, last_good_ms, sequence;
  uint32_t angle_raw, turns_raw, status, alarm;
  float angle_deg;
  uint8_t frame[16];
  /* [千问工作助理 2026-10-28] 追加于结构体末尾，保持以上字段偏移不变。
     实测器件响应延迟，用于取代「按 2 ms 超时门限估算延迟」的假设，
     并作为位置环相位预算的实测依据。resp_latency_max_ms 为本次上电以来
     的最大值，resp_latency_sum_ms/samples 可求平均。 */
  uint32_t resp_latency_max_ms, resp_latency_sum_ms, resp_latency_samples;
  /* Codex: DWT timestamps resolve the latency hidden by 1 ms HAL ticks.
     max includes startup/precharge stalls; last is the latest completed frame. */
  uint32_t resp_latency_last_us, resp_latency_max_us;
} mpt_t485_t;
extern volatile mpt_t485_t g_mpt_t485;
/* 0=valid, 1=CRC, 2=format, 3=status. Never sends encoder reset commands. */
uint32_t MptT485_Decode(const uint8_t *frame, uint32_t length,
                       uint32_t *angle, uint32_t *turns);
uint8_t MptT485_Init(void);
void MptT485_Task(void);
#endif
