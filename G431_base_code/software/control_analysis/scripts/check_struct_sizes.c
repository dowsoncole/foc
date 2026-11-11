/* [千问工作助理 2026-10-28] 结构体尺寸实测，用于校正硬件脚本中硬编码的
   savemem 长度。
   背景：本轮给 mpt_t485_t 追加了 3 个 uint32_t 延迟统计字段、给
   output_position_test_t 追加了 5 个扫频字段。而
   test_mpt_position_sine.py:14 与 commission_mpt_position.py:15 把长度
   硬编码为 mpt=88 / trajectory=40，若不更新，savemem 会静默少读尾部字节，
   新增字段全部读不到——不报错，但 §8.5 的 M2/M3 实测会拿到假数据。
   本文件只打印尺寸，不修改任何工程状态。 */
#include <stdio.h>
#include <stddef.h>
#include "mpt_t485.h"
#include "output_position.h"

int main(void)
{
    printf("sizeof(mpt_t485_t)               = %u\n", (unsigned)sizeof(mpt_t485_t));
    printf("sizeof(output_position_t)        = %u\n", (unsigned)sizeof(output_position_t));
    printf("sizeof(output_position_test_t)   = %u\n", (unsigned)sizeof(output_position_test_t));
    printf("sizeof(output_position_trace_t)  = %u\n", (unsigned)sizeof(output_position_trace_t));
    printf("trace array bytes                = %u\n",
           (unsigned)(sizeof(output_position_trace_t) * OUTPUT_POSITION_TRACE_CAPACITY));
    /* 关键字段偏移，供脚本按字节寻址核对。 */
    printf("offsetof(mpt_t485_t, resp_latency_max_ms)     = %u\n",
           (unsigned)offsetof(mpt_t485_t, resp_latency_max_ms));
    printf("offsetof(mpt_t485_t, resp_latency_sum_ms)     = %u\n",
           (unsigned)offsetof(mpt_t485_t, resp_latency_sum_ms));
    printf("offsetof(mpt_t485_t, resp_latency_samples)    = %u\n",
           (unsigned)offsetof(mpt_t485_t, resp_latency_samples));
    printf("offsetof(output_position_test_t, frequency_hz)= %u\n",
           (unsigned)offsetof(output_position_test_t, frequency_hz));
    printf("offsetof(output_position_test_t, duration_s)  = %u\n",
           (unsigned)offsetof(output_position_test_t, duration_s));
    printf("offsetof(output_position_test_t, trace_interval_ms)   = %u\n",
           (unsigned)offsetof(output_position_test_t, trace_interval_ms));
    printf("offsetof(output_position_test_t, saturated_samples)   = %u\n",
           (unsigned)offsetof(output_position_test_t, saturated_samples));
    printf("offsetof(output_position_test_t, peak_motor_rpm)      = %u\n",
           (unsigned)offsetof(output_position_test_t, peak_motor_rpm));
    printf("offsetof(output_position_t, max_motor_rpm)    = %u\n",
           (unsigned)offsetof(output_position_t, max_motor_rpm));
    printf("offsetof(output_position_t, velocity_deg_s)   = %u\n",
           (unsigned)offsetof(output_position_t, velocity_deg_s));
    return 0;
}
