#ifndef MT6701_SPI3_TEST_H
#define MT6701_SPI3_TEST_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 临时测试默认启用。置 0 可在编译时完全移除 main 中的初始化和调度。 */
#ifndef MT6701_SPI3_TEST_ENABLE
#define MT6701_SPI3_TEST_ENABLE 1U
#endif

typedef struct
{
  /* 调试器可写：enabled=0 暂停；poll_period_ms 默认 10，0 按 1 ms 处理。 */
  uint32_t enabled;
  uint32_t poll_period_ms;
  uint32_t initialized;
  uint32_t last_hal_status;
  uint32_t last_hal_error;
  uint32_t frame_count;           /* 发起收发的次数，含 HAL 错误 */
  uint32_t good_frame_count;      /* CRC 正确且非全零/全一，与磁场状态无关 */
  uint32_t spi_error_count;
  uint32_t crc_error_count;
  uint32_t blank_frame_count;
  uint32_t status_warning_count;  /* 有效帧中 Mg[3:0] 非零的次数 */
  uint32_t angle_change_count;    /* 相邻有效帧角度发生变化的次数 */
  uint32_t consecutive_good;
  uint32_t sample_valid;          /* 最近一次帧 CRC 正确且非空帧 */
  uint32_t communication_ok;      /* 连续 3 帧有效；遇错或暂停立即清零 */
  uint32_t last_sample_tick_ms;
  uint32_t last_good_tick_ms;
  uint32_t raw_frame;             /* [23:10] angle, [9:6] Mg, [5:0] CRC */
  uint32_t angle_raw;             /* 最近一次有效角度，0..16383 */
  float angle_deg;                /* 最近一次有效角度，0..360 度 */
  uint8_t rx[3];                  /* 最近一次 HAL_OK 的原始数据，高字节先 */
  uint8_t status;                 /* Mg[1:0]: 0正常/1过强/2过弱/3保留 */
  uint8_t magnetic_field_status;
  uint8_t push_detected;          /* Mg[2] */
  uint8_t loss_of_track;          /* Mg[3]，新版中文手册称超速 */
  uint8_t crc_received;
  uint8_t crc_calculated;
} mt6701_spi3_test_t;

extern volatile mt6701_spi3_test_t g_mt6701_spi3_test;

/* 初始化失败保留在观测变量中，由 Task 按周期重试，不阻断主程序。 */
HAL_StatusTypeDef Mt6701Spi3Test_Init(void);
/* 主循环调用；不得从控制中断调用。默认每 10 ms 阻塞读一帧，超时 2 ms。 */
void Mt6701Spi3Test_Task(void);

#ifdef __cplusplus
}
#endif
#endif
