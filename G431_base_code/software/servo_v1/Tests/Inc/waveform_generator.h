#ifndef WAVEFORM_GENERATOR_H
#define WAVEFORM_GENERATOR_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RS485 test stream: 0xA5 0x5A, type, sequence, signed sample, CRC8. */
#define WAVEFORM_RS485_BAUDRATE       921600U
#define WAVEFORM_SAMPLE_RATE_HZ       1000U
#define WAVEFORM_SOURCE_RATE_HZ       20000U
#define WAVEFORM_DECIMATION           (WAVEFORM_SOURCE_RATE_HZ / WAVEFORM_SAMPLE_RATE_HZ)

HAL_StatusTypeDef WaveformGenerator_Init(UART_HandleTypeDef *huart);

/* Call once for every current-loop sample (ISR-safe, no peripheral access). */
void WaveformGenerator_SampleTick(void);

/* Call from the main loop; sends at most one queued frame per invocation. */
void WaveformGenerator_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* WAVEFORM_GENERATOR_H */
