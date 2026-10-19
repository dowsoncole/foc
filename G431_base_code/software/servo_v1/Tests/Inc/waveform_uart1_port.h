#ifndef WAVEFORM_UART1_PORT_H
#define WAVEFORM_UART1_PORT_H

#include "stm32g4xx_hal.h"
#include "waveform_generator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WAVEFORM_UART1_BAUDRATE  921600U

/* 当前板上 USART1 经过 PA8 控制的 485 收发器；该细节仅存在于端口适配层。 */
HAL_StatusTypeDef WaveformUart1Port_Init(waveform_transport_t *transport,
                                        UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* WAVEFORM_UART1_PORT_H */
