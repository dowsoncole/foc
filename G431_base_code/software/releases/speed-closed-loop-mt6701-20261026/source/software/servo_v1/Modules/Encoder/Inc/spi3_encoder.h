#ifndef SPI3_ENCODER_H
#define SPI3_ENCODER_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 临时 MT6701 SSI 配置：PB6=软件 CSN，PB3=SPI3_SCK，PB4=SPI3_MISO。
   不需要 MOSI，不配置 PB5。SPI3 使用 APB1，当前 168 MHz / 256 = 656.25 kHz。
   Mode 1：CLK 空闲低、上升沿由芯片更新数据、下降沿采样，24 个时钟一帧。
   禁用硬件 CRC 和 NSS pulse，由测试层校验 SSI 帧内的 CRC6。 */
#define MT6701_SPI3_CSN_PIN           GPIO_PIN_6
#define MT6701_SPI3_CSN_PORT          GPIOB
#define MT6701_SPI3_FRAME_BYTES       3U
#define MT6701_SPI3_PRESCALER         SPI_BAUDRATEPRESCALER_256

extern SPI_HandleTypeDef hspi3;

/* 允许在传输失败后重新调用；先释放 CSN，再复位 SPI3，清除残留 FIFO/状态。 */
HAL_StatusTypeDef MX_SPI3_Encoder_Init(void);
/* 仅从主循环调用，保持中断开启。发送虚拟字节只为产生精确的 24 个时钟。
   HAL_OK 仅表示主机收发完成，芯片是否回应由测试层的 CRC/空帧判断。 */
HAL_StatusTypeDef SPI3_Encoder_ReadFrame(uint8_t frame[MT6701_SPI3_FRAME_BYTES],
                                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
