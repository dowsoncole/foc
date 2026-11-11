/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi1_encoder.h
 * @brief   MT6835 磁编码器 SPI 配置及收发抽象（L1 配置层）。
  *
  *          引脚映射（STM32G431VB，LQFP100；数据手册 DS12589 Table 13，
  *          四脚的 SPI1 复用全部在 AF5，无歧义）：
  *            PA4 -> 软件片选 CSN（普通推挽输出，不用硬件 NSS）
  *            PA5 -> SPI1_SCK
 *            硬件 SPI 模式：PA6 -> MISO，PA7 -> MOSI
 *            软件 SPI：默认与硬件 SPI 同映射；历史交叉接线须显式选择
  *
  *          为什么片选用软件 GPIO 而不用硬件 NSS：
  *            1. 普通寄存器事务是 24 bit，角度 burst 事务是 48 bit；
  *               CSN 必须覆盖整个事务，不能在各 8 bit 字节间脉冲释放；
  *            2. 手册要求每帧「CSN 下降沿开始、上升沿结束」，且 CSN 下降沿会
  *               锁存角度寄存器 0x003~0x006（手册 §7.6.9），软件片选才能精确
  *               控制锁存时刻。
  *            PA4 兼作 DAC1_OUT1 / ADC2_IN17，但只要不使能 DAC1 通道 1，
  *            它就是干净的数字 GPIO；上电复位后为浮空输入，本文件会显式
  *            配成输出并立即置高（CSN 空闲高）。
  *
  *          SPI 工作模式由 MT6835 决定（手册 §7.6.2）：模式 3（CPOL=1, CPHA=1），
  *          数据在 SCK 下降沿变化、上升沿采样；SCK 上限 16 MHz。本配置取
  *          APB2 168 MHz 的 32 分频 = 5.25 MHz；输出边沿配置由
  *          SPI1_ENC_OUTPUT_GPIO_SPEED 选择，不改变 SCK 分频。
  *          启动可靠性和最高转速仍需分别实测验证。
  *
  *          本文件只配置外设并交出句柄，不知道 MT6835 的寄存器语义；
  *          器件读写由 L2 的 mt6835.c 完成。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SPI1_ENCODER_H__
#define __SPI1_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "mt6701_ssi.h"

/* --------------------------------------------------------------------------*/
/* 硬件常量                                                                  */
/* --------------------------------------------------------------------------*/

/* Diagnostic selection: GPIO SPI uses the SAME normal pin mapping by default.
   Hardware SPI remains the delivery default after the bounded comparison. */
#define MT6835_USE_SOFTWARE_SPI       0U
#define MT6835_SOFT_SPI_CROSSWIRED     0U

/* SPI1 挂在 APB2；APB2 不分频时 SPI1 内核时钟 = SYSCLK = 168 MHz。 */
#define SPI1_ENC_PERIPHERAL_CLOCK_HZ   168000000UL

/* SPI only supports power-of-two divisors. 168 MHz / 32 = 5.25 MHz is the
   closest available setting to the requested 5 MHz.
   A 24-clock MT6701 SSI frame takes 4.571 us at this SCK frequency.
   CRC, sensor alarms and the configured feedback-age limit remain active. */
#define SPI1_ENC_BAUDRATE_PRESCALER    SPI_BAUDRATEPRESCALER_32
#define SPI1_ENC_SCK_HZ                (SPI1_ENC_PERIPHERAL_CLOCK_HZ / 32UL)
/* Shared CS/SCK/MOSI edge selection; does not alter SCK frequency or the
   sensor's MISO driver. MEDIUM failed the 8400-rpm board comparison on
   2026-10-23, so retain the original VERY_HIGH baseline, not a claimed fix. */
#define SPI1_ENC_OUTPUT_GPIO_SPEED    GPIO_SPEED_FREQ_VERY_HIGH

#if SPI1_ENC_SCK_HZ > 16000000UL
#error "MT6835 SPI clock exceeds the 16 MHz sensor limit"
#endif

/* 软件片选引脚。空闲必须为高（手册：非通信状态 SCK 保持高、CSN 释放）。 */
#if MT6701_SSI_ENABLE
#define MT6835_CSN_Pin GPIO_PIN_6
#define MT6835_CSN_GPIO_Port GPIOB
#else
#define MT6835_CSN_Pin                 GPIO_PIN_4
#define MT6835_CSN_GPIO_Port           GPIOA
#endif
#define MT6835_CAL_EN_Pin              GPIO_PIN_4
#define MT6835_CAL_EN_GPIO_Port        GPIOC
#define MT6835_SCK_Pin                 GPIO_PIN_5
#define MT6835_SCK_GPIO_Port           GPIOA
#if (MT6835_SOFT_SPI_CROSSWIRED != 0U)
/* Historical crosswired assembly only; not the verified current assembly. */
#define MT6835_SOFT_MOSI_Pin           GPIO_PIN_6
#define MT6835_SOFT_MOSI_GPIO_Port     GPIOA
#define MT6835_SOFT_MISO_Pin           GPIO_PIN_7
#define MT6835_SOFT_MISO_GPIO_Port     GPIOA
#else
#define MT6835_SOFT_MOSI_Pin           GPIO_PIN_7
#define MT6835_SOFT_MOSI_GPIO_Port     GPIOA
#define MT6835_SOFT_MISO_Pin           GPIO_PIN_6
#define MT6835_SOFT_MISO_GPIO_Port     GPIOA
#endif
#define MT6835_CSN_SELECT()            (MT6835_CSN_GPIO_Port->BSRR = \
                                        ((uint32_t)MT6835_CSN_Pin << 16U))
#define MT6835_CSN_RELEASE()           (MT6835_CSN_GPIO_Port->BSRR = MT6835_CSN_Pin)
#define MT6835_CAL_EN_DISABLE()        HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, \
                                                          MT6835_CAL_EN_Pin, \
                                                          GPIO_PIN_RESET)

/* --------------------------------------------------------------------------*/
/* 导出变量与函数                                                            */
/* --------------------------------------------------------------------------*/

extern SPI_HandleTypeDef hspi1;

typedef void (*mt6835_spi_async_callback_t)(HAL_StatusTypeDef status,
                                             void *context);

/**
  * @brief  【配置线】配置 SPI1 为主机、模式 3、8 bit、软件片选，并把 PA4
  *         配成空闲为高的推挽输出。
  *
  * 引脚的复用配置（PA5/PA6/PA7 = AF5）在 MSP 层
  * （Core/Src/stm32g4xx_hal_msp.c 的 HAL_SPI_MspInit）完成，由
  * HAL_SPI_Init() 自动回调。
  *
  * 遵循本工程 L1 约定：不含 main.h、不调 Error_Handler()，失败返回
  * HAL_ERROR 交给 main.c 处置。只能由 main() 在时钟配好之后调用一次。
  *
  * @retval HAL_OK；HAL_SPI_Init 失败时返回 HAL_ERROR。
  */
HAL_StatusTypeDef MX_SPI1_Encoder_Init(void);

/**
  * @brief  编码器 SPI 收发抽象。上层不关心当前使用 SPI1 外设还是 GPIO 模拟。
  * @note   此阻塞接口仅供线程上下文；与异步 DMA 互斥，忙时不操作 CSN。
  */
HAL_StatusTypeDef MT6835_SPI_TransmitReceive(const uint8_t *tx, uint8_t *rx,
                                             uint16_t size, uint32_t timeout);

/* 收发抽象统一拥有 CSN，调用方不得另外拉低/释放片选。同步与异步互斥。
   DMA 异步事务：函数只负责启动，CSN 在完成/错误回调后释放。回调运行在
   DMA 中断上下文，不得阻塞。任一时刻只允许一个事务。 */
HAL_StatusTypeDef MT6835_SPI_TransmitReceiveAsync(
    const uint8_t *tx, uint8_t *rx, uint16_t size,
    mt6835_spi_async_callback_t callback, void *context);
HAL_StatusTypeDef MT6835_SPI_AbortAsync(void);
uint8_t MT6835_SPI_IsBusy(void);
/* Bounded master-peripheral resynchronization after a bad CRC. Only when
   software ownership is idle; also clears stuck BSY/DMA state. Never touches EEPROM and
   never waits for a timeout. Does not acknowledge motor/controller faults. */
HAL_StatusTypeDef MT6835_SPI_ResyncIdle(void);

/* 中断文件通过这两个入口分发 DMA IRQ，避免暴露 DMA 句柄。 */
void SPI1_Encoder_DmaRxIrqHandler(void);
void SPI1_Encoder_DmaTxIrqHandler(void);
void SPI1_Encoder_SpiIrqHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI1_ENCODER_H__ */
