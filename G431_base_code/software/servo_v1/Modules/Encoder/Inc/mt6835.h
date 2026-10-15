/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6835.h
  * @brief   MT6835 磁编码器的 SPI 寄存器接口（L2 器件驱动层，纯业务线）。
  *
  *          职责边界：只负责「如何正确读写这颗芯片的寄存器」——24 bit 帧的
  *          组包、软件片选时序、角度帧的 CRC 校验与位域剥离。
  *          不做的事：不配置 SPI 外设（L1 的职责，本层只用 hspi1 句柄与片选
  *          宏）、不做角度到电角度的换算与闭环消费（上层职责）、不决定
  *          轮询节奏（监控/测试层的职责）。
  *
  *          SPI 帧格式（手册 v1.3 §7.6.3 图-18，逐位核对过，共 24 个 SCK 周期，
  *          无奇偶校验位——与 MT6816/MT6825 系列不同，勿混用）：
  *            bit  1~ 4 : C3~C0   命令字（0011 读 / 0110 写 / 1010 连读角度）
  *            bit  5~16 : A11~A0  12 bit 寄存器地址
  *            bit 17~24 : DI/DO   8 bit 数据；读模式下 MISO 仅在这 8 拍驱动，
  *                                前 16 拍为高阻
  *          模式 3（CPOL=1, CPHA=1），MSB 先行；CSN 下降沿开始并锁存角度
  *          寄存器，上升沿结束（§7.6.2 / §7.6.9）。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MT6835_H__
#define __MT6835_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* --------------------------------------------------------------------------*/
/* 命令字（手册 §7.6.3 真值表）                                               */
/* --------------------------------------------------------------------------*/
#define MT6835_CMD_READ              0x3U   /* 0011 读寄存器 */
#define MT6835_CMD_WRITE             0x6U   /* 0110 写寄存器 */
#define MT6835_CMD_BURN_EEPROM       0xCU   /* 1100 烧录 EEPROM（⚠ 勿在测试中调用） */
#define MT6835_CMD_AUTO_ZERO         0x5U   /* 0101 自动设置零点（⚠ 同上） */
#define MT6835_CMD_BURST_READ_ANGLE  0xAU   /* 1010 连续读角度寄存器 */

/* --------------------------------------------------------------------------*/
/* 寄存器映射（手册 §10，第 31~32 页）                                        */
/* --------------------------------------------------------------------------*/
#define MT6835_REG_USER_ID           0x001U  /* EEPROM，客户保留 */
#define MT6835_REG_ANGLE_H           0x003U  /* ANGLE[20:13]，只读 */
#define MT6835_REG_ANGLE_M           0x004U  /* ANGLE[12:5]，只读 */
#define MT6835_REG_ANGLE_L           0x005U  /* ANGLE[4:0]<<3 | STATUS[2:0]，只读 */
#define MT6835_REG_CRC               0x006U  /* 角度帧 CRC，只读 */
#define MT6835_REG_ABZ_RES_H         0x007U  /* EEPROM */
#define MT6835_REG_ABZ_RES_L         0x008U  /* EEPROM */
#define MT6835_REG_ZERO_POS_H        0x009U  /* EEPROM */
#define MT6835_REG_ZERO_POS_L        0x00AU  /* EEPROM */
#define MT6835_REG_CAL_STATUS        0x113U  /* bit[7:6] 自校准状态，§9.2 */

/* 0x005 的位域：高 5 位是 ANGLE[4:0]，低 3 位是 STATUS[2:0] */
#define MT6835_ANGLE_L_ANGLE_MASK    0xF8U
#define MT6835_ANGLE_L_ANGLE_SHIFT   3U
#define MT6835_ANGLE_L_STATUS_MASK   0x07U

/* STATUS[2:0] 各位（手册 §10 (2)，第 25/32 页）：'1' 为报警 */
#define MT6835_STATUS_OVERSPEED      (1U << 0)  /* 转速过快（>120000 rpm） */
#define MT6835_STATUS_WEAK_FIELD     (1U << 1)  /* 外加磁场太弱 */
#define MT6835_STATUS_UNDERVOLTAGE   (1U << 2)  /* 芯片供电欠压 */
#define MT6835_STATUS_ALARM_MASK     0x07U

/* 0x113 bit[7:6] 自校准状态（手册 §9.2） */
#define MT6835_CAL_NOT_RUN           0x0U
#define MT6835_CAL_RUNNING           0x1U
#define MT6835_CAL_FAILED            0x2U
#define MT6835_CAL_SUCCESS           0x3U

/* 角度分辨率：21 bit 对 360°。ANGLE[i] 合成后 θ = Σ·360°/2^21。 */
#define MT6835_ANGLE_FULL_SCALE      (1UL << 21)

/* --------------------------------------------------------------------------*/
/* CRC 参数（手册：多项式 X8+X2+X+1 = 0x07，覆盖 ANGLE[20:0]+STATUS[2:0] 共
   24 bit，ANGLE[20] 为最高位最先移入）。
   ⚠ 手册未给出 CRC 寄存器初值，此处按 CRC-8 常规取 0x00。若实测校验恒不
   通过，优先尝试把初值改为 0xFF 再验证，并在 README 中固化结论。 */
#define MT6835_CRC_POLY              0x07U
#define MT6835_CRC_INIT              0x00U

/* 上电等待：手册 §5 T_PwrUp = 2 ms（电源斜坡 < 100 µs 时）。放宽到 5 ms，
   兼容上电斜坡较慢的板级电源。ABZ 输出需 64 ms 才稳定，但 SPI 不受影响。 */
#define MT6835_POWERUP_WAIT_MS       5U

/* 单次 SPI 事务的超时（ms）。10.5 MHz 下 24 bit 约 2.3 µs，超时只为兜底
   芯片未焊或断线的情况。 */
#define MT6835_SPI_TIMEOUT_MS        10U

/* --------------------------------------------------------------------------*/
/* 导出类型                                                                  */
/* --------------------------------------------------------------------------*/

/* 报警状态位域，与手册 STATUS[2:0] 逐位对应 */
typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t overspeed:1;    /* STATUS[0] */
    uint8_t weak_field:1;   /* STATUS[1] */
    uint8_t undervolt:1;    /* STATUS[2] */
    uint8_t rsvd:5;
  } bit;
} mt6835_status_t;

/* 一次完整的角度帧读取结果 */
typedef struct
{
  uint32_t        angle_raw;   /* ANGLE[20:0]，0 ~ 2097151 */
  mt6835_status_t status;      /* 与角度同帧锁存的报警状态 */
  uint8_t         crc;         /* 0x006 读回的 CRC */
  uint8_t         crc_ok;      /* 1 = 本地重算与读回一致 */
  uint8_t         raw[4];      /* MISO 原始字节：依次为 0x003~0x006 的内容，
                                    调试器诊断用（全 0xFF = MISO 未被驱动） */
} mt6835_angle_t;

/* --------------------------------------------------------------------------*/
/* 对上（测试层 / 未来监控层）提供的接口                                      */
/* --------------------------------------------------------------------------*/

/**
  * @brief  上电等待：保证 T_PwrUp 已过，之后才允许发起 SPI 通讯。
  * @note   只应在初始化阶段调用一次。内部用 HAL_Delay，禁止放进 ISR。
  */
void MT6835_PowerUpWait(void);

/**
  * @brief  读单个寄存器（24 bit 帧：命令 0011 + 地址 + 8 bit 数据）。
  * @param  reg     寄存器地址（12 bit）
  * @param  value   读回值存放处，不可为 NULL
  * @param  timeout SPI 超时（ms）
  * @retval HAL_OK / HAL_ERROR（参数非法或传输失败）/ HAL_TIMEOUT
  */
HAL_StatusTypeDef MT6835_ReadRegister(uint16_t reg, uint8_t *value,
                                      uint32_t timeout);

/**
  * @brief  写单个寄存器（24 bit 帧：命令 0110 + 地址 + 8 bit 数据）。
  * @note   MT6835 无写保护锁，但 EEPROM 寄存器写后需烧录命令才持久化；
  *         本函数只做 RAM 写入，不触发烧录。
  */
HAL_StatusTypeDef MT6835_WriteRegister(uint16_t reg, uint8_t value,
                                       uint32_t timeout);

/**
  * @brief  单独读取报警状态：借道 0x005（ANGLE[4:0]+STATUS[2:0]）剥离低 3 位。
  * @param  status  状态存放处，不可为 NULL
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef MT6835_ReadStatus(mt6835_status_t *status, uint32_t timeout);

/**
  * @brief  读取一整帧角度数据：单帧内连读 0x003~0x006 四个寄存器。
  *
  * CSN 在整个 4×24 bit 事务期间保持拉低，因此四个寄存器来自同一次锁存
  * （手册 §7.6.9：CSN 下降沿锁存 0x003~0x006，直到被读走），STATUS 与
  * ANGLE 不会错位。函数内部完成位域拼装与 CRC 重算比对。
  *
  * @param  frame   角度帧存放处，不可为 NULL
  * @retval HAL_OK 表示四个字节都读到了（CRC 是否通过看 frame->crc_ok）；
  *         HAL_ERROR / HAL_TIMEOUT 表示传输本身失败。
  * @note   单次事务约 9 µs（10.5 MHz），允许在主循环与测试层调用；
  *         是否放进 20 kHz 电流环由上层按耗时预算决定。
  */
HAL_StatusTypeDef MT6835_ReadAngle(mt6835_angle_t *frame, uint32_t timeout);

/**
  * @brief  对 24 bit 角度数据重算 CRC-8（多项式 X8+X2+X+1，MSB 先入）。
  * @param  data24  ANGLE[20:0] 与 STATUS[2:0] 按 ANGLE[20] 为最高位拼成的
  *                 24 bit 值（即 (reg003<<16)|(reg004<<8)|reg005）
  * @retval 8 bit CRC
  * @note   公开给测试层做独立的校验判据使用。
  */
uint8_t MT6835_CalcCrc24(uint32_t data24);

#ifdef __cplusplus
}
#endif

#endif /* __MT6835_H__ */
