/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6835.c
  * @brief   MT6835 磁编码器的 SPI 寄存器接口（L2 器件驱动层）实现。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mt6835.h"
#include "spi1_encoder.h"

/* 24 bit 帧的前两个字节：命令字占 bit1~4，地址占 bit5~16。
   byte0 = C3~C0 + A11~A8，byte1 = A7~A0。第三个字节是数据段：
   写模式下由 MOSI 驱动，读模式下由 MISO 在第 17~24 拍驱动。 */
#define MT6835_FRAME_BYTE0(cmd, reg) \
  ((uint8_t)((((uint8_t)(cmd)) << 4) | (((uint16_t)(reg) >> 8) & 0x0FU)))
#define MT6835_FRAME_BYTE1(reg)      ((uint8_t)((uint16_t)(reg) & 0xFFU))

void MT6835_PowerUpWait(void)
{
  HAL_Delay(MT6835_POWERUP_WAIT_MS);
}

uint8_t MT6835_CalcCrc24(uint32_t data24)
{
  uint8_t crc = MT6835_CRC_INIT;
  uint8_t bit;

  /* 24 bit 自最高位（ANGLE[20]）逐位移入，多项式 X8+X2+X+1 = 0x07。
     反馈位 = CRC 寄存器最高位 XOR 当前数据位。 */
  for (bit = 0U; bit < 24U; bit++)
  {
    uint8_t feedback = (uint8_t)(((crc >> 7) ^
                                   (uint8_t)(data24 >> (23U - bit))) & 0x01U);
    crc = (uint8_t)(crc << 1);
    if (feedback != 0U)
    {
      crc = (uint8_t)(crc ^ MT6835_CRC_POLY);
    }
  }
  return crc;
}

HAL_StatusTypeDef MT6835_ReadRegister(uint16_t reg, uint8_t *value,
                                      uint32_t timeout)
{
  uint8_t tx[3];
  uint8_t rx[3] = {0U, 0U, 0U};

  if ((value == NULL) || (reg > 0x0FFFU))
  {
    return HAL_ERROR;
  }

  tx[0] = MT6835_FRAME_BYTE0(MT6835_CMD_READ, reg);
  tx[1] = MT6835_FRAME_BYTE1(reg);
  tx[2] = 0x00U;

  MT6835_CSN_SELECT();
  HAL_StatusTypeDef st = MT6835_SPI_TransmitReceive(tx, rx, 3U, timeout);
  MT6835_CSN_RELEASE();

  if (st != HAL_OK)
  {
    return st;
  }

  *value = rx[2];
  return HAL_OK;
}

HAL_StatusTypeDef MT6835_WriteRegister(uint16_t reg, uint8_t value,
                                       uint32_t timeout)
{
  uint8_t tx[3];
  uint8_t rx[3];

  if (reg > 0x0FFFU)
  {
    return HAL_ERROR;
  }

  tx[0] = MT6835_FRAME_BYTE0(MT6835_CMD_WRITE, reg);
  tx[1] = MT6835_FRAME_BYTE1(reg);
  tx[2] = value;

  MT6835_CSN_SELECT();
  HAL_StatusTypeDef st = MT6835_SPI_TransmitReceive(tx, rx, 3U, timeout);
  MT6835_CSN_RELEASE();

  return st;
}

HAL_StatusTypeDef MT6835_ReadStatus(mt6835_status_t *status, uint32_t timeout)
{
  uint8_t reg005;

  if (status == NULL)
  {
    return HAL_ERROR;
  }

  if (MT6835_ReadRegister(MT6835_REG_ANGLE_L, &reg005, timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  status->raw = (uint8_t)(reg005 & MT6835_ANGLE_L_STATUS_MASK);
  return HAL_OK;
}

HAL_StatusTypeDef MT6835_ReadAngle(mt6835_angle_t *frame, uint32_t timeout)
{
  /* MT6835 §7.6.9 burst angle read：一个 CSN 事务内先发 16 bit 的
     1010 + 地址 0x003（即 A0 03），随后连续读出 0x003~0x006 四个字节。
     因此完整事务是 6 字节、48 个 SCK，不能拆成四个独立 CSN。 */
  uint8_t tx[6] =
  {
    MT6835_FRAME_BYTE0(MT6835_CMD_BURST_READ_ANGLE, MT6835_REG_ANGLE_H),
    MT6835_FRAME_BYTE1(MT6835_REG_ANGLE_H),
    0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t rx[6] = {0U};

  if (frame == NULL)
  {
    return HAL_ERROR;
  }

  MT6835_CSN_SELECT();
  HAL_StatusTypeDef st = MT6835_SPI_TransmitReceive(tx, rx, 6U, timeout);
  MT6835_CSN_RELEASE();

  if (st != HAL_OK)
  {
    return st;
  }

  uint8_t reg003 = rx[2];
  uint8_t reg004 = rx[3];
  uint8_t reg005 = rx[4];
  uint8_t reg006 = rx[5];

  uint32_t data24 = ((uint32_t)reg003 << 16) |
                    ((uint32_t)reg004 << 8)  |
                    ((uint32_t)reg005);

  frame->angle_raw = (data24 >> 3) & 0x1FFFFFUL;  /* ANGLE[20:0] */
  frame->status.raw = (uint8_t)(reg005 & MT6835_ANGLE_L_STATUS_MASK);
  frame->crc = reg006;
  frame->crc_ok = (MT6835_CalcCrc24(data24) == reg006) ? 1U : 0U;
  frame->raw[0] = reg003;
  frame->raw[1] = reg004;
  frame->raw[2] = reg005;
  frame->raw[3] = reg006;

  return HAL_OK;
}
