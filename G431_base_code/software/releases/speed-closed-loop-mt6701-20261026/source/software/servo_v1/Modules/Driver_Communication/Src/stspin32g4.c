/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stspin32g4.c
  * @brief   STSPIN32G4 的 I2C 寄存器接口实现（L2 器件驱动层）。
  *          本层只使用 L1 交出的 hi2c3 句柄与 GD_*_Pin 宏，自身不初始化任何外设。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stspin32g4.h"

HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                          uint32_t timeout)
{
  return STSPIN32G4_ReadRegisters(reg, value, 1U, timeout);
}

HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                           uint32_t timeout)
{
  return STSPIN32G4_WriteRegisters(reg, &value, 1U, timeout);
}

HAL_StatusTypeDef STSPIN32G4_ReadRegisters(uint8_t reg, uint8_t *data,
                                           uint16_t size, uint32_t timeout)
{
  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c3, STSPIN32G4_I2C_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, data, size, timeout);
}

HAL_StatusTypeDef STSPIN32G4_WriteRegisters(uint8_t reg, const uint8_t *data,
                                            uint16_t size, uint32_t timeout)
{
  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Write(&hi2c3, STSPIN32G4_I2C_ADDRESS, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, size,
                           timeout);
}

HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout)
{
  uint8_t  status = 0U;
  uint32_t tick_start;

  /* 上电后故障处于锁存状态：必须写 CLEAR 芯片才会进入可动作状态。
     LOCK 寄存器本身不受保护，所以即使还没解锁也能完成清除。 */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_CLEAR, STSPIN32G4_CLEAR_FAULTS,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (STSPIN32G4_ReadRegister(STSPIN32G4_REG_STATUS, &status,
                              timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((status & (STSPIN32G4_STATUS_VDS_P | STSPIN32G4_STATUS_THSD |
                 STSPIN32G4_STATUS_VCC_UVLO)) != 0U)
  {
    return HAL_ERROR;
  }

  /* 只看寄存器视图是不够的：READY/nFAULT 才是硬件真正用来门控功率级的引脚，
     所以两者必须一致。READY 是开漏输出，芯片可用时置高；nFAULT 也是开漏输出，
     报告故障时拉低。

     上面那次 STATUS 读取已经在总线上花了约 390 µs，稳定时间是足够的，
     这里再留一个短的有界等待来覆盖临界情况。出现瞬态不一致时让调用者重试，
     而不是硬性判失败。 */
  tick_start = HAL_GetTick();
  while ((STSPIN32G4_IsReady() == 0U) || (STSPIN32G4_IsFault() == 0U))
  {
    if ((HAL_GetTick() - tick_start) > STSPIN32G4_PIN_SETTLE_MS)
    {
      return HAL_BUSY;
    }
  }

  return HAL_OK;
}

uint8_t STSPIN32G4_IsReady(void)
{
  return (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET)
         ? 1U : 0U;
}

uint8_t STSPIN32G4_IsFault(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin);
}

uint8_t STSPIN32G4_PinsAllowSwitching(void)
{
  uint32_t pin_levels = GD_READY_GPIO_Port->IDR;

  return (((pin_levels & GD_READY_Pin) != 0U) &&
          ((pin_levels & GD_NFAULT_Pin) != 0U)) ? 1U : 0U;
}

HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout)
{
  return STSPIN32G4_WriteRegister(STSPIN32G4_REG_CLEAR, STSPIN32G4_CLEAR_FAULTS,
                                  timeout);
}

HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout)
{
  uint32_t tick_start = HAL_GetTick();

  /* WAKE 高有效，驱动芯片侧自带内部下拉 */
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

  /* VCC 越过 VCC(On) 之后经过 tWAKE,LDO（10 µs），READY 才会释放 */
  while (STSPIN32G4_IsReady() == 0U)
  {
    if ((HAL_GetTick() - tick_start) > timeout)
    {
      HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);
      return HAL_TIMEOUT;
    }
  }

  /* 撤销 WAKE，否则后续的待机请求会被它打断而无法生效 */
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);

  /* EnterStandby() 里无法完成重新上锁：等 STBY 生效时 I2C 接口已经死了。
     而在 LOCK == ~NLOCK（解锁）期间，栅极驱动输出是被强制拉低的
     （DS13630 Table 23），所以芯片在重新上锁之前无法驱动 MOSFET。
     LOCK 不受保护，因此只要芯片有电，这次写入总能成功。
     如果待机期间 MCU 掉过电，寄存器本来就已经回到默认值（LOCK = 0），
     这次写入就是一次无害的空操作。 */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_LOCK, STSPIN32G4_LOCK_LOCK,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 清除上电 / 退出待机产生的锁存故障，并校验引脚电平 */
  return STSPIN32G4_Init(timeout);
}

/**
  * @brief  请求进入低功耗待机模式。
  * @param  stby_reg_en 1 = 使能待机稳压器（待机期间继续给内嵌 MCU 供电），
  *                     0 = 不使能。
  *
  * 警告 —— 电流预算：待机稳压器最多只能提供 IREG3V3,Stby = 6 mA
  * （IREGStby,lim 典型值 5 mA）。STM32G431 跑在 168 MHz 时的电流远超此值，
  * 因此这里传 1 只在「MCU 随后立即进入 Stop 模式」时才成立。否则待机稳压器
  * 会触发自身的过流保护并复位芯片。
  *
  * 本函数是整个序列的最后一次 I2C 访问：STBY 位锁存后经过 tSTBY（100 µs），
  * 芯片即进入低功耗模式且 I2C 接口变为不可用，所以之后不能再尝试重新上锁
  * ——那次传输必然超时。重新上锁的工作由 STSPIN32G4_Wake() 完成。
  *
  * 调用者必须已经关闭功率级。不过即便没有关，解锁受保护寄存器这个动作本身
  * 也会把所有栅极驱动输出强制拉低。
  */
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en, uint32_t timeout)
{
  uint8_t powmng = 0U;

  /* STBY 和 POWMNG 都是受保护寄存器：先解锁 */
  if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_LOCK, STSPIN32G4_LOCK_UNLOCK,
                               timeout) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (stby_reg_en != 0U)
  {
    /* 用读-改-写，以保留原有的 VCC 电压档位和各稳压器使能位 */
    if (STSPIN32G4_ReadRegister(STSPIN32G4_REG_POWMNG, &powmng,
                                timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }

    powmng |= STSPIN32G4_POWMNG_STBY_REG_EN;

    if (STSPIN32G4_WriteRegister(STSPIN32G4_REG_POWMNG, powmng,
                                 timeout) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return STSPIN32G4_WriteRegister(STSPIN32G4_REG_STBY, STSPIN32G4_STBY_ENTER,
                                  timeout);
}
