#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电流采样模块落地补丁：保留每个文件原有的 BOM 与行尾风格，逐处精确替换。"""
import os, sys

ROOT = r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code"
SW = os.path.join(ROOT, "software", "servo_v1")


def patch(path, pairs):
    with open(path, "rb") as f:
        raw = f.read()
    bom = b""
    body = raw
    if raw.startswith(b"\xef\xbb\xbf"):
        bom = b"\xef\xbb\xbf"
        body = raw[3:]
    crlf = b"\r\n" in body
    text = body.decode("utf-8")
    for old, new in pairs:
        if crlf:
            old = old.replace("\n", "\r\n")
            new = new.replace("\n", "\r\n")
        n = text.count(old)
        if n != 1:
            print("FAIL %s: 匹配 %d 次（需恰好 1 次）\n---\n%s\n---" % (
                os.path.basename(path), n, old[:300]))
            sys.exit(1)
        text = text.replace(old, new)
    out = bom + text.encode("utf-8")
    with open(path, "wb") as f:
        f.write(out)
    print("OK %-32s %6d -> %6d bytes  BOM=%-5s CRLF=%s" % (
        os.path.basename(path), len(raw), len(out), bool(bom), crlf))


# ---------------------------------------------------------------- 1. hal_conf
patch(os.path.join(SW, "Core", "Inc", "stm32g4xx_hal_conf.h"), [
    ("  /*#define HAL_ADC_MODULE_ENABLED   */",
     "#define HAL_ADC_MODULE_ENABLED"),
    ("/*#define HAL_OPAMP_MODULE_ENABLED   */",
     "#define HAL_OPAMP_MODULE_ENABLED"),
])

# --------------------------------------------------------------------- 2. MSP
MSP_CALLBACKS = '''
/**
  * @brief  OPAMP GPIO configuration for the phase-current amplifiers.
  *         OPAMP1: PA1 (VINP0) / PA3 (VINM0) / PA2 (OUT)   -> U phase
  *         OPAMP3: PB0 (VINP0) / PB2 (VINM0) / PB1 (OUT)   -> W phase
  *
  *         All six pins must sit in analog mode. PB2 doubles as BOOT1, so it
  *         has to be configured explicitly or the leftover boot-mode setting
  *         will disturb the inverting input.
  *
  *         PA2 and PB1 are also the ADC1_IN3 / ADC1_IN12 inputs: the amplifier
  *         outputs come back through the external pins (not the internal
  *         VOPAMP channels) because the schematic hangs a post-stage RC
  *         network on those nodes.
  *
  *         The STM32G4 OPAMPs have no dedicated RCC gate, so only the GPIO
  *         clocks are enabled here; the amplifier itself is turned on through
  *         its own CSR enable bit inside HAL_OPAMP_Init().
  */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hopamp->Instance == OPAMP1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (hopamp->Instance == OPAMP3)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**OPAMP3 GPIO Configuration
    PB0     ------> OPAMP3_VINP
    PB1     ------> OPAMP3_VOUT
    PB2     ------> OPAMP3_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp)
{
  if (hopamp->Instance == OPAMP1)
  {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  }
  else if (hopamp->Instance == OPAMP3)
  {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  }
}

/**
  * @brief  ADC1 clock enable for the injected current-sampling group.
  *
  *         No ADC interrupt is configured here on purpose: the injected group
  *         is consumed inside the TIM1_CC ISR that tim1_pwm.c already owns
  *         (priority 0), so a second vector would only add latency.
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
  }
}

/* USER CODE BEGIN 1 */'''

patch(os.path.join(SW, "Core", "Src", "stm32g4xx_hal_msp.c"), [
    ('/* USER CODE BEGIN Includes */\n#include "tim1_pwm.h"\n/* USER CODE END Includes */',
     '/* USER CODE BEGIN Includes */\n#include "tim1_pwm.h"\n'
     '#include "adc1_current_sampling.h"\n/* USER CODE END Includes */'),
    ("/* USER CODE BEGIN 1 */", MSP_CALLBACKS),
])

# -------------------------------------------------------------------- 3. main
patch(os.path.join(SW, "Core", "Src", "main.c"), [
    ('#include "tim1_pwm_verify.h"\n#include "measurement_manager.h"',
     '#include "tim1_pwm_verify.h"\n#include "measurement_manager.h"\n'
     '#include "adc1_current_sampling.h"'),
    ('''  /* TODO: MX_ADC1_Init() + 电流环。功率级只能在
   * DrvMonitor_GetState() == DRV_STATE_READY 且 TIM1->BDTR 的 MOE 仍置位时
   * 才允许使能。 */''',
     '''  /* ------------------------------------------------------------------------
   * 电流采样外设配置（模块 F：Modules/Current_Sampling/）
   * ------------------------------------------------------------------------
   * 顺序有意义：两个运放必须先于 ADC 配好。ADC1 采的是运放输出脚
   * （PA2 = ADC1_IN3，PB1 = ADC1_IN12），运放没起来时那两个节点上没有有效
   * 信号，先校准 ADC 只会把噪声当成基线记下来。
   *
   * MX_ADC1_Init() 传入 software_trigger_flag = 1，即注入组由软件触发、
   * 不挂 TIM1_CH4。这是刻意的：本固件当前运行的是模块 D 的栅极验证波形，
   * 此时 TIM1_CH4 已经以 20 kHz 在跑，若让注入组跟随它触发，每个载波周期
   * 都会多做一轮 11.43 µs 的过采样转换（见第 12 节待核对项 I 的时序余量
   * 问题），而目前还没有任何消费者去读这些数据。
   *
   * 配置只把硬件准备好，不接通信号通路：这里不启动运放、不启动注入转换。
   * 启动属于未来的业务层，且必须在 DrvMonitor_GetState() == DRV_STATE_READY
   * 且 TIM1->BDTR 的 MOE 仍置位时才允许。
   *
   * 电流环真正上线时，把下面这一次调用换成 software_trigger_flag = 0
   * （由 ADC_EXTERNALTRIGINJEC_T1_CC4 触发），并先做一轮零电流偏置标定，
   * 结果写入 ia_offsets / ib_offsets。 */
  if (MX_OPAMP1_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_OPAMP3_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (MX_ADC1_Init(1U) != HAL_OK)
  {
    Error_Handler();
  }

  /* TODO: 电流环。功率级只能在
   * DrvMonitor_GetState() == DRV_STATE_READY 且 TIM1->BDTR 的 MOE 仍置位时
   * 才允许使能。 */'''),
])

print("全部补丁完成")
