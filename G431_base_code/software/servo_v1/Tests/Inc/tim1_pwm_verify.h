/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim1_pwm_verify.h
  * @brief   单相 MOSFET 栅极驱动测量波形（L5 测试/验证层，纯业务线）。
  *
  *          g_measurement_manager 中选定的那一相会输出一个高侧 PWM 波形。
  *          另外两相被命令为 0 % 高侧占空比，这让它们的低侧 MOSFET 保持导通，
  *          把相端钳位到 PGND。改 config.mos_driver.phase 即可选择 U、V 或 W。
  *
  *          运行时必须脱开电机，并使用带限流的直流母线。该波形刻意施加了
  *          非零的线电压，用途是探测单个半桥，而不是用来转动电机。
  *
  * ---------------------------------------------------------------------------
  * 调试操作手册（Runbook）
  * ---------------------------------------------------------------------------
  *   1. 上电顺序。给板子上电，先在调试器里确认
  *      DrvMonitor_GetState() == DRV_STATE_READY，然后再做任何其他事。
  *      此时不要接入任何承受不了电机保持电流的负载。
  *   2. 探测选中相的栅源电压及其 U/V/W 相端。相端必须以 20 kHz、
  *      按所配置的占空比翻转。
  *   3. 确认另外两相的相端电压贴近 PGND。
  *   4. 核对寄存器期望值。占空比标度是 0..1000，比较值 = duty × 4200 / 1000：
  *        当前默认 duty = 50（即 5 %）→ TIM1->CCR1 = 210，CCR2/3 = 0
  *        若改为 duty = 100（即 10 %）→ TIM1->CCR1 = 420
  *      TIM1->ARR 恒为 4199，运行时 BDTR.DTG 为 84，BDTR.MOE 为 1。
  *   5. 波形会持续输出，直到 MOE 字段被清零或调用 Tim1PwmVerify_Stop()。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TIM1_PWM_VERIFY_H__
#define __TIM1_PWM_VERIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "measurement_manager.h"

/* --------------------------------------------------------------------------*/
/* 导出函数                                                                  */
/* --------------------------------------------------------------------------*/

/**
  * @brief  准备并启动验证。
  *
  * 装载选中相的波形、等待自举电容预充电，然后使能栅极输出。波形是连续输出的：
  * MOE 只会被配置变更、硬件故障，或显式调用 Tim1PwmVerify_Stop() 清除。
  *
  * @param  manager     统一测量对象。其中的 MOS 驱动配置决定被测相、
  *                     占空比和启动延时。
  * @retval HAL_OK 表示输出已使能；波形被拒绝或输出被 MOE 挡住时返回 HAL_ERROR。
  * @note   本函数内部有一次 startup_delay_ms 的阻塞延时（默认 50 ms），
  *         因此只应在初始化阶段调用，不要在实时路径里调。
  */
HAL_StatusTypeDef Tim1PwmVerify_Start(measurement_manager_t *manager);

/**
  * @brief  服务当前测量。由主循环调用。
  *
  * 非阻塞的「比对并同步」：把待生效的 phase / duty / MOE 修改应用下去，
  * 并在输出不再使能时判定测量失败——清掉 MOE 就是这样结束测试的。
  *
  * @param  manager     传给 Start() 的同一个测量对象。
  * @retval 测量运行中或空闲时返回 HAL_OK；测量已失败或配置被拒绝时返回 HAL_ERROR。
  */
HAL_StatusTypeDef Tim1PwmVerify_Task(measurement_manager_t *manager);

/**
  * @brief  停止验证并关断输出。
  *         任何时候调用都安全，包括正常结束之后。
  */
void              Tim1PwmVerify_Stop(measurement_manager_t *manager);

/**
  * @brief  查询序列是否仍在输出波形。
  * @retval 运行中返回 1；已结束、已中止或已停止后返回 0。
  */
uint8_t           Tim1PwmVerify_IsRunning(const measurement_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_PWM_VERIFY_H__ */
