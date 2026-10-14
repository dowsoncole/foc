/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc_interface.c
  * @brief   FOC 控制器的生命周期与调度，通过 foc_port_t 函数指针端口访问硬件。
  *
  *          本文件属于业务逻辑线，且刻意做到零硬件依赖：不 include 任何
  *          芯片/外设头文件，所有与硬件的往来都通过上层注册的回调完成。
  *          更换硬件只需更换回调实现，本文件与 foc_algorithm.c 一行都不用改。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "foc_interface.h"

#include <string.h>

/**
  * @brief  初始化控制器：清零对象、装入端口、按参数初始化算法。
  * @note   任一入参为空指针时直接返回，对象保持调用方原状（不清零），
  *         避免把调用方已有的内容意外抹掉。
  */
void FOC_ControllerInit(foc_controller_t *controller,
                        const foc_parameters_t *parameters,
                        const foc_port_t *port)
{
  if ((controller == 0) || (parameters == 0) || (port == 0))
  {
    return;
  }

  (void)memset(controller, 0, sizeof(*controller));
  controller->port = *port;
  FOC_AlgorithmInit(&controller->algorithm, parameters);
}

/**
  * @brief  置运行标志。真正的输出使能由 port.disable_output / write_voltage
  *         的对端（三相驱动层的 MOE）决定，本函数不触碰硬件。
  */
void FOC_ControllerStart(foc_controller_t *controller)
{
  if (controller != 0)
  {
    controller->running = 1U;
  }
}

/**
  * @brief  停止控制器：清运行标志、复位积分器，并把零电压与安全关断
  *         下发到硬件，确保停下后功率级不会残留在上一次的输出状态。
  * @note   两个端口回调都先判空再调用——允许调用方只注册其中一部分。
  */
void FOC_ControllerStop(foc_controller_t *controller)
{
  if (controller == 0)
  {
    return;
  }

  controller->running = 0U;
  FOC_AlgorithmReset(&controller->algorithm);
  if (controller->port.write_voltage != 0)
  {
    controller->port.write_voltage(controller->port.context,
                                    &controller->algorithm.output);
  }
  if (controller->port.disable_output != 0)
  {
    controller->port.disable_output(controller->port.context);
  }
}

/**
  * @brief  单次电流环迭代。必须由 20 kHz 的 TIM1_CC 中断调用（ADC 采样完成后），
  *         不能放在主循环——主循环没有确定性周期，电流环会失效。
  *
  *         安全门：每一步都先问 port.is_power_stage_safe()。该回调应绑定到
  *         DrvMonitor_IsPowerStageSafeFast()（只读 READY/nFAULT 引脚，绝不走 I2C，
  *         执行时间恒定），因此在 20 kHz 中断里调用是安全的。
  *         一旦报告不安全，立即停环并关断输出，不等待上层轮询发现。
  */
void FOC_ControllerStep(foc_controller_t *controller)
{
  if ((controller == 0) || (controller->running == 0U))
  {
    return;
  }

  /* 四个回调缺一不可，短路求值保证：只要有一个为空，就不会执行到最后一个
     条件里的 is_power_stage_safe() 调用。 */
  if ((controller->port.is_power_stage_safe == 0) ||
      (controller->port.read_feedback == 0) ||
      (controller->port.write_voltage == 0) ||
      (controller->port.disable_output == 0) ||
      (controller->port.is_power_stage_safe(controller->port.context) == 0U))
  {
    controller->running = 0U;
    FOC_AlgorithmReset(&controller->algorithm);

    /* 修复：进入本分支的原因之一可能就是 disable_output 为空指针，
       原先这里无条件调用会对空指针解引用并触发 HardFault。
       端口不完整时能做的只有停环与复位积分器，关断动作必须判空后执行。 */
    if (controller->port.disable_output != 0)
    {
      controller->port.disable_output(controller->port.context);
    }
    return;
  }

  controller->port.read_feedback(controller->port.context, &controller->input);
  controller->input.enabled = 1U;
  FOC_AlgorithmStep(&controller->algorithm, &controller->input);
  controller->port.write_voltage(controller->port.context,
                                  &controller->algorithm.output);
}

/**
  * @brief  取最近一次迭代输出的只读指针，供上层做遥测/限幅判断。
  * @retval 控制器为空时返回 0。
  */
const foc_output_t *FOC_ControllerGetOutput(const foc_controller_t *controller)
{
  if (controller == 0)
  {
    return 0;
  }
  return &controller->algorithm.output;
}
