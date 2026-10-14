#ifndef FOC_INTERFACE_H
#define FOC_INTERFACE_H

#include "foc_algorithm.h"

/* --------------------------------------------------------------------------*/
/* FOC 与硬件之间的唯一接缝（依赖倒置）                                      */
/* --------------------------------------------------------------------------*/
/* 本层刻意不 include 任何芯片/外设头文件：所有与硬件的往来都通过下面这四个
   函数指针完成。更换一套硬件只需替换回调实现，本层与 foc_algorithm.c
   一行都不用改。

   上层注册回调时的约定：
     is_power_stage_safe : 绑定 DrvMonitor_IsPowerStageSafeFast()。
                           必须只读引脚、绝不走 I2C，因为它会在 20 kHz
                           电流环里被调用，执行时间必须恒定。
     read_feedback       : 填 phase_a / phase_b（两相电流，第三相由
                           ia+ib+ic=0 推出）、electrical_angle_rad（编码器
                           机械角 × 极对数）、d_reference / q_reference。
     write_voltage       : 把 foc_output_t 的电压换算成占空比后写入
                           Tim1Pwm_SetPhaseDuty()。
     disable_output      : 绑定 Tim1Pwm_EmergencyStop()，用于安全关断。 */
typedef struct
{
  uint8_t (*is_power_stage_safe)(void *context);
  void (*read_feedback)(void *context, foc_input_t *input);
  void (*write_voltage)(void *context, const foc_output_t *output);
  void (*disable_output)(void *context);
  void *context;
} foc_port_t;

/* 控制器实例：算法 + 端口 + 最近一次输入 + 运行标志 */
typedef struct
{
  foc_algorithm_t algorithm;
  foc_port_t port;
  foc_input_t input;
  uint8_t running;
} foc_controller_t;

/* --------------------------------------------------------------------------*/
/* 对上（未来的电流环宿主 / L6）提供的接口                                   */
/* --------------------------------------------------------------------------*/

/* 初始化：清零对象、装入端口、按参数初始化算法。任一入参为空时直接返回。 */
void FOC_ControllerInit(foc_controller_t *controller,
                        const foc_parameters_t *parameters,
                        const foc_port_t *port);

/* 置运行标志。真正的输出使能由三相驱动层的 MOE 决定，本层不触碰硬件。 */
void FOC_ControllerStart(foc_controller_t *controller);

/* 停止：清运行标志、复位积分器，并把零电压与安全关断下发到硬件。 */
void FOC_ControllerStop(foc_controller_t *controller);

/* 单次电流环迭代。必须由 20 kHz 的 TIM1_CC 中断调用（ADC 采样完成后），
   不能放主循环——主循环没有确定性周期，电流环会失效。
   每步先问 is_power_stage_safe()，一旦不安全立即停环并关断输出。 */
void FOC_ControllerStep(foc_controller_t *controller);

/* 取最近一次迭代输出的只读指针，供上层做遥测与限幅判断。 */
const foc_output_t *FOC_ControllerGetOutput(const foc_controller_t *controller);

#endif
