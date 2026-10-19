#ifndef FOC_INTERFACE_H
#define FOC_INTERFACE_H

#include "foc_algorithm.h"

typedef uint32_t foc_fault_flags_t;

#define FOC_FAULT_NONE               0UL
#define FOC_FAULT_CURRENT_INVALID    (1UL << 0)
#define FOC_FAULT_ANGLE_INVALID      (1UL << 1)
#define FOC_FAULT_REFERENCE_INVALID  (1UL << 2)
#define FOC_FAULT_POWER_STAGE        (1UL << 3)
#define FOC_FAULT_PORT_CONFIG        (1UL << 4)
#define FOC_FAULT_ALGORITHM          (1UL << 5)
#define FOC_FAULT_PWM_WRITE          (1UL << 6)

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
     read_feedback       : 每周期先填安培单位的 phase_a / phase_b、
                           electrical_angle_rad、安培单位的 d_reference /
                           q_reference，再分别设置 current_valid、
                           angle_valid、reference_valid。任一无效都禁止输出。
     write_voltage       : 把 foc_output_t 已计算好的三相占空比交给
                           定时器层；成功返回 1，失败返回 0 并锁存故障。
     disable_output      : 绑定 Tim1Pwm_EmergencyStop()，用于安全关断。 */
typedef struct
{
  uint8_t (*is_power_stage_safe)(void *context);
  void (*read_feedback)(void *context, foc_input_t *input);
  uint8_t (*write_voltage)(void *context, const foc_output_t *output);
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
  /* ADC ISR 在“输入全有效 + 算法完成 + CCR 已写入”后发布。 */
  volatile uint8_t feedback_ready;
  /* 电流环 ISR 只锁存快速故障；恢复策略由 Servo Manager 决定。 */
  volatile foc_fault_flags_t fault_flags;
  struct
  {
    /* 当前 PWM/电流环周期，PID 参数仍由 algorithm.parameters 统一管理。 */
    float pwm_period_s;
    uint32_t cycle_count;
    uint32_t last_sample_sequence;
  } timing;
} foc_controller_t;

/* --------------------------------------------------------------------------*/
/* 对上（未来的电流环宿主 / L6）提供的接口                                   */
/* --------------------------------------------------------------------------*/

/* 初始化：清零对象、装入端口、按参数初始化算法。任一入参为空时直接返回。 */
void FOC_ControllerInit(foc_controller_t *controller,
                        const foc_parameters_t *parameters,
                        const foc_port_t *port);

/* 只由 Servo Manager 调用：启动实时电流环，不触碰 MOE。 */
void FOC_ControllerStart(foc_controller_t *controller);

/* 停止：清运行标志、复位积分器，并把零电压与安全关断下发到硬件。 */
void FOC_ControllerStop(foc_controller_t *controller);

/* 单次电流环迭代。必须由 20 kHz 的 ADC 注入序列完成回调调用，
   不能放主循环——主循环没有确定性周期，电流环会失效。
   任一安全条件失效时立即关断 MOE、退出 running 并锁存故障，
   后台 Servo Manager 负责状态切换、上报和复位策略。 */
void FOC_ControllerStep(foc_controller_t *controller);

/* 故障只允许在停止状态由 Servo Manager 显式清除。 */
void FOC_ControllerClearFaults(foc_controller_t *controller);
foc_fault_flags_t FOC_ControllerGetFaults(const foc_controller_t *controller);

/* 取最近一次迭代输出的只读指针，供上层做遥测与限幅判断。 */
const foc_output_t *FOC_ControllerGetOutput(const foc_controller_t *controller);

#endif
