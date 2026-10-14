# G431 舵机驱动固件

本工程面向 STM32G431VBTx + STSPIN32G4 三相栅极驱动器，使用 Keil MDK 和 STM32 HAL。代码按“上层业务调用下层接口、下层不反向调用上层”的方向组织。

## 1. 工程入口与分组

Keil 工程：`software/servo_v1/MDK-ARM/servo_v1.uvprojx`

Keil 工程中的分组与磁盘目录保持一致：

```text
Application/Startup
Application/Core
Modules/Driver Communication
Modules/Three-Phase Driver
FOC
Tests
Drivers/STM32G4xx_HAL_Driver
Drivers/CMSIS
```

`FOC` 是独立分组，不放在 `Core` 中。每次新增或移动源文件时，必须同时更新 `servo_v1.uvprojx` 和 `servo_v1.cproject.yml`，再在 Keil 工程树中确认分组与文件路径一致。

## 2. 分层和依赖方向

```text
Application/Core
    ↓ 调用
Modules/Driver Communication       L1 配置、L2 器件接口、L3 监控
Modules/Three-Phase Driver          L1 TIM1 配置、L4 PWM 业务
FOC                                 L5 纯算法和硬件端口中间层
Tests                               验证业务
    ↓ 使用
STM32 HAL / CMSIS / 芯片寄存器
```

具体约束：

- `main.c` 可以调用所有下层公开接口，负责初始化顺序和任务调度。
- `gpio.c`、`i2c3_bus.c`、`tim6_heartbeat.c`、`tim1_pwm.c` 不得包含 `main.h`，也不得调用 `Error_Handler()`。初始化失败通过返回值交给 `main.c` 处理。
- `main.h` 只属于应用入口，不再保存 GPIO 引脚定义。
- GPIO 定义和 GPIO 初始化接口属于 `Core/Inc/gpio.h` / `Core/Src/gpio.c`。
- `FOC/Src/foc_algorithm.c` 不包含 HAL、GPIO、定时器或器件驱动头文件；硬件访问只能通过 `foc_port_t` 回调完成。

## 3. 目录职责

```text
software/servo_v1/
├── Core/
│   ├── Inc/main.h                  应用声明和 Error_Handler
│   ├── Inc/gpio.h                  GPIO 引脚定义与 GPIO 初始化声明
│   └── Src/main.c                  初始化编排、主循环、后台任务
├── Modules/
│   ├── Driver_Communication/
│   │   ├── Inc/i2c3_bus.h          I2C3 配置接口
│   │   ├── Inc/tim6_heartbeat.h    TIM6 1 秒心跳配置接口
│   │   ├── Inc/stspin32g4.h        芯片寄存器接口
│   │   └── Inc/drv_monitor.h       驱动状态机和监控快照
│   └── Three_Phase_Driver/
│       ├── Inc/tim1_pwm.h          三相 PWM 配置和功率级接口
│       └── Src/tim1_pwm.c
├── FOC/
│   ├── Inc/foc_algorithm.h         参数、输入、输出、算法对象
│   ├── Inc/foc_interface.h         算法控制器和硬件端口
│   └── Src/
└── Tests/                          产线或调试验证，不属于算法层
```

## 4. L1 外设配置接口详解

L1 只配置时钟、GPIO、复用、NVIC 和外设句柄，不做状态判断，不访问器件寄存器。初始化函数返回 `HAL_OK` 或错误码；只有应用层 `main()` 决定错误处理方式。

### 4.1 GPIO：`gpio.h` / `gpio.c`

```c
void MX_GPIO_Init(void);
```

`MX_GPIO_Init()` 开启 GPIOA、GPIOE、GPIOF 时钟并配置栅极驱动控制脚：

```c
#define GD_WAKE_Pin          GPIO_PIN_7
#define GD_WAKE_GPIO_Port    GPIOE
#define GD_READY_Pin         GPIO_PIN_14
#define GD_READY_GPIO_Port   GPIOE
#define GD_NFAULT_Pin        GPIO_PIN_15
#define GD_NFAULT_GPIO_Port  GPIOE
```

这些定义位于 `Core/Inc/gpio.h`，不位于 `main.h`。有效电平约定如下：

| 信号          | 方向          | 有效电平 | 含义         |
| ------------- | ------------- | -------- | ------------ |
| `GD_WAKE`   | MCU → 驱动器 | 高       | 退出待机     |
| `GD_READY`  | 驱动器 → MCU | 高       | 驱动器可工作 |
| `GD_NFAULT` | 驱动器 → MCU | 低有效   | 0 表示故障   |

### 4.2 I2C3：`i2c3_bus.h` / `i2c3_bus.c`

```c
extern I2C_HandleTypeDef hi2c3;
HAL_StatusTypeDef MX_I2C3_Init(void);
```

该接口只初始化 I2C3 和 PC8/PC9（AF8）。它不知道 STSPIN32G4 的寄存器语义，器件读写由 `stspin32g4.c` 完成。I2C 总线只允许在初始化阶段和 L3 的低频监控任务中使用，禁止放进 PWM/电流环 ISR。

### 4.3 TIM6：`tim6_heartbeat.h` / `tim6_heartbeat.c`

```c
extern TIM_HandleTypeDef htim6;
HAL_StatusTypeDef MX_TIM6_Init(void);
```

TIM6 以 10 kHz 计数、10000 个计数产生 1 秒更新中断。中断只置位 `g_drv_monitor_tick`，不进行 I2C 传输、不运行状态机、不调用延时函数。

### 4.4 TIM1：`tim1_pwm.h` / `tim1_pwm.c`

```c
extern TIM_HandleTypeDef htim1;
HAL_StatusTypeDef MX_TIM1_Init(void);
```

`MX_TIM1_Init()` 配置并启动 20 kHz 中央对齐互补 PWM、500 ns 死区和 CH4 采样触发。它保持 `MOE=0`，因此初始化不会接通功率级。`main()` 必须检查返回值。六路栅极 GPIO 由 HAL 调用 `HAL_TIM_MspPostInit()` 配置。

## 5. L2 器件接口：STSPIN32G4

`stspin32g4.c` 只封装寄存器访问和控制脚语义，提供：

```c
HAL_StatusTypeDef STSPIN32G4_ReadRegister(uint8_t reg, uint8_t *value,
                                           uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_WriteRegister(uint8_t reg, uint8_t value,
                                            uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Init(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_ClearFaults(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_Wake(uint32_t timeout);
HAL_StatusTypeDef STSPIN32G4_EnterStandby(uint8_t stby_reg_en,
                                           uint32_t timeout);
uint8_t STSPIN32G4_IsReady(void);
uint8_t STSPIN32G4_IsFault(void);
uint8_t STSPIN32G4_PinsAllowSwitching(void);
```

`STSPIN32G4_PinsAllowSwitching()` 一次读取 GPIOE 的输入寄存器，同时检查 `READY=1` 和 `nFAULT=1`。它不访问 I2C，适合实时路径；详细 STATUS、VDS、热关断、欠压和通信错误仍由 L3 低频任务读取。

## 6. L3 驱动监控与故障策略

### 6.1 初始化和后台任务

```c
HAL_StatusTypeDef DrvMonitor_Init(void);
void DrvMonitor_Task(void);
void DrvMonitor_PostRequest(drv_request_t request);
void DrvMonitor_RequestStandby(uint8_t stby_reg_en);
HAL_StatusTypeDef DrvMonitor_GetSnapshot(drv_monitor_t *snapshot);
drv_state_t DrvMonitor_GetState(void);
uint8_t DrvMonitor_IsPowerStageSafeFast(void);
```

`DrvMonitor_Init()` 在无实时任务时清除上电锁存故障、检查器件状态并启动 TIM6。`DrvMonitor_Task()` 在主循环中反复调用；没有 1 秒 tick 时立即返回，有 tick 时才执行一次状态寄存器轮询和状态机推进。

监控状态包括：`INIT`、`STANDBY`、`READY`、`FAULT`、`COMM_LOST`。详细寄存器错误通过 `drv_monitor_t` 快照发布。工作副本只在后台任务中修改，发布和读取快照使用短临界区，因此中断不会读到半更新结构体。

### 6.2 电流环中的快速检查

电流环不应每次通过 I2C 查询 STATUS。I2C 传输有不可预测的等待时间，会破坏 20 kHz PWM 时序。实时路径只检查两个硬件输入：

```c
if (DrvMonitor_IsPowerStageSafeFast() == 0U)
{
    /* 通过 FOC 端口关闭 MOE，禁止继续输出 */
}
```

该接口只读 READY 和 nFAULT GPIO，执行时间固定，不改变监控快照，也不触发通信。I2C 详细诊断、故障寄存器锁存和通信重试统一放在 1 秒 `DrvMonitor_Task()` 中。这样形成两条互补路径：GPIO 负责每个 PWM 周期的快速关断条件，I2C 负责低频故障原因和诊断信息。

## 7. L4 三相 PWM 业务接口

```c
HAL_StatusTypeDef Tim1Pwm_SetPhaseDuty(uint32_t channel, uint16_t duty);
uint16_t Tim1Pwm_GetPhaseDuty(uint32_t channel);
void Tim1Pwm_SetNeutral(void);
void Tim1Pwm_EnableOutputs(void);
void Tim1Pwm_EmergencyStop(void);
uint8_t Tim1Pwm_IsOutputEnabled(void);
```

占空比范围为 0~1000，500 表示中性点。`SetPhaseDuty()` 只做一次比较寄存器写入，可由实时路径调用。`EnableOutputs()` 只能在监控确认 READY 且 nFAULT 未触发后调用；任何快速故障都应调用 `Tim1Pwm_EmergencyStop()` 清除 MOE。

## 8. FOC 算法层和中间层接口

FOC 分为两部分：`foc_algorithm.c` 是纯数学算法，不知道 MCU、HAL、GPIO、I2C 和 TIM1；`foc_interface.c` 是控制器生命周期和调度，通过回调把算法与硬件隔离。

### 8.1 算法参数结构体

```c
typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_min;
    float integral_max;
} foc_pi_t;

typedef struct {
    float sample_period_s;
    float voltage_limit;
    float current_limit;
    foc_pi_t d_controller;
    foc_pi_t q_controller;
} foc_parameters_t;
```

`sample_period_s` 是电流环实际调用周期，单位秒；`voltage_limit` 限制 d/q 合成电压矢量的幅值；`current_limit` 限制 d/q 参考值；两个 PI 控制器分别维护自己的积分状态和积分上下限。参数属于算法对象，不放在 `main.h` 或外设驱动中。

### 8.2 算法输入和输出

```c
typedef struct {
    float phase_a;
    float phase_b;
    float electrical_angle_rad;
    float d_reference;
    float q_reference;
    uint8_t enabled;
} foc_input_t;

typedef struct {
    float d_current;
    float q_current;
    float d_voltage;
    float q_voltage;
    float alpha_voltage;
    float beta_voltage;
    float phase_a_voltage;
    float phase_b_voltage;
    float phase_c_voltage;
    uint8_t saturated;
} foc_output_t;
```

输入的 `phase_a/phase_b` 是两相采样电流，算法先计算 Clarke 变换，再按 `electrical_angle_rad` 做 Park 变换得到 d/q 电流。PI 输出经过电压矢量限幅后执行反 Park 变换并生成三相电压指令。`enabled=0` 时积分器和输出复位，避免停机后积分项残留。

### 8.3 纯算法 API

```c
void FOC_AlgorithmInit(foc_algorithm_t *algorithm,
                       const foc_parameters_t *parameters);
void FOC_AlgorithmReset(foc_algorithm_t *algorithm);
void FOC_AlgorithmStep(foc_algorithm_t *algorithm,
                       const foc_input_t *input);
```

上电或参数变更后调用一次 `FOC_AlgorithmInit()`；停机、故障或切换控制模式时调用 `FOC_AlgorithmReset()`；每个固定采样周期调用一次 `FOC_AlgorithmStep()`，不要在主循环中调用电流环。算法函数只处理内存中的结构体，不能读 GPIO、读 I2C 或写 PWM 寄存器。

### 8.4 FOC 中间层端口

```c
typedef struct {
    uint8_t (*is_power_stage_safe)(void *context);
    void (*read_feedback)(void *context, foc_input_t *input);
    void (*write_voltage)(void *context, const foc_output_t *output);
    void (*disable_output)(void *context);
    void *context;
} foc_port_t;
```

端口是 FOC 与底层的唯一接缝：`is_power_stage_safe` 调用 `DrvMonitor_IsPowerStageSafeFast()` 只读两个 GPIO；`read_feedback` 读取 ADC 电流、编码器角度和 d/q 参考；`write_voltage` 将三相电压换算为 0~1000 占空比并调用 `Tim1Pwm_SetPhaseDuty()`；`disable_output` 调用 `Tim1Pwm_EmergencyStop()` 清除 TIM1 MOE。

### 8.5 控制器 API 和执行流程

```c
void FOC_ControllerInit(foc_controller_t *controller,
                        const foc_parameters_t *parameters,
                        const foc_port_t *port);
void FOC_ControllerStart(foc_controller_t *controller);
void FOC_ControllerStop(foc_controller_t *controller);
void FOC_ControllerStep(foc_controller_t *controller);
const foc_output_t *FOC_ControllerGetOutput(
                        const foc_controller_t *controller);
```

推荐执行顺序：

```text
TIM1_CH4 采样时刻
  └─ FOC_ControllerStep()
       ├─ is_power_stage_safe()       GPIO 快速安全检查，无 I2C
       ├─ read_feedback()             获取 ADC/编码器反馈
       ├─ FOC_AlgorithmStep()         Clarke → Park → PI → 反 Park
       └─ write_voltage()             输出三相 PWM 指令
```

如果安全回调缺失、返回 0，或其他必需回调为空，控制器会复位算法、停止运行并调用 `disable_output()`。因此 FOC 控制器不会在没有硬件安全门的情况下继续驱动功率级。`FOC_ControllerStop()` 同样先复位算法，再写出零输出并关闭功率级。

## 9. `main()` 初始化和任务调度

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
if (MX_I2C3_Init() != HAL_OK) Error_Handler();
if (MX_TIM6_Init() != HAL_OK) Error_Handler();
if (MX_TIM1_Init() != HAL_OK) Error_Handler();
if (DrvMonitor_Init() != HAL_OK) Error_Handler();
```

主循环只运行后台任务：

```c
while (1)
{
    DrvMonitor_Task();
    Tim1PwmVerify_Task(&g_measurement_manager);
}
```

实时 FOC 不应放进这个循环，而应由 TIM1 采样/比较中断驱动。主循环中的监控任务没有 tick 时立即返回；有 tick 时才做 I2C 访问。

## 10. 修改和验证规则

1. 修改引脚、时钟、外设初始化时，优先修改 CubeMX 配置并重新生成，再检查 L1 返回值和 Keil 分组。
2. 新增业务模块时，头文件只暴露业务接口，不从 `main.h` 借用宏。
3. 任何实时 ISR 禁止 I2C、`HAL_Delay()`、无界循环和日志输出。
4. FOC 算法修改后必须验证 PI 积分限幅、电压矢量限幅和 `enabled=0` 复位路径。
5. 使用 Keil 的 Build Target 构建，确认 `0 Error(s), 0 Warning(s)`；再检查工程树中的分组和源文件路径没有出现旧文件名。
