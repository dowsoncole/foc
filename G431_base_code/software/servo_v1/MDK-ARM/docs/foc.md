# FOC 执行逻辑、接口与数据管理

## 一句话理解

FOC 每次拿到最新的电流和转子角度，计算当前磁场和转矩，再通过 PI 控制器算出需要施加的电压，最后交给三相 PWM 驱动层。

当前工程使用标定后的 S16 电流作为 FOC 输入。电流环、坐标变换、PI 调节和 PWM 比较值更新已经连接起来，但电气角度和 d/q 给定目前仍是固定值，TIM1 的 MOE 也保持关闭。

## 一、实际执行链

```text
TIM1_CH4 硬件事件
    ↓
只触发 ADC 注入序列
    ↓
ADC 完成 U/V 两路 16 倍过采样
    ↓
HAL_ADCEx_InjectedConvCpltCallback()
    ↓
保存 raw_u/raw_v
    ↓
调用 ADC 完成回调
    ↓
MainFoc_OnAdcConversion()
    ↓
FOC_ControllerStep()
    ↓
安全检查 → 读取反馈 → FOC 算法 → 写回 PWM
```

`HAL_TIM_OC_DelayElapsedCallback()` 不在这条链路中。TIM1_CH4 只负责触发 ADC；ADC 完成前不能读取本轮 JDR，也不能执行 FOC。

## 二、FOC 的两个层次

`foc_interface.c` 是控制器接口层，负责生命周期、调度、安全检查和硬件回调；`foc_algorithm.c` 是算法层，只负责数学计算，不直接访问 ADC、GPIO、TIM 或驱动芯片。

主要接口：

```c
FOC_ControllerInit();
FOC_ControllerStart();
FOC_ControllerStop();
FOC_ControllerStep();
FOC_ControllerGetOutput();
```

## 三、硬件接口 `foc_port_t`

```c
typedef struct
{
  uint8_t (*is_power_stage_safe)(void *context);
  void (*read_feedback)(void *context, foc_input_t *input);
  void (*write_voltage)(void *context, const foc_output_t *output);
  void (*disable_output)(void *context);
  void *context;
} foc_port_t;
```

可以把它理解为 FOC 和硬件之间的插座：

```text
is_power_stage_safe  功率级是否安全
read_feedback        读取电流、角度和目标值
write_voltage        把计算出的电压写入 PWM
disable_output      故障时关闭输出
context              传给回调函数的对象地址
```

当前工程的连接关系：

```text
is_power_stage_safe → DrvMonitor_IsPowerStageSafeFast()
read_feedback       → MainFoc_ReadFeedback()
write_voltage       → MainFoc_WriteVoltage()
disable_output      → Tim1Pwm_EmergencyStop()
```

## 四、控制器生命周期

```text
FOC_ControllerInit()
    ↓ 建立参数、算法和硬件接口
FOC_ControllerStart()
    ↓ running = 1
FOC_ControllerStep()
    ↓ 每次 ADC 完成时运行一次
FOC_ControllerStop()
    ↓ 清积分器、输出零电压、关闭功率级
```

`FOC_ControllerStart()` 只设置软件运行标志，不等于打开 MOE。

## 五、`FOC_ControllerStep()` 伪代码

```text
FOC_ControllerStep(controller):
    如果 controller 为空或没有运行:
        返回

    如果回调不完整或功率级不安全:
        running = 0
        清零 PI 积分器
        关闭输出
        返回

    调用 read_feedback()
    input.enabled = 1
    调用 FOC_AlgorithmStep()
    cycle_count += 1
    调用 write_voltage()
```

安全检查在算法之前完成，故障时不会继续使用旧数据输出电压。

## 六、输入数据 `foc_input_t`

```c
typedef struct
{
  int16_t phase_a;
  int16_t phase_b;
  float electrical_angle_rad;
  float d_reference;
  float q_reference;
  uint8_t enabled;
} foc_input_t;
```

含义：

```text
phase_a / phase_b       U/V 两相标定 S16 电流
electrical_angle_rad    转子电气角度
d_reference/q_reference d/q 轴目标电流
enabled                 本次输入是否有效
```

采样模块还会计算 W 相，但当前 Clarke 变换只需要 U/V 两相。

当前工程实际填入：

```text
phase_a = CurrentSampling_ReadFeedback_s16() 的 U 相
phase_b = CurrentSampling_ReadFeedback_s16() 的 V 相
electrical_angle_rad = 0
d_reference = 0
q_reference = 0
```

## 七、输出数据 `foc_output_t`

```c
typedef struct
{
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

它保存最近一次计算结果：

```text
d_current/q_current         d/q 轴电流
d_voltage/q_voltage         PI 输出的 d/q 电压
alpha_voltage/beta_voltage α/β 坐标电压
phase_a/b/c_voltage         三相电压
saturated                   是否触发电压限幅
```

## 八、算法计算顺序

```text
U/V S16 电流
    ↓
Clarke 变换：得到 α/β 电流
    ↓
Park 变换：得到 d/q 电流
    ↓
d_reference - d_current
q_reference - q_current
    ↓
两个 PI 控制器
    ↓
电压限幅
    ↓
反 Park 变换：得到 α/β 电压
    ↓
得到 U/V/W 三相电压
```

关键公式：

```text
alpha_current = phase_a
beta_current  = (phase_a + 2 × phase_b) / √3

d_current = alpha × cos(angle) + beta × sin(angle)
q_current = -alpha × sin(angle) + beta × cos(angle)

alpha_voltage = d_voltage × cos(angle) - q_voltage × sin(angle)
beta_voltage  = d_voltage × sin(angle) + q_voltage × cos(angle)
```

三相电压：

```text
phase_a = alpha
phase_b = -0.5 × alpha + 0.866 × beta
phase_c = -0.5 × alpha - 0.866 × beta
```

## 九、数据是怎样流动的

```text
ADC JDR1/JDR2
    ↓
current_sampling_t 保存 raw_u/raw_v
    ↓
扣除 offset、修正 polarity
    ↓
CurrentSampling_ReadFeedback_s16()
    ↓
MainFoc_ReadFeedback()
    ↓
foc_input_t.phase_a/phase_b
    ↓
FOC 算法
    ↓
foc_output_t
    ↓
MainFoc_WriteVoltage()
    ↓
TIM1 PWM 比较寄存器
```

`foc_controller_t` 是总控制器对象，里面包含：

```text
algorithm  算法参数、PI 积分器和最近输出
port       四个硬件回调
input      最近一次输入
running    是否运行
timing     PWM 周期、运行次数、最近采样序号
```

## 十、当前参数和边界

当前参数：

```text
采样/电流环频率：20 kHz
电压限幅：0.95
电流限幅：10 A
Kp：0.1
Ki：5.0
```

已经具备：

```text
ADC 完成调度
S16 电流反馈
Clarke/Park 变换
d/q PI 控制
电压限幅
PWM 比较值更新
故障快速关断
```

尚未完成：

```text
真实编码器角度接入
速度环和位置环
非零 d/q 给定
正式的功率级 MOE 使能状态机
```

因此当前 FOC 是“电流环控制框架”，还不是完整的位置伺服系统；而且由于 MOE 关闭，计算出的 PWM 比较值不会形成实际栅极驱动波形。

## 十一、先伪代码，再结合源代码

### 1. 完整伪代码

```text
系统启动:
    初始化 MCU、TIM1、ADC、运放和驱动器
    配置 FOC 参数和硬件回调
    初始化 FOC 控制器
    注册 ADC 完成回调
    启动 ADC
    请求零点校准

主循环:
    推进校准状态机
    如果校准完成且 FOC 未运行:
        FOC_ControllerStart()

TIM1_CH4 事件:
    只触发 ADC 开始转换

ADC 注入序列完成:
    读取 JDR1/JDR2
    保存 U/V 原始值
    如果正在校准:
        累加样本
    如果注册了回调:
        调用回调函数

ADC 完成回调:
    调用 MainFoc_OnAdcConversion()
    调用 FOC_ControllerStep()

FOC_ControllerStep:
    如果未运行:
        返回
    检查功率级安全
    如果不安全:
        停止 FOC、清 PI、关 PWM
        返回
    读取 S16 电流、角度和 d/q 给定
    Clarke 变换
    Park 变换
    d/q PI 控制
    电压限幅
    反 Park 变换
    写入三相 PWM 比较值
```

### 2. 对照 `main.c`

初始化由 `HAL_Init()`、`MX_TIM1_Init()`、`MX_ADC1_Init()`、`DrvMonitor_Init()` 等函数完成。FOC 参数和硬件接口在下面的调用中组装：

```c
FOC_ControllerInit(&g_foc_controller,
                   &foc_parameters,
                   &foc_port);
```

ADC 完成回调注册为：

```c
CurrentSampling_SetConversionCallback(MainFoc_OnAdcConversion,
                                      &g_foc_controller);
```

主循环先执行：

```c
CurrentSampling_Service();
```

校准完成后才执行：

```c
FOC_ControllerStart(&g_foc_controller);
```

### 3. 对照 `current_sampling.c`

ADC 完成后进入：

```c
HAL_ADCEx_InjectedConvCpltCallback()
```

此时 JDR1/JDR2 才是本轮完整结果：

```c
g_current_sampling.raw_u = (uint16_t)ADC1->JDR1;
g_current_sampling.raw_v = (uint16_t)ADC1->JDR2;
```

之后，如果存在回调，就执行：

```c
if (s_conversion_callback != NULL)
{
    s_conversion_callback(s_conversion_context);
}
```

当前保存的回调是 `MainFoc_OnAdcConversion`，保存的上下文是 `&g_foc_controller`，所以这行等价于：

```c
MainFoc_OnAdcConversion(&g_foc_controller);
```

这里不经过 `HAL_TIM_OC_DelayElapsedCallback()`；TIM1_CH4 只负责触发 ADC。

### 4. 对照 `main.c` 中的 FOC 回调

```c
static void MainFoc_OnAdcConversion(void *context)
{
    读取三相安培值；
    WaveformGenerator_Publish3FromIsr(...); // 仅入队，不访问串口
    根据 g_foc_control.active_mode 执行电压开环、
        虚拟角度电流环或正式电流环；
}
```

这说明 ADC 完成后，才正式进入一次 FOC 电流环。

### 5. 对照 `foc_interface.c`

```c
FOC_ControllerStep(controller):
    判断 controller 和 running
    调用 is_power_stage_safe
    调用 read_feedback
    设置 input.enabled = 1
    调用 FOC_AlgorithmStep
    cycle_count 加一
    调用 write_voltage
```

对应源代码中的四个端口回调：

```text
is_power_stage_safe → DrvMonitor_IsPowerStageSafeFast()
read_feedback       → MainFoc_ReadFeedback()
write_voltage       → MainFoc_WriteVoltage()
disable_output      → Tim1Pwm_EmergencyStop()
```

### 6. 对照 `foc_algorithm.c`

```c
alpha_current = (float)input->phase_a;
beta_current  = ((float)input->phase_a +
                 2.0f * (float)input->phase_b) *
                0.5773502692f;
```

这对应 Clarke 变换。之后根据电气角度进行 Park 变换，得到 d/q 电流；再用 d/q 目标值减去实际值，分别进入 d 轴和 q 轴 PI 控制器。

最后算法得到三相电压和三相归一化占空比，由 `MainFoc_WriteVoltage()` 调用：

```c
Tim1Pwm_SetPhaseVoltage(output->phase_a_duty,
                         output->phase_b_duty,
                         output->phase_c_duty);
```

于是完整关系是：

```text
ADC 完成
  → 采样模块保存数据
  → 回调通知 MainFoc
  → FOC_ControllerStep
  → FOC_AlgorithmStep
  → MainFoc_WriteVoltage
  → TIM1 比较寄存器
```

## 十二、四个端口回调的通俗解释

FOC 算法通过 foc_port_t 中的四个回调使用硬件：

    is_power_stage_safe：功率级是否安全
    read_feedback：读取电流、角度和目标值
    write_voltage：把计算出的电压写入 PWM
    disable_output：发生故障时关闭输出

### 1. is_power_stage_safe

函数类型：

    uint8_t (*is_power_stage_safe)(void *context);

它回答：现在能不能继续执行 FOC 并输出电压？

当前绑定：

    DrvMonitor_IsPowerStageSafeFast()

伪代码：

    is_power_stage_safe():
        读取 READY
        读取 nFAULT
        如果 READY 和 nFAULT 都正常:
            返回 1
        否则:
            返回 0

FOC_ControllerStep 每次运行前都会调用它。不安全时立即停止 FOC、清零 PI 积分器并关闭输出。这个回调必须很快，不能在实时路径中执行 I2C 或阻塞操作。

### 2. read_feedback

函数类型：

    void (*read_feedback)(void *context, foc_input_t *input);

它负责填写本次 FOC 计算需要的输入：

    U/V/W 电流
    电气角度
    d 轴目标电流
    q 轴目标电流

当前绑定：

    MainFoc_ReadFeedback()

伪代码：

    read_feedback(input):
        调用 CurrentSampling_ReadFeedback_s16()
        input.phase_a = U 相 S16 电流
        input.phase_b = V 相 S16 电流
        input.electrical_angle_rad = 当前电气角度
        input.d_reference = d 轴目标
        input.q_reference = q 轴目标

当前角度和 d/q 给定仍是占位值 0。

### 3. write_voltage

函数类型：

    void (*write_voltage)(void *context,
                          const foc_output_t *output);

它接收算法计算出的 alpha_voltage 和 beta_voltage。

当前绑定：

    MainFoc_WriteVoltage()

内部调用：

    Tim1Pwm_SetPhaseVoltage(output->phase_a_duty,
                             output->phase_b_duty,
                             output->phase_c_duty);

伪代码：

    write_voltage(output):
        读取 alpha_voltage
        读取 beta_voltage
        转换为三相 PWM 比较值
        更新 TIM1 CH1/CH2/CH3

更新比较寄存器不等于功率级已经输出。当前 MOE 关闭时，比较值可以变化，但栅极波形不会被放行。

### 4. disable_output

函数类型：

    void (*disable_output)(void *context);

它负责故障时立即停止功率级输出。

当前绑定：

    Tim1Pwm_EmergencyStop()

伪代码：

    disable_output():
        关闭 TIM1 MOE
        确保 PWM 不再送到栅极驱动器

它可能在功率级不安全、主动停止 FOC、回调接口不完整或驱动器故障时执行。

### 5. 一次正常运行的顺序

    ADC 转换完成
        ↓
    FOC_ControllerStep()
        ↓
    is_power_stage_safe()
        ↓
    read_feedback()
        ↓
    FOC_AlgorithmStep()
        ↓
    write_voltage()

故障时：

    is_power_stage_safe() 返回 0
        ↓
    停止 FOC
        ↓
    清零 PI 积分器
        ↓
    disable_output()

四个回调把硬件职责和 FOC 数学算法分开。同一套 FOC 算法以后可以更换 ADC、PWM、驱动芯片或 MCU，只需要重新绑定这些回调，不必修改 Clarke、Park 和 PI 算法。

## 十三、SPWM 计算与定时器硬件层的边界

本工程把“电压指令如何变成三相占空比”和“占空比如何写进定时器 CCR”分成两层。

FOC 算法层负责：

    α/β 电压
        ↓
    α/β 到 U/V/W 三相电压
        ↓
    零序分量注入
        ↓
    三相归一化占空比 phase_a_duty、phase_b_duty、phase_c_duty

定时器硬件层负责：

    三相归一化占空比 0..1
        ↓
    转换成 0..1000 的驱动层占空比
        ↓
    转换成 TIM1 CCR 比较值
        ↓
    写入 CH1、CH2、CH3

因此，SPWM 或 SVPWM 的计算属于 foc_algorithm.c。Tim1Pwm_SetPhaseVoltage() 只接收 FOC 已经算好的三相占空比，完成占空比标度和 CCR 写入。

算法输出结构中现在包括：

    phase_a_voltage、phase_b_voltage、phase_c_voltage
    phase_a_duty、phase_b_duty、phase_c_duty

其中 voltage 是算法中的三相电压结果，duty 是经过零序平移后的归一化占空比。FOC 的 write_voltage 回调把三个 duty 传给：

    Tim1Pwm_SetPhaseVoltage(phase_a_duty,
                            phase_b_duty,
                            phase_c_duty)

定时器函数内部只做：

    检查输入是否为有效浮点数
    将 0..1 转换为 0..1000
    调用 Tim1Pwm_SetPhaseDuty()
    将 0..1000 转换为 CCR
    写入 TIM1 CH1/CH2/CH3

这样阅读代码时，可以把职责记成：

    foc_algorithm.c：
        计算三相电压和三相占空比

    MainFoc_WriteVoltage：
        通过 write_voltage 回调传递算法结果

    tim1_pwm.c：
        把占空比转换成 CCR，并写入定时器硬件




FOC链条业务逻辑梳理：



算法执行判定：run flag

算法硬件输出判定：moe

foc执行前置条件：电流校准完成（），
