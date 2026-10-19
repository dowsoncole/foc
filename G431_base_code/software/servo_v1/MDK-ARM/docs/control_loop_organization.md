# FOC 动态模式与波形输出组织

## 1. 当前实现

三种控制模式现在全部编译进同一个固件，不再通过宏裁剪：

| 枚举值 | 模式 | 电流反馈 | 电流 PI | 电角度 | 生命周期 |
|---|---|---|---|---|---|
| `FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP` | 电压开环 | 不参与控制 | 不运行 | 虚拟角度 | 测试运行时 |
| `FOC_CONTROL_MODE_CURRENT_OPEN_LOOP` | 电流开环测试 | ADC实际电流 | 运行 | 虚拟角度 | 测试运行时 |
| `FOC_CONTROL_MODE_NORMAL` | 正常模式 | ADC实际电流 | 运行 | 正式角度接口 | Servo Manager |

“电流开环”沿用测试命名；从算法角度看，它是电流闭环、角度开环。

## 2. 统一管理结构

所有模式都由一个全局对象管理：

```c
foc_control_manager_t g_global_mode_control;
```

对象内部结构：

```text
g_global_mode_control
  requested_mode       外部请求的模式
  active_mode          当前实际运行模式
  switch_count         已完成的安全切换次数
  voltage_open_loop    电压开环命令、虚拟角度和斜坡状态
  current_open_loop    dq电流命令、虚拟角度、dq反馈和PI输出电压
  normal_loop          正常模式dq指令、正式电角度及有效标志
  runtime              输出请求、MOE、预充电、故障和引脚诊断
```

默认值：

```text
requested_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP
active_mode    = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP
Id*            = 0.10 A
Iq*            = 0 A
虚拟电角频率   = 2 Hz
runtime.output_request = 0
```

## 3. 动态切换方法

程序或调试器只写：

```c
g_global_mode_control.requested_mode = FOC_CONTROL_MODE_VOLTAGE_OPEN_LOOP;
g_global_mode_control.requested_mode = FOC_CONTROL_MODE_CURRENT_OPEN_LOOP;
g_global_mode_control.requested_mode = FOC_CONTROL_MODE_NORMAL;
```

不得直接修改 `active_mode`。主循环检测到请求变化后依次执行：

1. 清除测试输出请求；
2. 置 `precharge_in_progress`，阻止测试环继续写PWM；
3. 正常模式请求 Servo OFF；
4. 关闭MOE并停止FOC控制器；
5. 在短临界区内清PI、清算法输出并装入新模式参数；
6. 重置虚拟角度和开环斜坡；
7. 更新 `active_mode` 和 `switch_count`；
8. 保持输出关闭，等待新模式再次显式启动。

因此模式切换不会把上一模式的占空比、PI积分或输出请求带进下一模式。

## 4. 三种环路

### 4.1 电压开环

```text
dq电压(V) -> 除以母线电压 -> 逆Park(虚拟角度) -> SVPWM -> PWM
```

参数位于 `g_global_mode_control.voltage_open_loop`。切换后算法可以在MOE关闭时运行，
只有把 `g_global_mode_control.runtime.output_request` 写1才进行预充电并打开输出。

### 4.2 电流开环测试

```text
ADC U/V电流(A) -> Clarke/Park(虚拟角度)
  -> Id/Iq误差 -> dq PI -> dq电压 -> 逆Park/SVPWM -> PWM
```

参数和遥测位于 `g_global_mode_control.current_open_loop`。初始安全限制：

- Id给定0.10 A，Iq给定0 A；
- 电流指令限幅0.5 A；
- 电压矢量限幅0.02 pu，24 V母线约0.48 V；
- Kp=0.05 pu/A，Ki=2.0 pu/(A·s)；
- 积分限幅±0.02 pu。

### 4.3 正常模式

```text
正式电角度 + 上层dq电流指令 + ADC电流
  -> FOC电流内环 -> PWM
Servo Manager负责校准、Servo ON/OFF、MOE和故障恢复
```

正式输入统一放在 `g_global_mode_control.normal_loop`。当前真实编码器同步接口尚未接入，
所以 `angle_valid` 和 `reference_valid` 默认都是0，不会误用假角度启动。

## 5. 波形打印分层

波形功能位于测试验证层，分成两部分：

```text
ADC/算法中断
  -> WaveformGenerator_PublishFromIsr()
     仅降采样、复制变量、入队，不访问外设

主循环
  -> WaveformGenerator_Task()
     编码通用波形帧
  -> waveform_transport_t.send()
     当前绑定USART1端口适配器
```

`waveform_generator.c` 不包含HAL头文件，不知道UART、CAN或RS485。以后增加CAN
或其他485端口时，只需提供新的 `waveform_transport_t.send` 实现。

当前采集内容：

- 电压开环预设：U/V/W三相电流；
- 电流开环预设：D/Q电流指令与D/Q电流反馈；
- 自定义预设：从通道枚举中选择1～4个同步量；
- 源采样率20 kHz；
- 输出采样率1 kHz；
- 各通道按原单位乘1000后编码，帧内通道ID决定名称和单位。

## 6. USART1帧协议

USART1配置为921600、8-N-1。当前板上PA8控制485收发器方向，但该硬件细节只
存在于 `waveform_uart1_port.c`。

波形协议已升级为 V2 可变通道帧，帧长为 `12 + 5*N`（N=1..4），小端序：

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 2 | 帧头 `A5 5A` |
| 2 | 1 | 协议版本，当前为2 |
| 3 | 1 | 波形输出模式 |
| 4 | 1 | 通道数量N，范围1..4 |
| 5 | 2 | 波形帧序号 |
| 7 | 4 | ADC源采样序号 |
| 11 | N | N个通道ID |
| 11+N | 4N | N个有符号int32值，小端，原值×1000 |
| 最后1字节 | 1 | CRC8，多项式0x07 |

状态可在 `g_waveform_generator` 中观察：

- `requested_mode/active_mode`：请求模式与已生效模式；
- `active_channels[]`：当前帧通道语义；
- `published_samples`：收到的20 kHz源样本数；
- `sent_frames`：成功发送帧数；
- `dropped_samples`：队列满丢包数；
- `transport_errors`：底层发送失败次数；
- `queued_samples`：当前队列深度。

后续打印其它变量时，先扩展 `waveform_source_snapshot_t` 和通道枚举，再由自定义模式
选择；不需要修改UART1端口代码。

## 7. 建议测试步骤

1. 上电确认 `g_current_sampling.calibration_state` 为 `CAL_DONE`；
2. 确认三相静态电流接近0 A；
3. PC端以921600、8-N-1接收，确认帧头、CRC和源序号连续；
4. 观察 `g_waveform_generator.dropped_samples` 是否保持0；
5. 模式切换前保持 `g_global_mode_control.runtime.output_request=0`；
6. 写 `requested_mode`，确认 `active_mode`跟随且`switch_count`增加；
7. 电流测试使用限流电源，再把`runtime.output_request`写1；
8. 若正Id给定使Id反馈持续向负方向增长，立即停止并检查采样极性和相序。
