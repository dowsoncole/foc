# USART1 多模式波形输出测试

## 1. 测试目标

验证以下四层相互独立且连接正确：

1. FOC/采样层产生同步业务快照；
2. `g_waveform_generator` 根据模式选择曲线；
3. 波形模块降采样、入队并编码 V2 可变长度帧；
4. USART1/RS485 端口发送，PC 按通道 ID 显示曲线。

USART1 参数为 921600、8-N-1；PA9=TX、PA10=RX、PA8=DE/nRE。

## 2. 固件健康检查

Keil Watch 添加：

```text
g_current_sampling.sample_sequence
g_waveform_generator.requested_mode
g_waveform_generator.active_mode
g_waveform_generator.active_channel_count
g_waveform_generator.active_channels
g_waveform_generator.published_samples
g_waveform_generator.queued_samples
g_waveform_generator.sent_frames
g_waveform_generator.dropped_samples
g_waveform_generator.transport_errors
g_waveform_generator.selection_errors
```

预期：`sample_sequence` 与 `published_samples` 持续增长；`sent_frames` 约每秒增长 1000；稳定运行时 `queued_samples` 较小，错误计数不持续增长。

## 3. 电压开环预设测试

```text
g_waveform_generator.requested_mode = WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT
```

等待一次 ADC 回调后应看到：

```text
active_mode = WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT
active_channel_count = 3
active_channels = {1, 2, 3, 0}
```

PC 每帧应为 27 字节，曲线为 U/V/W 三相电流，单位 A。W 相由 `-(U+V)` 重构，不是独立 ADC 通道。

## 4. 电流开环预设测试

```text
g_waveform_generator.requested_mode = WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ
```

预期：

```text
active_channel_count = 4
active_channels = {4, 5, 6, 7}
```

PC 每帧应为 32 字节，依次显示 D 电流指令、D 电流反馈、Q 电流指令、Q 电流反馈。改变 `g_global_mode_control.current_open_loop.d_current_reference_a` 或 `q_current_reference_a` 后，对应指令曲线应立即改变；打开输出后反馈曲线应跟随。

## 5. 自定义模式测试

推荐调用：

```c
const waveform_channel_t channels[] =
{
  WAVEFORM_CHANNEL_PHASE_U_CURRENT_A,
  WAVEFORM_CHANNEL_D_VOLTAGE_OUTPUT_PU,
  WAVEFORM_CHANNEL_Q_VOLTAGE_OUTPUT_PU,
  WAVEFORM_CHANNEL_ELECTRICAL_ANGLE_RAD
};
WaveformGenerator_ConfigureCustom(channels, 4U);
```

用 Watch 手动配置时，先填写数量和 ID，最后切换模式并递增版本：

```text
custom_channel_count = 4
custom_channels = {1, 8, 9, 13}
requested_mode = WAVEFORM_OUTPUT_CUSTOM
selection_revision++
```

预期 `active_channels` 与自定义配置一致。测试 1、2、3、4 通道配置，帧长应分别为 17、22、27、32 字节。

非法测试：数量设为 0/5，或通道 ID 设为 0/大于等于 `WAVEFORM_CHANNEL_COUNT`。预期拒绝新配置、保留上一次有效 `active_*`，并使 `selection_errors` 增长。

## 6. RS485与协议验收

- 示波器观察 PA8：仅发送期间为高。
- PA9 应有 921600 baud 字节流；差分 A/B 上应有对应活动。
- 帧头固定 `A5 5A`，版本固定 2。
- 通道数 N 为 1～4，帧长为 `12 + 5*N`。
- 通道 ID 后紧跟 N 个小端 `int32`，数值除以 1000 得到原量。
- CRC8 多项式 `0x07`，覆盖最后 CRC 前的全部字节。
- 相邻帧 ADC 序号通常相差 20，对应 20 kHz 降采样到 1 kHz。

完整解析方法见 [pc_waveform_protocol.md](pc_waveform_protocol.md)。

## 7. 无波形时的排查顺序

1. `sample_sequence` 不增长：排查 TIM1_CH4、ADC JEOS 和 `ADC1_2_IRQHandler()`。
2. `published_samples` 不增长：确认校准完成并进入 `MainFoc_OnAdcConversion()`。
3. `selection_errors` 增长：检查模式、通道数量和通道 ID。
4. `published_samples` 增长但无队列：确认模式不是 `DISABLED`。
5. `queued_samples` 增长但 `sent_frames` 不增长：检查主循环是否执行 `WaveformGenerator_Task()`。
6. `transport_errors` 增长：检查 UART 状态、超时和 PA8 方向控制。
7. MCU 发送成功但 PC 无数据：检查 A/B 极性、共地、终端电阻和适配器波特率。

调试器中 `TDR=0` 不代表从未发送；发送完成后 TDR 为空属于正常状态。当前波形业务只负责发送，`RDR=0` 也不表示发送链路异常。
