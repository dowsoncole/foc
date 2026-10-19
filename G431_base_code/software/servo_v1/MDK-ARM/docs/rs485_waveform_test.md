# USART1 三相电流波形输出测试

## 1. 功能边界

波形测试层与通信硬件已经解耦：

- `waveform_generator.c`：降采样、队列、帧编码和CRC，不包含HAL；
- `waveform_uart1_port.c`：USART1初始化、HAL发送以及PA8方向控制；
- `MainFoc_OnAdcConversion()`：读取同步的U/V/W三相电流并发布给波形层。

替换为CAN或其它串口时，只需实现新的 `waveform_transport_t.send()`。

## 2. 当前硬件和速率

- USART1：921600 baud，8-N-1；
- PA9：TX；
- PA10：RX；
- PA8：板载485收发器DE/nRE，高电平发送；
- ADC源采样率：20 kHz；
- 波形输出率：1 kHz；
- 每帧：U、V、W三相电流。

ADC中断只执行降采样和内存入队，串口发送位于主循环，不延长FOC的阻塞时间。

## 3. 24字节帧

所有多字节字段均为小端序：

```text
偏移  长度  内容
0     2     A5 5A
2     1     协议版本 = 01
3     1     信号组 = 01（三相电流）
4     1     通道数 = 03
5     2     uint16 波形帧序号
7     4     uint32 ADC源采样序号
11    4     int32 U相电流，mA
15    4     int32 V相电流，mA
19    4     int32 W相电流，mA
23    1     CRC8，覆盖字节0..22，多项式07
```

PC端还原安培值：

```text
phase_current_a = signed_current_ma / 1000.0
```

## 4. Watch验证

推荐观察：

```text
g_current_sampling.sample_sequence
g_current_sampling.calibration_state
g_waveform_generator.enabled
g_waveform_generator.published_samples
g_waveform_generator.queued_samples
g_waveform_generator.sent_frames
g_waveform_generator.dropped_samples
g_waveform_generator.transport_errors
huart1.State
USART1->ISR
GPIOA->ODR
```

正常现象：

- `published_samples` 以约20 kHz增加；
- `sent_frames` 以约1 kHz增加；
- `queued_samples` 应保持较低；
- `dropped_samples`、`transport_errors` 应保持0；
- PA8仅在每帧发送期间为高。

可把 `g_waveform_generator.enabled` 写0暂停采集和发送，写1重新启用。

## 5. 排查顺序

1. `sample_sequence`不增加：检查TIM1 CH4、ADC注入触发和JEOS中断；
2. `published_samples`不增加：检查校准是否仍在SETTLING/COLLECTING；
3. `published_samples`增加但`sent_frames`不增加：检查队列和传输错误计数；
4. MCU发送成功但PC无数据：检查PA9、PA8、A/B极性、共地和终端电阻；
5. CRC错误：确认PC按24字节帧、小端字段以及0x07多项式解析。

USART1的TDR只是当前发送寄存器，不保存历史数据。调试器暂停时TDR显示0不能说明
从未发送，应以外部波形和上述计数器为准。
