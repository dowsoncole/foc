# USART1/RS485 波形发送测试说明

本文用于排查 Keil 调试器中 USART1 的 `RDR`、`TDR` 显示为 0，以及确认正弦波形测试链路是否真正运行。文档只描述当前代码行为，不把调试器截图中的寄存器显示当作持续数据缓存。

## 1. 当前代码路径

初始化和发送路径如下：

1. `main()` 调用 `WaveformGenerator_PortInit(&huart1)`。
2. `waveform_generator.c` 配置 USART1：921600 baud、8 数据位、1 停止位、无校验、收发使能。
3. `HAL_UART_MspInit()` 配置：
   - PA9：USART1_TX / 485_TX
   - PA10：USART1_RX / 485_RX
   - PA8：RS485 DE/nRE，发送前置高，发送完成后置低
4. ADC 注入转换完成后进入 `HAL_ADCEx_InjectedConvCpltCallback()`，再调用 `MainFoc_OnAdcConversion()` 和 `WaveformGenerator_SampleTick()`。
5. 每 20 次 20 kHz 采样生成一个 1 kHz 正弦样本，写入环形队列。
6. 主循环调用 `WaveformGenerator_Task()`，从队列取出样本，通过 `HAL_UART_Transmit()` 发送 8 字节帧。

因此，串口发送依赖 ADC 注入转换中断；仅完成 USART1 初始化并不会自动产生数据。

## 2. Keil 寄存器显示的正确解释

截图中典型值：

- `CR1 = 0x0000000D`：`UE`、`TE`、`RE` 已置位，USART 已使能，收发功能已打开。
- `BRR = 0x000000B6`：在当前 168 MHz 时钟下对应约 921600 baud。
- `ISR` 中的 `TXE`、`TC` 置位：表示发送数据寄存器当前为空、线路当前空闲。
- `TDR = 0`：发送函数已经返回或尚未进入发送函数时，TDR 为空是正常现象；TDR 不是发送历史记录。
- `RDR = 0`：当前没有收到外部字节是正常现象。当前固件没有调用 `HAL_UART_Receive()`，也没有 USART1 接收中断，因此不会主动消费或处理接收数据。

调试器暂停在任意时刻时，不能以 `TDR/RDR` 是否为 0 判断串口“从未发送/接收”。应观察发送函数断点、队列计数、USART 状态位和外部总线。

## 3. Keil 侧重测试流程

### 3.1 上电前检查

- 不接电机功率级，保持 `MOE=0`。
- 使用支持 921600 baud 的 USB-RS485 适配器。
- 适配器设置为 `921600, 8-N-1`，关闭硬件流控。
- 确认适配器 A/B 极性和板端 RS485 收发器共地。
- PA8 的有效电平按当前代码假设为高电平发送、低电平接收。

### 3.2 代码断点和 Watch 窗口

按以下顺序设置断点或观察变量：

1. `WaveformGenerator_PortInit()` 返回处：确认返回 `HAL_OK`。
2. `HAL_UART_MspInit()`：确认进入 `USART1` 分支。
3. `HAL_ADCEx_InjectedConvCpltCallback()`：确认 `g_current_sampling.sample_sequence` 持续递增。
4. `WaveformGenerator_SampleTick()`：确认 `s_source_count` 每 20 次归零，`s_head` 递增。
5. `WaveformGenerator_Task()` 中 `HAL_UART_Transmit()` 前：确认 `s_tail != s_head`。
6. `HAL_UART_Transmit()` 返回后：确认 PA8 先为高，随后等待 `TC` 后恢复低。

推荐 Watch 项：

```text
g_current_sampling.sample_sequence
g_current_sampling.calibration_state
s_source_count
s_head
s_tail
huart1.State
USART1->CR1
USART1->BRR
USART1->ISR
USART1->TDR
USART1->RDR
GPIOA->ODR bit 8
```

### 3.3 发送链路验收

外部串口工具应收到连续帧：

```text
A5 5A 01 seq sample_lo sample_hi 00 crc8
```

验收条件：

- 帧头固定为 `A5 5A`。
- `seq` 连续递增（8 位溢出后从 0 重新开始）。
- `sample` 为有符号 16 位数，约每 1 秒完成一个正弦周期。
- CRC8 按前 7 字节计算，生成多项式为 `0x07`。
- 示波器观察 PA9 或 RS485 A/B 线时，能看到约 1 kHz 的数据帧活动；PA8 在帧发送期间为高电平。

### 3.4 Keil 中看不到任何发送时的排查顺序

1. `g_current_sampling.sample_sequence` 不递增：先排查 TIM1_CH4 触发、ADC JEOS 中断和 `ADC1_2_IRQHandler()`，不要先排查 USART。
2. `sample_sequence` 递增但 `s_head == s_tail`：确认是否仍处于校准的 `SETTLING/COLLECTING` 状态；校准期间回调被刻意屏蔽。
3. 队列有数据但 `HAL_UART_GetState()` 非 `HAL_UART_STATE_READY`：检查是否卡在上一次发送或 USART 外设错误状态。
4. `HAL_UART_Transmit()` 返回 `HAL_TIMEOUT/HAL_ERROR`：检查 USART1 时钟、PA9 复用、波特率时钟源以及 RS485 DE 电平。
5. MCU 端发送函数成功但 PC 无数据：检查 A/B 线极性、适配器方向、终端电阻和共地；用示波器直接看 PA9 可以区分 MCU 与总线侧问题。

## 4. RDR 接收专项测试

当前代码是“发送测试”，不是完整收发协议。要验证 `RDR`：

1. 外部 RS485 主机向板端发送任意字节。
2. 在调试器中观察 `USART1->ISR` 的 `RXNE` 和 `USART1->RDR`。
3. 由于当前固件没有接收 API/中断，`RDR` 可能保持接收字节，直到调试器或后续代码读取它；这不代表发送链路异常。

如果后续需要命令控制波形，必须新增接收状态机（建议中断或 DMA），并明确帧格式、超时、CRC 和错误清除策略。

## 5. 测试结论记录模板

```text
日期：
固件版本/提交：
板卡：
USART1 配置：921600 / 8-N-1
ADC sample_sequence 是否递增：是 / 否
WaveformGenerator 队列是否有数据：是 / 否
HAL_UART_Transmit 返回值：
PA8 DE 波形：
PA9 TX 波形：
RS485 适配器收到帧数：
CRC 校验：通过 / 失败
结论：
```
