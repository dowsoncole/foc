# 编码器使用说明

更新：2026-10-20。本说明对应 `codex/encoder-test` 分支中的正式 MT6835 编码器实现。

## 1. 当前实现

编码器已从一次性通信测试改为持续运行模块：

```text
TIM1 周期边界（20 kHz）
  → 启动 SPI1 DMA，不等待
  → MT6835 锁存并返回角度、状态和 CRC
  → DMA 完成中断解析数据
  → 跨圈处理、速度估算、电角度换算
  → 发布一致快照
  → FOC 在 ADC 完成时读取上一份快照
```

原来的 `mt6835_verify.c/.h` 已删除。正式运行期间不再出现“采满 256 帧后停止”的行为。

模块职责：

| 文件 | 职责 |
|---|---|
| `spi1_encoder.c` | SPI1、DMA1 Channel 1/2、CS 和异步传输 |
| `mt6835.c` | MT6835 命令、响应解码、状态和 CRC |
| `encoder.c` | 连续采样、有效性、跨圈、速度、电角度、快照和耗时统计 |
| `main.c` | 正常模式下把编码器电角度交给 FOC |

## 2. 使用前必须配置

初始化已经在 `main()` 中自动执行：

```c
if (Encoder_Init() == 0U)
{
    Error_Handler();
}
```

上层完成方向、极对数和电角零位标定后，在功率输出关闭状态调用：

```c
Encoder_ConfigureMotor(direction, pole_pairs, electrical_offset_rad);
```

参数含义：

- `direction`：机械正方向，取 `+1` 或 `-1`。
- `pole_pairs`：电机极对数，必须大于 0。
- `electrical_offset_rad`：标定得到的电角零位，单位 rad。

未配置前仍会持续读取机械角度，但 `electrical_angle_valid=0`，正常 FOC 不会把机械角误当电角度使用。修改配置会清空跨圈和速度历史，需要重新积累约 1 ms 的速度窗口。

## 3. 上层接口

头文件：`Modules/Encoder/Inc/encoder.h`。

```c
uint8_t Encoder_ReadSnapshot(encoder_snapshot_t *snapshot);
uint8_t Encoder_ReadElectricalAngleForFoc(float *electrical_angle_rad);
void Encoder_GetDiagnostics(encoder_diagnostics_t *diagnostics);
```

`Encoder_ReadSnapshot()` 用于速度环、位置环、通信和调试。主要字段：

| 字段 | 含义 |
|---|---|
| `raw_angle` | MT6835 的 21 位单圈原始角度 |
| `multi_turn_count` | 软件跨圈后的有符号累计计数，断电不保存 |
| `mechanical_angle_rad` | 统一方向后的单圈机械角，范围 `[0, 2π)` |
| `mechanical_speed_rad_s` | 机械角速度，单位 rad/s |
| `electrical_angle_rad` | 极对数和零位换算后的电角度 |
| `sample_cycle` | CS 拉低附近的 DWT 周期计数 |
| `sequence` | 有效帧序号 |
| `fault_flags` | 通信、CRC、报警、过期或未配置 |
| `angle_valid` | 当前机械角有效且没有过期 |
| `speed_valid` | 速度窗口已经建立 |
| `electrical_angle_valid` | 电机参数已配置且当前角度有效 |

返回值为 1 表示当前机械角有效。速度和电角度仍应检查各自有效位。数据超过 3 个 FOC 周期，也就是约 150 μs 未更新，会被判定为过期；断线不会被伪装成零速度。

FOC 使用 `Encoder_ReadElectricalAngleForFoc()`。该接口只复制内存快照和检查时间，不启动 SPI，也不等待 DMA。

## 4. 速度计算

每个有效角度样本先进行跨圈修正，再进入 20 点窗口：

```text
机械速度 = 窗口内累计角度变化 / 窗口实际时间
```

采样率为 20 kHz，速度窗口覆盖约 1 ms；随后使用时间常数 10 ms 的一阶低通。启动或重新配置后，前 20 个样本只有角度有效，速度尚未有效。

当前硬件没有把 MT6835 的 A/B 脉冲接回 MCU，因此这里使用 SPI 绝对角度差分，不是经典 AB 脉冲 M/T 法。

## 5. 编码器在什么时刻通信

通信由 TIM1 更新事件启动，也就是每个 PWM 周期的固定边界，频率 20 kHz、周期 50 μs：

```text
TIM1 周期边界：CS 拉低并启动 DMA
     ↓ 48 个 SCK
DMA 完成：释放 CS，解析并发布快照
     ↓
ADC 注入转换完成：FOC 读取最近完整快照
```

SPI1 时钟为 10.5 MHz，一帧 6 字节、48 个 SCK，理论线上时间：

```text
48 / 10.5 MHz = 4.57 μs
```

DMA 完成中断优先级为 2，ADC/FOC 优先级为 1。编码器完成处理不能抢占 FOC。若 DMA 在 ADC 中断到来时尚未完成，FOC 使用上一份有效快照，并记录一次 `foc_reads_while_busy`。

## 6. 如何测量对 FOC 的影响

使用：

```c
encoder_diagnostics_t diagnostics;
Encoder_GetDiagnostics(&diagnostics);
```

使用 Keil Watch 时也可以直接观察只读镜像 `g_encoder_snapshot` 和
`g_encoder_diagnostics`；业务代码仍应调用接口，禁止直接写这两个对象。

周期数除以 `SystemCoreClock` 得到秒；当前 168 MHz 下，周期数除以 168 得到微秒。

重点字段：

| 字段 | 判断内容 |
|---|---|
| `transaction_cycles_latest/min/max` | 从启动到 DMA 完成回调的实际通信墙钟时间 |
| `start_cycles_latest/max` | TIM1 ISR 中启动 DMA 的 CPU 开销 |
| `completion_cycles_latest/max` | DMA 回调解析、测速和发布快照的 CPU 开销 |
| `foc_read_cycles_latest/max` | FOC 读取快照增加的 CPU 开销 |
| `foc_sample_age_cycles_latest/max` | FOC 使用的角度距离采样时刻有多旧 |
| `foc_isr_cycles_latest/max` | 完整 ADC/FOC ISR 的执行时间 |
| `foc_deadline_overrun_count` | FOC ISR 是否达到或超过 50 μs |
| `busy_trigger_count` | 新周期到来时上一次通信是否仍未结束 |
| `crc_error_count` | CRC 错误数量 |
| `transfer_error_count` | DMA/SPI 错误或超时数量 |

判断标准：

1. `foc_deadline_overrun_count` 必须保持 0。
2. `foc_isr_cycles_max` 必须明显小于 `168 MHz / 20 kHz = 8400 cycles`。
3. `busy_trigger_count` 正常应为 0。
4. `transaction_cycles_max` 应明显小于 8400 cycles；理论线传输约 768 cycles。
5. `foc_read_cycles_max` 应只占 50 μs 预算的一小部分。
6. 持续观察 `crc_error_count` 和 `transfer_error_count`，不能只看一瞬间。

`ADC1_2_IRQHandler` 仍通过 PA0 输出高电平测试脉冲，可以用示波器同时核对 `foc_isr_cycles_max`。当前编译通过只能证明代码和链接正确；以上实际数值需要烧录后在目标板测量。

## 7. 故障行为

- SPI/DMA 失败、CRC 错误或 MT6835 报警：立即撤销角度、速度和电角度有效性。
- 超过 100 μs 未完成的事务由后台服务中止并记录通信错误。
- 超过约 150 μs 没有新样本：读取时增加 `ENCODER_FAULT_STALE`。
- 正常 FOC 读取不到有效电角度时，现有 FOC 安全逻辑锁存 `FOC_FAULT_ANGLE_INVALID` 并关闭输出。

详细实现决策和回溯记录见 [编码器实现日志](encoder_analysis_log_2026-10-20.md)。
