# 上位机波形协议与模式选择说明

## 1. 设计目标

波形打印属于测试验证层的业务功能。FOC 与采样模块只发布同步数据，不决定“打印哪几条曲线”；`g_waveform_generator` 根据模式选择通道，再由 USART1/RS485 端口发送。

当前支持：

| 模式     |                                                枚举值 | 自动选择的曲线                 |
| -------- | ----------------------------------------------------: | ------------------------------ |
| 关闭     |                      `WAVEFORM_OUTPUT_DISABLED = 0` | 不入队、不发送新帧             |
| 电压开环 | `WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT = 1` | U、V、W 三相电流               |
| 电流开环 |          `WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ = 2` | D 指令、D 反馈、Q 指令、Q 反馈 |
| 自定义   |                        `WAVEFORM_OUTPUT_CUSTOM = 3` | 用户指定的 1～4 个通道         |

波形模式和 FOC 控制模式相互独立。例如控制器处于电流开环时，也可以临时选择自定义模式观察三相 duty。

## 2. 固件侧使用方法

### 2.1 API 选择

```c
WaveformGenerator_SelectMode(WAVEFORM_OUTPUT_VOLTAGE_OPEN_LOOP_UVW_CURRENT);
WaveformGenerator_SelectMode(WAVEFORM_OUTPUT_CURRENT_OPEN_LOOP_DQ);
WaveformGenerator_SelectMode(WAVEFORM_OUTPUT_DISABLED);
```

自定义示例：打印 U 相电流、D/Q 电压输出和电角度。

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

### 2.2 Keil Watch 直接切换

预设模式只需修改：

```text
g_waveform_generator.requested_mode = 1  // 电压开环：UVW 电流
g_waveform_generator.requested_mode = 2  // 电流开环：DQ 指令/反馈
g_waveform_generator.requested_mode = 0  // 关闭
```

ISR 会更新 `active_mode` 和 `active_channels[]`。不要直接写 `active_*` 字段。

自定义模式必须按顺序操作：

```text
g_waveform_generator.custom_channel_count = 3
g_waveform_generator.custom_channels[0] = 1
g_waveform_generator.custom_channels[1] = 8
g_waveform_generator.custom_channels[2] = 13
g_waveform_generator.requested_mode = 3
g_waveform_generator.selection_revision++
```

最后递增 `selection_revision` 相当于“提交配置”，防止 ISR 在配置写到一半时使用它。

## 3. 通道 ID 和单位

| ID | 枚举                      | 原始单位 | 帧中整数单位 |
| -: | ------------------------- | -------- | ------------ |
|  1 | `PHASE_U_CURRENT_A`     | A        | mA           |
|  2 | `PHASE_V_CURRENT_A`     | A        | mA           |
|  3 | `PHASE_W_CURRENT_A`     | A        | mA           |
|  4 | `D_CURRENT_REFERENCE_A` | A        | mA           |
|  5 | `D_CURRENT_FEEDBACK_A`  | A        | mA           |
|  6 | `Q_CURRENT_REFERENCE_A` | A        | mA           |
|  7 | `Q_CURRENT_FEEDBACK_A`  | A        | mA           |
|  8 | `D_VOLTAGE_OUTPUT_PU`   | 标幺     | 0.001 pu     |
|  9 | `Q_VOLTAGE_OUTPUT_PU`   | 标幺     | 0.001 pu     |
| 10 | `PHASE_U_DUTY`          | 0～1     | 0.001        |
| 11 | `PHASE_V_DUTY`          | 0～1     | 0.001        |
| 12 | `PHASE_W_DUTY`          | 0～1     | 0.001        |
| 13 | `ELECTRICAL_ANGLE_RAD`  | rad      | mrad         |
| 14 | `DC_BUS_VOLTAGE_V`      | V        | mV           |

所有值统一按 `int32 = 原始浮点值 × 1000` 编码。上位机解析后除以 1000，再根据通道 ID 显示单位。

## 4. 串口参数

| 参数       | 配置                               |
| ---------- | ---------------------------------- |
| 接口       | USART1，经 PA8 控制的 RS485 收发器 |
| 波特率     | 921600                             |
| 格式       | 8-N-1，无硬件流控                  |
| 字节序     | 小端序                             |
| 源采样率   | 20 kHz                             |
| 默认输出率 | 1 kHz                              |
| 协议版本   | 2                                  |

## 5. V2 可变长度帧

帧长度取决于通道数量 `N`：

```text
frame_length = 12 + 5 * N，N = 1..4
```

|      偏移 | 长度 | 字段                                        |
| --------: | ---: | ------------------------------------------- |
|         0 |    2 | 帧头 `A5 5A`                              |
|         2 |    1 | 版本，固定 `2`                            |
|         3 |    1 | 波形模式                                    |
|         4 |    1 | 通道数量 N                                  |
|         5 |    2 | 帧序号，小端 `uint16`                     |
|         7 |    4 | ADC 源采样序号，小端 `uint32`             |
|        11 |    N | N 个通道 ID                                 |
|      11+N | 4×N | N 个小端 `int32` 数值，与通道 ID 顺序一致 |
| 最后1字节 |    1 | CRC8                                        |

CRC 初值 0，多项式 `0x07`，计算范围为帧头到最后一个数据字节。

常见长度：三相电流 27 字节，DQ 指令/反馈 32 字节。

## 6. Python解析核心

```python
import struct

HEADER = b"\xA5\x5A"

def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (((crc << 1) ^ 0x07) if crc & 0x80 else crc << 1) & 0xff
    return crc

def try_decode(buffer: bytearray):
    pos = buffer.find(HEADER)
    if pos < 0:
        buffer[:] = buffer[-1:] if buffer[-1:] == b"\xA5" else b""
        return None
    del buffer[:pos]
    if len(buffer) < 5:
        return None
    if buffer[2] != 2 or not 1 <= buffer[4] <= 4:
        del buffer[0]
        return None
    count = buffer[4]
    length = 12 + 5 * count
    if len(buffer) < length:
        return None
    frame = bytes(buffer[:length])
    if crc8(frame[:-1]) != frame[-1]:
        del buffer[0]
        return None
    del buffer[:length]
    mode = frame[3]
    frame_seq = struct.unpack_from("<H", frame, 5)[0]
    adc_seq = struct.unpack_from("<I", frame, 7)[0]
    ids = list(frame[11:11 + count])
    values = struct.unpack_from("<" + "i" * count, frame, 11 + count)
    return mode, frame_seq, adc_seq, list(zip(ids, (v / 1000 for v in values)))
```

上位机必须按每帧中的通道 ID 绑定曲线，不能假设第 1 条永远是 U 相电流。模式切换后，队列中可能短暂保留一两帧旧模式数据；按帧内模式和 ID 解析即可安全过渡。

## 7. 固件状态判断

```text
g_waveform_generator.requested_mode
g_waveform_generator.active_mode
g_waveform_generator.active_channel_count
g_waveform_generator.active_channels[]
g_waveform_generator.sent_frames
g_waveform_generator.dropped_samples
g_waveform_generator.transport_errors
g_waveform_generator.selection_errors
```

- `selection_errors` 增长：自定义数量或通道 ID 非法。
- `published_samples` 增长但 `sent_frames` 不增长：检查主循环和 UART 端口。
- `dropped_samples` 增长：发送/主循环消费速度低于入队速度。
- `transport_errors` 增长：底层 UART 忙、超时或发送失败。
