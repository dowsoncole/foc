# MT6701 / SPI3 临时通信验证

测试业务在 `Tests/Src/mt6701_spi3_test.c`，外设配置在
`Modules/Encoder/Src/spi3_encoder.c`，主循环已经调用初始化和周期任务。
Keil 工程与 CMSIS 工程均已登记源文件。测试默认启用，不请求伺服上电或运动，
不把 MT6701 角度接入现有控制反馈。

## 接线与时序

| MCU 引脚 | MT6701 信号 | 配置 |
| --- | --- | --- |
| PB6 | CSN | 普通推挽输出，低有效，空闲高；软件控制 |
| PB3 | CLK | SPI3_SCK，AF6 |
| PB4 | DO | SPI3_MISO，AF6，内部上拉 |

共地，芯片处于 SSI 接口模式（MODE 引脚为高；核对模块板的 MODE、Z/CSN 接法）。
无需 MOSI，不占用 PB5。PB3/PB4 不可同时用于 SWO/JTAG，调试器使用 SWD。

SPI 配置为 Mode 1（CPOL=0、CPHA=1），MSB first、8 bit、软件 NSS、禁用 NSS pulse。
芯片在 CLK 上升沿更新数据，主机在下降沿采样。使用双线全双工发送 3 个虚拟字节，
只为生成 24 个时钟；不使用会持续产生时钟的主机 RXONLY。
不要直接换成 Mode 2 并保留相同位移：其首个下降沿早于数据更新，会引入首位偏移。

当前 APB1 为 168 MHz，256 分频得到 656.25 kHz；24 个时钟约 36.6 us，
实际片选低电平还包含软件等待和中断抢占。CSN 建立/保持各留至少 1 us。
每 10 ms 在主循环读一帧，HAL 收发超时为 2 ms，中断保持开启。
不在 FOC、ADC 或 TIM1 中断内调用本测试。

协议依据：麦歌恩 [MT6701 Rev.1.9 中文手册](https://atta.szlcsc.com/upload/public/pdf/source/20260616/FEB700C1922508565884FF0A90656B50.pdf)
第 7.8 节。帧格式为 14 位角度、4 位状态、6 位 CRC：

```text
raw_frame[23:10] = angle[13:0]
raw_frame[9:6]   = Mg[3:0]
raw_frame[5:0]   = CRC[5:0]
angle_deg       = angle_raw * 360 / 16384
```

CRC 对前 18 位按 MSB first 计算，多项式 x^6+x+1（0x03），初值 0，无反射或末尾异或。

## 调试器观察

在 Watch 中加入 `g_mt6701_spi3_test`，下载运行后观察：

| 字段 | 判定/用途 |
| --- | --- |
| `initialized` | 1 表示 SPI3 配置成功；不代表芯片有响应 |
| `frame_count` | 通常约每秒增加 100；为 0 时检查主循环是否运行到任务 |
| `communication_ok` | 连续 3 帧 CRC 正确且非全零/全一后为 1；任一错误立即清零 |
| `sample_valid` | 最近一次接收是否为有效帧 |
| `angle_raw` / `angle_deg` | 最近有效角度；手转磁铁应连续变化并在一圈处回绕 |
| `angle_change_count` | 相邻有效帧角度发生变化的次数 |
| `good_frame_count` | 有效帧数量，应持续增加 |
| `spi_error_count` / `last_hal_status` / `last_hal_error` | 主机 HAL 错误；出错后释放 CSN，下一次采样复位 SPI3 重试 |
| `crc_error_count` | 帧校验错误，检查时序、接线或信号质量 |
| `blank_frame_count` | 全零/全一帧，优先检查供电、DO、CSN、MODE |
| `raw_frame` / `rx` | 最近一次 HAL_OK 的原始数据；HAL 失败时保留上一帧 |
| `crc_received` / `crc_calculated` | 接收与计算的 CRC，应相同 |
| `magnetic_field_status` | 0 正常，1 磁场过强，2 太弱，3 保留 |
| `push_detected` / `loss_of_track` | Mg[2] 按压 / Mg[3] 超速或失跟踪 |
| `status_warning_count` | CRC 正确但状态不为零的帧数，不计为通信错误 |
| `last_sample_tick_ms` / `last_good_tick_ms` | 最近采样 / 最近有效数据的毫秒时间，判断是否仍在更新 |

有效角度在错误后保留，读取它时同时检查 `sample_valid` 和时间戳。
`communication_ok` 只说明通信帧连续通过，不保证磁场或角度测量正常。
全零也可能是恰好零角度与正常状态，本临时测试保守视为空帧；转动磁铁后复核。
仅有 `last_hal_status=HAL_OK` 不足以证明芯片通信成功。

逻辑分析仪应看到：PB6 平时高，每约 10 ms 拉低一次；每个低电平窗口内 PB3
恰好 24 个时钟，PB4 有数据。解码选择 Mode 1、MSB first、8 bit。

## 暂停与移除

- 运行时把 `g_mt6701_spi3_test.enabled` 写为 0 暂停，写为 1 恢复。
- 修改 `poll_period_ms` 可改变采样周期，0 按 1 ms 处理。
- 编译时定义 `MT6701_SPI3_TEST_ENABLE=0`，或修改测试头文件中的默认值，
  可移除 main 中的初始化与调度，PB3/PB4/PB6 不再由本测试配置。

## 已执行的软件验证

`Tests/Host/run_mt6701_spi3_tests.ps1` 编译实际的传输与测试源文件，注入独立 CRC
向量、损坏帧、全零/全一、HAL 超时、初始化失败以及 tick 回绕；覆盖采样节拍、
暂停、片选释放与错误恢复。该验证不等同于连接真实 MT6701 的通信实测。

## 2026-10-23 烧录实测结果

烧录及 68,008 字节回读校验成功。默认 Mode 1 下固定 10 秒窗口收发 1,000 帧：
有效帧 0、HAL 错误 0、CRC 错误 1,000。通信验证未通过，尚无有效角度数据。
已恢复默认配置并保持测试运行，电机输出关闭。原始快照、烧录文件及旧固件备份见
[本次实测记录](../../control_analysis/results/mt6701_spi3_verify_20261023_150246/verification.md)。
