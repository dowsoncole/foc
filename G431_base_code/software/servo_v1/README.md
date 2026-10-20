# servo_v1 工程说明

本目录的架构、L1 外设接口、驱动监控策略和 FOC 算法接口统一维护在仓库根目录的 [README.md](../../README.md)。

Keil 工程入口：`MDK-ARM/servo_v1.uvprojx`

编码器当前分层、业务流程、测速选择和拟定 FOC 接口见
[编码器使用说明](MDK-ARM/docs/encoder/encoder_user_guide.md)；
代码证据、设计约定和后续验收清单见
[2026-10-20 编码器分析记录](MDK-ARM/docs/encoder/encoder_analysis_log_2026-10-20.md)。

电流 PI 模型整定、阶跃/正弦采集、时域指标及闭环带宽分析见
[电流环整定与性能评价手册](MDK-ARM/docs/current_loop_tuning_and_evaluation.md)。

自动调试所需硬件资料、信号、数据格式、设备接口及可填写清单见
[电流环自动调试数据与接口要求](MDK-ARM/docs/automated_hardware_debug_data_requirements.md)。

电流环验收之后的编码器、电角标定、速度环、轨迹与位置环分阶段安排见
[伺服闭环实现计划](MDK-ARM/docs/servo_closed_loop_implementation_plan.md)。

FOC 现有独立调制模块，支持线性 SVPWM、两段过调制与六步极限，基波增益补偿、
板级采样降额和测试方式见 [FOC/README.md](FOC/README.md)。

当天需求、代码、数学推导、测试证据及遗留项详见
[2026-10-18 详细开发记录](MDK-ARM/docs/daily_changes_2026-10-18.md)。

工程分组必须与磁盘目录一致：

```text
Application/Core
Modules/Driver Communication
Modules/Three-Phase Driver
FOC
Tests
Drivers/STM32G4xx_HAL_Driver
Drivers/CMSIS
```

构建前请确认 Keil 工程树中的 `FOC` 为独立分组，且没有残留的 `i2c.c`、`tim.c`、`i2c.h` 或 `tim.h` 路径。
