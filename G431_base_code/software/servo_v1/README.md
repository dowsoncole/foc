# servo_v1 工程说明

本目录的架构、L1 外设接口、驱动监控策略和 FOC 算法接口统一维护在仓库根目录的 [README.md](../../README.md)。

Keil 工程入口：`MDK-ARM/servo_v1.uvprojx`

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
