# servo_v1 工程说明

本目录的架构、L1 外设接口、驱动监控策略和 FOC 算法接口统一维护在仓库根目录的 [README.md](../../README.md)。

Keil 工程入口：`MDK-ARM/servo_v1.uvprojx`

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
