# foc 代码审查报告

- 仓库：dowsoncole/foc
- 分支：main
- 审查时间：2026-10-17（Asia/Shanghai）
- 范围：FOC、ADC 采样、PWM、驱动监控、编码器和启动流程
- 约束：仅审查并补充文档，未修改业务代码

## 结论

当前版本适合继续做台架验证，但不建议直接带载运行。关键控制路径依赖 STM32 外设和实机时序，本次未宣称目标工程构建通过。

## 潜在问题

### 高风险

1. **FOC 电流环在 ADC 中断中执行浮点和 PWM 写入**：证据为 FOC/Src/foc_algorithm.c:120、Modules/Current_Sampling/Src/current_sampling.c:69、Modules/Three_Phase_Driver/Src/tim1_pwm.c:154。sinf/cosf/sqrtf/isfinite 位于 20 kHz JEOS 回调链路，最坏执行时间可能造成 ADC overrun、PWM 延迟和抖动。建议测量最坏时间，考虑 CMSIS-DSP/查表或定点实现，并设置 overrun 安全停机。

2. **ADC 注入序列使用 2.5 周期采样，模拟建立时间未经硬件验证**：证据为 Modules/Current_Sampling/Src/adc1_current_sampling.c:189。外部 RC、走线和功率开关噪声可能导致采样电容未充分建立。建议比较不同采样周期并用示波器确认触发提前量。

3. **故障诊断与快速关断路径分离，可能自动重新启动**：证据为 Modules/Driver_Communication/Src/drv_monitor.c:297、FOC/Src/foc_interface.c:95、Core/Src/main.c:291。快速 GPIO 路径可停止控制器，但主循环没有不可自动清除的故障锁存。建议增加 fault latch，启动前同时检查驱动状态和锁存原因。

### 中风险

4. **PI 输出限幅复用积分限幅**：证据为 FOC/Src/foc_algorithm.c:24。建议拆分输出限幅和积分限幅，采用 conditional integration 或 back-calculation。

5. **校准窗口过短且无离群/方差检查**：证据为 Modules/Current_Sampling/Src/current_sampling.c:14、Core/Src/main.c:259。1 ms/20 个样本可能把过渡态或噪声写入 offset。建议记录均值、方差、最小/最大值，异常时保持 CAL_ERROR 并禁止启动。

6. **编码器结果未接入 FOC 角度**：证据为 Core/Src/main.c:97。electrical_angle_rad 始终为 0，编码器验证任务与控制环没有数据通路。建议明确开环/闭环模式，闭环必须检查数据新鲜度、CRC 和角度范围。

7. **驱动器故障 API 命名容易反转语义**：证据为 Modules/Driver_Communication/Src/stspin32g4.c:94-103。IsFault() 返回 nFAULT 原始高电平，而调用方将高电平解释为无故障。建议改为语义化命名并加入高低电平测试。

## 结构优化建议

1. 拆成 BSP/HAL、device、control、services、tests 五层，禁止 control 直接包含 STM32 HAL。
2. 将采样时间、比例、极性、限幅和时钟派生值集中到只读配置结构并启动校验。
3. 为 Clarke/Park、SVPWM、PI 饱和、NaN/Inf 和边界角度增加主机端纯 C 测试。
4. 增加 ISR 最大耗时、ADC overrun、PWM 更新、驱动故障锁存和编码器 CRC 错误遥测。
5. 将 .o/.axf/.map/.htm 等生成物移出源码目录或加入忽略规则。
6. 文档统一放在 docs/，为“已处理”事项附代码或测试证据。

## 验证计划

- 测量 JEOS ISR 最坏耗时和 20 kHz 周期裕量。
- 确认 TIM1_CH4、注入序列结束和 PWM 更新的时序关系。
- 注入 READY/nFAULT 拉低、I2C 超时、CRC 错误和 ADC overrun，确认输出保持关闭且故障锁存。
- 统计重复上电时的 offset 均值和方差。
- 使用 Keil/ARM GCC 完整构建；本次环境未发现可直接运行的目标编译器。

## 本次变更

仅新增本审查报告文档；未修改现有 C/H/工程配置文件。
