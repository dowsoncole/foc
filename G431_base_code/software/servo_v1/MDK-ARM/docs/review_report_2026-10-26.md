# FOC 日常审查复核 — 2026-10-26

## 基线与范围

仓库：dowsoncole/foc。fetch 后远程 main 为 7ab9e57f600ae67b8f6c57348694fcc4c069c99a，与上次已推送审查相同，没有新增代码提交。
本次为既有问题复核，不将历史问题重复计为新发现。使用独立审查克隆，原开发工作区未被操作。下列位置相对 G431_base_code/software/servo_v1。

## 复核结果

| 问题 | 级别 | 当前状态与证据 |
| --- | --- | --- |
| 工程依赖文件未提交 | P1 | 仍存在。FOC/Inc/foc_algorithm.h:6 包含缺失的 foc_pi.h；Tests/Host/run_tests.ps1:7–10 引用缺失源文件。七个文件存在性检查全部为 False，Git 跟踪清单也无对应文件。 |
| 大偏移归一化无法终止 | P2 | 仍存在。Modules/Encoder/Src/encoder.c:64–74 使用反复加减 2π；308–325 仅检查有限性后，在关闭中断期间归一化。float32 单步复核显示 1e10f 减去 2π 后保持不变。 |
| 校准结束时多字段读取不一致 | P2 | 相关路径仍未修改。Modules/Current_Sampling/Src/current_sampling.c:123–136 在 COLLECTING 状态分别读取累加和与计数；超时且未满目标样本时仍可能被采样 ISR 插入。沿用 10-18 报告的触发条件，未新增实机证据。 |
| ISR 故障邮箱读后清零丢事件 | P2（接口潜在风险） | Servo/Src/servo_manager.c:153–154 仍分步读取及清空 pending_faults，与 126 行 ISR 置位存在交错窗口。沿用 10-19 报告的接口适用范围，不声称实际运行已发生。 |
| 待机参数为 0 不清旧使能位 | P2（条件性） | Modules/Driver_Communication/Src/stspin32g4.c:183 仍只处理非零参数。入口时旧位为 1 的条件下存在配置残留，硬件状态保持情况未实测。 |

缺失文件：
- FOC/Inc/foc_pi.h
- FOC/Src/foc_pi.c
- FOC/Inc/foc_current_tuning.h
- FOC/Src/foc_current_tuning.c
- Tests/Inc/current_loop_test.h
- Tests/Src/current_loop_test.c
- Tests/Host/test_current_loop.c

影响与边界：
- 缺失头文件和源文件阻断从当前版本树完整构建；仓库中已有二进制不能替代可重现源码构建。
- 编码器问题仅在配置接口收到相应大幅度有限输入时触发，不意味着默认启动必然卡死。数值验证采用 Python struct 模拟 float32 舍入，没有对目标板调用危险输入。
- 其他历史问题没有因本次复核扩大影响范围；开环启停路径本次未重新作完整交错分析。

## 结构优化及处理顺序

1. 先补齐受版本管理的构建输入，在干净检出中验证工程与主机测试引用。为 Keil、CMSIS 和测试脚本增加统一源文件清单或引用存在性检查。
2. 将编码器归一化与参数校验移出关中断区间，采用有界计算或明确输入范围；临界区只发布已验证配置。将数学处理与 HAL 分离后覆盖正负大数、2π 边界和非法值。
3. 为校准快照和故障邮箱定义统一的中断共享数据规则：冻结并复制一致快照、原子取出并清空故障。测试应显式安排 ISR 交错，而不只检查串行调用。
4. 寄存器配置接口应设置完整目标状态，同时覆盖置位和清位；增加“原位为 1，要求为 0”的 mock 场景。
5. 在上述问题修复后再做目标板时序、故障注入和输出生命周期验证。本次只提出建议，不实施代码修复。

## 本次验证

- git fetch 成功，远程无新增提交；独立克隆开始时干净。
- 再次核对七个缺失文件的本地存在性与 Git 跟踪清单。
- 再次检查编码器相关行号并执行 float32 单步验证：subtraction makes progress=False。
- PythonExample 目录执行 python3.15 -m unittest discover -s tests：20 项通过。
- 未执行 C 编译、固件构建或实机测试；缺失源文件问题仍阻断完整构建，不宣称固件验证通过。
- 本次仅新增本报告，提交前检查暂存范围和空白错误。
