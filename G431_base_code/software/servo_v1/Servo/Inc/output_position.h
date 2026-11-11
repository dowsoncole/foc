#ifndef OUTPUT_POSITION_H
#define OUTPUT_POSITION_H
#include <stdint.h>
/* Current recipe is defined in output_position.c; current ABI is ELF-derived.
   The long dated notes below describe earlier iterations. In particular their
   "unchanged 2500 rpm", 81 rows and 32-byte FRD statements are historical;
   current ceiling=8500 rpm, trace=64 rows, FRD=40 bytes, trajectory=84 bytes. */
/* Codex 2026-10-28 correction to historical notes below: a causal response
   CAN have positive phase (e.g. velocity feedforward / lead compensation).
   reject=6 is retired; analytic DFT tests, not a phase sign gate, verify sign.
   frequency_hz=-1 selects a finite step with 50 ms pre-trigger. */
/* ==========================================================================
   [千问工作助理 2026-10-28] 位置环带宽优化：解锁增益与动态限幅
   ==========================================================================
   任务：用户要求「±2° 正弦扫频时带宽尽量靠近 20 Hz」。
   本轮已确认的用户决策：
     1) 目标口径 = 尽量靠近 20 Hz（非 ±2° 满幅跟踪）；
     2) 反馈源   = 维持输出轴 MPT，不切电机侧；
     3) 电流保护 = 保持 FLT_MAX，仅硬件 OCP，不恢复软件限流。

   改动 1：新增 OUTPUT_POSITION_MAX_KP_PER_S = 200.0f
   ------------------------------------------------------------------------
   原 output_position_math.c:8 硬编码 `kp>10` 即判参数无效并 trip(3)。
   纯 P 位置环 + 积分器对象的交越频率 fc ≈ kp·|G_speed|/(2π)。
     kp=2.0（默认）→ fc ≈ 0.32 Hz
     kp=10 （原上限）→ fc ≈ 1.59 Hz   ← 带宽被锁死在此，与 20 Hz 差 12.6 倍
   20 Hz 处速度闭环实测增益 −2.00 dB(=0.794)、相位 −21.1°
   （results/speed_response_20261028/balanced_matlab_measured_evaluation.json:247,268），
   要 fc=20 Hz 需 kp = 20·2π/0.794 ≈ 158.3 /s。故上限取 200 留约 26% 余量。

   改动 2：OUTPUT_POSITION_RAMP_RPM_S 2000 → 200000
   ------------------------------------------------------------------------
   幅值 A[rpm]、频率 f 的正弦，其速度指令峰值变化率 = A·2πf。
   斜坡限幅 R 把可跟踪频率上限压到 R/(A·2π)：
     R=2000 时，20 Hz 小信号（电机速度幅值 1152 rpm）需 1152·2π·20
       = 144,765 rpm/s，当前值不足 72 倍 → 正弦被完全削成锯齿，
       任何带宽测量都是饱和伪影，没有意义。
     ±2° 幅值下更是需要 1.06e6 rpm/s（不足 532 倍）。
   取 200,000 rpm/s 可同时覆盖「20 Hz 小信号」(144,765) 与
   「±2° 电流极限 7.23 Hz」(139,000) 两个工况，留约 38% 余量。
   注意：这是速度/加速度限幅，不是电流限幅；max_motor_rpm 与
   硬件 OCP 仍然生效。

   改动 3：轨迹测试支持可配频率（见 output_position_test_t.frequency_hz）
   ------------------------------------------------------------------------
   原实现频率硬编码为 0.1 Hz（output_position.c:95 的 0.6283185307f
   = 2π×0.1），无法做 20 Hz 扫频，属测试能力缺失。

   未改动项及理由：
   - max_motor_rpm 上限保持 2500：20 Hz 小信号（±0.272°）电机峰值仅
     1152 rpm，有 2.2 倍余量；且它同时决定前馈一致性上限（见下）。
   - velocity_deg_s 上限保持 ±75：该值 = 2500×6/202 = 74.26 °/s，是与
     max_motor_rpm 严格一致的物理限幅，非任意值。20 Hz 小信号前馈需求
     仅 34.2 °/s，余量 2.2 倍。
   - 保持纯 P，不加积分：速度环已有积分，位置环再加积分即「堆叠两个
     强积分器」（2026-10-22_闭环整定模块实施方案_规划记录.md:227 明确警告）。
     纯 P + 速度环积分已使稳态误差被限制在死区内（实测 ±1° 阶跃末态
     误差 0.008°~0.022°，见 results/mpt_position_20261026_175813/）。
   ========================================================================== */
/* R14 commissioning ceiling is below the existing 9000 rpm speed ceiling.
   Ordinary default stays 5500; higher values are explicit finite trials. */
#define OUTPUT_POSITION_MAX_MOTOR_RPM 8500.0f
#define OUTPUT_POSITION_MAX_VELOCITY_DEG_S (OUTPUT_POSITION_MAX_MOTOR_RPM*6.0f/202.0f)
#define OUTPUT_POSITION_MAX_KP_PER_S 200.0f
/* R14: ordinary targets/steps keep the lower acceleration. The 1.2M recipe
   applies only to a latched, finite positive-frequency sine. */
#define OUTPUT_POSITION_RAMP_RPM_S 300000.0f
#define OUTPUT_POSITION_SWEEP_RAMP_RPM_S 1200000.0f
/* Codex 2026-10-28: target SRAM is shared with the 512-row speed recorder.
   64 rows recover 272 bytes; host resolves array size from the ELF. */
#define OUTPUT_POSITION_TRACE_CAPACITY 64U
/* [千问工作助理 2026-10-28] 扫频幅值安全上限。位置环 P 输出在
   |error| > max_motor_rpm/(kp·202/6) 时饱和；超出线性区的扫频结果
   是饱和伪影，不能当作带宽。kp=158.3、max=2500 时线性区为 ±0.47°。 */
#define OUTPUT_POSITION_SWEEP_MAX_AMPLITUDE_DEG 90.0f
typedef struct {
  /* command: 1=arm at current position, 2=stop, 3=clear fault while OFF.
     Boot automatically calibrates and arms at the current position.
     Set target_deg AFTER state becomes 1 and the speed inner loop is running. */
  uint32_t command, state, fault;
  /* Explicit commissioning gate: set only after real CRC/angle/direction
     and timing validation. Direction is +1/-1 motor RPM for increasing angle. */
  uint32_t commissioned;
  int32_t motor_direction;
  float target_deg, actual_deg, error_deg, motor_rpm;
  float kp_per_s, max_motor_rpm, deadband_deg;
  uint32_t updates;
  /* Live output-shaft velocity feedforward, deg/s. Zero for fixed targets. */
  float velocity_deg_s;
} output_position_t;
extern volatile output_position_t g_output_position;
/* Optional finite commissioning trajectory. command 1 starts while position
   mode is active; 2 aborts and stops. No automatic start at boot.
   10 s smooth move to -amplitude, 2 cosine cycles at 0.1 Hz, 10 s return. */
/* [千问工作助理 2026-10-28] 追加 frequency_hz / trace_interval_ms /
   saturated_samples / peak_motor_rpm 四个字段于结构体末尾，原有字段偏移
   保持不变。
   - frequency_hz = 0（默认）时行为与改动前完全一致：2 个 0.1 Hz 余弦周期，
     总时长 40 s（10 s 平滑入 + 20 s 正弦 + 10 s 平滑出）。
   - frequency_hz > 0 时按该频率生成正弦，用于带宽扫频；此时不做 10 s
     平滑过渡（高频下平滑段相对周期过长会淹没有效数据），直接从中心起振，
     运行 duration_s 后结束。duration_s = 0 时默认取 max(2 s, 4 个周期)。
   - trace_interval_ms = 0（默认）时沿用原 500 ms 记录间隔。
     20 Hz 扫频必须显著缩短（建议 5~10 ms）才能采到波形；500 ms 间隔在
     20 Hz 下每个周期只有 0.1 个点，无法分析。
   - saturated_samples / peak_motor_rpm 用于判定扫频是否进入限幅：
     饱和样本占比必须为 0，否则测出的「带宽」是饱和伪影，不可采信。 */
typedef struct {
  uint32_t command, state, fault; /* state: 0 idle, 1 running, 2 done, 3 failed */
  float amplitude_deg, center_deg, elapsed_s, peak_error_deg, squared_error;
  uint32_t samples, trace_count;
  /* --- [千问工作助理 2026-10-28] 以下为本轮追加，追加于末尾 --- */
  float frequency_hz;        /* 0 = legacy 0.1 Hz；>0 = 扫频频率，上限 100 Hz */
  float duration_s;          /* 0 = 自动（legacy 40 s / 扫频 max(2 s, 4 周期)） */
  uint32_t trace_interval_ms;/* 0 = legacy 500 ms */
  uint32_t saturated_samples;/* 电机速度指令触及 max_motor_rpm 的样本数 */
  float peak_motor_rpm;      /* 本次试验电机速度指令峰值绝对值 */
  /* Codex 2026-10-28: 0 measures feedback-only bandwidth; 1 includes velocity
     feedforward. Latched at start. Existing fields retain their offsets. */
  float feedforward_gain;
  uint32_t slew_limited_samples; /* Requested command slew exceeds speed ramp. */
  /* Read-only ADC-rate sums, one per physical phase; no current clamp/trip.
     Sines count only the same steady window as FRD; steps count while active. */
  float phase_squared_sum[3];
  uint32_t current_samples;
} output_position_test_t;
#define OUTPUT_POSITION_SWEEP_MAX_FREQUENCY_HZ 100.0f
/* [千问工作助理 2026-10-28] 单频解调的最小累积周期数。
   位置环实际周期取决于主循环，未必精确 1 ms，无法保证窗口恰含整数个周期，
   故用「足够多周期」把频谱泄漏压到可忽略：泄漏幅度按 1/N 衰减，N>=20 时
   增益/相位偏差已远小于判读精度（数值验证见
   control_analysis/scripts/verify_quadrature_demod_sign.py case 5）。 */
#define OUTPUT_POSITION_SWEEP_MIN_CYCLES 20U
/* [千问工作助理 2026-10-28] 单频扫频的闭环频响结果，扫频结束时由固件自动发布。
   为什么需要这个全局结构体：调试器（pyocd commander）只能 savemem/write32，
   **无法调用函数**，所以仅有 OutputPosition_ReadSweepFrd() 时上位机拿不到
   结果。发布为全局后，主机用一次 savemem 即可读到，且读到的就是固件内
   经过全部有效性判定后的结论，不必在主机侧重复实现同一套数学（避免两份
   实现分叉）。字段顺序与下方 sizeof 注释一一对应，共 32 字节。 */
typedef struct {
  float gain_db;        /* 0  闭环幅频 |H|，dB */
  float phase_deg;      /* 4  arg(H), deg, principal phase; positive is allowed. */
  float coherence;      /* 8  gamma^2 = |Sxy|^2/(Sxx*Syy)，门槛 0.9 */
  float cycles;         /* 12 实际累积周期数，门槛 OUTPUT_POSITION_SWEEP_MIN_CYCLES */
  float frequency_hz;   /* 16 本次扫频频率 */
  float amplitude_deg;  /* 20 本次扫频幅值（输出轴） */
  uint32_t valid;       /* 24 1=可采信；0=不可采信，禁止当作带宽证据 */
  uint32_t reject;      /* 28 不可采信的原因码，见下 */
  /* --- [千问工作助理 2026-10-28] 以下为修复相干性缺陷而追加（原字段偏移不变） --- */
  float coherence_bin;  /* 32 单 bin 相干性。**恒等于 1**，仅为留证保留，见下方警告 */
  uint32_t segments;    /* 36 Welch 分段数；<2 时相干性无意义，判 reject=8 */
  /* reject 原因码：
       0 = 无（valid=1）
       1 = 未累积到样本（未做过 frequency_hz>0 的扫频）
       2 = saturated_samples>0，速度指令触及限幅 → 结果是饱和伪影
       3 = 参考能量退化或非有限
       4 = 累积周期数不足，频谱泄漏未压够
       5 = Welch 相干性 < 0.9 → 反馈在该频点不能被参考线性解释（被污染）
       6 = retired: positive phase alone does not violate causality
       7 = 幅值退化或非有限
       8 = Welch 分段数 < 2 → 相干性无法估计

     ⚠️ 相干性缺陷修正记录（千问工作助理 2026-10-28）：
     首版按单频单 bin 计算 gamma^2 = |Sxy|^2/(Sxx*Syy)，其中
     H = Sxy/Sxx，故 |H|^2 = |Sxy|^2/Sxx^2，代入得
       gamma^2 = |Sxy|^2/(Sxx*Syy) = |H|^2*Sxx/Syy = (|Sxy|^2/Sxx^2)*Sxx/Syy
               = |Sxy|^2/(Sxx*Syy)  ≡ 1  （代数恒等式）
     即**单 bin 相干性恒等于 1**，与噪声大小无关（数值仿真确认：注入 ±0.3°
     噪声、乃至参考与反馈频率完全失配，coh 仍为 1.0000）。这意味着首版的
     reject=5 门槛是**永假死代码**——反馈被污染时仍会报 valid=1，正是工程
     验收文档 2026-10-22_闭环整定模块实施方案_规划记录.md:157「设计频带相干
     性不低于 0.9」所要防止的「污染数据冒充带宽」。
     修复：改用 Welch 分段估计。按整数个周期把窗口切成多段，
       H1  = mean(Y_i * conj(X_i)) / mean(|X_i|^2)
       gamma^2 = |mean(Y_i*conj(X_i))|^2 / (mean(|X_i|^2) * mean(|Y_i|^2))
     单段时仍退化为 1，故要求 segments>=2（reject=8）。仿真验证：20 段、
     参考 20 Hz 而反馈含 23.7 Hz 失配分量时 gamma^2 = 0.2519 < 0.9，门槛有效。
     仿真脚本：control_analysis/scripts/verify_quadrature_demod_sign.py */
} output_position_sweep_frd_t;
extern volatile output_position_sweep_frd_t g_output_position_sweep_frd;
#define OUTPUT_POSITION_SWEEP_FRD_BYTES 40U
/* [千问工作助理 2026-10-28] Welch 分段的最小段数。单段时相干性退化为恒等 1，
   必须至少 2 段才有统计意义。 */
#define OUTPUT_POSITION_SWEEP_MIN_SEGMENTS 2U
typedef struct { float elapsed_s, target_deg, actual_deg, motor_rpm; } output_position_trace_t;
/* Customer 2025-10-24 protocol: 40 discrete commands/cycle, 0..2..0 deg,
   1..30 Hz, 2 s work + 2 s hold. Explicit start only, no boot motion.
   test_mode=0: existing trajectory; 1: Word 40-point protocol;
   2: literal XLSX 300 Hz frames at 1..15 Hz. Latched at start. */
typedef struct {
  uint32_t test_mode, frequency_hz, resting, completed;
} output_position_customer_t;
typedef struct {
  float frequency_hz, reference_amplitude_deg, actual_amplitude_deg;
  float gain_db, phase_deg;
  uint32_t samples, flags, missed_commands;
  /* flags: 1=fit/energy invalid, 2=speed clipping, 4=slew clipping,
     8=command slots skipped, 16=aborted, 32=incomplete time coverage.
     A numerical short-window estimate is usable only when flags & 55 == 0;
     complete protocol reproduction additionally requires flags==0.
     This is NOT the steady sine FRD validity/coherence contract. */
} output_position_customer_result_t;
/* Mutually exclusive capture modes share SRAM: the G431 had only 176 bytes
   free in R17. Trace keeps 64 rows; customer results keep all 30 frequencies. */
typedef union {
  output_position_trace_t trace[OUTPUT_POSITION_TRACE_CAPACITY];
  output_position_customer_result_t customer[30];
} output_position_capture_t;
extern volatile output_position_customer_t g_output_position_customer;
extern volatile output_position_capture_t g_output_position_capture;
#define g_output_position_trace (g_output_position_capture.trace)
#define g_output_position_customer_results (g_output_position_capture.customer)
extern volatile output_position_test_t g_output_position_test;
/* Accumulated-degree P controller. Returns 0 for invalid input. */
uint8_t OutputPosition_Compute(float target, float actual, float kp,
                              float max_rpm, float deadband, int32_t direction,
                              float *rpm);
void OutputPosition_Service(uint8_t output_off);
/* Called at carrier rate from the existing continuous speed loop. */
uint8_t OutputPosition_Check(void);
uint8_t OutputPosition_Update(float *rpm);
/* Called by existing 1 kHz speed loop before its ramp and PI. */
uint8_t OutputPosition_Reference(float *rpm);
/* Same selection is used by the speed executor and FRD clipping check. */
float OutputPosition_RampRpmS(void);
void OutputPosition_ObservePhaseCurrents(float u, float v, float w);
/* [千问工作助理 2026-10-28] 读取上一次扫频的单频闭环频响（正交解调结果）。
   返回 1 表示读取流程正常；*valid=1 才表示该频点数据可采信。
   *cycles_out 为实际累积的周期数，用于判断泄漏是否已压够。
   *valid=0 的情况（此时 gain_db/phase_deg 不得作为带宽证据）：
     - 未做过 frequency_hz>0 的扫频，或尚未累积到样本；
     - saturated_samples>0，速度指令触及 max_motor_rpm，结果为饱和伪影；
     - 参考能量退化或非有限；
     - 累积周期数 < OUTPUT_POSITION_SWEEP_MIN_CYCLES，频谱泄漏未压够；
     - 相干性 gamma^2 < 0.9；
     - **相位为超前（>0）**：因果闭环跟踪相位只可能滞后，算出超前即说明
       符号约定、参考/反馈接线或方向极性有问题。
   coherence 即 gamma^2 = |Sxy|^2/(Sxx*Syy)，与工程验收文档
   2026-10-22_闭环整定模块实施方案_规划记录.md:157「设计频带相干性不低于
   0.9」的门槛一致。
   ⚠️ 本接口的相位符号约定曾出错并已修正，推导与数值验证见实现处注释及
   control_analysis/scripts/verify_quadrature_demod_sign.py。 */
uint8_t OutputPosition_ReadSweepFrd(float *gain_db,float *phase_deg,
                                   float *coherence,float *cycles_out,
                                   uint32_t *valid);
#endif
