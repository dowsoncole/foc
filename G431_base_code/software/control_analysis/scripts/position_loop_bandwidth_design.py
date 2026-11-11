"""Position-loop bandwidth design from MEASURED inner-loop FRF.

Author : 千问工作助理
Date   : 2026-10-28
Purpose: 定量求解「±2° 正弦扫频带宽尽量靠近 20 Hz」目标下，位置环
         纯 P 增益 kp 的可达上限与推荐值，以及对应的时域/频域预期。

为什么可以这样算（关键推导）
----------------------------
固件链路：error_deg --P(kp)--> rpm_cmd --速度闭环--> rpm_actual
          --减速比/单位--> output_deg_s --积分--> output_deg

  rpm_cmd      = kp * (202/6) * error_deg        (output_position_math.c:41)
  rpm_actual   = H_speed(s) * rpm_cmd            (实测闭环 FRF)
  output_deg_s = rpm_actual * (6/202)            (减速比反算)
  output_deg   = output_deg_s / s                (位置是速度的积分)

代入后 **减速比 202 完全抵消**：

    L(s) = kp * H_speed(s) / s

这是一个非常干净的结论：位置环开环 = kp × 实测速度闭环 × 积分器。
因此无需知道电机惯量、Kt、减速比，只需用实测 H_speed 频响即可设计。

额外延迟 T_d（未包含在 H_speed 实测中）：
  - 位置环 ZOH：0.5 × OUTPUT_POSITION_PERIOD_MS
  - MPT RS485 线时间：48 us（mpt_t485.c，2.5 Mbaud × 12 字节）
  本轮已把轮询周期由 5 ms 改为 1 ms，故 T_d 由 ~2.55 ms 降到 ~0.55 ms。
  下方对 T_d 做敏感性扫描，避免把单点假设当结论。

数据来源（全部实测，无合成）
----------------------------
results/speed_response_20261028/balanced_matlab_measured_evaluation.json
  frequency_domain[1] label="balanced"（当前生效整定 Kp=0.0306/Ki=0.1348）
  该 JSON 自带警告：「稀疏 -3dB 插值不是精密带宽测量」，故本脚本不直接
  引用其 bandwidth 字段，而是在密集频点上重新插值求解交越。

注意：JSON 中 interpolated_bandwidth_hz=85.89（闭环 -3dB）与
derived_open_loop.gain_crossover_hz=37.93（开环交越）是两个不同的量，
并不矛盾；本脚本使用后者所在频段的实测闭环幅相数据。

运行：python position_loop_bandwidth_design.py
纯标准库，不依赖 numpy/matplotlib，不访问硬件。
"""
import json
import math
from pathlib import Path

RESULTS = Path(__file__).resolve().parents[1] / "results"
SRC = RESULTS / "speed_response_20261028" / "balanced_matlab_measured_evaluation.json"

# 验收门槛（MDK-ARM/docs/电流环速度环验收标准.md 与
# docs/log/2026-10-22_闭环整定模块实施方案_规划记录.md）
PM_MIN_DEG = 45.0     # 相位裕度下限
GM_MIN_DB = 6.0       # 增益裕度下限
TARGET_BW_HZ = 20.0   # 用户目标

GEAR = 202.0
RPM_TO_DEG_S_OUT = 6.0 / GEAR       # 电机 rpm -> 输出轴 deg/s
FF_LIMIT_DEG_S = 75.0               # output_position.c 前馈硬限幅
MAX_KP = 200.0                      # 本轮解锁后的 kp 上限
MAX_RPM = 2500.0                    # OUTPUT_POSITION_MAX_MOTOR_RPM
I_RATED_A = 5.0
I_MAX_A = 7.4
KT_NM_A = 0.0252                    # 厂家标称 25.2 mN·m/A
J_KG_M2 = 8.651e-6                  # 厂家标称转子惯量（未含负载反射）


def load_inner_frf():
    """读取实测速度闭环 FRF，返回 (freq_hz, mag, phase_deg) 列表。"""
    data = json.loads(SRC.read_text(encoding="utf-8"))
    frd = [d for d in data["frequency_domain"] if d.get("label") == "balanced"]
    assert len(frd) == 1, f"expected exactly 1 'balanced' dataset, got {len(frd)}"
    d = frd[0]
    f = d["frequency_hz"]
    g = [10.0 ** (x / 20.0) for x in d["gain_db"]]          # dB -> 线性
    p = d["unwrapped_phase_deg"]
    assert len(f) == len(g) == len(p) and f == sorted(f)
    return f, g, p, data["derived_open_loop"]


def interp_loglog(f, y, x):
    """在 log10(f) 域线性插值；超出范围则钳位到端点（并标记外推）。"""
    if x <= f[0]:
        return y[0], True
    if x >= f[-1]:
        return y[-1], True
    for i in range(len(f) - 1):
        if f[i] <= x <= f[i + 1]:
            a, b = math.log10(f[i]), math.log10(f[i + 1])
            t = (math.log10(x) - a) / (b - a) if b != a else 0.0
            return y[i] + t * (y[i + 1] - y[i]), False
    raise AssertionError("unreachable")


def interp_phase(f, p, x):
    """相位在 log10(f) 域插值（unwrapped，故线性插值合理）。"""
    v, extrap = interp_loglog(f, p, x)
    return v, extrap


def open_loop(kp, f_hz, t_d_s, fm, fp):
    """L(jw) = kp * H_speed(jw) * e^{-jw T_d} / (jw)"""
    w = 2.0 * math.pi * f_hz
    mag = kp * fm / w
    phase = -90.0 + fp - math.degrees(w * t_d_s)
    return mag, phase


def find_crossover(kp, f_arr, g_arr, p_arr, t_d_s):
    """求 |L|=1 的交越频率（对数域二分），返回 (fc_hz, PM_deg) 或 None。"""
    lo, hi = f_arr[0], f_arr[-1]

    def lmag(x):
        m, _ = interp_loglog(f_arr, g_arr, x)
        ph, _ = interp_phase(f_arr, p_arr, x)
        mag, _ = open_loop(kp, x, t_d_s, m, ph)
        return 20.0 * math.log10(mag) if mag > 0 else -1e9

    if lmag(lo) < 0.0 or lmag(hi) > 0.0:
        return None
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if lmag(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    fc = math.sqrt(lo * hi)
    m, _ = interp_loglog(f_arr, g_arr, fc)
    ph, _ = interp_phase(f_arr, p_arr, fc)
    _, phase = open_loop(kp, fc, t_d_s, m, ph)
    return fc, 180.0 + phase


def find_gain_margin(kp, f_arr, g_arr, p_arr, t_d_s):
    """求相位穿越（phase=-180°）处的增益裕度 dB。无穿越返回 None。"""
    lo = hi = None
    for i in range(len(f_arr) - 1):
        a, b = f_arr[i], f_arr[i + 1]
        pa, _ = interp_phase(f_arr, p_arr, a)
        pb, _ = interp_phase(f_arr, p_arr, b)
        # 相位穿越 -180（考虑 unwrap 后单调下降）
        if (pa + 180.0) * (pb + 180.0) <= 0.0 and pa != pb:
            lo, hi = a, b
            break
    if lo is None:
        return None, None
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        ph, _ = interp_phase(f_arr, p_arr, mid)
        if ph > -180.0:
            lo = mid
        else:
            hi = mid
    fx = math.sqrt(lo * hi)
    m, _ = interp_loglog(f_arr, g_arr, fx)
    ph, _ = interp_phase(f_arr, p_arr, fx)
    mag, _ = open_loop(kp, fx, t_d_s, m, ph)
    gm = -20.0 * math.log10(mag) if mag > 0 else float("inf")
    return fx, gm


def closed_loop_at(kp, f_hz, f_arr, g_arr, p_arr, t_d_s):
    """闭环 T = L/(1+L) 在 f_hz 处的幅值(dB)与相位(deg)。"""
    w = 2.0 * math.pi * f_hz
    m, _ = interp_loglog(f_arr, g_arr, f_hz)
    ph, _ = interp_phase(f_arr, p_arr, f_hz)
    mag, phase = open_loop(kp, f_hz, t_d_s, m, ph)
    # L = mag * e^{j phase}
    lr = mag * math.cos(math.radians(phase))
    li = mag * math.sin(math.radians(phase))
    tr = lr / ((1.0 + lr) ** 2 + li ** 2)
    ti = li / ((1.0 + lr) ** 2 + li ** 2)
    tmag = math.hypot(tr, ti)
    tph = math.degrees(math.atan2(ti, tr))
    return 20.0 * math.log10(tmag) if tmag > 0 else -1e9, tph, mag


def sensitivity(kp, amp_out_deg, f_hz):
    """该工况下的执行器需求，用于判定是否进入限幅（限幅数据是伪影）。"""
    amp_rad = math.radians(amp_out_deg)
    w = 2.0 * math.pi * f_hz
    out_peak_deg_s = amp_out_deg * w                       # 输出轴角速度峰值
    motor_peak_rpm = out_peak_deg_s / RPM_TO_DEG_S_OUT     # 电机 rpm 峰值
    motor_peak_acc = motor_peak_rpm * (2.0 * math.pi / 60.0) * w  # rad/s^2
    torque = J_KG_M2 * motor_peak_acc
    current = torque / KT_NM_A
    ramp_need = motor_peak_rpm * w                         # rpm/s
    return dict(out_peak_deg_s=out_peak_deg_s, motor_peak_rpm=motor_peak_rpm,
                motor_peak_acc_rad_s2=motor_peak_acc, torque_nm=torque,
                current_a=current, ramp_need_rpm_s=ramp_need)


def latency_model(period_ms):
    """反馈延迟 T_d 的严格分解（秒）。

    三段串联，全部是真实存在的：
      1) MPT 采样保持：器件 Update Rate=50 kHz(20 us) 可忽略，但主机按
         period_ms 轮询，测量值平均要等半个轮询周期才被取用 → 0.5*period
      2) RS485 线时间：(1 命令 + 11 响应) × 10 bit / 2.5 Mbaud = 48 us
      3) 位置环 ZOH：位置环自身也以 period_ms 更新，输出被保持到下个周期，
         等效再延迟半个周期 → 0.5*period

    注意：mpt_t485.c:103 把 last_good_ms 记为**请求时刻** started_ms，
    所以轮询周期与线时间都真实计入延迟，不能只算线时间。

    同时给出乐观下界（仅 0.5*period + 线时间，忽略位置环 ZOH）用于
    敏感性对比，但**判定一律采用严格值**，避免用乐观模型得出虚高带宽。
    """
    wire_s = (1.0 + 11.0) * 10.0 / 2.5e6
    strict = period_ms / 1000.0 + wire_s          # 0.5+0.5 个周期 + 线时间
    optimistic = 0.5 * period_ms / 1000.0 + wire_s
    return strict, optimistic


def main():
    f_arr, g_arr, p_arr, derived = load_inner_frf()
    print("=" * 78)
    print("位置环带宽设计（基于实测速度闭环 FRF）")
    print(f"数据源: {SRC.relative_to(RESULTS.parent)}")
    print(f"实测频点: {f_arr[0]}~{f_arr[-1]} Hz，共 {len(f_arr)} 点")
    print(f"内环实测开环交越 {derived['gain_crossover_hz']:.2f} Hz, "
          f"PM {derived['phase_margin_deg']:.1f}°, GM {derived['gain_margin_db']:.2f} dB")
    print("=" * 78)

    # --- 1. 延迟敏感性：轮询周期改造前后的对比 ---
    print("\n【1】反馈延迟 T_d 敏感性（严格模型：轮询半周期 + 线时间 + 位置环 ZOH 半周期）")
    print(f"{'周期(ms)':>9} {'T_d严格(ms)':>12} {'T_d乐观(ms)':>12} "
          f"{'20Hz处PM严格(°)':>16} {'判定':>6}")
    for period_ms in (5.0, 2.0, 1.0):
        t_d, t_d_opt = latency_model(period_ms)
        m, _ = interp_loglog(f_arr, g_arr, TARGET_BW_HZ)
        ph, _ = interp_phase(f_arr, p_arr, TARGET_BW_HZ)
        # 取 20 Hz 交越所需 kp（|L|=1）
        w20 = 2.0 * math.pi * TARGET_BW_HZ
        kp20 = w20 / m
        _, phase = open_loop(kp20, TARGET_BW_HZ, t_d, m, ph)
        pm = 180.0 + phase
        _, phase_o = open_loop(kp20, TARGET_BW_HZ, t_d_opt, m, ph)
        print(f"{period_ms:9.1f} {t_d*1000:12.2f} {t_d_opt*1000:12.2f} "
              f"{pm:16.1f} {'PASS' if pm >= PM_MIN_DEG else 'FAIL':>6}"
              f"   (乐观模型 {180.0+phase_o:.1f}°)")
    print("  → 严格模型下 5 ms 轮询在 20 Hz 处 PM 仍可能达标，但裕度显著小于")
    print("    1 ms；且 5 ms 轮询使可达带宽上限明显降低（见【2】对比）。")
    print("    判定一律采用严格值，乐观值仅用于显示模型不确定性。")

    # --- 2. 扫 kp，求满足 PM/GM 的最大带宽 ---
    t_d, _ = latency_model(1.0)
    t_d5, _ = latency_model(5.0)
    print(f"\n【2】kp 扫描（严格延迟模型）")
    print("  注意：**交越频率 fc 与反馈延迟无关**——纯延迟 e^(-jwTd) 的幅值恒为 1，")
    print("        只旋转相位。所以延迟吃掉的是**相位裕度**，不是带宽。")
    print("        这正是 5 ms→1 ms 改造的真实收益所在（对比 PM@1ms 与 PM@5ms）。")
    print(f"{'kp(/s)':>8} {'fc(Hz)':>8} {'PM@1ms(°)':>10} {'PM@5ms(°)':>10} "
          f"{'GM(dB)':>8} {'20Hz增益(dB)':>13} {'判定':>6}")
    rows = []
    for kp in (2.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0,
               158.3, 170.0, 180.0, 200.0):
        cx = find_crossover(kp, f_arr, g_arr, p_arr, t_d)
        cx5 = find_crossover(kp, f_arr, g_arr, p_arr, t_d5)
        fx, gm = find_gain_margin(kp, f_arr, g_arr, p_arr, t_d)
        g20, p20, _ = closed_loop_at(kp, TARGET_BW_HZ, f_arr, g_arr, p_arr, t_d)
        pm5 = f"{cx5[1]:.1f}" if cx5 else "—"
        if cx is None:
            print(f"{kp:8.1f} {'—':>8} {'—':>10} {pm5:>10} "
                  f"{(f'{gm:.2f}' if gm is not None else '—'):>8} {g20:13.2f} {'N/A':>6}")
            continue
        fc, pm = cx
        ok = pm >= PM_MIN_DEG and (gm is None or gm >= GM_MIN_DB)
        rows.append((kp, fc, pm, gm, g20, p20, ok))
        print(f"{kp:8.1f} {fc:8.2f} {pm:10.1f} {pm5:>10} "
              f"{(f'{gm:.2f}' if gm is not None else 'inf'):>8} "
              f"{g20:13.2f} {'PASS' if ok else 'FAIL':>6}")

    # --- 3. 满足门槛的最大带宽与推荐 kp ---
    good = [r for r in rows if r[6]]
    print("\n【3】满足验收门槛（PM≥45°, GM≥6dB）的解")
    if not good:
        print("  无解：需先提升内环带宽或进一步降低反馈延迟。")
    else:
        best = max(good, key=lambda r: r[1])
        print(f"  最大可达交越频率 fc = {best[1]:.2f} Hz @ kp = {best[0]:.1f} /s")
        print(f"  该点 PM = {best[2]:.1f}°, GM = "
              f"{best[3]:.2f} dB" if best[3] is not None else "  GM = inf")
        # 20 Hz 专项
        at20 = [r for r in good if abs(r[1] - TARGET_BW_HZ) < 3.0]
        if at20:
            r = min(at20, key=lambda x: abs(x[1] - TARGET_BW_HZ))
            print(f"\n  最接近 20 Hz 目标的解：kp = {r[0]:.1f} /s")
            print(f"    fc = {r[1]:.2f} Hz, PM = {r[2]:.1f}°, GM = "
                  f"{(f'{r[3]:.2f} dB' if r[3] is not None else 'inf')}")
            print(f"    20 Hz 处闭环增益 {r[4]:.2f} dB, 相位 {r[5]:.1f}°")
            print(f"    → 这就是扫频实测应当看到的目标值（±1 dB / ±5° 内即吻合）")

    # --- 4. 执行器限幅：判定 ±2° 满幅是否物理可达 ---
    print("\n【4】执行器限幅核查（限幅数据是饱和伪影，不可当带宽）")
    print(f"{'幅值(°)':>9} {'频率(Hz)':>9} {'电机峰值rpm':>12} {'前馈需求deg/s':>14} "
          f"{'峰值电流A':>10} {'判定':>6}")
    for amp, f_hz in ((0.272, 20.0), (0.331, 20.0), (1.0, 20.0), (2.0, 20.0),
                      (2.0, 7.23), (2.0, 8.80)):
        s = sensitivity(0.0, amp, f_hz)
        ff_ok = s["out_peak_deg_s"] <= FF_LIMIT_DEG_S
        i_ok = s["current_a"] <= I_MAX_A
        rpm_ok = s["motor_peak_rpm"] <= MAX_RPM
        verdict = "PASS" if (ff_ok and i_ok and rpm_ok) else "FAIL"
        why = []
        if not ff_ok:
            why.append(f"前馈超{FF_LIMIT_DEG_S:.0f}deg/s")
        if not i_ok:
            why.append(f"电流超{I_MAX_A}A")
        if not rpm_ok:
            why.append(f"转速超{MAX_RPM:.0f}rpm")
        print(f"{amp:9.3f} {f_hz:9.2f} {s['motor_peak_rpm']:12.1f} "
              f"{s['out_peak_deg_s']:14.1f} {s['current_a']:10.1f} {verdict:>6}"
              + (f"  ({', '.join(why)})" if why else ""))
    print(f"\n  ±2°@20Hz 需电流 "
          f"{sensitivity(0,2.0,20.0)['current_a']:.1f} A = 额定 {I_RATED_A} A 的 "
          f"{sensitivity(0,2.0,20.0)['current_a']/I_RATED_A:.1f} 倍 → 物理不可达。")

    # 20 Hz 下受各限幅约束的最大幅值（反解）。
    # 记 A = 输出轴幅值(rad)，w = 2*pi*f。
    #   电机侧幅值      A_m   = A * GEAR          (减速比放大位移)
    #   电机侧峰值加速度 acc  = A_m * w^2
    #   所需扭矩        T    = J * acc            (J 为电机侧总惯量)
    #   所需电流        I    = T / Kt
    # 反解：A_deg = deg(I*Kt / (J*w^2)) / GEAR
    #
    # ⚠️ 修正记录（千问工作助理 2026-10-28）：首版把 GEAR 乘在了分子上，
    #    得出「±15798°」这种物理上荒谬的结果。减速比是把输出轴位移**放大**
    #    到电机侧，所以电机侧惯量/电流需求随 GEAR 增大，反解回输出轴幅值
    #    时必须**除以** GEAR。修正后数值与【4】表格的正向计算一致。
    w20 = 2.0 * math.pi * TARGET_BW_HZ
    amp_ff = FF_LIMIT_DEG_S / w20                       # 输出轴 deg
    amp_rpm = MAX_RPM * RPM_TO_DEG_S_OUT / w20           # 输出轴 deg

    def amp_from_current(i_a):
        return math.degrees(i_a * KT_NM_A / (J_KG_M2 * w20 ** 2)) / GEAR

    amp_i = amp_from_current(I_MAX_A)
    amp_i_rated = amp_from_current(I_RATED_A)
    binding = min(amp_ff, amp_rpm, amp_i_rated)

    def mark_a(v):
        return "   ← 最紧" if abs(v - binding) < 1e-12 else ""

    print(f"\n  20 Hz 下各硬限幅对应的最大输出轴幅值：")
    print(f"    前馈 ±75 deg/s        → ±{amp_ff:.3f}°{mark_a(amp_ff)}")
    print(f"    转速 ±{MAX_RPM:.0f} rpm       → ±{amp_rpm:.3f}°{mark_a(amp_rpm)}")
    print(f"    电流 {I_RATED_A} A(额定)       → ±{amp_i_rated:.3f}°{mark_a(amp_i_rated)}")
    print(f"    电流 {I_MAX_A} A(最大连续)   → ±{amp_i:.3f}°")
    binding_name = ("前馈限幅" if abs(binding - amp_ff) < 1e-12 else
                    "转速限幅" if abs(binding - amp_rpm) < 1e-12 else "额定电流")
    print(f"    ⇒ 20 Hz 可做带宽测量的幅值上限约 ±{binding:.3f}°（受{binding_name}约束），")
    print(f"       而非 ±2°。±2° 满幅 20 Hz 属物理不可达（差 {2.0/binding:.1f} 倍）。")
    print(f"    注：电流项按 J=转子标称值计算，**未含减速器与负载反射惯量**，")
    print(f"       故该上限偏乐观；工程多处标注 J「待辨识」。实测只会更严。")

    # ±2° 满幅下受电流约束的最大频率
    # I = J*(A_rad*GEAR)*w^2/Kt  =>  w = sqrt(I*Kt/(J*A_rad*GEAR))
    a2_rad = math.radians(2.0)

    def freq_from_current(i_a):
        return math.sqrt(i_a * KT_NM_A / (J_KG_M2 * a2_rad * GEAR)) / (2 * math.pi)

    f_i_rated = freq_from_current(I_RATED_A)
    f_i_max = freq_from_current(I_MAX_A)
    f_ff = FF_LIMIT_DEG_S / (2.0 * math.pi * 2.0)
    f_rpm = MAX_RPM * RPM_TO_DEG_S_OUT / (2.0 * math.pi * 2.0)
    f_binding = min(f_ff, f_rpm, f_i_rated)

    def mark_f(v):
        return "   ← 最紧" if abs(v - f_binding) < 1e-12 else ""

    print(f"\n  ±2° 满幅下各限幅对应的最大频率：")
    print(f"    前馈 ±75 deg/s        → {f_ff:.2f} Hz{mark_f(f_ff)}")
    print(f"    转速 ±{MAX_RPM:.0f} rpm       → {f_rpm:.2f} Hz{mark_f(f_rpm)}")
    print(f"    电流 {I_RATED_A} A(额定)       → {f_i_rated:.2f} Hz{mark_f(f_i_rated)}")
    print(f"    电流 {I_MAX_A} A(最大连续)   → {f_i_max:.2f} Hz")
    print(f"    ⇒ ±2° 满幅的物理上限约 {f_binding:.1f} Hz，仅为 20 Hz 目标的 "
          f"{f_binding/TARGET_BW_HZ*100:.0f}%。")
    print(f"    注意：电流项未含负载反射惯量，属乐观值；前馈与转速是与惯量无关的")
    print(f"          硬限幅，故「±2° 满幅上限 ≈ {f_binding:.1f} Hz」这一结论稳健。")

    # --- 5. 结论 ---
    print("\n" + "=" * 78)
    print("【结论】")
    if good:
        best = max(good, key=lambda r: r[1])
        print(f"  1. 解锁 kp（10→{MAX_KP:.0f}）+ 轮询周期 5→1 ms 后，位置环交越频率")
        print(f"     可由 1.59 Hz 提升到 {best[1]:.1f} Hz，"
              f"{'达到' if best[1] >= TARGET_BW_HZ else '接近'} 20 Hz 目标。")
        print(f"     （1.59 Hz = 原 kp 上限 10 / 2π；交越频率与延迟无关，延迟只吃相位裕度）")
        at20 = [r for r in good if abs(r[1] - TARGET_BW_HZ) < 3.0]
        if at20:
            r = min(at20, key=lambda x: abs(x[1] - TARGET_BW_HZ))
            print(f"  2. 推荐 kp = {r[0]:.1f} /s（fc={r[1]:.1f} Hz, PM={r[2]:.1f}°, "
                  f"GM={(f'{r[3]:.2f} dB' if r[3] is not None else 'inf')}）。")
            print(f"     该点 20 Hz 处闭环增益 {r[4]:.2f} dB、相位 {r[5]:.1f}°，")
            print(f"     即扫频实测应当复现的目标值（±1 dB / ±5° 内视为吻合）。")
        print(f"  3. 上述为**小信号线性带宽**。幅值受硬限幅约束：")
        print(f"     - 20 Hz 下最大幅值 ±{binding:.3f}°（受{binding_name}），非 ±2°；")
        print(f"     - ±2° 满幅下最大频率 {f_binding:.1f} Hz，仅为目标的 "
              f"{f_binding/TARGET_BW_HZ*100:.0f}%。")
        print(f"     ⇒ 「±2° 满幅 @ 20 Hz」物理不可达；可达的是「小信号 20 Hz 带宽」")
        print(f"       或「±2° 满幅 @ {f_binding:.1f} Hz」，二者不可兼得。")
        print(f"  4. 全部为离线设计值，**必须用固件内正交解调实测复核**。")
        print(f"     可采信条件：saturated_samples == 0 且相干性 ≥ 0.9；")
        print(f"     违反任一条即说明进入了限幅，测出的「带宽」是饱和伪影。")
        print(f"  5. 本脚本的 J 用转子标称值、未含减速器与负载反射惯量，")
        print(f"     故电流类结论偏乐观；前馈/转速类限幅与惯量无关，结论稳健。")
    else:
        print("  无满足 PM≥45°/GM≥6dB 的解：需先提升内环带宽或进一步降低反馈延迟。")
    print("=" * 78)


if __name__ == "__main__":
    main()
