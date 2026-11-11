#include "output_position.h"
#include "customer_pwm_table.h"
#include "mpt_t485.h"
#include "speed_commission.h"
#include "stm32g4xx_hal.h"
#include <math.h>
volatile output_position_t g_output_position={
  /* This assembly was physically commissioned with positive output direction.
     Calibration and command=1 remain mandatory after each boot. */
  .commissioned=1U,.motor_direction=1,
  /* R17, Codex 2026-10-29: user confirmed the afternoon inertia disc.
     Kp72 completed +/-2 deg at 18.5 Hz, -2.97 dB, max phase RMS 4.91 A.
     Higher Kp80 reached the 15 A host peak criterion at 17 Hz. Ordinary
     commands retain 5500 rpm; finite sine trials explicitly select 7500.
     See docs/log/2026-10-29_位置环续调与反馈链路检查.md for load dependence. */
  .kp_per_s=72.0f,.max_motor_rpm=5500.0f,.deadband_deg=0.0055f
};
volatile output_position_test_t g_output_position_test={.amplitude_deg=90.0f,.feedforward_gain=0.6f};
volatile output_position_capture_t g_output_position_capture;
volatile output_position_customer_t g_output_position_customer;
static uint32_t test_mode;
/* Foreground-only least squares [DC, cos(wt), sin(wt)]. Fit the actual
   delivered staircase as X, not an ideal sine or the requested 2 deg span. */
static struct {
  float c,s,cc,ss,cs,x,y,xc,xs,yc,ys;
  uint32_t n,slot,missed,saturated,slew,first_ms,last_ms;
} customer_fit;
static void customer_finish(uint32_t flags)
{
  uint32_t f=g_output_position_customer.frequency_hz;
  if(!f || f>30 || g_output_position_customer.completed>=f) return;
  float n=(float)customer_fit.n;
  float cc=customer_fit.cc-customer_fit.c*customer_fit.c/fmaxf(n,1);
  float ss=customer_fit.ss-customer_fit.s*customer_fit.s/fmaxf(n,1);
  float cs=customer_fit.cs-customer_fit.c*customer_fit.s/fmaxf(n,1);
  float det=cc*ss-cs*cs;
  float xc=customer_fit.xc-customer_fit.x*customer_fit.c/fmaxf(n,1);
  float xs=customer_fit.xs-customer_fit.x*customer_fit.s/fmaxf(n,1);
  float yc=customer_fit.yc-customer_fit.y*customer_fit.c/fmaxf(n,1);
  float ys=customer_fit.ys-customer_fit.y*customer_fit.s/fmaxf(n,1);
  output_position_customer_result_t r={0};
  r.frequency_hz=(float)f; r.samples=customer_fit.n;
  r.missed_commands=customer_fit.missed;
  uint32_t slots=test_mode==2U ? 600U : 80U*f;
  if(customer_fit.slot<slots) r.missed_commands+=slots-customer_fit.slot;
  r.gain_db=r.phase_deg=NAN;
  if(customer_fit.n<20 || !isfinite(det) || det<=1e-6f) flags|=1U;
  else {
    float a=(xc*ss-xs*cs)/det, b=(xs*cc-xc*cs)/det;
    float u=(yc*ss-ys*cs)/det, v=(ys*cc-yc*cs)/det;
    r.reference_amplitude_deg=hypotf(a,b);
    r.actual_amplitude_deg=hypotf(u,v);
    if(!isfinite(r.reference_amplitude_deg) || !isfinite(r.actual_amplitude_deg) ||
       r.reference_amplitude_deg<1e-5f || r.actual_amplitude_deg<1e-5f) flags|=1U;
    else {
      r.gain_db=20.0f*log10f(r.actual_amplitude_deg/r.reference_amplitude_deg);
      r.phase_deg=atan2f(u*b-v*a,u*a+v*b)*57.29577951f;
    }
  }
  if(g_output_position_test.saturated_samples!=customer_fit.saturated) flags|=2U;
  if(g_output_position_test.slew_limited_samples!=customer_fit.slew) flags|=4U;
  if(r.missed_commands) flags|=8U;
  if(customer_fit.first_ms>2U || customer_fit.last_ms<1997U || customer_fit.n<1900U) flags|=32U;
  r.flags=flags;
  g_output_position_customer_results[f-1]=r;
  g_output_position_customer.completed=f; /* publish last */
}
static void customer_reset_fit(void)
{
  customer_fit.c=customer_fit.s=customer_fit.cc=customer_fit.ss=customer_fit.cs=0;
  customer_fit.x=customer_fit.y=customer_fit.xc=customer_fit.xs=0;
  customer_fit.yc=customer_fit.ys=0;
  customer_fit.n=customer_fit.slot=customer_fit.missed=0;
  customer_fit.first_ms=customer_fit.last_ms=0;
  customer_fit.saturated=g_output_position_test.saturated_samples;
  customer_fit.slew=g_output_position_test.slew_limited_samples;
}
static float customer_target(uint32_t elapsed)
{
  uint32_t f=elapsed/4000U+1U, ms=elapsed%4000U;
  if(g_output_position_customer.frequency_hz!=f) {
    customer_finish(32U); /* catches a foreground stall across a boundary */
    g_output_position_customer.frequency_hz=f;
    customer_reset_fit();
  }
  uint32_t last_frequency=test_mode==2U ? 15U : 30U;
  if(elapsed>=last_frequency*4000U) {
    g_output_position_customer.frequency_hz=last_frequency;
    g_output_position_customer.resting=1U;
    g_output_position_test.state=2U;
    return 0;
  }
  g_output_position_customer.resting=ms>=2000U;
  if(ms>=2000U) { customer_finish(0U); return 0; }
  /* 1501..1520..1500: first point is issued at t=0, zero-order held.
     Absolute-time indexing prevents frequency drift from integer periods. */
  uint32_t slot=test_mode==2U ? ms*300U/1000U+1U : ms*40U*f/1000U+1U;
  if(slot>customer_fit.slot+1U) customer_fit.missed+=slot-customer_fit.slot-1U;
  customer_fit.slot=slot;
  if(test_mode==2U) return 0.1f*(float)customer_pwm_table[f-1U][(slot-1U)%300U];
  uint32_t k=slot%40U;
  return 0.1f*(float)(k<=20U ? k : 40U-k);
}
static void customer_observe(uint32_t elapsed,float x,float y)
{
  if(g_output_position_customer.resting) return;
  uint32_t ms=elapsed%4000U;
  float phase=6.283185307f*(float)g_output_position_customer.frequency_hz*(float)ms*0.001f;
  float c=cosf(phase),s=sinf(phase);
  customer_fit.c+=c; customer_fit.s+=s;
  customer_fit.cc+=c*c; customer_fit.ss+=s*s; customer_fit.cs+=c*s;
  customer_fit.x+=x; customer_fit.y+=y;
  customer_fit.xc+=x*c; customer_fit.xs+=x*s;
  customer_fit.yc+=y*c; customer_fit.ys+=y*s;
  if(!customer_fit.n) customer_fit.first_ms=ms;
  customer_fit.last_ms=ms; customer_fit.n++;
}
static uint32_t test_start_ms, next_trace_ms;
static float test_amplitude, test_center;
/* [千问工作助理 2026-10-28] 扫频参数，在 command=1 时与增益一同锁存。
   test_frequency_hz==0 表示沿用改动前的 0.1 Hz 五次多项式平滑轨迹。 */
static float test_frequency_hz, test_duration_s;
/* R15: latch current-observation boundaries in foreground. Never divide in
   the 20 kHz current observer; R14 captured a PWM commit at its 2 us guard. */
static float test_current_window_start_s, test_current_window_end_s;
static float test_feedforward_gain;
/* A one-cycle high-frequency envelope adds a substantial acceleration
   transient. Use at least two cycles / 0.5 s and exclude BOTH ramps in FRD. */
static float sweep_ramp_s(float frequency)
{
  float duration=2.0f/frequency;
  return duration<0.5f ? 0.5f : duration;
}
float OutputPosition_RampRpmS(void)
{
  return g_output_position.state==1U && g_output_position_test.state==1U &&
         test_mode==0U && test_frequency_hz>0.0f ? OUTPUT_POSITION_SWEEP_RAMP_RPM_S :
                                 OUTPUT_POSITION_RAMP_RPM_S;
}
void OutputPosition_ObservePhaseCurrents(float u, float v, float w)
{
  if(g_output_position.state!=1U || g_output_position_test.state!=1U) return;
  float elapsed=g_output_position_test.elapsed_s;
  if(elapsed<test_current_window_start_s || elapsed>=test_current_window_end_s) return;
  g_output_position_test.phase_squared_sum[0]+=u*u;
  g_output_position_test.phase_squared_sum[1]+=v*v;
  g_output_position_test.phase_squared_sum[2]+=w*w;
  g_output_position_test.current_samples++;
}
static uint32_t test_trace_interval_ms;
static uint32_t sequence, previous_raw, last_update_ms;
static int64_t unwrapped_count;
static int32_t direction;
static float kp,max_rpm,deadband;
volatile uint32_t g_output_position_fault_capture[6];
static uint8_t trip(uint32_t fault)
{
  if(g_output_position.state!=2) {
    g_output_position_fault_capture[0]=HAL_GetTick();
    g_output_position_fault_capture[1]=g_mpt_t485.last_good_ms;
    g_output_position_fault_capture[2]=g_mpt_t485.good_frames;
    g_output_position_fault_capture[3]=g_mpt_t485.enabled;
    g_output_position_fault_capture[4]=g_mpt_t485.valid;
    g_output_position_fault_capture[5]=g_speed_test.state;
  }
  g_output_position.fault=fault; g_output_position.state=2;
  if(g_output_position_test.state==1) {
    g_output_position_test.state=3; g_output_position_test.fault=fault;
  }
  g_output_position.velocity_deg_s=0;
  g_output_position.motor_rpm=0; g_speed_run.target_rpm=0; g_speed_run.enable=0;
  return 0;
}
uint8_t OutputPosition_Check(void)
{
  if(g_output_position.state!=1) return g_output_position.state!=2;
  if(g_output_position.command==2) {
    if(g_output_position_test.state==1) g_output_position_test.state=3;
    g_output_position.velocity_deg_s=0;
    g_speed_run.enable=0; g_speed_run.target_rpm=0;
    g_output_position.motor_rpm=0; g_output_position.state=0;
    g_output_position.command=0; return 1;
  }
  if(!g_mpt_t485.enabled || !g_mpt_t485.valid ||
     (uint32_t)(HAL_GetTick()-g_mpt_t485.last_good_ms)>OUTPUT_POSITION_MAX_AGE_MS) return trip(1);
  if(!g_output_position.commissioned || !g_speed_run.enable ||
     g_speed_run.fault || g_speed_test.fault || g_speed_test.state==4) return trip(2);
  if(!isfinite(g_output_position.target_deg) || !isfinite(g_output_position.velocity_deg_s) ||
     fabsf(g_output_position.velocity_deg_s)>OUTPUT_POSITION_MAX_VELOCITY_DEG_S) return trip(3);
  return 1;
}
/* [千问工作助理 2026-10-28] 扫频正交解调累积量。单频闭环频响只需
   sum(x*cos)、sum(x*sin) 四个 float，不占用 81 点 trace 缓冲，
   因此可在 1 kHz 位置周期上累积数千个样本而 RAM 增量可忽略。
   仅在 ramp-in 结束后（纯稳态正弦段）累积，符合工程验收文档
   2026-10-22_闭环整定模块实施方案_规划记录.md:153「先等待暂态衰减，
   再采集完整整数周期」的要求。 */
static float qd_tc,qd_ts,qd_ac,qd_as;
static uint32_t qd_n;
static uint8_t qd_active;
/* [千问工作助理 2026-10-28] Welch 分段相干性累积量。
   为什么必须分段：单 bin 相干性在代数上恒等于 1。因为 H=Sxy/Sxx，故
   |H|²=|Sxy|²/Sxx²，代入 gamma²=|H|²·Sxx/Syy 得
     gamma² = (|Sxy|²/Sxx²)·Sxx/Syy = |Sxy|²/(Sxx·Syy) ≡ 1
   与噪声大小**完全无关**（数值仿真确认：注入 ±0.3° 噪声、乃至参考与反馈
   频率完全失配，coh 仍为 1.0000）。这意味着首版的 reject=5 门槛是**永假死
   代码**——反馈被污染时仍会报 valid=1，正是工程验收文档
   2026-10-22_闭环整定模块实施方案_规划记录.md:157「设计频带相干性不低于
   0.9」所要防止的「污染数据冒充带宽」。
   修复：按整数个信号周期把窗口切成多段，每段独立估计 X_i、Y_i 后做 Welch
   平均：
     H1     = mean(Y_i·conj(X_i)) / mean(|X_i|²)
     gamma² = |mean(Y_i·conj(X_i))|² / (mean(|X_i|²)·mean(|Y_i|²))
   单段时仍退化为 1，故要求 segments>=OUTPUT_POSITION_SWEEP_MIN_SEGMENTS
   （否则 reject=8）。仿真验证：20 段、参考 20 Hz 而反馈含 23.7 Hz 失配分量
   时 gamma²=0.2519<0.9，门槛确实生效。RAM 增量仅 9 个标量。 */
static float qd_wxy_r,qd_wxy_i,qd_wxx,qd_wyy;   /* Welch 跨段累积 */
static float qd_sxc,qd_sxs,qd_syc,qd_sys;       /* 当前段累积 */
static uint32_t qd_sn,qd_seg_n,qd_seg_len;      /* 段内样本数、已完成段数、段长 */
/* [千问工作助理 2026-10-28] 扫频频响结果的全局发布槽，共 32 字节。
   调试器只能读写内存、无法调用函数，故必须发布为全局，上位机才能取到。 */
volatile output_position_sweep_frd_t g_output_position_sweep_frd;
/* 前向声明：sweep_frd_compute 的定义在本文件后部（紧邻读取接口），
   但扫频结束时需要在此处之前调用它。 */
static void sweep_frd_compute(output_position_sweep_frd_t *out);
/* [千问工作助理 2026-10-28] Welch 分段结算：把当前段累积量结算进跨段累积量。
   sweep_frd_flush_segment   处理满段（qd_sn == qd_seg_len）
   sweep_frd_flush_partial_segment 处理扫频结束时的尾部不足一段，避免丢数据 */
static void sweep_frd_flush_segment(void);
static void sweep_frd_flush_partial_segment(void);
uint8_t OutputPosition_Update(float *rpm)
{
  if(!OutputPosition_Check()) return 0;
  if(g_output_position.state!=1) return 1;
  uint32_t now=HAL_GetTick();
  if((uint32_t)(now-last_update_ms)<OUTPUT_POSITION_PERIOD_MS) {
    *rpm=g_output_position.motor_rpm; return 1;
  }
  uint32_t update_elapsed_ms=now-last_update_ms;
  last_update_ms=now;
  if(g_output_position_test.command==2) {
    if(test_mode!=0U && g_output_position_test.state==1U) customer_finish(16U);
    g_output_position_test.command=0; g_output_position_test.state=3;
    g_output_position_test.fault=6; return trip(6);
  }
  if(g_output_position_test.command==1) {
    g_output_position_test.command=0;
    float amp=g_output_position_test.amplitude_deg;
    float center=g_output_position.actual_deg;
    /* [千问工作助理 2026-10-28] 可配频率扫频。frequency_hz==0 → 完全沿用
       改动前行为（0.1 Hz、40 s、五次多项式平滑过渡），既有 ±10/±45/±90°
       验收记录与主机测试断言均不受影响。
       frequency_hz>0 → 按该频率直接起振，不做 10 s 平滑（高频下平滑段
       相对周期过长，会淹没有效采样数据）。 */
    float freq=g_output_position_test.frequency_hz;
    float dur=g_output_position_test.duration_s;
    uint32_t trace_iv=g_output_position_test.trace_interval_ms;
    uint32_t mode=g_output_position_customer.test_mode;
    if(mode>2U) return trip(6);
    if(mode!=0U) {
      /* Explicit customer mode owns the fixed recipe, including FF=0. */
      amp=2.0f; freq=0; dur=0;
    }
    if(!isfinite(freq) || (freq<0 && freq!=-1.0f) || freq>OUTPUT_POSITION_SWEEP_MAX_FREQUENCY_HZ ||
       (freq!=0 && (!isfinite(dur) || dur<0 || dur>60)) ||
       !isfinite(g_output_position_test.feedforward_gain) ||
       g_output_position_test.feedforward_gain<0 || g_output_position_test.feedforward_gain>1 ||
       !isfinite(amp) || amp<=0 ||
       amp>OUTPUT_POSITION_SWEEP_MAX_AMPLITUDE_DEG) return trip(6);
    if(mode!=0U) {
      if(center<5 || center+amp>355 || g_output_position_test.state==1 ||
         fabsf(g_output_position.error_deg)>0.1f ||
         !isfinite(g_speed_run.actual_rpm) || fabsf(g_speed_run.actual_rpm)>50) return trip(6);
      dur=mode==2U ? 60.0f : 120.0f;
    } else if(freq==0.0f) {
      /* legacy 0.1 Hz profile: keep the original 40 s envelope exactly. */
      if(dur!=0.0f) return trip(6); /* duration is not settable in legacy mode */
      dur=40.0f;
    } else if(dur==0.0f) {
      /* [千问工作助理 2026-10-28] 扫频默认时长必须保证累积周期数达标。
         解调排除两端的起落振包络（当前由 sweep_ramp_s 统一计算），故
           累积周期数 = (dur - 2*ramp)*freq
         要满足 >= OUTPUT_POSITION_SWEEP_MIN_CYCLES，需
           dur >= 2*ramp + MIN_CYCLES/freq
         原写法取 max(2 s, 4 周期) 会在 5~10 Hz 处只累积 9~19 个周期，
         低于 20 周期门槛，导致 OutputPosition_ReadSweepFrd 的 valid 恒为 0、
         整个扫频测不出结果（静默失效，比报错更难发现）。
         下限仍保留 2 s（低频时避免过短），上限 60 s（避免低频时时长失控；
         若 60 s 内仍凑不够周期数，valid 会如实返回 0，而不是伪造结果）。 */
      dur=freq<0 ? 1.0f : 2.0f*sweep_ramp_s(freq)+
          (float)(OUTPUT_POSITION_SWEEP_MIN_CYCLES+1U)/freq;
      if(dur<2.0f) dur=2.0f;
      if(dur>60.0f) dur=60.0f;
    }
    if(mode!=0U) {
      /* Bounds checked above. No center-amplitude motion in this protocol. */
    } else if(freq==0.0f) {
      if(center-amp<5 || center+amp>355 ||
         g_output_position_test.state==1 || fabsf(g_output_position.error_deg)>0.1f ||
         fabsf(g_output_position.motor_rpm)>5) return trip(6);
    } else {
      /* Sweep keeps the same absolute-angle envelope guard, and additionally
         checks the LINEAR REGION: |error| must stay below the P-saturation
         threshold max_motor_rpm/(kp*202/6), otherwise the measured "bandwidth"
         is a saturation artifact and must not be trusted. */
      /* Codex 2026-10-28: reference amplitude is NOT tracking error. Permit
         large-signal tests; actual clipping below invalidates linear FRD. */
      if(center-amp<5 || center+amp>355 ||
         g_output_position_test.state==1 || fabsf(g_output_position.error_deg)>0.1f ||
         !isfinite(g_speed_run.actual_rpm) || fabsf(g_speed_run.actual_rpm)>50.0f) return trip(6);
    }
    test_mode=mode;
    test_amplitude=amp; test_center=center; test_start_ms=now; next_trace_ms=0;
    test_frequency_hz=freq; test_duration_s=dur;
    test_current_window_start_s=freq>0.0f ? sweep_ramp_s(freq) : 0.0f;
    test_current_window_end_s=dur-test_current_window_start_s;
    test_feedforward_gain=g_output_position_test.feedforward_gain;
    test_trace_interval_ms=trace_iv ? trace_iv : 500U;
    g_output_position_test.center_deg=center;
    g_output_position_test.frequency_hz=freq;
    g_output_position_test.duration_s=dur;
    g_output_position_test.trace_interval_ms=test_trace_interval_ms;
    g_output_position_test.fault=0; g_output_position_test.peak_error_deg=0;
    g_output_position_test.squared_error=0; g_output_position_test.samples=0;
    g_output_position_test.saturated_samples=0; g_output_position_test.peak_motor_rpm=0;
    g_output_position_test.slew_limited_samples=0;
    for(unsigned phase=0;phase<3;phase++) g_output_position_test.phase_squared_sum[phase]=0;
    g_output_position_test.current_samples=0;
    g_output_position_test.trace_count=0; g_output_position_test.state=1;
    if(mode!=0U) {
      g_output_position_test.amplitude_deg=amp;
      g_output_position_test.feedforward_gain=0;
      g_output_position_customer.frequency_hz=g_output_position_customer.completed=0;
      g_output_position_customer.resting=0;
      for(unsigned i=0;i<30;i++) {
        output_position_customer_result_t empty={0};
        empty.gain_db=empty.phase_deg=NAN; empty.flags=32U;
        g_output_position_customer_results[i]=empty;
      }
      customer_reset_fit();
    }
    g_output_position_sweep_frd.valid=0;
    /* [千问工作助理 2026-10-28] 复位正交解调累积量，防止上一次试验残留污染。
       同时复位 Welch 分段累积量，并按「整数个信号周期」计算段长：
         seg_len = round(1 / (freq * period_s))
       使每段恰好含整数个周期，段内 DFT bin 无泄漏；段长至少 1 个样本。 */
    qd_tc=qd_ts=qd_ac=qd_as=0.0f; qd_n=0;
    qd_wxy_r=qd_wxy_i=qd_wxx=qd_wyy=0.0f;
    qd_sxc=qd_sxs=qd_syc=qd_sys=0.0f;
    qd_sn=0; qd_seg_n=0;
    if(freq>0.0f) {
      float seg=1.0f/(freq*(float)OUTPUT_POSITION_PERIOD_MS*0.001f);
      uint32_t sl=(uint32_t)(seg+0.5f);
      qd_seg_len=sl ? sl : 1U;
    } else {
      qd_seg_len=0;
    }
    qd_active=(freq>0.0f) ? 1U : 0U;
  }
  if(g_output_position_test.state==1) {
    float t=(now-test_start_ms)*0.001f, offset, velocity;
    g_output_position_test.elapsed_s=t;
    if(test_mode!=0U) {
      offset=customer_target(now-test_start_ms); velocity=0;
    } else if(test_frequency_hz<0.0f) {
      /* Codex: finite step capture, 50 ms pre-trigger then positive amplitude.
         Host uses signed motor direction; output returns to session origin. */
      velocity=0;
      offset=t<0.05f ? 0 : test_amplitude;
      if(t>=test_duration_s) { offset=0; g_output_position_test.state=2; }
    } else if(test_frequency_hz==0.0f) {
      /* ---- legacy 0.1 Hz profile: byte-for-byte unchanged behaviour ----
         10 s smooth move to -amplitude, 2 cosine cycles at 0.1 Hz,
         10 s smooth return. Keeps every existing ±10/±45/±90° acceptance
         record and the host-test trace_count==80 / peak_velocity assertions. */
      if(t>=40.0f) {
        offset=velocity=0; g_output_position_test.state=2;
      } else if(t<10.0f || t>=30.0f) {
        float u=(t<10.0f?t:t-30.0f)*0.1f;
        float q=u*u*u*(10.0f+u*(-15.0f+6.0f*u));
        float dq=3.0f*u*u*(1.0f-u)*(1.0f-u);
        offset=test_amplitude*(t<10.0f?-q:q-1.0f);
        velocity=test_amplitude*dq*(t<10.0f?-1.0f:1.0f);
      } else {
        float phase=0.6283185307f*(t-10.0f);
        offset=-test_amplitude*cosf(phase);
        velocity=test_amplitude*0.6283185307f*sinf(phase);
      }
    } else {
      /* ---- [千问工作助理 2026-10-28] configurable-frequency sweep ----
         Raised-cosine envelope over sweep_ramp_s(f) so the velocity command
         starts from 0 instead of stepping to A*omega (a step would be a
         large-signal disturbance, not a linear FRF measurement). After the
         complete ramp the envelope is exactly 1: a pure steady-state sine.
           offset(t)   = -A*env(t)*cos(2*pi*f*t)
           velocity(t) = d(offset)/dt, computed analytically so the
                         feedforward stays consistent with the trajectory.
         env(t) = 0.5*(1-cos(pi*t/ramp)) for t<ramp, else 1
         d(env)/dt = pi/(2*ramp)*sin(pi*t/ramp) for t<ramp, else 0 */
      float w=6.283185307f*test_frequency_hz;
      float phase=w*t;
      float ramp=sweep_ramp_s(test_frequency_hz);
      float env,denv;
      if(t<ramp) {
        /* Half cosine rises monotonically 0->1 over the complete ramp. */
        float ep=3.141592654f*t/ramp;
        env=0.5f*(1.0f-cosf(ep));
        denv=1.570796327f/ramp*sinf(ep);
      } else if(t>test_duration_s-ramp) {
        float ep=3.141592654f*(test_duration_s-t)/ramp;
        env=0.5f*(1.0f-cosf(ep));
        denv=-1.570796327f/ramp*sinf(ep);
      } else { env=1.0f; denv=0.0f; }
      if(t>=test_duration_s) {
        offset=velocity=0; g_output_position_test.state=2;
        qd_active=0;
        /* [千问工作助理 2026-10-28] 扫频结束即发布频响结果。
           必须在 qd_active=0 之后、且此处 qd_n 已停止增长，故结果稳定。
           主机随后用一次 savemem 读 32 字节即可，无需调用函数。
           先算到局部变量再逐字段赋值，**不**对 volatile 全局做限定符强转
           （通过非 volatile 左值写 volatile 对象属未定义行为）。 */
        {
          output_position_sweep_frd_t r;
          sweep_frd_compute(&r);
          g_output_position_sweep_frd.gain_db=r.gain_db;
          g_output_position_sweep_frd.phase_deg=r.phase_deg;
          g_output_position_sweep_frd.coherence=r.coherence;
          g_output_position_sweep_frd.cycles=r.cycles;
          g_output_position_sweep_frd.frequency_hz=r.frequency_hz;
          g_output_position_sweep_frd.amplitude_deg=r.amplitude_deg;
          /* [千问工作助理 2026-10-28] Welch 相干性与段数。
             coherence_bin 恒等于 1，仅留证；判定用的是 coherence（Welch）。
             segments<2 时 reject=8，主机应据此判定「扫频太短，加长时长重测」。 */
          g_output_position_sweep_frd.coherence_bin=r.coherence_bin;
          g_output_position_sweep_frd.segments=r.segments;
          g_output_position_sweep_frd.reject=r.reject;
          /* valid 最后写：主机若在此刻读入，看到 valid=1 即表示其余字段已就绪。 */
          g_output_position_sweep_frd.valid=r.valid;
        }
      } else {
        offset=-test_amplitude*env*cosf(phase);
        velocity=-test_amplitude*(denv*cosf(phase)-env*w*sinf(phase));
        velocity*=test_feedforward_gain;
        /* Feedforward limit guard: exceeding ±75 deg/s trips fault 3 by design
           (OutputPosition_Check). Refuse to run a sweep that would trip. */
        if(fabsf(velocity)>OUTPUT_POSITION_MAX_VELOCITY_DEG_S) {
          offset=velocity=0; g_output_position_test.state=3;
          g_output_position_test.fault=7; qd_active=0;
          return trip(7);
        }
      }
    }
    g_output_position.target_deg=test_center+offset;
    g_output_position.velocity_deg_s=velocity;
  }
  if(sequence!=g_mpt_t485.sequence) {
    uint32_t raw=g_mpt_t485.angle_raw;
    int32_t delta=(int32_t)raw-(int32_t)previous_raw;
    if(delta>65536) delta-=131072;
    if(delta<-65536) delta+=131072;
    /* At <=2500 motor RPM /202, even 17 ms is <460 encoder counts.
       Reject discontinuities instead of guessing a lost revolution. */
    if(delta>512 || delta<-512) return trip(4);
    unwrapped_count+=delta; previous_raw=raw; sequence=g_mpt_t485.sequence;
    g_output_position.actual_deg=unwrapped_count*(360.0f/131072.0f);
  }
  float target=g_output_position.target_deg, actual=g_output_position.actual_deg;
  if(!OutputPosition_Compute(target,actual,kp,max_rpm,deadband,direction,rpm)) return trip(3);
  g_output_position.error_deg=target-actual;
  *rpm += g_output_position.velocity_deg_s*(202.0f/6.0f)*direction;
  if(*rpm>max_rpm) *rpm=max_rpm;
  if(*rpm<-max_rpm) *rpm=-max_rpm;
  if(g_output_position_test.state==1) {
    float error=fabsf(target-actual);
    if(error>10 || actual<test_center-test_amplitude-2 || actual>test_center+test_amplitude+2)
      return trip(6);
    if(error>g_output_position_test.peak_error_deg) g_output_position_test.peak_error_deg=error;
    g_output_position_test.squared_error+=error*error; g_output_position_test.samples++;
    /* [千问工作助理 2026-10-28] 饱和与峰值统计。saturated_samples 必须为 0，
       否则本次扫频测出的「带宽」是限幅伪影，不可采信。
       工程验收文档 2026-10-22_闭环整定模块实施方案_规划记录.md:231 明确要求
       「防止饱和实验冒充线性带宽测试」。 */
    {
      float ar=fabsf(*rpm);
      if(ar>g_output_position_test.peak_motor_rpm) g_output_position_test.peak_motor_rpm=ar;
      if(ar>=max_rpm-0.5f) g_output_position_test.saturated_samples++;
      float et=(now-test_start_ms)*0.001f;
      float settle=test_frequency_hz>0 ? sweep_ramp_s(test_frequency_hz) : 0;
      uint8_t steady=test_frequency_hz<=0 ||
          (et>=settle && et<test_duration_s-settle);
      if(steady && fabsf(*rpm-g_output_position.motor_rpm)>
          OutputPosition_RampRpmS()*(float)update_elapsed_ms*0.001f+0.5f)
        g_output_position_test.slew_limited_samples++;
    }
    /* [千问工作助理 2026-10-28] 单频正交解调，直接得到闭环频响。
       仅在纯稳态正弦段累积（跳过升余弦起振段），符合「先等暂态衰减再采集
       完整整数周期」的要求。x=参考偏移，y=实际偏移，均为输出轴度数。
         Syx_c = sum(y*cos(w*t)), Syx_s = sum(y*sin(w*t))
         Sxx_c = sum(x*cos(w*t)), Sxx_s = sum(x*sin(w*t))
       H(f) = (Syx_c*Sxx_c + Syx_s*Sxx_s + j*(Syx_s*Sxx_c - Syx_c*Sxx_s))
              / (Sxx_c^2 + Sxx_s^2)
       由 OutputPosition_ReadSweepFrd() 在停机后换算为 dB 与度。
       只用 4 个 float 累积量，故可在 1 kHz 周期上累积数千样本而 RAM 增量可忽略。

       同时维护 Welch 分段累积量（qd_sxc/sxs/syc/sys 为当前段），每满
       qd_seg_len 个样本（= 整数个信号周期）结算一段到跨段累积量：
         X_i = sxc - j*sxs ,  Y_i = syc - j*sys
         Y_i*conj(X_i) = (syc*sxc + sys*sxs) + j*(syc*sxs - sys*sxc)
         |X_i|^2 = sxc^2 + sxs^2 ,  |Y_i|^2 = syc^2 + sys^2
       段长取整数周期是为了让每段的 DFT bin 无泄漏；不足一段的尾部样本在
       扫频结束时由 sweep_frd_flush_partial_segment() 结算，避免丢弃数据。 */
    float settle_s=test_frequency_hz>0 ? sweep_ramp_s(test_frequency_hz) : 0;
    if(test_frequency_hz>0.0f && qd_active &&
       (now-test_start_ms)*0.001f>=settle_s &&
       (now-test_start_ms)*0.001f<test_duration_s-settle_s) {
      float t=(now-test_start_ms)*0.001f;
      float w=6.283185307f*test_frequency_hz;
      float ph=w*t, c=cosf(ph), s=sinf(ph);
      float x=target-test_center;
      float y=actual-test_center;
      qd_tc+=x*c; qd_ts+=x*s; qd_ac+=y*c; qd_as+=y*s; qd_n++;
      qd_sxc+=x*c; qd_sxs+=x*s; qd_syc+=y*c; qd_sys+=y*s; qd_sn++;
      if(qd_seg_len && qd_sn>=qd_seg_len) sweep_frd_flush_segment();
    }
    /* Keep the final steady-state window for sweeps instead of filling the
       entire 81-row buffer with startup. Legacy 40-second traces unchanged. */
    float trace_end=test_frequency_hz>0 ? test_duration_s-sweep_ramp_s(test_frequency_hz) : test_duration_s;
    float trace_start=trace_end-(OUTPUT_POSITION_TRACE_CAPACITY-1U)*test_trace_interval_ms*0.001f;
    if(test_mode==0U && (test_frequency_hz<=0 || ((now-test_start_ms)*0.001f>=trace_start &&
       (now-test_start_ms)*0.001f<trace_end)) &&
       (now-test_start_ms)>=next_trace_ms && g_output_position_test.trace_count<OUTPUT_POSITION_TRACE_CAPACITY) {
      uint32_t i=g_output_position_test.trace_count;
      g_output_position_trace[i]=(output_position_trace_t){(now-test_start_ms)*0.001f,target,actual,*rpm};
      g_output_position_test.trace_count=i+1; next_trace_ms=(now-test_start_ms)+test_trace_interval_ms;
    }
    if(test_mode!=0U) customer_observe(now-test_start_ms,target-test_center,actual-test_center);
  }
  g_output_position.motor_rpm=*rpm; g_output_position.updates++;
  return 1;
}
/* [千问工作助理 2026-10-28] 读取上一次扫频的单频闭环频响。
   返回 1 且 *valid=1 表示该频点数据可采信；否则 *valid=0，
   调用方不得把结果当作带宽证据。判定条件：
   1) 必须做过频率>0 的扫频且已累积到样本（qd_n>0）；
   2) saturated_samples 必须为 0（限幅数据是伪影）；
   3) 参考能量不得退化（Sxx_c^2+Sxx_s^2 > 0）；
   4) 累积周期数必须 >= OUTPUT_POSITION_SWEEP_MIN_CYCLES，否则频谱泄漏
      会同时偏差增益与相位（见下方泄漏说明）；
   5) 相对相干性 gamma^2 = |Sxy|^2/(Sxx*Syy) 必须 >= 0.9，
      与工程验收文档 2026-10-22_闭环整定模块实施方案_规划记录.md:157
      「设计频带相干性不低于 0.9」一致。
   增益为闭环幅频 |H|，相位为 arg(H)，即参考→实际的跟踪响应。

   ⚠️ 符号约定修正记录（千问工作助理 2026-10-28）：
   首版写成 im=(qd_as*qd_tc-qd_ac*qd_ts)/sxx，该式在 x=cos(wt)、y=sin(wt)
   （纯 -90° 滞后）工况下给出 **+90°（超前）**。因果闭环系统的跟踪相位
   只可能滞后，不可能超前，故首版符号是错的，会使全部相位反号、进而把
   「稳定」误判为「不稳定」。已用数值实验独立验证并修正为
   im=(qd_ac*qd_ts-qd_as*qd_tc)/sxx。
   验证脚本：control_analysis/scripts/verify_quadrature_demod_sign.py
   推导：取共轭 DFT bin
     Y = sum(y*conj(e)) = qd_ac - j*qd_as
     X = sum(x*conj(e)) = qd_tc - j*qd_ts
     H = Y/X = (qd_ac - j*qd_as)(qd_tc + j*qd_ts)/sxx
       = [qd_ac*qd_tc + qd_as*qd_ts + j*(qd_ac*qd_ts - qd_as*qd_tc)]/sxx
   即实部不变，虚部符号与首版相反。

   频谱泄漏说明：单频解调要求窗口含整数个周期，否则泄漏会偏差结果。
   位置环实际周期取决于主循环，未必精确 1 ms，故无法保证严格整数周期；
   改为要求足够多的周期数把泄漏压到可忽略——泄漏幅度按 1/N 衰减，
   N>=OUTPUT_POSITION_SWEEP_MIN_CYCLES 时增益/相位偏差已远小于本函数
   的判读精度（数值实验见验证脚本 case 5）。 */
/* [千问工作助理 2026-10-28] Welch 分段结算。
   把当前段的 X_i = qd_sxc - j*qd_sxs、Y_i = qd_syc - j*qd_sys 结算进跨段累积：
     Y_i*conj(X_i) = (syc*sxc + sys*sxs) + j*(syc*sxs - sys*sxc)
     |X_i|^2       = sxc^2 + sxs^2
     |Y_i|^2       = syc^2 + sys^2
   注意符号：Y*conj(X) 的虚部是 (syc*sxs - sys*sxc)，与单 bin 版 H 的虚部
   (qd_ac*qd_ts - qd_as*qd_tc) 形式一致，即两者共用同一套已验证的符号约定。
   段内样本数为 0 时直接返回，避免把空段计入分母。 */
static void sweep_frd_flush_segment(void)
{
  if(!qd_sn) return;
  qd_wxy_r+=qd_syc*qd_sxc+qd_sys*qd_sxs;
  qd_wxy_i+=qd_syc*qd_sxs-qd_sys*qd_sxc;
  qd_wxx  +=qd_sxc*qd_sxc+qd_sxs*qd_sxs;
  qd_wyy  +=qd_syc*qd_syc+qd_sys*qd_sys;
  qd_seg_n++;
  qd_sxc=qd_sxs=qd_syc=qd_sys=0.0f; qd_sn=0;
}
/* 扫频结束时结算尾部不足一段的样本。段长按整数信号周期设定，而总时长未必是
   段长的整数倍，故通常会有一个尾部残段。丢弃它会让相干性少一段统计，虽然
   不致命，但计入更充分利用数据；该残段同样按同一公式结算，无特殊处理。 */
static void sweep_frd_flush_partial_segment(void)
{
  sweep_frd_flush_segment();
}
/* [千问工作助理 2026-10-28] 单频解调的**唯一**计算实现。
   发布路径（扫频结束时写入 g_output_position_sweep_frd）与读取接口
   （OutputPosition_ReadSweepFrd）都调用它，确保固件内只有一份数学。
   这样做的动机：本轮首版解调曾把虚部符号写反，且因无任何测试覆盖而未被
   发现（详见上方修正记录）。若再让主机侧另写一份换算，同样的分叉风险会
   重演——两份实现各自"看起来对"，却在符号约定上分道扬镳。

   判定顺序（先拦最硬性的、再拦统计性的）：
     1 无样本 → 2 饱和伪影 → 3 能量退化 → 4 周期数不足
     → 7 幅值退化 → 8 分段不足 → 6 因果性 → 5 相干性
   顺序有讲究：因果性(6)排在相干性(5)之前，因为符号/接线错误是**结构性**
   缺陷，比统计污染更根本；先报它能让排查直接指向根因。 */
static void sweep_frd_compute(output_position_sweep_frd_t *out)
{
  /* 显式逐字段清零，不用 memset：本文件未包含 <string.h>，
     依赖 HAL 头间接引入不可靠。 */
  out->gain_db=0; out->phase_deg=0; out->coherence=0; out->cycles=0;
  out->frequency_hz=test_frequency_hz; out->amplitude_deg=test_amplitude;
  out->valid=0; out->reject=0;
  out->coherence_bin=0; out->segments=0;
  /* 先把尾部残段结算掉，使段数统计完整。结算只改 Welch 累积量，
     不影响单 bin 累积量，故对 gain/phase 无副作用。 */
  sweep_frd_flush_partial_segment();
  if(!qd_n) { out->reject=1; return; }
  if(g_output_position_test.saturated_samples || g_output_position_test.slew_limited_samples) {
    out->reject=2; return;
  }
  float sxx=qd_tc*qd_tc+qd_ts*qd_ts;
  float syy=qd_ac*qd_ac+qd_as*qd_as;
  if(!(sxx>0.0f) || !(syy>0.0f) || !isfinite(sxx) || !isfinite(syy)) { out->reject=3; return; }
  /* 累积周期数不足则拒绝判定（泄漏未压够）。qd_n 为样本数，乘以周期即周期数。 */
  out->cycles=(float)qd_n*test_frequency_hz*(float)OUTPUT_POSITION_PERIOD_MS*0.001f;
  if(!(out->cycles>=(float)OUTPUT_POSITION_SWEEP_MIN_CYCLES)) { out->reject=4; return; }
  float re=(qd_ac*qd_tc+qd_as*qd_ts)/sxx;
  float im=(qd_ac*qd_ts-qd_as*qd_tc)/sxx;   /* 符号已修正，见上方推导 */
  float mag2=re*re+im*im;
  if(!isfinite(mag2) || mag2<=0.0f) { out->reject=7; return; }
  /* 单 bin 相干性：代数上恒等于 1，仅作留证保留，**不得**用于判定。 */
  out->coherence_bin=(mag2*sxx)/syy;
  out->gain_db=10.0f*log10f(mag2);
  out->phase_deg=atan2f(im,re)*57.29577951f;
  /* Welch 相干性：分段估计后才有统计意义。 */
  out->segments=qd_seg_n;
  /* 分段门槛（防御性）。⚠️ 当前段长 seg_len = round(1/(freq*period_s)) 恰好
     等于 1 个信号周期的样本数，而 cycles = qd_n*freq*period_s = qd_n/seg_len、
     segments ≈ qd_n/seg_len，两者数值相同。故上面的 cycles>=MIN_CYCLES(20)
     必然推出 segments>=20>=MIN_SEGMENTS(2)，**本门槛被 reject=4 严格支配，
     在当前段长选择下不可单独触发**（主机测试用例 7 已记录此结论）。
     保留它是为了：若将来把段长改为多个信号周期（降低相干性估计方差），
     该门槛才会变得可达，届时不必重新想起要加这道检查。 */
  if(qd_seg_n<OUTPUT_POSITION_SWEEP_MIN_SEGMENTS) { out->reject=8; return; }
  if(!(qd_wxx>0.0f) || !(qd_wyy>0.0f) || !isfinite(qd_wxx) || !isfinite(qd_wyy)) {
    out->reject=3; return;
  }
  out->coherence=(qd_wxy_r*qd_wxy_r+qd_wxy_i*qd_wxy_i)/(qd_wxx*qd_wyy);
  /* 因果性自检：闭环跟踪相位必须滞后（<=0）。若算出超前，说明符号约定、
     参考/反馈接线或方向极性有问题，该数据一律不可采信。 */
  /* Codex 2026-10-28 correction: causality does NOT prohibit phase lead.
     Velocity feedforward can produce a positive tracking phase. Analytic
     lag/lead tests establish the DFT convention; do not censor measurements. */
  if(!(out->coherence>=0.9f)) { out->reject=5; return; }
  out->valid=1;
}
uint8_t OutputPosition_ReadSweepFrd(float *gain_db,float *phase_deg,
                                   float *coherence,float *cycles_out,
                                   uint32_t *valid)
{
  if(!gain_db || !phase_deg || !coherence || !cycles_out || !valid) return 0;
  output_position_sweep_frd_t r;
  sweep_frd_compute(&r);
  /* 同步发布到全局，使主机用一次 savemem 就能读到与函数返回完全一致的结果。
     调试器只能读写内存、无法调用函数，故这是上位机取得结论的唯一途径。 */
  g_output_position_sweep_frd=r;
  *gain_db=r.gain_db; *phase_deg=r.phase_deg;
  *coherence=r.coherence; *cycles_out=r.cycles; *valid=r.valid;
  return 1;
}
uint8_t OutputPosition_Reference(float *rpm)
{
  if(!OutputPosition_Check()) return 0;
  if(g_output_position.state==1) *rpm=g_output_position.motor_rpm;
  return 1;
}
void OutputPosition_Service(uint8_t off)
{
  /* Position math belongs to the foreground, not the current-loop ISR. */
  if(test_mode!=0U && g_output_position_test.state==3U) customer_finish(16U);
  if(!g_output_position.command) {
    if(g_output_position.state==1) {
      float rpm;
      (void)OutputPosition_Update(&rpm);
    }
    return;
  }
  uint32_t mask=__get_PRIMASK(); __disable_irq();
  uint32_t command=g_output_position.command;
  if(command==2) {
    if(g_output_position_test.state==1) g_output_position_test.state=3;
    g_output_position.velocity_deg_s=0;
    g_speed_run.enable=0; g_speed_run.target_rpm=0;
    g_output_position.motor_rpm=0;
    if(g_output_position.state!=2) g_output_position.state=0;
    g_output_position.command=0;
  } else if(command==3 && off) {
    g_output_position.state=g_output_position.fault=g_output_position.command=0;
  } else if(command==1) {
    g_output_position.command=0;
    float ignored;
    if(!off || g_output_position.state || !g_output_position.commissioned ||
       !g_mpt_t485.enabled || !g_mpt_t485.valid ||
       (uint32_t)(HAL_GetTick()-g_mpt_t485.last_good_ms)>OUTPUT_POSITION_MAX_AGE_MS ||
       !g_speed_test.calibration_ok || g_speed_test.state || g_speed_test.command ||
       g_speed_run.enable || g_speed_test.fault || g_speed_run.fault ||
       !OutputPosition_Compute(0,0,g_output_position.kp_per_s,
           g_output_position.max_motor_rpm,g_output_position.deadband_deg,
           g_output_position.motor_direction,&ignored)) {
      /* Refusal must never stop an unrelated running speed test. */
      g_output_position.fault=5;
    } else {
      kp=g_output_position.kp_per_s; max_rpm=g_output_position.max_motor_rpm;
      deadband=g_output_position.deadband_deg; direction=g_output_position.motor_direction;
      last_update_ms=HAL_GetTick()-OUTPUT_POSITION_PERIOD_MS;
      previous_raw=g_mpt_t485.angle_raw; sequence=g_mpt_t485.sequence;
      unwrapped_count=(int32_t)previous_raw;
      g_output_position.target_deg=g_output_position.actual_deg=g_mpt_t485.angle_deg;
      g_output_position.error_deg=g_output_position.motor_rpm=0;
      g_output_position.velocity_deg_s=0;
      g_output_position.fault=0; g_output_position.state=1;
      g_speed_run.target_rpm=0; g_speed_run.enable=1;
    }
  }
  if(g_output_position.state==1 && (g_speed_run.fault || g_speed_test.state==4)) (void)trip(2);
  if(!mask) __enable_irq();
}
