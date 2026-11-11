#include "output_position.h"
#include "customer_pwm_table.h"
#include "mpt_t485.h"
#include "speed_commission.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
uint32_t test_tick;
speed_run_t g_speed_run;
speed_commission_t g_speed_test;
volatile mpt_t485_t g_mpt_t485;
static void crc(uint8_t *f) { f[10]=0; for(unsigned i=0;i<10;i++) f[10]^=f[i]; }
/* [千问工作助理 2026-10-28] arm 辅助函数参数化。
   原因：Service 在 state==1 时收到 command=1 会拒绝并置 fault=5，且**不会**
   重新锁存 kp/max_rpm/deadband（这三个值只在成功 arm 时由 Service 拷入静态
   变量）。故高增益扫频测试必须在 arm **之前**就设好参数，不能 arm 后再改。
   ready() 保持原默认值（kp=0.5, max=100, deadband=0.01），既有断言不受影响。 */
static void ready_cfg(uint32_t raw,float kp,float max_rpm,float deadband) {
  memset((void *)&g_output_position,0,sizeof(g_output_position));
  memset((void *)&g_output_position_test,0,sizeof(g_output_position_test));
  memset((void *)&g_output_position_customer,0,sizeof(g_output_position_customer));
  g_output_position_test.feedforward_gain=0.0f;
  memset((void *)&g_mpt_t485,0,sizeof(g_mpt_t485));
  memset(&g_speed_run,0,sizeof(g_speed_run));
  memset(&g_speed_test,0,sizeof(g_speed_test));
  g_speed_test.calibration_ok=1;
  g_mpt_t485.enabled=g_mpt_t485.valid=1;
  test_tick=g_mpt_t485.last_good_ms=100;
  g_mpt_t485.angle_raw=raw; g_mpt_t485.angle_deg=raw*360.0f/131072;
  g_output_position.commissioned=1; g_output_position.motor_direction=1;
  g_output_position.kp_per_s=kp; g_output_position.max_motor_rpm=max_rpm;
  g_output_position.deadband_deg=deadband;
  g_output_position.command=1; OutputPosition_Service(1);
  assert(g_output_position.state==1 && g_speed_run.enable);
  assert(g_output_position.target_deg==g_mpt_t485.angle_deg);
}
static void ready(uint32_t raw) { ready_cfg(raw,0.5f,100.0f,0.01f); }
int main(void) {
  uint8_t f[11]={0x1a,0,0x45,0x23,1,0x17,0xfe,0xff,0xff,0,0};
  uint32_t a=0,t=0; crc(f);
  assert(MptT485_Decode(f,11,&a,&t)==0 && a==0x12345 && t==0xfffffe);
  for(unsigned i=0;i<11;i++) for(unsigned bit=0;bit<8;bit++) {
    f[i]^=1U<<bit; assert(MptT485_Decode(f,11,&a,&t)!=0); f[i]^=1U<<bit;
  }
  assert(MptT485_Decode(f,10,&a,&t)==2);
  assert(MptT485_Decode(f,12,&a,&t)==2);
  f[4]=2;crc(f);assert(MptT485_Decode(f,11,&a,&t)==2);
  f[4]=0;f[1]=0x40;crc(f);assert(MptT485_Decode(f,11,&a,&t)==3);
  f[1]=0;f[9]=0x80;crc(f);assert(MptT485_Decode(f,11,&a,&t)==3);
  memset(f,0,11);f[0]=0x1a;f[5]=0x17;crc(f);
  assert(MptT485_Decode(f,11,&a,&t)==0 && a==0);
  f[0]=0x02;crc(f);assert(MptT485_Decode(f,11,&a,&t)==2);
  float rpm;
  assert(OutputPosition_Compute(1,0,1,500,0,1,&rpm) && fabsf(rpm-202.0f/6)<0.001f);
  assert(OutputPosition_Compute(0,359,1,100,0,1,&rpm) && rpm==-100);
  assert(OutputPosition_Compute(360,1,1,100,0,-1,&rpm) && rpm==-100);
  assert(OutputPosition_Compute(10,10.005f,1,100,.01f,1,&rpm) && rpm==0);
  assert(!OutputPosition_Compute(NAN,0,1,100,0,1,&rpm));
  assert(!OutputPosition_Compute(0,0,1,100,0,0,&rpm));
  assert(OutputPosition_Compute(720,0,1,100,0,1,&rpm) && rpm==100);
  assert(OutputPosition_Compute(-180,0,1,100,0,1,&rpm) && rpm==-100);
  /* [千问工作助理 2026-10-28] kp 上限边界断言。
     由变异测试补入：变异 M9（移除 `kp>OUTPUT_POSITION_MAX_KP_PER_S` 检查）
     此前**未被任何断言捕获**，而 kp 上限正是本轮最核心的改动——原硬编码
     `kp>10` 使交越频率锁死在 10/2π=1.59 Hz，与 20 Hz 目标差 12.6 倍。
     若将来有人静默把上限改回 10、或彻底移除校验，以下断言会立即失败。
     20 Hz 设计点 kp=158.3 /s（见调试日志 §8.2）必须可用。 */
  assert(OutputPosition_Compute(1,0,OUTPUT_POSITION_MAX_KP_PER_S,2500,0,1,&rpm));      /* 边界值可用 */
  assert(OutputPosition_Compute(1,0,158.3f,2500,0,1,&rpm));                            /* 20 Hz 设计点可用 */
  assert(!OutputPosition_Compute(1,0,OUTPUT_POSITION_MAX_KP_PER_S+0.001f,2500,0,1,&rpm)); /* 超上限必须拒绝 */
  assert(!OutputPosition_Compute(1,0,0.0f,2500,0,1,&rpm));                             /* kp=0 必须拒绝 */
  assert(!OutputPosition_Compute(1,0,-1.0f,2500,0,1,&rpm));                            /* 负增益必须拒绝 */
  assert(OUTPUT_POSITION_MAX_KP_PER_S>=158.3f);   /* 上限必须容得下 20 Hz 设计点 */
  assert(OUTPUT_POSITION_RAMP_RPM_S>=20.0f*2.0f*3.141592654f*1152.0f); /* 需覆盖 20 Hz 小信号的 144765 rpm/s */
  ready(65536);g_output_position.target_deg=181;rpm=0;
  assert(OutputPosition_Update(&rpm) && rpm>0);
  float held=rpm; uint32_t updates=g_output_position.updates;
  /* [千问工作助理 2026-10-28] 原写法 `test_tick++` 隐含假设「1 ms < 一个轮询
     周期」。本轮把 OUTPUT_POSITION_PERIOD_MS 由 5 改为 1 后该假设失效，断言
     在第 58 行失败（rpm!=held）。
     断言的真实意图是「未到下一个周期前重复调用不得重算」，因此改为在**同一
     tick** 上重复调用：这对任意周期值都成立（含周期=1 ms 的极端情形，因为
     tick 分辨率就是 1 ms，无法表达比 1 ms 更小的间隔）。被测源码未改动。 */
  g_output_position.target_deg=182;
  assert(OutputPosition_Update(&rpm) && rpm==held && g_output_position.updates==updates);
  test_tick+=OUTPUT_POSITION_PERIOD_MS;
  assert(OutputPosition_Update(&rpm) && rpm>held && g_output_position.updates==updates+1);
  test_tick=118;assert(!OutputPosition_Check() && !g_speed_run.enable && g_output_position.fault==1);
  g_mpt_t485.last_good_ms=118;assert(!OutputPosition_Check()); /* latched */
  ready(65536);g_mpt_t485.angle_raw+=513;g_mpt_t485.sequence++;
  assert(!OutputPosition_Update(&rpm) && g_output_position.fault==4);
  ready(131071);g_mpt_t485.angle_raw=0;g_mpt_t485.sequence++;
  assert(OutputPosition_Update(&rpm) && g_output_position.actual_deg==360);
  test_tick+=OUTPUT_POSITION_PERIOD_MS;
  g_mpt_t485.angle_raw=1;g_mpt_t485.sequence++;
  assert(OutputPosition_Update(&rpm) && g_output_position.actual_deg>360);
  ready(1);g_mpt_t485.angle_raw=131071;g_mpt_t485.sequence++;
  assert(OutputPosition_Update(&rpm) && g_output_position.actual_deg<0);
  ready(65536);g_output_position.command=2;
  assert(OutputPosition_Check() && !g_speed_run.enable && g_output_position.state==0);
  ready(0);g_output_position.target_deg=720;
  for(unsigned i=1;i<=1024;i++) {
    test_tick+=OUTPUT_POSITION_PERIOD_MS;
    g_mpt_t485.last_good_ms=test_tick;
    g_mpt_t485.angle_raw=(i*256U)%131072U;
    g_mpt_t485.sequence++;
    assert(OutputPosition_Update(&rpm));
  }
  assert(g_output_position.actual_deg==720 && rpm==0);
  g_output_position.target_deg=-360;
  for(unsigned i=1;i<=1536;i++) {
    test_tick+=OUTPUT_POSITION_PERIOD_MS;
    g_mpt_t485.last_good_ms=test_tick;
    g_mpt_t485.angle_raw=(131072U-(i*256U)%131072U)%131072U;
    g_mpt_t485.sequence++;
    assert(OutputPosition_Update(&rpm));
  }
  assert(g_output_position.actual_deg==-360 && rpm==0);
  ready(65536);g_output_position.target_deg=INFINITY;
  assert(!OutputPosition_Check() && g_output_position.fault==3);
  ready(65536);g_output_position.command=2;OutputPosition_Service(0);
  g_output_position.commissioned=0;g_output_position.command=1;OutputPosition_Service(1);
  assert(!g_speed_run.enable && g_output_position.fault==5);
  assert(OutputPosition_Compute(100,0,2,2500,0,1,&rpm) && rpm==2500);
  assert(!OutputPosition_Compute(0,0,2,OUTPUT_POSITION_MAX_MOTOR_RPM+1,0,1,&rpm));
  assert(OutputPosition_Compute(0,0,2,OUTPUT_POSITION_MAX_MOTOR_RPM,0,1,&rpm));
  ready(65536);g_output_position.velocity_deg_s=INFINITY;
  assert(!OutputPosition_Check() && g_output_position.fault==3);
  ready(65536);g_output_position_test.amplitude_deg=91;g_output_position_test.command=1;
  assert(!OutputPosition_Update(&rpm) && g_output_position.fault==6);
  ready(65536);g_output_position_test.amplitude_deg=90;g_output_position_test.command=1;
  float min_target=360,max_target=0,peak_velocity=0;
  for(unsigned i=0;i<=8000;i++) {
    test_tick=100+i*5; g_mpt_t485.last_good_ms=test_tick;
    /* Ideal plant follows the previous tick; exercise wrap/age, trajectory and completion. */
    g_mpt_t485.angle_raw=(uint32_t)(g_output_position.target_deg*131072.0f/360.0f);
    g_mpt_t485.sequence++;
    assert(OutputPosition_Update(&rpm));
    float target=g_output_position.target_deg;
    if(target<min_target) min_target=target;
    if(target>max_target) max_target=target;
    if(fabsf(g_output_position.velocity_deg_s)>peak_velocity) peak_velocity=fabsf(g_output_position.velocity_deg_s);
  }
  assert(g_output_position_test.state==2 && g_output_position_test.trace_count==OUTPUT_POSITION_TRACE_CAPACITY);
  assert(fabsf(min_target-90)<.01f && fabsf(max_target-270)<.01f);
  assert(fabsf(peak_velocity-56.54867f)<.01f);
  assert(g_output_position.target_deg==180 && g_output_position.velocity_deg_s==0);
  ready(65536);g_output_position_test.command=2;
  assert(!OutputPosition_Update(&rpm) && !g_speed_run.enable);

  /* ==========================================================================
     [千问工作助理 2026-10-28] 扫频与正交解调回归测试
     --------------------------------------------------------------------------
     背景：本轮新增了可配频率扫频与 MCU 内单频正交解调，但既有测试只覆盖
     legacy 0.1 Hz 路径，新代码零覆盖。事实上首版解调的**虚部符号写反**，
     在纯 -90° 滞后工况下算出 +90°（超前）——因果闭环不可能超前，属严重
     缺陷，却因无测试而未被自动发现。以下测试即为锁死该符号约定而写。

     验证方法是**独立**的：不预设答案，而是用一个离散一阶滞后环节仿真被控
     对象（它天然因果、相位必为负），再解析地算出该环节的离散频响作为期望
     值，与解调输出比对。期望值推导：
       alpha = dt/(tau+dt),  tau = 1/(2*pi*fc)
       H_plant(z) = alpha / (1 - (1-alpha) z^-1),  z^-1 = e^{-j w dt}
       仿真中 raw 由上一拍的 target 驱动，故再叠加一拍纯延迟 e^{-j w dt}
     ========================================================================== */
  {
    const double PI=3.14159265358979323846;
    /* A quantized holding P command is not motion. Gate measured motor speed. */
    ready_cfg(65536,95,4500,.0055f);
    g_output_position.motor_rpm=26.36f;
    g_output_position.error_deg=.00824f;
    g_output_position_test.amplitude_deg=2;
    g_output_position_test.frequency_hz=5;
    g_output_position_test.command=1;
    assert(OutputPosition_Update(&rpm) && !g_output_position.fault);
    ready_cfg(65536,95,4500,.0055f);
    g_speed_run.actual_rpm=51;
    g_output_position_test.amplitude_deg=2;
    g_output_position_test.frequency_hz=5;
    g_output_position_test.command=1;
    assert(!OutputPosition_Update(&rpm) && g_output_position.fault==6);
    /* Codex: finite step and monotone ramp-in boundary regression. */
    ready_cfg(65536,20,2500,0);
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_RAMP_RPM_S);
    g_output_position_test.amplitude_deg=2;
    g_output_position_test.frequency_hz=-1;
    g_output_position_test.duration_s=1;
    g_output_position_test.trace_interval_ms=5;
    g_output_position_test.command=1;
    assert(OutputPosition_Update(&rpm));
    /* A live edit must not grant the fast sine ramp to a latched step. */
    g_output_position_test.frequency_hz=20;
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_RAMP_RPM_S);
    OutputPosition_ObservePhaseCurrents(3,4,0);
    OutputPosition_ObservePhaseCurrents(0,-4,3);
    assert(g_output_position_test.current_samples==2);
    assert(g_output_position_test.phase_squared_sum[0]==9);
    assert(g_output_position_test.phase_squared_sum[1]==32);
    assert(g_output_position_test.phase_squared_sum[2]==9);
    for(unsigned j=1;j<=1000;j++) {
      test_tick++; g_mpt_t485.last_good_ms=test_tick;
      assert(OutputPosition_Update(&rpm));
      if(j<50) assert(g_output_position.target_deg==180);
      if(j>=50 && j<1000) assert(g_output_position.target_deg==182);
    }
    assert(g_output_position_test.state==2 && g_output_position.target_deg==180);
    OutputPosition_ObservePhaseCurrents(100,100,100);
    assert(g_output_position_test.current_samples==2);
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_RAMP_RPM_S);
    assert(g_output_position_test.trace_count==OUTPUT_POSITION_TRACE_CAPACITY);
    ready_cfg(65536,20,2500,0);
    g_output_position_test.amplitude_deg=.25f;
    g_output_position_test.frequency_hz=20;
    g_output_position_test.feedforward_gain=1;
    g_output_position_test.command=1;
    assert(OutputPosition_Update(&rpm));
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_SWEEP_RAMP_RPM_S);
    OutputPosition_ObservePhaseCurrents(100,100,100);
    assert(g_output_position_test.current_samples==0);
    g_output_position_test.frequency_hz=-1;
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_SWEEP_RAMP_RPM_S);
    float prev=g_output_position.target_deg;
    for(unsigned j=0;j<500;j++) {
      test_tick++; g_mpt_t485.last_good_ms=test_tick;
      assert(OutputPosition_Update(&rpm));
      assert(fabsf(g_output_position.target_deg-prev)<.04f);
      prev=g_output_position.target_deg;
      if(j==124) {
        /* Analytic 20 Hz at t=0.125s, quarter of its 0.5s envelope:
           carrier cos(5*pi)=-1 and sin(5*pi)=0. */
        float env=0.5f*(1.0f-cosf(3.141592654f/4.0f));
        assert(fabsf(g_output_position.target_deg-(180.0f+.25f*env))<.0001f);
        assert(fabsf(g_output_position.velocity_deg_s-
                     .25f*3.141592654f*sinf(3.141592654f/4.0f))<.001f);
        OutputPosition_ObservePhaseCurrents(100,100,100);
        assert(g_output_position_test.current_samples==0);
      }
    }
    OutputPosition_ObservePhaseCurrents(6,-3,-3);
    assert(g_output_position_test.current_samples==1);
    test_tick+=3000; g_mpt_t485.last_good_ms=test_tick;
    assert(OutputPosition_Update(&rpm) && g_output_position_test.state==2);
    assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_RAMP_RPM_S);
    /* [千问工作助理 2026-10-28] 结构体尺寸守卫。
       上位机用 savemem 按**硬编码字节数**读取这些结构体
       （test_mpt_position_sine.py:14 / commission_mpt_position.py:15）。
       若固件侧增删字段而未同步脚本，savemem 会**静默少读尾部字节**——
       不报错，但新字段全读不到。本轮就因此把 mpt 88→100、trajectory 40→60。
       以下断言把「脚本应使用的字节数」钉死在固件测试里，字段一变即失败。 */
    assert(sizeof(output_position_sweep_frd_t)==OUTPUT_POSITION_SWEEP_FRD_BYTES);
    assert(sizeof(output_position_t)==56);
    assert(sizeof(output_position_test_t)==84);
    assert(sizeof(mpt_t485_t)==108);
    assert(sizeof(output_position_trace_t)==16);
    assert(OUTPUT_POSITION_TRACE_CAPACITY==64);
    const double fc=20.0, dt=0.001, f_sig=20.0;
    const double tau=1.0/(2.0*PI*fc), alpha=dt/(tau+dt);
    const double w=2.0*PI*f_sig, cr=cos(w*dt), ci=sin(w*dt);
    /* 1 - (1-alpha)*(cr - j*ci) = dr + j*di */
    double dr=1.0-(1.0-alpha)*cr, di=(1.0-alpha)*ci;
    double den=dr*dr+di*di;
    double pr=alpha*dr/den, pi_=-alpha*di/den;      /* H_plant */
    double hr=pr*cr+pi_*ci, hi=pi_*cr-pr*ci;       /* × 一拍延迟 */
    double exp_db=10.0*log10(hr*hr+hi*hi);
    double exp_ph=atan2(hi,hr)*180.0/PI;
    assert(exp_ph<0.0);   /* 期望值本身必须是滞后，否则测试构造有误 */

    /* --- 1. 超出线性区的扫频幅值必须被拒绝（防饱和伪影冒充带宽） ---
       lin_deg = max_rpm/(kp*202/6) = 2500/(158.3*33.6667) = 0.4694° */
    ready_cfg(65536,158.3f,2500.0f,0.0f);
    g_output_position_test.amplitude_deg=2.0f;
    g_output_position_test.feedforward_gain=0.0f;
    g_output_position_test.frequency_hz=20.0f;
    g_output_position_test.command=1;
    assert(OutputPosition_Update(&rpm) && !g_output_position.fault);
    /* Large reference is allowed, but actual speed clipping must never pass
       the linear-bandwidth validity gate. FF=0 must permit +/-2 deg at 20Hz. */
    for(unsigned j=0;j<2200 && g_output_position_test.state==1;j++) {
      test_tick++; g_mpt_t485.last_good_ms=test_tick;
      assert(OutputPosition_Update(&rpm));
      assert(g_output_position.velocity_deg_s==0);
    }
    assert(g_output_position_test.state==2);
    assert(g_output_position_test.saturated_samples>0);
    assert(!g_output_position_sweep_frd.valid && g_output_position_sweep_frd.reject==2);

    /* --- 2. 扫频频率超上限必须被拒绝 --- */
    ready_cfg(65536,158.3f,2500.0f,0.0f);
    g_output_position_test.amplitude_deg=0.25f;
    g_output_position_test.frequency_hz=OUTPUT_POSITION_SWEEP_MAX_FREQUENCY_HZ+1.0f;
    g_output_position_test.command=1;
    assert(!OutputPosition_Update(&rpm) && g_output_position.fault==6);

    /* --- 3. 合法扫频 + 因果一阶滞后被控对象：核对增益与相位符号 --- */
    ready_cfg(65536,158.3f,2500.0f,0.0f);
    g_output_position_test.amplitude_deg=0.25f;
    g_output_position_test.frequency_hz=20.0f;
    g_output_position_test.duration_s=0.0f;         /* 自动：2 s */
    g_output_position_test.trace_interval_ms=10;
    g_output_position_test.command=1;
    double y=0.0, xprev=0.0;
    uint32_t k;
    for(k=0;k<2200;k++) {
      test_tick+=1;
      g_mpt_t485.last_good_ms=test_tick;
      y+=alpha*(xprev-y);                            /* 因果一阶滞后 */
      {
        float actual_deg=180.0f+(float)y;
        uint32_t raw=(uint32_t)(actual_deg*131072.0f/360.0f+0.5f);
        if(raw!=g_mpt_t485.angle_raw) { g_mpt_t485.angle_raw=raw; g_mpt_t485.sequence++; }
      }
      assert(OutputPosition_Update(&rpm));
      xprev=(double)(g_output_position.target_deg-180.0f);
      if(g_output_position_test.state!=1) break;
    }
    assert(g_output_position_test.state==2 && !g_output_position_test.fault);
    assert(g_output_position_test.saturated_samples==0);
    assert(g_output_position_test.peak_motor_rpm>0.0f);
    assert(g_output_position_test.samples>1000);     /* 1 kHz 周期确实生效 */
    /* --- 3b. 扫频结束时固件必须已自动发布结果到全局槽 ---
       这是上位机取得频响的**唯一**途径：调试器只能 savemem/write32，
       无法调用函数。若此处不成立，实测时主机会拿到全零结果却毫不知情。 */
    assert(g_output_position_sweep_frd.valid==1);
    {
      float db,phase,coh,cycles;uint32_t valid;
      g_output_position_test.slew_limited_samples=1;
      assert(OutputPosition_ReadSweepFrd(&db,&phase,&coh,&cycles,&valid));
      assert(!valid && g_output_position_sweep_frd.reject==2);
      g_output_position_test.slew_limited_samples=0;
      assert(OutputPosition_ReadSweepFrd(&db,&phase,&coh,&cycles,&valid) && valid);
    }
    assert(g_output_position_sweep_frd.reject==0);
    assert(fabsf(g_output_position_sweep_frd.frequency_hz-20.0f)<1e-3f);
    assert(fabsf(g_output_position_sweep_frd.amplitude_deg-0.25f)<1e-3f);
    assert(g_output_position_sweep_frd.phase_deg<0.0f);   /* 因果性 */
    assert(fabs((double)g_output_position_sweep_frd.gain_db-exp_db)<0.3);
    assert(fabs((double)g_output_position_sweep_frd.phase_deg-exp_ph)<2.0);
    {
      float gdb,phdeg,coh,cyc; uint32_t valid;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid));
      assert(valid==1);                              /* 数据可采信 */
      assert(coh>=0.9f);
      assert(cyc>=(float)OUTPUT_POSITION_SWEEP_MIN_CYCLES);
      /* 符号约定：相位必须为负（滞后）。首版实现会得到 +50.4°，此断言即锁死它。 */
      assert(phdeg<0.0f);
      assert(fabs((double)phdeg-exp_ph)<2.0);
      assert(fabs((double)gdb-exp_db)<0.3);
      /* 饱和守卫：人为置位后必须判为不可采信（限幅数据是伪影）。 */
      g_output_position_test.saturated_samples=1;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid) && valid==0);
      /* 置位期间全局槽也必须同步为不可采信，且给出原因码 2。
         否则主机会读到「扫频结束时」那份 valid=1 的旧结果而误判。 */
      assert(g_output_position_sweep_frd.valid==0 && g_output_position_sweep_frd.reject==2);
      g_output_position_test.saturated_samples=0;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid) && valid==1);
      /* --- 3c. 关键不变量：函数返回值必须与全局发布逐字段完全一致 ---
         两条路径共用 sweep_frd_compute() 单一实现，此断言锁死该事实。
         若将来有人只在其中一处改了数学（例如再次弄反符号），本断言即失败。 */
      assert(g_output_position_sweep_frd.valid==valid);
      assert(g_output_position_sweep_frd.gain_db==gdb);
      assert(g_output_position_sweep_frd.phase_deg==phdeg);
      assert(g_output_position_sweep_frd.coherence==coh);
      assert(g_output_position_sweep_frd.cycles==cyc);
      assert(g_output_position_sweep_frd.reject==0);
    }

    /* --- 4. 累积周期数不足必须拒绝判定（频谱泄漏未压够） --- */
    ready_cfg(65536,158.3f,2500.0f,0.0f);
    g_output_position_test.amplitude_deg=0.25f;
    g_output_position_test.frequency_hz=20.0f;
    g_output_position_test.duration_s=1.5f; /* Two 0.5s ramps leave 10 cycles. */
    g_output_position_test.trace_interval_ms=10;
    g_output_position_test.command=1;
    y=0.0; xprev=0.0;
    for(k=0;k<1600;k++) {
      test_tick+=1;
      g_mpt_t485.last_good_ms=test_tick;
      y+=alpha*(xprev-y);
      {
        float actual_deg=180.0f+(float)y;
        uint32_t raw=(uint32_t)(actual_deg*131072.0f/360.0f+0.5f);
        if(raw!=g_mpt_t485.angle_raw) { g_mpt_t485.angle_raw=raw; g_mpt_t485.sequence++; }
      }
      assert(OutputPosition_Update(&rpm));
      xprev=(double)(g_output_position.target_deg-180.0f);
      if(g_output_position_test.state!=1) break;
    }
    assert(g_output_position_test.state==2 && !g_output_position_test.fault);
    {
      float gdb,phdeg,coh,cyc; uint32_t valid;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid));
      assert(valid==0);                              /* 周期数不足 → 不可采信 */
      assert(cyc<(float)OUTPUT_POSITION_SWEEP_MIN_CYCLES);
      assert(g_output_position_sweep_frd.reject==4); /* 原因码必须是「周期数不足」 */
    }

    /* --- 5. 因果性门槛：相位算出超前必须拒绝（reject=6） ---
       [千问工作助理 2026-10-28] 本用例由变异测试补入。
       变异 M2（移除 `if(out->phase_deg>0.0f) reject=6`）后原测试仍然 PASS，
       说明该门槛此前**没有任何用例覆盖**——而它正是防止「符号写反」这类
       缺陷流入实测数据的最后一道防线（本轮首版实现就写反过符号）。
       构造：稳态段参考 x = -A·cos(wt) = A∠180°，注入反馈
             y = A·cos(wt+225°) = A∠225°
       则 H = y/x = ∠(225-180) = **+45°（超前）**。因果闭环不可能超前，
       固件必须判为不可采信。纯音注入使相干性≈1，故触发的一定是因果性门槛
       而非相干性门槛（实现中因果性检查在相干性检查之前）。 */
    {
      const double A=0.25, w=2.0*PI*20.0;
      ready_cfg(65536,158.3f,2500.0f,0.0f);
      g_output_position_test.amplitude_deg=0.25f;
      g_output_position_test.frequency_hz=20.0f;
      g_output_position_test.duration_s=0.0f;        /* 自动 2 s */
      g_output_position_test.trace_interval_ms=10;
      g_output_position_test.command=1;
      for(k=0;k<2200;k++) {
        test_tick+=1;
        g_mpt_t485.last_good_ms=test_tick;
        double t=(double)k*0.001;
        double yoff=A*cos(w*t+225.0*PI/180.0);       /* 超前 45° */
        float actual_deg=180.0f+(float)yoff;
        uint32_t raw=(uint32_t)(actual_deg*131072.0f/360.0f+0.5f);
        if(raw!=g_mpt_t485.angle_raw) { g_mpt_t485.angle_raw=raw; g_mpt_t485.sequence++; }
        assert(OutputPosition_Update(&rpm));
        if(g_output_position_test.state!=1) break;
      }
      assert(g_output_position_test.state==2 && !g_output_position_test.fault);
      assert(g_output_position_test.saturated_samples==0);
      float gdb,phdeg,coh,cyc; uint32_t valid;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid));
      assert(valid==1); /* Phase lead alone is not a causality violation. */
      assert(phdeg>0.0f);                            /* 确实算出了超前 */
      assert(coh>=0.9f);                             /* 不是因相干性被拒 */
      assert(cyc>=(float)OUTPUT_POSITION_SWEEP_MIN_CYCLES);  /* 不是因周期数被拒 */
      assert(g_output_position_sweep_frd.reject==0);
    }

    /* --- 6. 相干性门槛：反馈含失配分量时必须拒绝（reject=5） ---
       [千问工作助理 2026-10-28] 本用例由变异测试补入（变异 M3 未被捕获），
       并因此暴露出一个**固件缺陷**：单 bin 相干性在代数上恒等于 1
       （|H|²=Syy/Sxx，代入 gamma²=|H|²·Sxx/Syy ≡ 1），与噪声无关，故原门槛
       是永假死代码——污染数据也会报 valid=1。已改为 Welch 分段估计，
       详见 output_position.h 中 output_position_sweep_frd_t 的修正记录。

       构造（参数由数值仿真选定，见下方"为何这样选"）：
         反馈 = A·cos(phase + 135°)  +  B·cos(2π·23.7·t)
                └─ 滞后 45° 的相关分量   └─ 与参考失配的干扰分量
       - 参考由固件生成：x = -A·cos(phase) = A∠180°，故滞后分量须取 +135°
         才能得到 H 相位为负。**首版误用 -45°，算出 +135°（超前），总是先被
         因果性门槛(reject=6)拦下，相干性门槛永远轮不到执行**——这正是变异
         M3 未被捕获的直接原因。
       - kp 取 10.0（非 158.3）：本用例的跟踪误差可达 ±0.69°，高增益下 P 输出
         会钳位到 max_motor_rpm 而先触发饱和门槛(reject=2)。kp=10 时线性区
         为 ±7.43°，余量充足。
       - 仿真结果：单 bin 相位 -38.52°（滞后，不触发因果性）、饱和 0 次、
         Welch 相干性 0.2519 < 0.9、单 bin 相干性仍为 1.0000（印证退化）。 */
    {
      const double A=0.25, w=2.0*PI*20.0, w2=2.0*PI*23.7, B=0.5;
      ready_cfg(65536,10.0f,2500.0f,0.0f);
      g_output_position_test.amplitude_deg=0.25f;
      g_output_position_test.frequency_hz=20.0f;
      g_output_position_test.duration_s=0.0f;
      g_output_position_test.trace_interval_ms=10;
      g_output_position_test.command=1;
      for(k=0;k<2200;k++) {
        test_tick+=1;
        g_mpt_t485.last_good_ms=test_tick;
        double t=(double)k*0.001;
        double yoff=A*cos(w*t+135.0*PI/180.0)+B*cos(w2*t);
        float actual_deg=180.0f+(float)yoff;
        uint32_t raw=(uint32_t)(actual_deg*131072.0f/360.0f+0.5f);
        if(raw!=g_mpt_t485.angle_raw) { g_mpt_t485.angle_raw=raw; g_mpt_t485.sequence++; }
        assert(OutputPosition_Update(&rpm));
        if(g_output_position_test.state!=1) break;
      }
      assert(g_output_position_test.state==2 && !g_output_position_test.fault);
      assert(g_output_position_test.saturated_samples==0);   /* 未被饱和门槛抢先 */
      float gdb,phdeg,coh,cyc; uint32_t valid;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid));
      assert(phdeg<=0.0f);                                   /* 未被因果性门槛抢先 */
      assert(cyc>=(float)OUTPUT_POSITION_SWEEP_MIN_CYCLES);
      assert(coh<0.9f);                                      /* Welch 相干性确实下降 */
      assert(valid==0);
      assert(g_output_position_sweep_frd.reject==5);         /* 原因码=相干性 */
      assert(g_output_position_sweep_frd.segments>=OUTPUT_POSITION_SWEEP_MIN_SEGMENTS);
      /* 关键留证：单 bin 相干性仍恒等于 1，证明它无法用于判定。
         若将来有人把判定改回单 bin 版本，此断言会立即失败。 */
      assert(fabsf(g_output_position_sweep_frd.coherence_bin-1.0f)<1e-3f);
      (void)gdb;
    }

    /* --- 7. 无任何解调样本时必须拒绝判定（reject=1）---
       [千问工作助理 2026-10-28] 本用例原意是验证「分段不足(reject=8)」，但
       构造后发现了门槛之间的**支配关系**，如实记录如下：
         段长 seg_len = round(1/(freq*period_s)) = round(1/(20*0.001)) = 50 样本
                      = 恰好 1 个信号周期
         cycles   = qd_n * freq * period_s = qd_n/50
         segments ≈ qd_n/50
       故 cycles 与 segments 数值相同，`cycles >= MIN_CYCLES(20)` 必然推出
       `segments >= 20 >= MIN_SEGMENTS(2)`。**reject=8 被 reject=4 严格支配，
       在当前段长选择下无法单独触发**，属防御性门槛（若将来把段长改为多个
       周期，它才会变得可达）。固件侧已加注释说明，避免后人误以为它可测。
       本用例改为覆盖此前**完全未测**的 reject=1 路径：duration 恰好等于
       升余弦起振段长度(1/freq)时，轨迹在解调开始的那一刻就结束，
       qd_n 保持 0，必须判为不可采信。 */
    {
      /* 复用上方定义的一阶滞后仿真（alpha），无需再声明 A/w。 */
      ready_cfg(65536,158.3f,2500.0f,0.0f);
      g_output_position_test.amplitude_deg=0.25f;
      g_output_position_test.frequency_hz=20.0f;
      g_output_position_test.duration_s=0.05f;         /* == 1/freq，解调窗口为空 */
      g_output_position_test.trace_interval_ms=10;
      g_output_position_test.command=1;
      y=0.0; xprev=0.0;
      for(k=0;k<200;k++) {
        test_tick+=1;
        g_mpt_t485.last_good_ms=test_tick;
        y+=alpha*(xprev-y);
        {
          float actual_deg=180.0f+(float)y;
          uint32_t raw=(uint32_t)(actual_deg*131072.0f/360.0f+0.5f);
          if(raw!=g_mpt_t485.angle_raw) { g_mpt_t485.angle_raw=raw; g_mpt_t485.sequence++; }
        }
        assert(OutputPosition_Update(&rpm));
        xprev=(double)(g_output_position.target_deg-180.0f);
        if(g_output_position_test.state!=1) break;
      }
      assert(g_output_position_test.state==2 && !g_output_position_test.fault);
      float gdb,phdeg,coh,cyc; uint32_t valid;
      assert(OutputPosition_ReadSweepFrd(&gdb,&phdeg,&coh,&cyc,&valid));
      assert(valid==0);
      assert(g_output_position_sweep_frd.reject==1);   /* 无样本 */
      assert(g_output_position_sweep_frd.segments==0);
      assert(g_output_position_sweep_frd.cycles==0.0f);
      (void)coh; (void)cyc; (void)phdeg; (void)gdb;
    }
  }

  /* Customer protocol: exact 120 s schedule, all 30 result slots, DC-safe
     amplitude/negative phase, 26..30 Hz slot loss and abort are observable. */
  ready_cfg(65536,0.5f,5500.0f,0.0f);
  g_output_position_customer.test_mode=1;
  g_output_position_test.command=1;
  uint32_t start=test_tick;
  for(uint32_t ms=0;ms<=120000;ms++) {
    test_tick=start+ms; g_mpt_t485.last_good_ms=test_tick;
    uint32_t f=ms/4000+1, within=ms%4000;
    float ref=0;
    if(ms<120000 && within<2000) {
      uint32_t delayed=(within+1000-5)%1000;
      uint32_t k=(delayed*40*f/1000+1)%40;
      ref=0.1f*(float)(k<=20 ? k : 40-k);
    }
    /* Half gain plus DC offset, five millisecond delay. */
    g_mpt_t485.angle_raw=(uint32_t)((180.3f+0.5f*ref)*131072.0f/360.0f+0.5f);
    g_mpt_t485.sequence++;
    assert(OutputPosition_Update(&rpm));
    if(ms<120000) {
      assert(g_output_position_test.state==1);
      assert(g_output_position.target_deg>=180 && g_output_position.target_deg<=182.001f);
      assert(g_output_position.velocity_deg_s==0);
      assert(OutputPosition_RampRpmS()==OUTPUT_POSITION_RAMP_RPM_S);
      if(within>=2000) assert(g_output_position.target_deg==180);
    }
  }
  assert(g_output_position_test.state==2 && g_output_position_customer.completed==30);
  assert(g_output_position_test.trace_count==0);
  for(unsigned i=0;i<30;i++) {
    output_position_customer_result_t r=g_output_position_customer_results[i];
    assert(r.frequency_hz==(float)(i+1) && r.samples==2000);
    assert(fabsf(r.gain_db+6.0206f)<0.08f);
    assert(fabsf(r.phase_deg+1.8f*(float)(i+1))<0.5f);
    assert(!(r.flags & 55U));
    if(i<25) assert(r.flags==0 && r.missed_commands==0);
    else assert(r.flags==8 && r.missed_commands==80*(i+1)-2000);
  }
  ready(65536);g_output_position_customer.test_mode=1;g_output_position_test.command=1;
  assert(OutputPosition_Update(&rpm));
  g_output_position_test.command=2;test_tick++;g_mpt_t485.last_good_ms=test_tick;
  assert(!OutputPosition_Update(&rpm) && !g_speed_run.enable);
  assert(g_output_position_customer_results[0].flags & 16U);
  ready(65536);g_output_position_customer.test_mode=3;g_output_position_test.command=1;
  assert(!OutputPosition_Update(&rpm) && !g_speed_run.enable);

  /* Literal workbook table, 300 frames/s, 15 frequencies, each second
     repeats twice. At 1 ms all 600 frame slots must be delivered per group. */
  ready_cfg(65536,0.5f,5500.0f,0.0f);
  g_output_position_customer.test_mode=2;g_output_position_test.command=1;
  start=test_tick;
  for(uint32_t ms=0;ms<=60000;ms++) {
    test_tick=start+ms;g_mpt_t485.last_good_ms=test_tick;
    uint32_t f=ms/4000U,within=ms%4000U;
    float ref=ms<60000 && within<2000 ?
        .1f*(float)customer_pwm_table[f][(within*300U/1000U)%300U] : 0;
    g_mpt_t485.angle_raw=(uint32_t)((180.3f+.5f*ref)*131072.0f/360.0f+.5f);
    g_mpt_t485.sequence++;
    assert(OutputPosition_Update(&rpm));
    assert(fabsf(g_output_position.target_deg-180-ref)<.0001f);
    if(ms==100) g_output_position_customer.test_mode=0; /* latched mode survives */
  }
  assert(g_output_position_test.state==2 && g_output_position_customer.completed==15);
  for(unsigned i=0;i<15;i++) {
    output_position_customer_result_t r=g_output_position_customer_results[i];
    assert(r.frequency_hz==(float)(i+1) && r.samples==2000);
    assert(r.flags==0 && r.missed_commands==0);
    assert(fabsf(r.gain_db+6.0206f)<.08f && fabsf(r.phase_deg)<.5f);
  }
  assert(g_output_position_customer_results[15].flags==32);
  /* Explicit stop finishes the partial result in foreground with IRQs on. */
  ready(65536);g_output_position_customer.test_mode=2;g_output_position_test.command=1;
  assert(OutputPosition_Update(&rpm));g_output_position.command=2;
  OutputPosition_Service(0);OutputPosition_Service(1);
  assert(!g_speed_run.enable && (g_output_position_customer_results[0].flags & 16));

  puts("PASS: T485 CRC/format/status, 202 ratio, limits, wrap, stale/jump, arming and stop, "
       "sweep/step trajectory, phase lead/lag, coherence/speed/slew/cycles guards, customer Word/XLSX protocols");
}
