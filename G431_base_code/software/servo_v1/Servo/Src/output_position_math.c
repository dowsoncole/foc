#include "output_position.h"
#include <math.h>
/* ==========================================================================
   [千问工作助理 2026-10-28] 位置环数学层：解除增益硬上限 + 死区连续化
   ------------------------------------------------------------------------
   改动 1：kp 上限 10 → OUTPUT_POSITION_MAX_KP_PER_S(200)
   原因：原 `kp>10` 使纯 P 环交越频率被锁死在 10/(2π)=1.59 Hz，
   与 20 Hz 目标差 12.6 倍。这是本轮识别的首要致命瓶颈（缺陷 D1）。
   依据见 output_position.h 顶部注释的完整推导。

   刻意未改动：死区语义保持「|error|<=deadband 时 P 输出为 0」。
   原实现使环增益在死区边界从 kp 突变为 0（本轮记为缺陷 D6）。曾考虑
   改为线性过渡，但撤回，理由：
   a) Tests/Host/test_output_position.c:49 有一条断言
      `OutputPosition_Compute(10,10.005f,1,100,.01f,1,&rpm) && rpm==0`
      显式编码了该行为，且「死区内位置 P 项为 0」是已对外文档化的语义
      （2026-10-26_MPT输出轴位置闭环接入与实机验收.md:96）。
      静默改变文档化行为属越界，且会使既有验收记录失效。
   b) 该非线性与 20 Hz 带宽目标无关：扫频时跟踪误差远大于 0.01° 死区，
      死区在扫频工况下基本不激活。
   ⚠️ 但需注意：kp 提高 79 倍会同比放大死区边界的输出跳变
   （deadband=0.01° 时，跨越边界的输出阶跃由 0.67 rpm 放大到 53 rpm），
   有激励极限环的风险。正确处置是**配置**而非改码：高增益扫频与高增益
   定位时应将 g_output_position.deadband_deg 设为 0（纯比例，天然无跳变；
   参数检查已允许 0）。详见调试日志第 8.4 节。

   其余未改动：202/6 减速比换算、对称限幅、direction 乘法、参数有效性
   检查的其余各项（max_rpm、deadband 范围、direction 取值）全部保持原样。
   ========================================================================== */
uint8_t OutputPosition_Compute(float target,float actual,float kp,
                              float max_rpm,float deadband,int32_t direction,float *rpm)
{
  if(!rpm || !isfinite(target) || !isfinite(actual) || !isfinite(kp) ||
     !isfinite(max_rpm) || !isfinite(deadband) ||
     kp<=0 || kp>OUTPUT_POSITION_MAX_KP_PER_S ||
     max_rpm<=0 || max_rpm>OUTPUT_POSITION_MAX_MOTOR_RPM ||
     deadband<0 || deadband>1 || (direction!=1 && direction!=-1)) return 0;
  float error=target-actual;
  if(!isfinite(error)) return 0;
  /* omega_output[deg/s] * gear_ratio / 6 = motor RPM. */
  float result=fabsf(error)<=deadband ? 0 : error*kp*(202.0f/6.0f);
  if(result>max_rpm) result=max_rpm;
  if(result<-max_rpm) result=-max_rpm;
  *rpm=result*(float)direction;
  return 1;
}
