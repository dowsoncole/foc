#include "current_sampling.h"
#include "stm32g4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
ADC_TypeDef test_adc;
TIM_TypeDef test_tim;
ADC_HandleTypeDef hadc1={&test_adc,0};
static uint32_t now,callbacks;
uint32_t HAL_GetTick(void) { return now; }
static void callback(void *ctx) { (void)ctx; callbacks++; }
static void reset(void) {
  memset(&g_current_sampling,0,sizeof(g_current_sampling));
  g_current_sampling.sample_rate_hz=20000;
  g_current_sampling.offset_u=123; g_current_sampling.offset_v=456;
  now=callbacks=test_tim.BDTR=0;
  CurrentSampling_SetConversionCallback(callback,0);
}
static void sample(uint16_t u,uint16_t v) {
  test_adc.JDR1=u; test_adc.JDR2=v;
  HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
}
int main(void) {
  reset(); CurrentSampling_RequestCalibration();
  assert(g_current_sampling.calibration_target_samples==20);
  for(unsigned i=0;i<19;i++) sample(1892+(i%2),1890+(i%2));
  CurrentSampling_Service();
  assert(!callbacks && g_current_sampling.offset_u==123 &&
      CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_COLLECTING);
  sample(1893,1891); now=1; CurrentSampling_Service();
  assert(CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_DONE &&
      g_current_sampling.offset_u==1892 && g_current_sampling.offset_v==1890);
  sample(1892,1890); assert(callbacks==1);
  reset(); CurrentSampling_RequestCalibration(); sample(1892,1890); now=5;
  CurrentSampling_Service(); assert(CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_ERROR &&
      g_current_sampling.offset_u==123);
  reset(); CurrentSampling_RequestCalibration();
  for(unsigned i=0;i<20;i++) sample(0,4095);
  CurrentSampling_Service(); assert(CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_ERROR);
  reset(); CurrentSampling_RequestCalibration(); test_tim.BDTR=TIM_BDTR_MOE;
  CurrentSampling_Service(); assert(CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_ERROR);
  reset(); now=0xFFFFFFFEU; CurrentSampling_RequestCalibration(); now=4;
  CurrentSampling_Service(); assert(CurrentSampling_GetCalibrationState()==CURRENT_SAMPLING_CAL_ERROR);
  puts("PASS 20-sample/1-ms fresh boot offsets, no partial commit, callback gate, timeout/wrap, rails and MOE interlock");
}
