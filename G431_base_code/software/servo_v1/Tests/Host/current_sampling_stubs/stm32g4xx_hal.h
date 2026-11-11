#ifndef CURRENT_TEST_HAL_H
#define CURRENT_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t JDR1,JDR2,ISR; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; uint32_t State; } ADC_HandleTypeDef;
typedef struct { uint32_t BDTR; } TIM_TypeDef;
extern ADC_TypeDef test_adc;
extern TIM_TypeDef test_tim;
extern ADC_HandleTypeDef hadc1;
#define ADC1 (&test_adc)
#define TIM1 (&test_tim)
#define TIM_BDTR_MOE (1U<<15)
#define ADC_FLAG_JEOC 1U
#define ADC_FLAG_JEOS 2U
#define HAL_ADC_STATE_INJ_EOC 1U
#define SET_BIT(reg,mask) ((reg)|=(mask))
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }
uint32_t HAL_GetTick(void);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc);
#endif
