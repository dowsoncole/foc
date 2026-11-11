#ifndef SPEED_TEST_STM32_HAL_H
#define SPEED_TEST_STM32_HAL_H
#include <stdint.h>
typedef struct { uint32_t CYCCNT; } speed_test_dwt_t;
extern speed_test_dwt_t speed_test_dwt;
extern uint32_t SystemCoreClock;
#define DWT (&speed_test_dwt)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }
#endif
