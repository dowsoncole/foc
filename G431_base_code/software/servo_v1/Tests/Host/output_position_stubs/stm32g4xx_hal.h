#ifndef OUTPUT_TEST_HAL_H
#define OUTPUT_TEST_HAL_H
#include <stdint.h>
extern uint32_t test_tick;
static inline uint32_t HAL_GetTick(void) { return test_tick; }
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
#endif
