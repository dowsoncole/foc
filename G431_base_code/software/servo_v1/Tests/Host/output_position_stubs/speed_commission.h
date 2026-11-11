#ifndef OUTPUT_TEST_SPEED_H
#define OUTPUT_TEST_SPEED_H
#include <stdint.h>
typedef struct { uint32_t enable, fault; float target_rpm, actual_rpm; } speed_run_t;
typedef struct { uint32_t calibration_ok,state,command,fault; } speed_commission_t;
extern speed_run_t g_speed_run;
extern speed_commission_t g_speed_test;
#endif
