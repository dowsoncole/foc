#ifndef PWM_COMMIT_DIAGNOSTICS_H
#define PWM_COMMIT_DIAGNOSTICS_H

#include <stdint.h>

#define PWM_COMMIT_FAULT_UDIS         1U
#define PWM_COMMIT_FAULT_BEFORE_GUARD 2U
#define PWM_COMMIT_FAULT_WRAPPED      3U
#define PWM_COMMIT_FAULT_AFTER_GUARD  4U
#define PWM_COMMIT_POSITION_UNREAD   UINT32_MAX

/* First ADC-context deadline rejection only. Non-halting host readers accept
   count==1; cleanup/foreground Stop writes must never overwrite this event.
   CCR reads are sequential peripheral observations, not a waveform capture. */
typedef struct {
  uint32_t version, count, reason, cycle, adc_entry_cycle, write_entry_cycle;
  uint32_t exception, position_before, position_after, guard_ticks;
  uint32_t cr1, cnt, arr, bdtr;
  uint32_t observed_ccr[4], requested_ccr[4];
} pwm_commit_fault_t;

/* Successful ADC commits only; foreground Stop must not pollute extrema.
   Observed at the existing post-write timer read, not a new deadline policy. */
typedef struct {
  uint32_t count, minimum_remaining_ticks, maximum_adc_to_commit_cycles;
  uint32_t last_remaining_ticks, last_adc_to_commit_cycles;
} pwm_commit_timing_t;

static inline void PwmCommit_RecordSuccess(volatile pwm_commit_timing_t *timing,
                                           uint32_t remaining, uint32_t cycles)
{
  if (timing == 0) return;
  if (timing->count == 0U || remaining < timing->minimum_remaining_ticks)
    timing->minimum_remaining_ticks = remaining;
  if (cycles > timing->maximum_adc_to_commit_cycles)
    timing->maximum_adc_to_commit_cycles = cycles;
  timing->last_remaining_ticks = remaining;
  timing->last_adc_to_commit_cycles = cycles;
  if (timing->count != UINT32_MAX) timing->count++;
}

/* Caller serializes writers. Publish count last; no clear/rearm operation. */
static inline void PwmCommit_FreezeFirst(volatile pwm_commit_fault_t *capture,
                                       const pwm_commit_fault_t *event)
{
  if (capture == 0 || event == 0 || capture->count != 0U) return;
  pwm_commit_fault_t value = *event;
  value.version = 1U;
  value.count = 0U;
  *capture = value;
  capture->count = 1U;
}

#endif
