#ifndef ENCODER_FRAME_GUARD_H
#define ENCODER_FRAME_GUARD_H
#include <stdint.h>

#define ENCODER_GUARD_COUNTS_PER_REV 2097152UL
#define ENCODER_GUARD_MARGIN_COUNTS 256UL
/* Check the shortest modulo-21-bit step, in either direction. The caller
   derives a rounded-up Q8 counts/cycle rate from the physical limit and clock.
   Long gaps naturally allow any position; this never invents a fresh sample. */
static inline uint8_t Encoder_RawStepPlausible(uint32_t previous, uint32_t next,
                                              uint32_t elapsed_cycles,
                                              uint32_t counts_per_256_cycles)
{
  if (!counts_per_256_cycles || previous >= ENCODER_GUARD_COUNTS_PER_REV ||
      next >= ENCODER_GUARD_COUNTS_PER_REV) return 0U;
  int32_t delta = (int32_t)next - (int32_t)previous;
  if (delta > (int32_t)(ENCODER_GUARD_COUNTS_PER_REV / 2UL))
    delta -= (int32_t)ENCODER_GUARD_COUNTS_PER_REV;
  else if (delta < -(int32_t)(ENCODER_GUARD_COUNTS_PER_REV / 2UL))
    delta += (int32_t)ENCODER_GUARD_COUNTS_PER_REV;
  uint32_t distance = (uint32_t)((delta < 0) ? -delta : delta);
  /* UMULL plus shifts on Cortex-M4; no division or FPU work in the IRQ.
     Keep the 64-bit product so long gaps cannot overflow the rate budget. */
  uint64_t allowed = (((uint64_t)elapsed_cycles * counts_per_256_cycles) >> 8) +
                     ENCODER_GUARD_MARGIN_COUNTS;
  return ((uint64_t)distance <= allowed) ? 1U : 0U;
}
#endif
