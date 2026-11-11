#ifndef SPEED_OVERLOAD_GUARD_H
#define SPEED_OVERLOAD_GUARD_H
#include <stdint.h>
#include <math.h>

/* User-selected sampled-RMS I2t model. Each ~1 ms block integrates the
   three-phase mean square before subtracting rated current squared.
   No cooling is assumed: below-rated blocks retain previous heat debt.
   Q16 A^2 * CPU cycles avoids lost small increments in a float accumulator.
   The 18 A / 5 s point defines energy, NOT an instantaneous trip threshold. */
#define SPEED_OVERLOAD_RATED_RMS_A 6UL
#define SPEED_OVERLOAD_REFERENCE_RMS_A 18UL
#define SPEED_OVERLOAD_REFERENCE_SECONDS 5U
#define SPEED_OVERLOAD_RATED_I2 (SPEED_OVERLOAD_RATED_RMS_A * SPEED_OVERLOAD_RATED_RMS_A)
#define SPEED_OVERLOAD_BUDGET_A2S ((SPEED_OVERLOAD_REFERENCE_RMS_A * \
    SPEED_OVERLOAD_REFERENCE_RMS_A - SPEED_OVERLOAD_RATED_I2) * SPEED_OVERLOAD_REFERENCE_SECONDS)
#define SPEED_OVERLOAD_RATED_I2_Q16 (SPEED_OVERLOAD_RATED_I2 * 65536UL)
typedef struct {
  uint32_t used, armed, started, expired, invalid;
  uint32_t has_sample, last_cycle, block_cycles, clock_hz;
  uint64_t block_i2_cycles_q16, excess_i2_cycles_q16, elapsed_cycles;
  uint64_t last_block_i2_cycles_q16;
  uint32_t last_block_cycles;
} speed_overload_guard_state_t;

static inline uint8_t SpeedOverload_ArmWithDebt(volatile speed_overload_guard_state_t *s,
    uint32_t request, uint32_t host_timeout, uint32_t clock_hz, float initial_a2s) {
  if(request>1U || (request && (!host_timeout || s->used || clock_hz<1000000U ||
      clock_hz>200000000U))) return 0U;
  uint32_t initial_q10=0U;
  if(request) {
    if(!isfinite(initial_a2s) || initial_a2s<0.0f ||
       initial_a2s>=(float)SPEED_OVERLOAD_BUDGET_A2S) return 0U;
    float scaled=initial_a2s*1024.0f;
    initial_q10=(uint32_t)scaled;
    if((float)initial_q10<scaled) initial_q10++; /* Round heat UP, never down. */
    if(initial_q10>=SPEED_OVERLOAD_BUDGET_A2S*1024U) return 0U;
  }
  uint64_t debt=s->excess_i2_cycles_q16;
  if(request) {
    uint64_t initial=(uint64_t)initial_q10*clock_hz*64U;
    if(initial>debt) debt=initial;
  }
  s->armed=request; s->started=s->expired=s->invalid=0U;
  s->has_sample=s->last_cycle=s->block_cycles=0U;
  s->block_i2_cycles_q16=s->elapsed_cycles=0U;
  s->excess_i2_cycles_q16=debt;
  s->last_block_i2_cycles_q16=0U; s->last_block_cycles=0U; s->clock_hz=clock_hz;
  if(request) s->used=1U;
  return 1U;
}
static inline uint8_t SpeedOverload_Arm(volatile speed_overload_guard_state_t *s,
    uint32_t request, uint32_t host_timeout, uint32_t clock_hz) {
  return SpeedOverload_ArmWithDebt(s,request,host_timeout,clock_hz,0.0f);
}
static inline uint8_t SpeedOverload_Check(volatile speed_overload_guard_state_t *s, uint32_t now) {
  if(!s->armed) return 1U;
  /* Missing feedback cannot freeze a thermal guard while leaving PWM on. */
  if(s->has_sample && (now-s->last_cycle)>s->clock_hz/2000U) s->invalid=1U;
  return !(s->expired || s->invalid);
}
static inline uint8_t SpeedOverload_Observe(volatile speed_overload_guard_state_t *s,
    uint32_t now, float mean_square_a2) {
  if(!s->armed) return 1U;
  if(!isfinite(mean_square_a2) || mean_square_a2<0.0f || mean_square_a2>65535.0f)
    s->invalid=1U;
  if(!SpeedOverload_Check(s,now)) return 0U;
  if(!s->has_sample) {
    s->has_sample=1U; s->last_cycle=now; return 1U;
  }
  uint32_t dt=now-s->last_cycle;
  if(!dt) { s->invalid=1U; return 0U; }
  s->last_cycle=now; s->elapsed_cycles+=dt; s->block_cycles+=dt;
  uint32_t square_q16=(uint32_t)(mean_square_a2*65536.0f);
  s->block_i2_cycles_q16+=(uint64_t)square_q16*dt;
  if(s->block_cycles>=s->clock_hz/1000U) {
    /* Publish integer block totals; float conversion/division belongs in
       background telemetry, not the coincident 1 kHz speed/current IRQ. */
    s->last_block_i2_cycles_q16=s->block_i2_cycles_q16;
    s->last_block_cycles=s->block_cycles;
    uint64_t rated=(uint64_t)SPEED_OVERLOAD_RATED_I2_Q16*s->block_cycles;
    if(s->block_i2_cycles_q16>rated) {
      s->excess_i2_cycles_q16+=s->block_i2_cycles_q16-rated;
      s->started=1U;
    }
    if(s->excess_i2_cycles_q16>=(uint64_t)SPEED_OVERLOAD_BUDGET_A2S*s->clock_hz*65536U)
      s->expired=1U;
    s->block_cycles=0U; s->block_i2_cycles_q16=0U;
  }
  return !s->expired;
}
/* Call only on a coherent background snapshot, never from the current ISR. */
static inline float SpeedOverload_MeanSquare(const speed_overload_guard_state_t *s) {
  return s->last_block_cycles ? (float)s->last_block_i2_cycles_q16 /
      ((float)s->last_block_cycles*65536.0f) : 0.0f;
}
#endif
