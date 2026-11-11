#ifndef CURRENT_POLICY_H
#define CURRENT_POLICY_H
/* User-selected policy: the external hardware owns current limiting.
   Disable ALL software current amplitude clamps/trips and I2t enforcement.
   Nonfinite-data, voltage/PWM, encoder, speed and travel checks remain.
   Override to 0 only in legacy regression tests, never in this board build. */
#ifndef CURRENT_LIMIT_HARDWARE_ONLY
#define CURRENT_LIMIT_HARDWARE_ONLY 1
#endif
#endif
