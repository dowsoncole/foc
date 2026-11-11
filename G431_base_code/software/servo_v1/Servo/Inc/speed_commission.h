#ifndef SPEED_COMMISSION_H
#define SPEED_COMMISSION_H
#include "foc_interface.h"
#include <float.h>
#include "mpt_t485.h"
#include "../../Tests/Inc/speed_response_test.h"
/* CURRENT_LIMIT_HARDWARE_ONLY=1 (current_policy.h) overrides legacy limits
   below for every mode: no phase-current trip, Iq clamp, d/q vector clamp or
   I2t enforcement. Legacy constants remain for explicit policy=0 host tests. */
/* Bounded tests plus explicitly enabled continuous speed mode.
   Boot seeds board direction/pole pairs, performs fixed 1.5 V / 3 s voltage
   zeroing, commits the new offset while OFF, then makes one speed-start
   request after fresh feedback. Failures never retry or use the old zero. */
/* User-fixed default: changes require explicit approval. */
/* Default: align once, then hold current output position (speed-only builds
   retain continuous speed start). A stop/fault never automatically rearms. */
#define SPEED_RUN_BOOT_AUTO_START 1U
#define SPEED_RUN_BOOT_READY_TIMEOUT_MS 200UL
#define SPEED_CONTINUOUS_KIND 7U
#define SPEED_ALIGNMENT_SWEEP_KIND 9U
#define SPEED_VOLTAGE_ALIGNMENT_KIND 10U
#define SPEED_VOLTAGE_ALIGNMENT_PROFILE_VERSION 2U
#define SPEED_VOLTAGE_ALIGNMENT_MAX_V 1.5f /* Legacy default; business module latches the user recipe. */
#define SPEED_VOLTAGE_ALIGNMENT_DURATION_S 3.0f
#define SPEED_VOLTAGE_ALIGNMENT_LIMIT_PU 0.10f /* Fits 1.5 V at 18..28 V nominal bus. */
/* Voltage alignment uses user-provided hardware current limiting, not a
   software current-amplitude trip. Nonfinite samples still stop output. */
extern volatile const uint32_t g_speed_voltage_alignment_profile_version;
/* ISR-owned command, not a host tuning interface. Main supplies nominal Vdc;
   Reference overwrites Ud/Uq every cycle, including readiness and completion. */
extern foc_voltage_command_t g_speed_voltage_alignment;
#define SPEED_ALIGNMENT_SWEEP_DURATION_S 12.0f
#define SPEED_ALIGNMENT_SWEEP_DISTANCE_RAD 12.5663706144f /* Two pole pairs: full mechanical turn. */
/* Separate, bounded SYNTHETIC-angle/current-loop diagnostic. Never consumes
   encoder angle/speed, and never changes normal speed-mode fault handling. */
#define ENCODER_DEBUG_OPEN_LOOP_KIND 8U
#define ENCODER_DEBUG_START_COMMAND 4U
#define ENCODER_DEBUG_MAX_DURATION_S 5.0f
#define ENCODER_DEBUG_MAX_CURRENT_A 0.30f
#define ENCODER_DEBUG_MAX_ELECTRICAL_HZ 1.0f
#define ENCODER_DEBUG_PHASE_TRIP_A 0.80f
#define ENCODER_DEBUG_VOLTAGE_LIMIT_PU 0.01f
/* Experimental command ceiling, NOT a validated operating rating. User
   confirmed gearbox input limit 10000 rpm; motor rated point is 8534 rpm.
   Automated trials retain the latched host deadman without adding a phase
   amplitude trip; explicitly requested I2t trials retain their heat budget. */
#define SPEED_RUN_MAX_RPM 9000.0f
#define SPEED_RUN_OVERSPEED_RAD_S 950.0f
#define SPEED_RUN_PHASE_TRIP_A 15.0f /* Alignment trip; continuous trip uses enable below. */
/* User-requested bypass of ONLY the sampled phase-amplitude software trip in
   active continuous speed mode. NaN/Inf, bounded tests and driver faults stay
   protected. Hardware OCP remains active; continuous Iq policy is below. */
#define SPEED_RUN_PHASE_TRIP_ENABLED 0U
/* User-selected hardware current limiting. FLT_MAX is the finite numerical
   sentinel: no finite continuous Iq command is clipped by the current loops.
   Optional explicitly armed I2t trials retain their separate configuration. */
#define SPEED_RUN_IQ_LIMIT_A FLT_MAX
/* Candidate test envelopes, NOT established power-stage or motor ratings.
   I2t's Imax determines heat budget, never a 6/18 A instantaneous clamp. */
#define SPEED_RUN_OVERLOAD_IQ_LIMIT_A 25.4558441f /* Separate command envelope: sqrt(2)*18 A RMS. */
/* Amplitude-invariant Clarke/Park: equivalent three-phase RMS = |Id|/sqrt(2)
   when Iq=0. User confirmed 5 A RMS alignment; this is NOT Id=5 A.
   The existing kind-9 sweep and legacy <=3 A commands retain their old ramp. */
#define SPEED_ALIGNMENT_PROFILE_VERSION 2U
#define SPEED_ALIGNMENT_RMS_LIMIT_A 5.0f
#define SPEED_ALIGNMENT_CURRENT_LIMIT_A 7.071067812f
#define SPEED_ALIGNMENT_CURRENT_RAMP_A_S 3.535533906f /* Reach 5 A RMS in 2 s. */
#define SPEED_ALIGNMENT_LEGACY_CURRENT_LIMIT_A 3.0f
#define SPEED_ALIGNMENT_LEGACY_RAMP_A_S 1.5f
extern volatile const uint32_t g_speed_alignment_profile_version;
#define SPEED_RUN_RAMP_RPM_S 300.0f
#define SPEED_RUN_START_DIRECTION_CHECK_MS 2000UL
#define SPEED_RUN_START_DIRECTION_COMMAND_RPM 50.0f
#define SPEED_RUN_START_DIRECTION_FEEDBACK_RPM 100.0f
/* ADC profile 1 allows ~0.528 pu linear / ~0.583 pu six-step fundamental. Modulation also
   clamps to the runtime sampling-derived duty window; never widen that window.
   The former 0.30 pu commissioning cap limited this assembly to ~4570 rpm. */
#define SPEED_RUN_VOLTAGE_LIMIT_PU 0.56f /* Candidate; sampling and overspeed guards remain enabled. */

//#define SPEED_RUN_MODULATION_MODE FOC_MODULATION_OVERMODULATION
#define SPEED_RUN_MODULATION_MODE FOC_MODULATION_LINEAR_ONLY


#define SPEED_RPM_TO_RAD_S 0.1047197551f
/* Boot seed only: zero offset is NOT a valid calibration. Load a stored zero
   or complete voltage alignment before speed operation.
   This does not write the MT6835 ZERO_POS registers. */
#define SPEED_BOARD_ENCODER_DIRECTION (-1)
#define SPEED_BOARD_ENCODER_POLE_PAIRS 2U
#define SPEED_BOARD_ENCODER_OFFSET_RAD 0.0f
#define SPEED_START_OUTPUT_ACTIVE      (1UL << 0)
#define SPEED_START_RUNTIME_FAULT      (1UL << 1)
#define SPEED_START_CONTROLLER_FAULT   (1UL << 2)
#define SPEED_START_POWER_UNSAFE       (1UL << 3)
#define SPEED_START_ADC_NOT_READY      (1UL << 4)
#define SPEED_START_MODE_INVALID       (1UL << 5)
#define SPEED_START_STAGE_INVALID      (1UL << 6)
#define SPEED_START_TEST_BUSY          (1UL << 7)
#define SPEED_START_COMMAND_PENDING    (1UL << 8)
#define SPEED_START_CALIBRATION_MISSING (1UL << 9)
#define SPEED_START_ENCODER_INVALID    (1UL << 10)
#define SPEED_START_ENCODER_TOLERANCE_INVALID (1UL << 11)
typedef struct {
  volatile uint32_t enable; /* 0: remove output; 1: request a fresh start while off */
  volatile float target_rpm; /* Live command; negative reverses direction. */
  volatile float applied_rpm, actual_rpm, turns, elapsed_s;
  volatile float iq_reference_a, iq_a, id_a, uq_pu;
  volatile uint32_t state, fault;
  volatile float phase_peak_a; /* Largest absolute sampled phase current since start. */
  volatile uint32_t feedback_status_raw; /* Last encoder status, frozen on speed fault. */
  volatile uint32_t start_block_mask; /* Why fault 0x0A refused the last start. */
  /* Maximum age of the LAST valid feedback, in carrier periods, not a count
     of bad frames. 1..10; latched by the application at an OFF start request.
     Live writes do not change the active limit. Appended to preserve offsets. */
  volatile uint32_t encoder_tolerance_cycles;
} speed_run_t;
extern speed_run_t g_speed_run;
/* Optional host test deadman, configured only while OFF. timeout_ms=0 keeps
   ordinary operator-controlled continuous operation unchanged. A nonzero
   100..10000 ms timeout is latched at arm and cannot be disabled in motion.
   Host increments heartbeat before expiry; fault 13 latches on expiry.
   Both ADC and background observe it; no automatic retry or encoder bypass. */
typedef struct {
  volatile uint32_t timeout_ms, heartbeat, active_timeout_ms, expired;
} speed_run_guard_t;
extern speed_run_guard_t g_speed_run_guard;
/* request=1 arms the sampled-RMS I2t model only while OFF with host deadman.
   initial_energy_a2s carries recorded heat into a new diagnostic image (OFF
   only, rounded upward and latched at arm). Other fields are read-only. One trial
   per boot; no automatic rearm/cooling/reset of accumulated thermal debt. */
typedef struct {
  volatile uint32_t request, armed, started, elapsed_ms, expired, used;
  volatile float energy_a2s, budget_a2s, current_rms_a;
  volatile uint32_t invalid;
  volatile float initial_energy_a2s;
} speed_run_overload_t;
extern speed_run_overload_t g_speed_run_overload;
extern volatile uint32_t g_encoder_debug_start_block_mask;
typedef struct {
  uint32_t kind; /* 1 current align,2 Iq,3 step,4 sine,5/6 PRBS,7 continuous,8 debug,9 sweep,10 voltage align */
  /* Kind 10 amplitude is positive physical Ud volts, bias is start field;
     current_limit_a retains the legacy 15 A config value, NOT a reference
     or an active phase-amplitude trip in voltage alignment. */
  float duration_s, amplitude, bias, frequency_hz, angle_end_rad;
  float kp, ki, current_limit_a, speed_limit_rad_s;
  uint32_t record_divider; /* outer ticks (1 ms) per record */
  uint32_t prbs_bit_ms;
} speed_test_config_t;
typedef struct {
  /* Kind 10 reuses iq_reference/voltage_q for applied Ud/Uq in volts.
     Other kinds retain the original A / pu units. ABI remains 32 bytes. */
  float time_s, reference, speed, id, iq, iq_reference, angle, voltage_q;
} speed_test_record_t;
typedef struct {
  /* command: 1 arm requested, 2 abort, 3 idle (OFF only), 4 preset debug start.
     state: 0 idle,1 armed,2 run,3 done,4 fault. */
  volatile uint32_t command, state, fault, count;
  speed_test_config_t requested, active;
  volatile uint32_t calibration_command, calibration_ok;
  int32_t direction;
  uint32_t pole_pairs;
  float electrical_offset_rad;
  float elapsed, integral, iq_reference, reference, angle;
  uint32_t first_cycle, last_cycle, tick_cycle, ticks, record_ticks;
  uint32_t max_interval_cycles;
  uint32_t feedback_fault, feedback_age_cycles;
  uint32_t ready_cycle, ready_count;
  uint8_t prbs_sequence[127]; /* MATLAB binary PRBS, uploaded only while off. */
  union {
    speed_test_record_t records[128];
    struct { float time_s, iq_command_a, speed_rad_s; } prbs_records[336];
  }; /* Reuse recorder RAM; modes are mutually exclusive. */
} speed_commission_t;
extern speed_commission_t g_speed_test;
void SpeedCommission_Service(uint8_t output_off);
/* Consume only the command observed by the application startup checks. */
void SpeedCommission_ServiceCommand(uint8_t output_off, uint32_t accepted_command);
void SpeedCommission_PublishRunTelemetry(void);
/* Startup only, after Encoder_Init, with power output OFF. No motor command
   or fault reset is issued; configuration failure leaves calibration invalid. */
uint8_t SpeedCommission_LoadBoardCalibration(void);
uint8_t SpeedCommission_PhaseCurrentFault(float u_a, float v_a, float w_a,
                                         float threshold_a);
uint8_t SpeedCommission_Reference(foc_input_t *input);
/* ADC: live angle/protection plus held Iq. PendSV: 1 kHz outer calculation.
   Returns pending status; the caller pends the low-priority exception. */
uint8_t SpeedCommission_FastReference(foc_input_t *input);
uint8_t SpeedCommission_OuterPending(void);
void SpeedCommission_OuterService(void);
#endif
