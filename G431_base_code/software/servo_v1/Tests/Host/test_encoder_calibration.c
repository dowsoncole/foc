#include "encoder_calibration.h"
#include "encoder_calibration_store.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now, starts, stops, writes, applies, fault;
static uint8_t save_ok, apply_ok, load_ok;
static float stored, applied;
static uint8_t start(void) { starts++; return 1; }
static void stop(void) { stops++; }
static uint32_t clock_ms(void) { return now; }
static uint32_t backend_fault(void) { return fault; }
static uint8_t load(int32_t d,uint32_t p,float *x) {
  assert(d==-1 && p==2); *x=stored; return load_ok;
}
static uint8_t save(int32_t d,uint32_t p,float x,const encoder_calibration_config_t *c) {
  assert(d==-1 && p==2 && c->d_voltage_v==1.0f && c->duration_s==.1f);
  writes++; if(save_ok==1) stored=x; return save_ok;
}
static uint8_t apply(int32_t d,uint32_t p,float x) {
  assert(d==-1 && p==2); applies++; applied=x; return apply_ok;
}
static const encoder_calibration_port_t port={start,stop,backend_fault,clock_ms,load,save,apply};
static void reset(void) {
  now=starts=stops=writes=applies=fault=0;
  save_ok=apply_ok=1; load_ok=0; stored=0;
  EncoderCalibration_Init(&port,-1,2);
}
static void arm(void) {
  g_encoder_calibration.requested=(encoder_calibration_config_t){1.0f,.1f,0};
  g_encoder_calibration.command=1;
  EncoderCalibration_Service(1);
  assert(starts==1 && g_encoder_calibration.state==ENCODER_CAL_QUEUED);
  /* Mutating user recipe after submission cannot alter this alignment. */
  g_encoder_calibration.requested=(encoder_calibration_config_t){2.0f,7.0f,0};
  assert(EncoderCalibration_Begin(-1,2));
  assert(g_encoder_calibration.active.d_voltage_v==1.0f &&
         g_encoder_calibration.active.duration_s==.1f);
}
int main(void) {
  reset(); assert(g_encoder_calibration.requested.d_voltage_v==1.5f &&
                  g_encoder_calibration.requested.duration_s==3.0f);
  assert(!EncoderCalibration_Load(1));
  arm(); assert(EncoderCalibration_Capture(100000U));
  EncoderCalibration_Service(0); assert(!writes && !applies);
  EncoderCalibration_Service(1);
  assert(writes==1 && applies==1 && g_encoder_calibration.valid &&
         g_encoder_calibration.state==ENCODER_CAL_DONE);
  assert(fabsf(applied-200000.0f*(6.28318530718f/2097152.0f))<1e-5f);
  EncoderCalibration_Service(1); assert(writes==1);
  float previous=stored;
  EncoderCalibration_Init(&port,-1,2); load_ok=1;
  assert(EncoderCalibration_Load(1) && g_encoder_calibration.loaded_from_storage);
  assert(g_encoder_calibration.offset_rad==previous && applied==previous);
  reset(); arm(); assert(EncoderCalibration_Capture(100000U)); save_ok=0;
  EncoderCalibration_Service(1);
  assert(g_encoder_calibration.fault==ENCODER_CAL_SAVE_FAILED && !applies);
  reset(); arm(); assert(EncoderCalibration_Capture(100000U)); save_ok=2;
  EncoderCalibration_Service(1);
  assert(g_encoder_calibration.state==ENCODER_CAL_CAPTURED && !applies);
  save_ok=1; EncoderCalibration_Service(1);
  assert(g_encoder_calibration.state==ENCODER_CAL_DONE && applies==1);
  reset(); arm(); g_encoder_calibration.command=2; EncoderCalibration_Service(0);
  assert(stops==1 && !writes && !EncoderCalibration_Capture(100000U));
  reset(); arm(); fault=1; EncoderCalibration_Service(0);
  assert(g_encoder_calibration.fault==ENCODER_CAL_BACKEND_FAULT && !writes);
  reset(); g_encoder_calibration.command=1; EncoderCalibration_Service(1);
  now=201; EncoderCalibration_Service(1);
  assert(g_encoder_calibration.fault==ENCODER_CAL_START_TIMEOUT);
  reset(); g_encoder_calibration.requested.d_voltage_v=NAN;
  g_encoder_calibration.command=1; EncoderCalibration_Service(1); assert(!starts);
  reset(); g_encoder_calibration.command=1; EncoderCalibration_Service(0);
  assert(!starts && !writes);
  reset();
  g_encoder_calibration.requested=(encoder_calibration_config_t){1.0f,.1f,1.0f};
  g_encoder_calibration.command=1; EncoderCalibration_Service(1);
  assert(EncoderCalibration_Begin(-1,2));
  assert(EncoderCalibration_Capture(100000U));
  EncoderCalibration_Service(1);
  assert(g_encoder_calibration.state==ENCODER_CAL_DONE && writes==1 && applies==1);
  assert(fabsf(applied-(200000.0f*(6.28318530718f/2097152.0f)+.7853981634f))<1e-5f);
  reset(); g_encoder_calibration.requested.q_voltage_v=-1.5f;
  g_encoder_calibration.requested.d_voltage_v=0;
  assert(EncoderCalibration_Begin(-1,2) && EncoderCalibration_Capture(0));
  assert(fabsf(g_encoder_calibration.candidate_offset_rad-4.7123889804f)<1e-5f);
  encoder_calibration_record_t r={0};
  r.magic=ENCODER_CAL_STORE_MAGIC; r.version=1; r.sequence=1;
  r.direction=-1; r.pole_pairs=2; r.offset_rad=.25f;
  r.d_voltage_v=1.5f; r.duration_s=3;
  r.checksum=EncoderCalibrationStore_Checksum(&r);
  assert(!EncoderCalibrationStore_Valid(&r)); /* Torn write before commit. */
  r.commit=ENCODER_CAL_STORE_COMMIT; assert(EncoderCalibrationStore_Valid(&r));
  for(unsigned byte=0;byte<sizeof(r);byte++) {
    encoder_calibration_record_t bad=r;
    ((unsigned char *)&bad)[byte]^=1;
    assert(!EncoderCalibrationStore_Valid(&bad));
  }
  puts("PASS configurable recipe, immutable latch, OFF-only save/apply, reboot load, cancellation/fault/timeout and CRC/torn record");
}
