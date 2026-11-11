#include "encoder_calibration_store.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

typedef char record_size_must_be_48[(sizeof(encoder_calibration_record_t) == 48) ? 1 : -1];
typedef char commit_must_be_last[(offsetof(encoder_calibration_record_t, commit) == 40) ? 1 : -1];
static uint8_t wait_flash(void) {
  uint32_t start = DWT->CYCCNT;
  while (FLASH->SR & FLASH_SR_BSY)
    if ((uint32_t)(DWT->CYCCNT - start) > SystemCoreClock / 10U) return 0;
  return (FLASH->SR & FLASH_FLAG_SR_ERRORS) == 0U;
}
static const encoder_calibration_record_t *newest(void) {
  const encoder_calibration_record_t *a = (const void *)ENCODER_CAL_STORE_PAGE0;
  const encoder_calibration_record_t *b = (const void *)ENCODER_CAL_STORE_PAGE1;
  uint8_t va = EncoderCalibrationStore_Valid(a), vb = EncoderCalibrationStore_Valid(b);
  if (!va) return vb ? b : 0;
  if (!vb) return a;
  return (int32_t)(a->sequence - b->sequence) > 0 ? a : b;
}
uint8_t EncoderCalibrationStore_Load(int32_t direction, uint32_t poles, float *offset) {
  const encoder_calibration_record_t *r = newest();
  if (!r || !offset || r->direction != direction || r->pole_pairs != poles) return 0;
  *offset = r->offset_rad;
  return 1;
}
uint8_t EncoderCalibrationStore_Save(int32_t direction, uint32_t poles,
                                    float offset, float voltage, float seconds) {
  if ((TIM1->BDTR & TIM_BDTR_MOE) || (__get_IPSR() != 0U)) return 0;
  const encoder_calibration_record_t *old = newest();
  uint32_t address = old == (const void *)ENCODER_CAL_STORE_PAGE0 ?
      ENCODER_CAL_STORE_PAGE1 : ENCODER_CAL_STORE_PAGE0;
  encoder_calibration_record_t r = {0};
  r.magic = ENCODER_CAL_STORE_MAGIC;
  r.version = 1;
  r.sequence = old ? old->sequence + 1U : 1U;
  r.direction = direction;
  r.pole_pairs = poles;
  r.offset_rad = offset;
  r.d_voltage_v = voltage;
  r.duration_s = seconds;
  r.checksum = EncoderCalibrationStore_Checksum(&r);
  r.commit = ENCODER_CAL_STORE_COMMIT;
  if (!EncoderCalibrationStore_Valid(&r)) return 0;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  uint32_t data_cache = FLASH->ACR & FLASH_ACR_DCEN;
  FLASH->ACR &= ~FLASH_ACR_DCEN;
  /* OFF-only operation. Same-bank Flash stalls control interrupts; never call
     from a live control cycle. SysTick cannot be used as an operation timer. */
  uint8_t ok = 0;
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  if (!(FLASH->CR & FLASH_CR_LOCK)) {
    FLASH->SR = FLASH_FLAG_SR_ERRORS | FLASH_SR_EOP;
    ok = wait_flash();
    if (ok) {
      FLASH->CR = (FLASH->CR & ~(FLASH_CR_PNB | FLASH_CR_PG)) |
          FLASH_CR_PER | (((address - FLASH_BASE) / FLASH_PAGE_SIZE) << FLASH_CR_PNB_Pos);
      FLASH->CR |= FLASH_CR_STRT;
      ok = wait_flash();
      FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
    }
    for (uint32_t i = 0; ok && i < sizeof(r); i += 8U) {
      uint32_t words[2];
      memcpy(words, (const uint8_t *)&r + i, sizeof(words));
      /* The last doubleword is the commit marker: incomplete writes cannot
         supersede the record in the untouched alternate page. */
      FLASH->CR |= FLASH_CR_PG;
      *(volatile uint32_t *)(address + i) = words[0];
      __ISB();
      *(volatile uint32_t *)(address + i + 4U) = words[1];
      __DSB();
      ok = wait_flash();
      FLASH->CR &= ~FLASH_CR_PG;
    }
    FLASH->CR |= FLASH_CR_LOCK;
  }
  FLASH->ACR |= FLASH_ACR_DCRST;
  FLASH->ACR &= ~FLASH_ACR_DCRST;
  FLASH->ACR |= data_cache;
  if (!mask) __enable_irq();
  return ok && EncoderCalibrationStore_Valid((const void *)address) &&
      memcmp((const void *)address, &r, sizeof(r)) == 0;
}
