#ifndef MT6835_CRC_H
#define MT6835_CRC_H
#include <stdint.h>
/* CRC-8 poly 0x07, init 0x00, MSB first over exactly 24 bits.
   16-byte nibble table replaces the data-dependent 24-iteration IRQ loop. */
static inline uint8_t MT6835_Crc24Fast(uint32_t data24)
{
  static const uint8_t table[16]={
    0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,
    0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d};
  uint8_t crc=0;
  for(int shift=16;shift>=0;shift-=8) {
    crc^=(uint8_t)(data24>>shift);
    crc=(uint8_t)((uint8_t)(crc<<4)^table[crc>>4]);
    crc=(uint8_t)((uint8_t)(crc<<4)^table[crc>>4]);
  }
  return crc;
}
#endif
