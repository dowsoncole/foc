#include "mpt_t485.h"
uint32_t MptT485_Decode(const uint8_t *f, uint32_t n,
                       uint32_t *angle, uint32_t *turns)
{
  uint8_t crc=0;
  uint32_t i;
  if (!f || !angle || !turns || n!=MPT_FRAME_LENGTH) return 2U;
  for(i=0;i<n;i++) crc^=f[i];
  if(crc) return 1U;
  /* Datasheet pp.14-15: Get all=0x1A, little endian 17-bit angle.
     B5 is printed as '17' without a radix. Preserve it in frame[] for
     hardware inspection; do not reject a valid position over this ambiguity. */
  if(f[0]!=0x1AU || (f[4]&0xFEU) || (f[1]&0x9FU)) return 2U;
  if(f[1] || f[9]) return 3U;
  *angle=(uint32_t)f[2]|((uint32_t)f[3]<<8)|((uint32_t)f[4]<<16);
  *turns=(uint32_t)f[6]|((uint32_t)f[7]<<8)|((uint32_t)f[8]<<16);
  return 0U;
}
