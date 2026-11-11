from pathlib import Path
root=Path('software/servo_v1')
p=root/'Modules/Encoder/Inc/spi1_encoder.h';s=p.read_text();s=s.replace('#include "stm32g4xx_hal.h"','#include "stm32g4xx_hal.h"\n#include "mt6701_ssi.h"');s=s.replace('#define MT6835_CSN_Pin                 GPIO_PIN_4\n#define MT6835_CSN_GPIO_Port           GPIOA','#if MT6701_SSI_ENABLE\n#define MT6835_CSN_Pin GPIO_PIN_6\n#define MT6835_CSN_GPIO_Port GPIOB\n#else\n#define MT6835_CSN_Pin                 GPIO_PIN_4\n#define MT6835_CSN_GPIO_Port           GPIOA\n#endif');p.write_text(s)
p=root/'Modules/Encoder/Src/spi1_encoder.c';s=p.read_text();s=s.replace('  __HAL_RCC_GPIOA_CLK_ENABLE();','  __HAL_RCC_GPIOA_CLK_ENABLE();\n  __HAL_RCC_GPIOB_CLK_ENABLE();');s=s.replace('  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;','#if MT6701_SSI_ENABLE\n  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;\n#else\n  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;\n#endif');s=s.replace('static void MT6835_SPI_CsDelay(void)\n{','static void MT6835_SPI_CsDelay(void)\n{\n#if MT6701_SSI_ENABLE\n  /* >= 1 us: covers SSI setup and half-clock hold at /128. */\n  for (volatile uint32_t n = 0; n < 168U; ++n) __NOP();\n#endif');p.write_text(s)
p=root/'Modules/Encoder/Inc/mt6701_ssi.h';s=p.read_text().replace('void Mt6701Ssi_Task(void);','void Mt6701Ssi_Task(void);\n/* Shared parser: DMA completion uses this without a second SPI owner. */\nvoid Mt6701Ssi_ProcessFrame(const uint8_t rx[3], HAL_StatusTypeDef status);');p.write_text(s)
p=root/'Modules/Encoder/Src/mt6701_ssi.c';s=p.read_text();s=s.replace('  uint32_t raw;\n  uint32_t angle;\n','');needle='  g_mt6701_ssi.last_sample_tick_ms = HAL_GetTick();';i=s.index(needle);s=s[:i]+'  Mt6701Ssi_ProcessFrame(rx, status);\n}\n\nvoid Mt6701Ssi_ProcessFrame(const uint8_t rx[3], HAL_StatusTypeDef status)\n{\n  uint32_t raw, angle;\n  g_mt6701_ssi.sample_valid = 0U;\n'+s[i:];s=s.replace('  g_mt6701_ssi.last_hal_error = HAL_SPI_GetError(&hspi_mt6701);\n  if (status != HAL_OK)','  if (status != HAL_OK)');p.write_text(s)
p=root/'Core/Src/main.c';s=p.read_text();a=s.index('#if MT6701_SSI_ENABLE\n  /* Dedicated SSI');b=s.index('\n  {',a);s=s[:a]+'  if (Encoder_Init() == 0U)'+s[b:];s=s.replace('#if MT6701_SSI_ENABLE\n    Mt6701Ssi_Task();\n#endif','/* Encoder SSI is sampled by TIM1/DMA; no foreground SPI owner. */');p.write_text(s)
p=root/'Modules/Encoder/Src/encoder.c';s=p.read_text();s=s.replace('#include "mt6835.h"','#include "mt6835.h"\n#include "mt6701_ssi.h"');s=s.replace('  f.computed_crc=MT6835_CalcCrc24(', '#if MT6701_SSI_ENABLE\n  f.computed_crc = g_mt6701_ssi.crc_calculated;\n  f.status = g_mt6701_ssi.status;\n#else\n  f.computed_crc=MT6835_CalcCrc24(');s=s.replace('  f.status=f.rx[4]&MT6835_ANGLE_L_STATUS_MASK;','  f.status=f.rx[4]&MT6835_ANGLE_L_STATUS_MASK;\n#endif')
a=s.index('  /* 实时刷新本帧 CRC');b=s.index('  if (status != HAL_OK)',a)
s=s[:a]+'''#if MT6701_SSI_ENABLE
  Mt6701Ssi_ProcessFrame(s_encoder.rx, status);
  /* Keep the common feedback ABI in 21-bit counts; MT6701 resolution remains
     14 bits. Scaling by 128 preserves existing speed, unwrap and step guards. */
  frame.angle_raw = g_mt6701_ssi.angle_raw << 7U;
  frame.status.raw = g_mt6701_ssi.status;
  frame.crc_ok = g_mt6701_ssi.sample_valid;
  s_encoder.last_computed_crc = g_mt6701_ssi.crc_calculated;
  s_encoder.last_crc_match = frame.crc_ok;
#else
'''+s[a:b]+'#endif\n'+s[b:]
s=s.replace('  else if (MT6835_DecodeAngleResponse', '#if !MT6701_SSI_ENABLE\n  else if (MT6835_DecodeAngleResponse');s=s.replace('  else if (frame.crc_ok == 0U)','#endif\n  else if (frame.crc_ok == 0U)');s=s.replace('  MT6835_PowerUpWait();\n  MT6835_BuildAngleReadCommand(s_encoder.tx);','''#if MT6701_SSI_ENABLE
  memset((void *)&g_mt6701_ssi, 0, sizeof(g_mt6701_ssi));
  g_mt6701_ssi.enabled = 1U;
  g_mt6701_ssi.initialized = 1U;
  /* Zero dummy TX, three bytes; no SSI command phase. */
#else
  MT6835_PowerUpWait();
  MT6835_BuildAngleReadCommand(s_encoder.tx);
#endif''');s=s.replace('s_encoder.tx, s_encoder.rx, sizeof(s_encoder.tx),','s_encoder.tx, s_encoder.rx,\n#if MT6701_SSI_ENABLE\n      3U,\n#else\n      sizeof(s_encoder.tx),\n#endif');p.write_text(s)
