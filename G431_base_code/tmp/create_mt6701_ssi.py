from pathlib import Path
root=Path('software/servo_v1')
h=(root/'Tests/Inc/mt6701_spi3_test.h').read_text(encoding='utf-8-sig')
for a,b in [('MT6701_SPI3_TEST_H','MT6701_SSI_H'),('MT6701_SPI3_TEST_ENABLE','MT6701_SSI_ENABLE'),('mt6701_spi3_test_t','mt6701_ssi_t'),('g_mt6701_spi3_test','g_mt6701_ssi'),('Mt6701Spi3Test','Mt6701Ssi')]:h=h.replace(a,b)
h=h.replace('poll_period_ms 默认 10','poll_period_ms 默认 1').replace('默认每 10 ms','默认每 1 ms')
extra='''/* MT6701 Rev.1.8 pp.24-26: PB6 CSN, PB3 CLK, PB4 DO; PB5 dummy MOSI.
   SPI1 AF5, Mode 1, 24 clocks; no command byte and no hardware CRC. */
#define MT6701_SSI_CSN_PIN GPIO_PIN_6
#define MT6701_SSI_CSN_PORT GPIOB
#define MT6701_SSI_FRAME_BYTES 3U
#define MT6701_SSI_PRESCALER SPI_BAUDRATEPRESCALER_128
extern SPI_HandleTypeDef hspi_mt6701;
HAL_StatusTypeDef Mt6701Ssi_TransportInit(void);
HAL_StatusTypeDef Mt6701Ssi_ReadFrame(uint8_t frame[MT6701_SSI_FRAME_BYTES], uint32_t timeout_ms);
'''
h=h.replace('typedef struct\n',extra+'\ntypedef struct\n',1)
(root/'Modules/Encoder/Inc/mt6701_ssi.h').write_text(h,encoding='utf-8')
transport=(root/'Modules/Encoder/Src/spi3_encoder.c').read_text(encoding='utf-8-sig')
for a,b in [('"spi3_encoder.h"','"mt6701_ssi.h"'),('hspi3','hspi_mt6701'),('Spi3Encoder','Mt6701Ssi'),('MX_SPI3_Encoder_Init','Mt6701Ssi_TransportInit'),('SPI3_Encoder_ReadFrame','Mt6701Ssi_ReadFrame'),('MT6701_SPI3_','MT6701_SSI_'),('SPI3','SPI1')]:transport=transport.replace(a,b)
transport=transport.replace('gpio.Pull = GPIO_NOPULL','gpio.Pull = GPIO_PULLUP')
protocol=(root/'Tests/Src/mt6701_spi3_test.c').read_text(encoding='utf-8-sig')
for a,b in [('"mt6701_spi3_test.h"','"mt6701_ssi.h"'),('#include "spi3_encoder.h"',''),('Mt6701Spi3Test','Mt6701Ssi'),('g_mt6701_spi3_test','g_mt6701_ssi'),('mt6701_spi3_test_t','mt6701_ssi_t'),('MX_SPI3_Encoder_Init','Mt6701Ssi_TransportInit'),('SPI3_Encoder_ReadFrame','Mt6701Ssi_ReadFrame'),('MT6701_SPI3_','MT6701_SSI_'),('hspi3','hspi_mt6701')]:protocol=protocol.replace(a,b)
protocol=protocol.replace('poll_period_ms = 10U','poll_period_ms = 1U')
protocol=protocol.replace('Rev.1.8/1.9','Rev.1.8').replace('初值 0，无反射，无末尾异或','采用初值 0、无反射、无末尾异或；需用实机完整帧核验')
(root/'Modules/Encoder/Src/mt6701_ssi.c').write_text(transport+'\n'+protocol,encoding='utf-8')
# Register the independent SSI diagnostic in both build descriptions.
p=root/'MDK-ARM/servo_v1.uvprojx';s=p.read_text(encoding='utf-8-sig');needle='''          <GroupName>Modules/Encoder</GroupName>
          <Files>''';addition='''
            <File><FileName>mt6701_ssi.c</FileName><FileType>1</FileType><FilePath>../Modules/Encoder/Src/mt6701_ssi.c</FilePath></File>
            <File><FileName>mt6701_ssi.h</FileName><FileType>5</FileType><FilePath>../Modules/Encoder/Inc/mt6701_ssi.h</FilePath></File>''';assert needle in s;s=s.replace(needle,needle+addition,1);p.write_text(s,encoding='utf-8')
p=root/'MDK-ARM/servo_v1.cproject.yml';s=p.read_text(encoding='utf-8-sig');needle='        - file: ../Modules/Encoder/Src/spi3_encoder.c';assert needle in s;s=s.replace(needle,'        - file: ../Modules/Encoder/Src/mt6701_ssi.c\n        - file: ../Modules/Encoder/Inc/mt6701_ssi.h\n'+needle,1);p.write_text(s,encoding='utf-8')
# Exercise the actual new transport and parser with independent CRC vectors.
stub=root/'Tests/Host/mt6701_ssi_stubs';stub.mkdir(exist_ok=True)
s=(root/'Tests/Host/spi3_stubs/stm32g4xx_hal.h').read_text(encoding='utf-8-sig').replace('SPI3','SPI1').replace('spi3','spi1');s=s.replace('#define GPIO_NOPULL 0U','#define GPIO_NOPULL 0U\n#define GPIO_PULLUP 1U\n#define SPI_BAUDRATEPRESCALER_128 (6U << 3)');(stub/'stm32g4xx_hal.h').write_text(s,encoding='utf-8')
s=(root/'Tests/Host/test_mt6701_spi3.c').read_text(encoding='utf-8-sig')
for a,b in [('"mt6701_spi3_test.h"','"mt6701_ssi.h"'),('#include "spi3_encoder.h"',''),('Mt6701Spi3Test','Mt6701Ssi'),('g_mt6701_spi3_test','g_mt6701_ssi'),('SPI3_Encoder_ReadFrame','Mt6701Ssi_ReadFrame'),('MX_SPI3_Encoder_Init','Mt6701Ssi_TransportInit'),('SPI_BAUDRATEPRESCALER_256','SPI_BAUDRATEPRESCALER_128'),('SPI3','SPI1'),('spi3','spi1'),('tick = 9U','tick = 0U')]:s=s.replace(a,b)
s=s.replace('assert(cs == GPIO_PIN_SET && config->Mode == GPIO_MODE_OUTPUT_PP);','assert(cs == GPIO_PIN_SET && config->Mode == GPIO_MODE_OUTPUT_PP);\n  assert(config->Pull == GPIO_PULLUP);')
(root/'Tests/Host/test_mt6701_ssi.c').write_text(s,encoding='utf-8')
