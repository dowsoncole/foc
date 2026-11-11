from pypdf import PdfReader
from pathlib import Path
out=Path('tmp/pdfs/mt6701_review');out.mkdir(parents=True,exist_ok=True)
for name,path in [('mt6701','C:/Users/penghao/Desktop/编码器芯片/6701.pdf'),('g431','hardware/芯片资料/G431VB_datasheet.pdf')]:
 d=PdfReader(path);print(name,'pages',len(d.pages))
 for i,p in enumerate(d.pages):
  t=p.extract_text();(out/f'{name}_{i+1:02}.txt').write_text(t,encoding='utf-8')
  if name=='mt6701' and any(x in t for x in ['SSI','CRC','串行']):print('SSI PAGE',i+1,t[:70].replace('\n',' '))
  if name=='g431' and 'PB3' in t and 'SPI1' in t:print('PIN PAGE',i+1)
