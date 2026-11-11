import os,sys,time,json,subprocess,re,shutil,struct,hashlib
from pathlib import Path
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe';sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('mt6701_ssi_off_on_off_%Y%m%d_%H%M%S');root.mkdir();image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
base.EXPRESSIONS['ssi']='&g_mt6701_ssi';base.LENGTHS['ssi']=92
class Probe(ProbeRun):
 def command(self,cmd):
  x=subprocess.run([str(base.PROBE),'commander','--no-config','--pack',base.PACK,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-M','attach','-W','-f','1000000','-c','; '.join(cmd)],capture_output=True,text=True,timeout=40)
  with (self.directory/'probe.log').open('a') as f:f.write('; '.join(cmd)+'\n'+x.stdout+x.stderr+'\n')
  if x.returncode or re.search(r'Error|No available|No connected',x.stdout+x.stderr):raise RuntimeError(x.stdout+x.stderr)
  return x.stdout
 def verify_flash(self):
  expected=base.flash_section(self.image);folder=self.directory/'flash_chunks';folder.mkdir();size=256
  offsets=list(range(0,len(expected),size))
  for start in range(0,len(offsets),16):
   batch=offsets[start:start+16]
   self.command([f'savemem 0x{0x08000000+i:x} {min(size,len(expected)-i)} {(folder/str(i)).as_posix()}' for i in batch])
  retried=[]
  for i in offsets:
   f=folder/str(i);want=expected[i:i+size]
   if f.read_bytes()!=want:
    retried.append(i)
    self.command([f'savemem 0x{0x08000000+i:x} {len(want)} {f.as_posix()}'])
   if f.read_bytes()!=want:raise RuntimeError('Flash mismatch at '+hex(0x08000000+i))
  actual=b''.join((folder/str(i)).read_bytes() for i in offsets)
  (self.directory/'flash.bin').write_bytes(actual)
  assert actual==expected
  print('FLASH VERIFIED',len(actual),'retried chunks',len(retried),flush=True)
  return {'image':str(self.image),'flash_bytes':len(actual),'flash_sha256':hashlib.sha256(actual).hexdigest(),'verification':'256-byte chunks, exact byte comparison','retried_chunks':retried}


p=Probe(image,root/'trial');r={'verification':p.verify_flash(),'protocol':'MT6701 SSI / SPI1 Mode 1, PB6 CS, PB3 CLK, PB4 DO, PB5 dummy MOSI'};rt=p.addresses['runtime'];vd=p.addresses['voltage'];ssi=p.addresses['ssi']
def commands(label):
 return [f'savemem 0x{addr:x} {n} {(root/(label+"_"+k+".bin")).as_posix()}' for k,addr,n in [('ssi',ssi,92),('runtime',rt,36),('voltage',vd,40),('bdtr',base.BDTR,4)]]
def decode(label):
 d=(root/(label+'_ssi.bin')).read_bytes();u=(root/(label+'_runtime.bin')).read_bytes();v=(root/(label+'_voltage.bin')).read_bytes()
 x={k:base.u32(d,i*4) for i,k in enumerate(['enabled','period_ms','initialized','hal_status','hal_error','frames','good','spi_errors','crc_errors','blank','warnings','angle_changes','consecutive_good','sample_valid','communication_ok','sample_tick','good_tick','raw_frame','angle_raw'])}
 x.update(angle_deg=base.f32(d,76),status=d[83],magnetic_status=d[84],crc_received=d[87],crc_calculated=d[88],moe=bool(base.u32((root/(label+'_bdtr.bin')).read_bytes())&0x8000),runtime=list(u[:6]),runtime_fault=base.u32(u,8),applied_ud=base.f32(v,24),applied_uq=base.f32(v,28),electrical_hz=base.f32(v,12));return x
try:
 p.command([f'write8 0x{rt+1:x} 0','sleep 100','write32 0x40012c44 0x02002054','read32 0x48000400 16','read32 0x48000420 4','read32 0x40013000 12'])
 p.command(commands('off_start')+['sleep 2000']+commands('off_end'))
 r['points']={k:decode(k) for k in ['off_start','off_end']};a=r['points']['off_start'];b=r['points']['off_end']
 assert not b['moe'] and b['initialized'] and b['communication_ok'] and (b['good']-a['good'])>10,'SSI off-state response not qualified; output not started'
 assert b['crc_errors']==a['crc_errors'] and b['blank']==a['blank'] and b['spi_errors']==a['spi_errors'],'SSI off-state communication errors; output not started'
 voltage_before=(root/'off_end_voltage.bin').read_bytes()
 assert abs(base.f32(voltage_before,0)-1.0)<0.001 and abs(base.f32(voltage_before,4))<0.001 and abs(b['electrical_hz']-10.0)<0.001,'Unexpected open-loop voltage/frequency'
 p.command([f'write8 0x{rt+1:x} 1','sleep 2500']+commands('on_start')+['sleep 3000']+commands('on_end')+[f'write8 0x{rt+1:x} 0','sleep 500']+commands('after_start')+['sleep 2000']+commands('after_end'))
 for k in ['on_start','on_end','after_start','after_end']:r['points'][k]=decode(k)
 r['stages']={}
 for name,a,z in [('off','off_start','off_end'),('on','on_start','on_end'),('off_after','after_start','after_end')]:
  x=r['points'][a];y=r['points'][z];r['stages'][name]={k:(y[k]-x[k])&0xffffffff for k in ['frames','good','crc_errors','spi_errors','blank','warnings','angle_changes']};r['stages'][name].update(moe_start=x['moe'],moe_end=y['moe'],elapsed_ms=(y['sample_tick']-x['sample_tick'])&0xffffffff,applied_ud=y['applied_ud'],electrical_hz=y['electrical_hz'],runtime_fault=y['runtime_fault'])
 print(json.dumps(r['stages'],indent=2),flush=True)
except Exception as e:r['error']=repr(e);print('ERROR',repr(e),flush=True)
finally:
 p.command([f'write8 0x{rt+1:x} 0','sleep 50','write32 0x40012c44 0x02002054'])
 p.command(commands('stopped'));r['stopped']=decode('stopped');(root/'summary.json').write_text(json.dumps(r,indent=2));print(root,flush=True)
