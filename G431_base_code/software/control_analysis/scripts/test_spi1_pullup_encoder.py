import os,sys,time,json,subprocess,re,shutil,struct,hashlib
from pathlib import Path
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe';sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('spi1_pullup_off_on_off_%Y%m%d_%H%M%S');root.mkdir();image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
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

p=Probe(image,root/'trial');r={'verification':p.verify_flash()};rt=p.addresses['runtime'];vd=p.addresses['voltage']
def snapcmd(label):
 return [f'savemem 0x{p.addresses[k]:x} {size} {(root/(label+"_"+k+".bin")).as_posix()}' for k,size in [('diagnostics',128),('runtime',36),('voltage',40),('encoder',48),('bdtr',4),('cycles',4)]]
try:
 p.command([f'write8 0x{rt+1:x} 0','sleep 100',f'write32 0x{base.BDTR:x} 0x02002054'])
 p.command(['savemem 0x4800000c 4 '+(root/'gpioa_pupdr.bin').as_posix()])
 pupdr=base.u32((root/'gpioa_pupdr.bin').read_bytes())
 r['gpioa_pupdr']=hex(pupdr);r['pullups']={str(pin):(pupdr>>(pin*2))&3 for pin in [4,5,6,7]}
 assert all(x==1 for x in r['pullups'].values()),'SPI1 internal pull-ups not active'
 blocks,r['before']=p.snapshot('before',full=True)
 assert not r['before']['moe'] and r['before']['mode']==[0,0]
 assert abs(base.f32(blocks['voltage'])-1)<1e-6 and base.f32(blocks['voltage'],4)==0 and base.f32(blocks['voltage'],12)==10
 cmd=['read32 0x1fff7590 12','sleep 500']+snapcmd('off_start')+['sleep 2000']+snapcmd('off_end')+[f'write8 0x{rt+1:x} 1','sleep 2500']+snapcmd('on_start')+['sleep 3000']+snapcmd('on_end')+[f'write8 0x{rt+1:x} 0','sleep 500']+snapcmd('after_start')+['sleep 2000']+snapcmd('after_end')
 r['run_log']=p.command(cmd);r['points']={}
 for label in ['off_start','off_end','on_start','on_end','after_start','after_end']:
  b={k:(root/(label+'_'+k+'.bin')).read_bytes() for k in ['diagnostics','runtime','voltage','encoder','bdtr','cycles']};d=b['diagnostics'];v=b['voltage'];e=b['encoder'];u=b['runtime']
  r['points'][label]={'completed':base.u32(d,4),'busy':base.u32(d,8),'start_errors':base.u32(d,12),'transfer_errors':base.u32(d,16),'crc_errors':base.u32(d,20),'alarms':base.u32(d,24),'step_errors':base.u32(d,104),'moe':bool(base.u32(b['bdtr'])&0x8000),'runtime':list(u[:6]),'fault':base.u32(u,8),'applied_ud':base.f32(v,24),'applied_uq':base.f32(v,28),'electrical_hz':base.f32(v,12),'angle':base.f32(v,16),'voltage_cycles':base.u32(v,20),'encoder_flags':base.u32(e,36),'cycles':base.u32(b['cycles'])}
 r['stages']={}
 for name,a,z in [('off','off_start','off_end'),('on','on_start','on_end'),('off_after','after_start','after_end')]:
  x=r['points'][a];y=r['points'][z];out={k:(y[k]-x[k])&0xffffffff for k in ['completed','busy','start_errors','transfer_errors','crc_errors','alarms','step_errors']};out.update(seconds=((y['cycles']-x['cycles'])&0xffffffff)/168e6,moe_start=x['moe'],moe_end=y['moe'],runtime_fault_start=x['fault'],runtime_fault_end=y['fault']);r['stages'][name]=out
 print(json.dumps(r['stages'],indent=2),flush=True)
except Exception as e:r['error']=repr(e);print('ERROR',repr(e),flush=True)
finally:
 p.command([f'write8 0x{rt+1:x} 0','sleep 50',f'write32 0x{base.BDTR:x} 0x02002054'])
 _,r['stopped']=p.snapshot('stopped',full=True);(root/'summary.json').write_text(json.dumps(r,indent=2));print('STOPPED MOE',r['stopped']['moe'],flush=True);print(root,flush=True)
