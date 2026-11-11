import os,sys,time,json,shutil,subprocess,re,struct
from pathlib import Path
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe'
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('encoder_voltage_scope_%Y%m%d_%H%M%S');root.mkdir()
image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
class Probe(ProbeRun):
 def command(self,cmds):
  x=subprocess.run([str(base.PROBE),'commander','--no-config','--pack',base.PACK,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-M','attach','-W','-f','500000','-c','; '.join(cmds)],capture_output=True,text=True,timeout=20)
  with (self.directory/'probe.log').open('a') as f:f.write('; '.join(cmds)+'\n'+x.stdout+x.stderr+'\n')
  if x.returncode or re.search('Error|No available',x.stdout+x.stderr):raise RuntimeError(x.stdout+x.stderr)
  return x.stdout

import pyvisa,numpy as np
from concurrent.futures import ThreadPoolExecutor
p=Probe(image,root/'trial');r={'verification':p.verify_flash(),'signals':{'1':'CS','2':'CLK','3':'SO'}};rt=p.addresses['runtime'];vd=p.addresses['voltage']
s=pyvisa.ResourceManager().open_resource('USB::0x0699::0x052C::C064843::INSTR');s.timeout=20000;s.chunk_size=1024*1024
pool=ThreadPoolExecutor(max_workers=1);future=None

def capture(label):
 for c in ['ACQUIRE:STATE STOP','SELECT:CH1 ON','SELECT:CH2 ON','SELECT:CH3 ON','HORIZONTAL:RECORDLENGTH 1000000','HORIZONTAL:SCALE 0.001','HORIZONTAL:POSITION 10','TRIGGER:A:TYPE EDGE','TRIGGER:A:MODE NORMAL','TRIGGER:A:EDGE:SOURCE CH2','TRIGGER:A:EDGE:SLOPE FALL','TRIGGER:A:EDGE:COUPLING DC','TRIGGER:A:LEVEL:CH2 1.65','ACQUIRE:STOPAFTER SEQUENCE']:
  s.write(c)
 for ch in [1,2,3]:
  for setting in ['SCALE 1','OFFSET 0','POSITION -2','COUPLING DC','BANDWIDTH FULL']:s.write(f'CH{ch}:{setting}')
 s.query('*OPC?');s.write('ACQUIRE:STATE RUN');deadline=time.monotonic()+8
 while s.query('ACQUIRE:STATE?').strip()!='0' and time.monotonic()<deadline:time.sleep(.1)
 if s.query('ACQUIRE:STATE?').strip()!='0':raise RuntimeError(label+' trigger timeout')
 meta={'channels':{},'signals':r['signals']};arrays={}
 for c in ['DATA:ENCDG RIBINARY','DATA:WIDTH 1','DATA:START 1','DATA:STOP 1000000']:s.write(c)
 for ch in [1,2,3]:
  s.write(f'DATA:SOURCE CH{ch}')
  meta['channels'][str(ch)]={k:float(s.query('WFMOUTPRE:'+k+'?')) for k in ['XINCR','XZERO','PT_OFF','YMULT','YOFF','YZERO']}
  arrays[f'ch{ch}']=np.asarray(s.query_binary_values('CURVE?',datatype='b',container=np.array),dtype=np.int8)
 np.savez_compressed(root/(label+'.npz'),**arrays);(root/(label+'.json')).write_text(json.dumps(meta,indent=2));print('CAPTURE',label,flush=True)
try:
 _,r['before']=p.snapshot('before',full=True)
 assert not r['before']['moe']
 capture('off')
 cmds=[f'write8 0x{rt+1:x} 1','sleep 2500',f'savemem 0x{rt:x} 36 {(root/'on_runtime.bin').as_posix()}',f'savemem 0x{vd:x} 40 {(root/'on_voltage.bin').as_posix()}',f'savemem 0x{base.BDTR:x} 4 {(root/'on_bdtr.bin').as_posix()}','sleep 4000',f'write8 0x{rt+1:x} 0',f'write32 0x{base.BDTR:x} 0x02002054']
 future=pool.submit(p.command,cmds);time.sleep(3.5);capture('on');r['run_log']=future.result(timeout=20)
 st=(root/'on_runtime.bin').read_bytes();v=(root/'on_voltage.bin').read_bytes()
 r['on']={'moe':bool(base.u32((root/'on_bdtr.bin').read_bytes())&0x8000),'runtime':list(st[:6]),'fault':base.u32(st,8),'ud':base.f32(v,24),'uq':base.f32(v,28),'frequency':base.f32(v,12)}
 print('ON',r['on'],flush=True)
except Exception as e:r['error']=repr(e);print('ERROR',repr(e),flush=True)
finally:
 if future:
  try:future.result(timeout=25)
  except Exception as e:r['run_error']=repr(e)
 p.command([f'write8 0x{rt+1:x} 0',f'write32 0x{base.BDTR:x} 0x02002054'])
 _,r['stopped']=p.snapshot('stopped',full=True)
 s.write('ACQUIRE:STATE STOP');s.close();pool.shutdown();(root/'summary.json').write_text(json.dumps(r,indent=2));print(root,flush=True)
