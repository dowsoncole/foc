import os,sys,json,time,struct,subprocess,re,shutil
from pathlib import Path
import pyvisa,numpy as np
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe'
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('magnet_separated_ud15_%Y%m%d_%H%M%S');root.mkdir()
image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
class Probe(ProbeRun):
 def command(self,commands):
  q=subprocess.run([str(base.PROBE),'commander','--no-config','--pack',base.PACK,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-M','attach','-W','-f','500000','-c','; '.join(commands)],capture_output=True,text=True,timeout=25)
  with (self.directory/'probe.log').open('a') as f:f.write('; '.join(commands)+'\n'+q.stdout+q.stderr+'\n')
  if q.returncode or re.search(r'Error|No connected|No available',q.stdout+q.stderr):raise RuntimeError(q.stdout+q.stderr)
  return q.stdout
p=Probe(image,root/'trial');p.lengths['mode']=12;r={'verification':p.verify_flash(),'test':'magnet separated, all wires connected; fixed electrical angle, Ud=1.5 V Uq=0 V; 3 second output request'}
s=pyvisa.ResourceManager().open_resource('USB::0x0699::0x052C::C064843::INSTR');s.timeout=30000;s.chunk_size=1024*1024
rt=p.addresses['runtime'];vd=p.addresses['voltage'];mode=p.addresses['mode'];prepared=False
f32=lambda x:struct.unpack('<I',struct.pack('<f',x))[0]
try:
 _,r['before']=p.snapshot('before',full=True);base.require_off(r['before'])
 prepared=True
 p.command([f'write32 0x{p.addresses["run"]:x} 0',f'write32 0x{p.addresses["test"]+4:x} 0',f'write8 0x{mode:x} 0','sleep 100',f'write32 0x{vd:x} {f32(1.5)}',f'write32 0x{vd+4:x} 0',f'write32 0x{vd+12:x} 0',f'write32 0x{vd+16:x} 0',f'write32 0x{vd+32:x} {f32(150.0)}'])
 blocks,r['prepared']=p.snapshot('prepared',full=True)
 assert blocks['mode'][0]==0 and blocks['mode'][1]==0
 assert not r['prepared']['moe'] and not blocks['runtime'][1] and not r['prepared']['runtime_fault']
 assert abs(base.f32(blocks['voltage'])-1.5)<1e-6 and base.f32(blocks['voltage'],4)==0 and base.f32(blocks['voltage'],12)==0
 for cmd in ['ACQUIRE:STATE STOP','SELECT:CH2 ON','CH2:COUPLING DC','CH2:OFFSET 0','CH2:SCALE 1','CH2:POSITION -2','CH2:BANDWIDTH FULL','HORIZONTAL:RECORDLENGTH 10000000','HORIZONTAL:SCALE 1','HORIZONTAL:POSITION 10','TRIGGER:A:TYPE EDGE','TRIGGER:A:MODE AUTO','TRIGGER:A:EDGE:SOURCE CH2','TRIGGER:A:EDGE:SLOPE FALL','TRIGGER:A:LEVEL:CH2 0.5','ACQUIRE:STOPAFTER RUNSTOP','ACQUIRE:STATE RUN']:s.write(cmd)
 time.sleep(2);r['horizontal']=s.query('HORIZONTAL?').strip()
 r['run_log']=p.command([f'write8 0x{rt+1:x} 1','sleep 500',f'savemem 0x{rt:x} 36 {(root/"during_runtime.bin").as_posix()}',f'savemem 0x{vd:x} 40 {(root/"during_voltage.bin").as_posix()}',f'read32 0x{base.BDTR:x} 4','sleep 2500',f'write8 0x{rt+1:x} 0',f'write32 0x{base.BDTR:x} 0x02002054'])
 s.write('ACQUIRE:STATE STOP')
 dr=(root/'during_runtime.bin').read_bytes();dv=(root/'during_voltage.bin').read_bytes()
 r['during']={'runtime':list(dr[:6]),'fault':base.u32(dr,8),'ud_command':base.f32(dv),'uq_command':base.f32(dv,4),'electrical_hz':base.f32(dv,12),'ud_applied':base.f32(dv,24),'uq_applied':base.f32(dv,28)}
 print('DURING',r['during'],flush=True)
 r['output_verified']=bool(dr[1] and dr[2] and dr[5] and base.u32(dr,8)==0)
 p.command([f'write8 0x{mode:x} 1','sleep 100'])
 _,r['stopped']=p.snapshot('stopped',full=True);base.require_off(r['stopped'])
 deadline=time.monotonic()+15
 while s.query('ACQUIRE:STATE?').strip()!='0' and time.monotonic()<deadline:time.sleep(.5)
 assert s.query('ACQUIRE:STATE?').strip()=='0','Scope still acquiring'
 for c in ['DATA:SOURCE CH2','DATA:ENCDG RIBINARY','DATA:WIDTH 1','DATA:START 1','DATA:STOP 10000000']:s.write(c)
 meta={k:float(s.query('WFMOUTPRE:'+k+'?')) for k in ['XINCR','XZERO','PT_OFF','YMULT','YOFF','YZERO']}
 a=np.asarray(s.query_binary_values('CURVE?',datatype='b',container=np.array),dtype=np.int8)
 np.savez_compressed(root/'vdd.npz',ch2=a);(root/'vdd.json').write_text(json.dumps(meta,indent=2))
except Exception as e:
 r['error']=repr(e);print('ERROR',repr(e),flush=True)
finally:
 if prepared:
  try:p.command([f'write8 0x{rt+1:x} 0',f'write32 0x{base.BDTR:x} 0x02002054',f'write8 0x{mode:x} 1',f'read32 0x{base.BDTR:x} 4'])
  except Exception as e:r['stop_error']=repr(e)
 s.write('ACQUIRE:STATE STOP');s.close();(root/'summary.json').write_text(json.dumps(r,indent=2));print(root,flush=True)
