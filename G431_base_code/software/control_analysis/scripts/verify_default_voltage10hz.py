import os,sys,time,json,shutil,subprocess,re,struct
from pathlib import Path
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe'
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('verify_default_voltage10hz_%Y%m%d_%H%M%S');root.mkdir()
image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
class Probe(ProbeRun):
 def command(self,cmds):
  x=subprocess.run([str(base.PROBE),'commander','--no-config','--pack',base.PACK,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-M','attach','-W','-f','500000','-c','; '.join(cmds)],capture_output=True,text=True,timeout=20)
  with (self.directory/'probe.log').open('a') as f:f.write('; '.join(cmds)+'\n'+x.stdout+x.stderr+'\n')
  if x.returncode or re.search('Error|No available',x.stdout+x.stderr):raise RuntimeError(x.stdout+x.stderr)
  return x.stdout
p=Probe(image,root/'trial');r={'verification':p.verify_flash()};rt=p.addresses['runtime'];vd=p.addresses['voltage']
try:
 cmds=['reset halt','go','sleep 2500']
 for i in range(2):
  cmds += [f'savemem 0x{rt:x} 36 {(root/f"runtime{i}.bin").as_posix()}',f'savemem 0x{vd:x} 40 {(root/f"voltage{i}.bin").as_posix()}',f'savemem 0x{base.BDTR:x} 4 {(root/f"bdtr{i}.bin").as_posix()}',f'savemem 0x40012c34 16 {(root/f"ccr{i}.bin").as_posix()}', 'sleep 20']
 cmds += [f'write8 0x{rt+1:x} 0','sleep 50',f'write32 0x{base.BDTR:x} 0x02002054']
 r['log']=p.command(cmds);r['during']=[]
 for i in range(2):
  v=(root/f'voltage{i}.bin').read_bytes();st=(root/f'runtime{i}.bin').read_bytes();bd=base.u32((root/f'bdtr{i}.bin').read_bytes())
  r['during'].append({'runtime':list(st[:6]),'fault':base.u32(st,8),'moe':bool(bd&0x8000),'ud':base.f32(v),'uq':base.f32(v,4),'frequency_hz':base.f32(v,12),'angle':base.f32(v,16),'cycle':base.u32(v,20),'applied_ud':base.f32(v,24),'applied_uq':base.f32(v,28),'ccr':list(struct.unpack('<4I',(root/f'ccr{i}.bin').read_bytes()))})
 print(json.dumps(r['during']),flush=True)
finally:
 p.command([f'write8 0x{rt+1:x} 0',f'write32 0x{base.BDTR:x} 0x02002054'])
 _,r['stopped']=p.snapshot('stopped',full=True)
 (root/'summary.json').write_text(json.dumps(r,indent=2));print(root,flush=True)
