import os,sys,time,json,subprocess,re,shutil,struct,hashlib
from pathlib import Path
os.environ["SERVO_PYOCD"]="C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe"
from test_highspeed_closed_loop import ProbeRun,base
class Probe(ProbeRun):
 def snapshot(self,name,full=False):
  blocks,state=super().snapshot(name,full)
  state.update(current_limit_a=base.f32(blocks['parameters'],24),
               applied_iq_a=base.f32(blocks['output'],12),
               current_reference_limited=bool(blocks['output'][32]))
  (self.directory/name/'state.json').write_text(json.dumps(state,indent=2))
  return blocks,state
 def command(self,cmd):
  if self.keepalive:
   refreshed=[]
   for item in cmd:
    if item.startswith('savemem '): refreshed.append(self.heartbeat())
    refreshed.append(item)
   cmd=refreshed
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

