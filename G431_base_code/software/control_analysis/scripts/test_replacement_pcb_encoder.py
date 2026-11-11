import os,sys,time,json,subprocess,re,shutil,struct
from pathlib import Path
os.environ['SERVO_PYOCD']='C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe';sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from test_highspeed_closed_loop import ProbeRun,base
root=Path('software/control_analysis/results')/time.strftime('replacement_pcb_off_on_off_%Y%m%d_%H%M%S');root.mkdir();image=(root/'firmware.axf').resolve();shutil.copy2('software/servo_v1/MDK-ARM/servo_v1/servo_v1.axf',image)
base.EXPRESSIONS['voltage']='&g_global_mode_control.voltage_open_loop';base.LENGTHS['voltage']=40
class Probe(ProbeRun):
 def command(self,cmd):
  x=subprocess.run([str(base.PROBE),'commander','--no-config','--pack',base.PACK,'-t','stm32g431vbtx','-u','E1007200D0D2139393740544','-M','attach','-W','-f','100000','-c','; '.join(cmd)],capture_output=True,text=True,timeout=40)
  with (self.directory/'probe.log').open('a') as f:f.write('; '.join(cmd)+'\n'+x.stdout+x.stderr+'\n')
  if x.returncode or re.search(r'Error|No available|No connected',x.stdout+x.stderr):raise RuntimeError(x.stdout+x.stderr)
  return x.stdout
p=Probe(image,root/'trial');r={'verification':p.verify_flash()};rt=p.addresses['runtime'];vd=p.addresses['voltage']
def snapcmd(label):
 return [f'savemem 0x{p.addresses[k]:x} {size} {(root/(label+"_"+k+".bin")).as_posix()}' for k,size in [('diagnostics',128),('runtime',36),('voltage',40),('encoder',48),('bdtr',4),('cycles',4)]]
try:
 p.command([f'write8 0x{rt+1:x} 0','sleep 100',f'write32 0x{base.BDTR:x} 0x02002054'])
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
