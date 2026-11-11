"""Bounded hardware commissioning; --run explicitly enables motor motion."""
import argparse,json,struct,subprocess,re,time,shutil,sys
from pathlib import Path
from mt6701_probe import Probe,base
from position_abi import position_symbols

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',action='store_true');ap.add_argument('--image',type=Path,default=Path('software/control_analysis/build_mpt/out/servo_v1/servo_v1/servo_v1.axf'));ap.add_argument('--max-motor-rpm',type=float,default=2500);args=ap.parse_args()
 if not args.run: ap.error('--run required')
 if not 1<=args.max_motor_rpm<=2500: ap.error('max motor RPM must be 1..2500')
 root=Path('software/control_analysis/results')/time.strftime('mpt_position_%Y%m%d_%H%M%S');root.mkdir()
 image=root/'firmware.axf';shutil.copy2(args.image,image)
 p=Probe(image,root/'probe');a=p.addresses
 g=subprocess.run([str(base.GDB),'-batch',str(image),'-ex','p/x &g_mpt_t485','-ex','p/x &g_output_position'],capture_output=True,text=True,check=True)
 vals=re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$',g.stdout,re.M);assert len(vals)==2
 a['mpt'],a['position']=[int(v,16) for v in vals]
 # [千问工作助理 2026-10-28] mpt 88→100：本轮给 mpt_t485_t 追加了 3 个
 # uint32_t 响应延迟统计字段（偏移 88/92/96），旧长度会静默少读尾部。
 # 数值来源：编译器实测，见 scripts/check_struct_sizes.c 与 scripts/struct_sizes.txt
 # ⚠️ 未改动 position=52：实测 sizeof(output_position_t)=56，本值少读 4 字节
 #   会漏掉偏移 52 的 velocity_deg_s，且与 test_mpt_position_sine.py 的 56 矛盾。
 #   属既有缺陷、非本轮引入，已记录于调试日志待用户确认，本轮不擅自扩大范围。
 abi=position_symbols(image,base.GDB)
 p.lengths.update(mpt=abi['mpt_size'],position=abi['position_size'])
 result={'samples':{}}
 verified=False
 bounded_pending=False
 def snap(name,full=False):
  b,s=p.snapshot(name,full)
  m=b['mpt'];o=b['position']
  s.update(mpt_good=base.u32(m,12),mpt_errors=list(struct.unpack_from('<5I',m,16)),mpt_valid=base.u32(m,40),mpt_deg=base.f32(m,68),pos_state=base.u32(o,4),pos_fault=base.u32(o,8),pos_target=base.f32(o,20),pos_actual=base.f32(o,24),pos_error=base.f32(o,28),pos_rpm=base.f32(o,32))
  result['samples'][name]=s;print(name,json.dumps({k:s[k] for k in ['moe','run_state','speed_test_state','speed_test_fault','actual_rpm','mpt_deg','mpt_errors','pos_state','pos_fault','pos_error','deadline_overruns']}),flush=True)
  return b,s
 def writef(addr,x):return f'write32 0x{addr:x} {struct.unpack("<I",struct.pack("<f",x))[0]}'
 def healthy(s,baseline):
  for k in ['crc_errors','step_errors','start_errors','transfer_errors','deadline_overruns','pwm_window_rejections','pwm_deadline_rejections']:
   assert s[k]==baseline[k],k
  assert s['mpt_errors']==baseline['mpt_errors'] and s['mpt_valid']
  assert not s['speed_test_fault'] and not s['runtime_fault'] and not s['controller_fault']
 try:
  _,s=snap('before');base.require_off(s)
  assert s['calibration']==3 and s['mode']==[1,1] and s['stage']==[2,2]
  assert s['encoder_valid'][1:3]==[1,1] and not s['encoder_flags']
  assert not s['runtime_fault'] and not s['controller_fault']
  assert s['mpt_valid'] and not any(s['mpt_errors']), 'Communication must qualify before motion'
  assert 20<s['mpt_deg']<340
  result['flash']=p.verify_flash()
  verified=True
  requested=(root/'probe/before/test.bin').read_bytes()[16:64]
  bounded_pending=True
  p.command([f'write32 0x{a["test"]+16:x} 10',f'write32 0x{a["test"]:x} 1','sleep 4000'])
  _,s=snap('aligned',True);assert s['calibration_ok'] and s['speed_test_state']==3 and not s['moe'];healthy(s,result['samples']['before'])
  p.command([f'write32 0x{a["test"]+16+i:x} {base.u32(requested,i)}' for i in range(0,48,4)]+[f'write32 0x{a["test"]:x} 3','sleep 200'])
  bounded_pending=False
  _,start=snap('direction_before');base.require_ready(start)
  bounded_pending=True
  # 2-second low-current speed test: about 2.6 output degrees, MCU timeout.
  p.command([writef(a['test']+24,5.0),f'write32 0x{a["test"]:x} 1','sleep 50',f'write8 0x{a["runtime"]+1:x} 1','sleep 2600'])
  _,end=snap('direction_after',True);healthy(end,start);assert not end['moe'] and end['speed_test_state']==3
  bounded_pending=False
  delta=end['mpt_deg']-start['mpt_deg'];assert .3<abs(delta)<6,delta
  direction=1 if delta>0 else -1;result['motor_direction']=direction;result['direction_delta_deg']=delta
  p.command([f'write32 0x{a["test"]:x} 3','sleep 200',f'write32 0x{a["guard"]:x} 6000',p.heartbeat(),writef(a['position']+40,args.max_motor_rpm),f'write32 0x{a["position"]+12:x} 1',f'write32 0x{a["position"]+16:x} {direction & 0xffffffff}',f'write32 0x{a["position"]:x} 1'])
  p.keepalive=True
  _,armed=snap('position_hold');healthy(armed,end);assert armed['pos_state']==1 and armed['run_state']==2 and armed['moe']
  origin=armed['pos_target'];result['origin_deg']=origin
  for label,target in [('plus_one',origin+1),('return',origin)]:
   p.command([p.heartbeat(),writef(a['position']+20,target)])
   for i in range(5):
    _,s=snap(f'{label}_{i}');healthy(s,end)
    assert s['pos_state']==1 and not s['pos_fault'] and s['moe'] and s['run_state']==2
    assert abs(s['mpt_deg']-origin)<3,'Travel exceeded trial envelope'
   assert abs(s['pos_error'])<.05,'Position has not settled within 0.05 degree'
  result['position_small_step_passed']=True
 except BaseException as e:
  result['error']=repr(e);print('ERROR',repr(e),flush=True)
 finally:
  try:
   if not verified: raise RuntimeError('No RAM writes: image identity not verified')
   p.keepalive=False
   abort=[f'write32 0x{a["test"]:x} 2'] if bounded_pending else []
   p.command(abort+[f'write32 0x{a["position"]:x} 2',f'write32 0x{a["run"]:x} 0',f'write8 0x{a["runtime"]+1:x} 0','sleep 100',f'write32 0x{base.BDTR:x} 0x02002054'])
   _,stopped=snap('stopped',True);base.require_off(stopped)
  except BaseException as e:result['cleanup_error']=repr(e)
  (root/'summary.json').write_text(json.dumps(result,indent=2));print(root,flush=True)
 if 'error' in result or 'cleanup_error' in result:raise SystemExit(1)
if __name__=='__main__':main()
