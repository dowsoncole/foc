"""Bounded hardware commissioning; --run explicitly enables motor motion."""
import argparse,json,struct,subprocess,re,time,shutil,sys
from pathlib import Path
from mt6701_probe import Probe,base
from position_abi import position_symbols

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',action='store_true');ap.add_argument('--image',type=Path,default=Path('software/control_analysis/build_mpt/out/servo_v1/servo_v1/servo_v1.axf'));args=ap.parse_args()
 if not args.run: ap.error('--run required')
 root=Path('software/control_analysis/results')/time.strftime('mpt_sine_%Y%m%d_%H%M%S');root.mkdir()
 image=root/'firmware.axf';shutil.copy2(args.image,image)
 p=Probe(image,root/'probe');a=p.addresses
 g=subprocess.run([str(base.GDB),'-batch',str(image),'-ex','p/x &g_mpt_t485','-ex','p/x &g_output_position','-ex','p/x &g_output_position_test','-ex','p/x &g_output_position_trace','-ex','p/x &g_system_clock_source'],capture_output=True,text=True,check=True)
 vals=re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$',g.stdout,re.M);assert len(vals)==5
 a['mpt'],a['position'],a['trajectory'],a['trace'],a['clock_source']=[int(v,16) for v in vals]
 # [千问工作助理 2026-10-28] mpt 88→100、trajectory 40→60。
 # 本轮给 mpt_t485_t 追加了 3 个 uint32_t 响应延迟统计字段（偏移 88/92/96）、
 # 给 output_position_test_t 追加了 5 个扫频字段（偏移 40/44/48/52/56）。
 # 若仍按旧长度 savemem，会**静默少读尾部字节**——不报错，但新增的
 # resp_latency_* 与 frequency_hz/saturated_samples 等全部读不到。
 # 数值来源：编译器实测，见 scripts/check_struct_sizes.c 与 scripts/struct_sizes.txt
 #   sizeof(mpt_t485_t)=100, sizeof(output_position_test_t)=60,
 #   sizeof(output_position_t)=56, sizeof(output_position_trace_t)=16
 # ⚠️ 未改动 position=56 与 trace 长度 1616：这两处与固件 HEAD 既有定义不一致
 #   （TRACE_CAPACITY=81U → 数组仅 1296 字节，1616 会越界读 320 字节；
 #     且 commission_mpt_position.py:15 用的是 position=52，与本文件矛盾）。
 #   属既有缺陷、非本轮引入，已记录于调试日志待用户确认后另行修复，
 #   本轮不擅自扩大改动范围。
 abi=position_symbols(image,base.GDB)
 p.lengths.update(mpt=abi['mpt_size'],position=abi['position_size'],trajectory=abi['trajectory_size'],clock_source=4)
 result={'samples':{}}
 verified=False
 bounded_pending=False
 def snap(name,full=False):
  b,s=p.snapshot(name,full)
  m=b['mpt'];o=b['position']
  s.update(mpt_good=base.u32(m,12),mpt_errors=list(struct.unpack_from('<5I',m,16)),mpt_valid=base.u32(m,40),mpt_deg=base.f32(m,68),pos_state=base.u32(o,4),pos_fault=base.u32(o,8),pos_target=base.f32(o,20),pos_actual=base.f32(o,24),pos_error=base.f32(o,28),pos_rpm=base.f32(o,32))
  s.update(trajectory_state=base.u32(b['trajectory'],4),trajectory_fault=base.u32(b['trajectory'],8),trajectory_peak_error=base.f32(b['trajectory'],24),clock_source=base.u32(b['clock_source'],0));result['samples'][name]=s;print(name,json.dumps({k:s[k] for k in ['moe','run_state','speed_test_state','speed_test_fault','actual_rpm','mpt_deg','mpt_errors','pos_state','pos_fault','pos_error','deadline_overruns']}),flush=True)
  return b,s
 def writef(addr,x):return f'write32 0x{addr:x} {struct.unpack("<I",struct.pack("<f",x))[0]}'
 def healthy(s,baseline):
  for k in ['crc_errors','step_errors','start_errors','transfer_errors','deadline_overruns','pwm_window_rejections','pwm_deadline_rejections']:
   assert s[k]==baseline[k],k
  assert s['mpt_errors']==baseline['mpt_errors'] and s['mpt_valid']
  assert not s['speed_test_fault'] and not s['runtime_fault'] and not s['controller_fault']
 try:
  result['flash']=p.verify_flash();verified=True
  _,s=snap('before');base.require_off(s)
  assert s['calibration']==3 and s['mode']==[1,1] and s['stage']==[2,2]
  assert s['encoder_valid'][1:3]==[1,1] and not s['encoder_flags']
  assert not s['runtime_fault'] and not s['controller_fault']
  assert s['mpt_valid'] and not any(s['mpt_errors']), 'Communication must qualify before motion'
  assert 20<s['mpt_deg']<340
  requested=(root/'probe/before/test.bin').read_bytes()[16:64]
  bounded_pending=True
  p.command([f'write32 0x{a["test"]+16:x} 10',f'write32 0x{a["test"]:x} 1','sleep 4000'])
  _,s=snap('aligned',True);assert s['calibration_ok'] and s['speed_test_state']==3 and not s['moe'];healthy(s,result['samples']['before'])
  p.command([f'write32 0x{a["test"]+16+i:x} {base.u32(requested,i)}' for i in range(0,48,4)]+[f'write32 0x{a["test"]:x} 3','sleep 200'])
  bounded_pending=False
  _,start=snap('direction_before');base.require_ready(start)
  # Direction +1 already physically verified in mpt_position_20261026_175813.
  # Start with the small position trajectory instead of repeating kind-3 calibration.
  end=start;direction=1;result['motor_direction']=direction
  result['direction_evidence']='mpt_position_20261026_175813'
  p.command([f'write32 0x{a["test"]:x} 3','sleep 200',f'write32 0x{a["guard"]:x} 6000',p.heartbeat(),f'write32 0x{a["position"]+12:x} 1',f'write32 0x{a["position"]+16:x} {direction & 0xffffffff}',f'write32 0x{a["position"]:x} 1'])
  p.keepalive=True
  _,armed=snap('position_hold');healthy(armed,end);assert armed['pos_state']==1 and armed['run_state']==2 and armed['moe']
  origin=armed['pos_target'];result['origin_deg']=origin
  result['trials']=[]
  for amplitude in [10.0,45.0,90.0]:
   p.command([p.heartbeat(),writef(a['trajectory']+12,amplitude),f'write32 0x{a["trajectory"]:x} 1'])
   deadline=time.monotonic()+60
   i=0
   while True:
    blocks,s=snap(f'amp_{amplitude:g}_{i}');healthy(s,end)
    assert s['pos_state']==1 and not s['pos_fault'] and s['moe'] and s['run_state']==2
    assert s['trajectory_state'] in [1,2] and not s['trajectory_fault']
    assert abs(s['mpt_deg']-origin)<amplitude+3
    if s['trajectory_state']==2: break
    assert time.monotonic()<deadline,'MCU trajectory did not finish'
    i+=1
   trace=root/f'trace_{amplitude:g}.bin'
   count=base.u32(blocks['trajectory'],36)
   if not 0<count<=abi['trace_size']//16: raise RuntimeError('Invalid trace count')
   p.command([p.heartbeat(),f'savemem 0x{a["trace"]:x} {count*16} {trace.as_posix()}'])
   rows=[struct.unpack_from('<4f',trace.read_bytes(),j*16) for j in range(count)]
   import csv
   with (root/f'trace_{amplitude:g}.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['elapsed_s','target_deg','actual_deg','motor_rpm_command']);w.writerows(rows)
   result['trials'].append({'amplitude_deg':amplitude,'peak_error_deg':s['trajectory_peak_error'],'final_error_deg':s['pos_error'],'trace':str(trace)})
   assert s['trajectory_peak_error']<2,'Tracking error above 2 degrees'
   assert abs(s['pos_error'])<.1,'Return position has not settled'
  result['position_sine_passed']=True
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
