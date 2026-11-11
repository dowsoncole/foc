"""Offline documentation QA; no probe or hardware tools are invoked."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote

root=Path.cwd()
names=['README.md','software/control_analysis/速度环快速响应_用户说明.md','software/docs/位置环调试与使用说明.md',
       'software/docs/架构说明.md','software/docs/编码器结构体与软件分层说明.md',
       'software/docs/log/README.md','software/docs/log/2026-10-29_位置环续调与反馈链路检查.md',
       'software/docs/log/2026-10-28_位置环实机扫频记录.md',
       'software/docs/log/2026-10-28_位置环带宽优化调研与可行性分析.md']
failures=[];links=0
for name in names:
    path=root/name; text=path.read_bytes().decode('utf-8')
    if '\ufffd' in text:failures.append(f'UTF-8 replacement character: {name}')
    if len(re.findall(r'^```',text,re.M))%2:failures.append(f'Unbalanced code fences: {name}')
    for target in re.findall(r'!?\[[^\]\n]*\]\(([^)\n]+)\)',text):
        target=target.strip('<>')
        if re.match(r'[A-Za-z][\w+.-]*://',target) or target.startswith('#'):continue
        target=unquote(target.split('#',1)[0])
        if not target:continue
        links+=1
        if not (path.parent/target).exists():failures.append(f'Broken link: {name} -> {target}')

script=root/'software/control_analysis/scripts/measure_position_response.py'
tree=ast.parse(script.read_text(encoding='utf-8'))
defaults={}
for node in ast.walk(tree):
    if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='add_argument':
        flags=[arg.value for arg in node.args if isinstance(arg,ast.Constant)]
        for keyword in node.keywords:
            if keyword.arg=='default':
                for flag in flags:defaults[flag]=ast.literal_eval(keyword.value)
assert defaults['--kp']==80 and defaults['--max-motor-rpm']==5500
assert defaults['--max-phase-peak']==defaults['--max-phase-rms']==0
help_result=subprocess.run([sys.executable,'-B',str(script),'--help'],capture_output=True,text=True,check=True)
assert '--compare-feedforward' in help_result.stdout

base=root/'software/control_analysis/results/position_tuning_20261029'
profile=json.loads((base/'loaded_position_recipe.json').read_text())
assert hashlib.sha256((base/'flash_r17/firmware.axf').read_bytes()).hexdigest()==profile['firmware_sha256']
assert (base/'flash_r17/verification/verified_flash.bin').stat().st_size==profile['flash_verified_bytes']==88080
sys.path.insert(0,str(root/'software/control_analysis/scripts'))
from verify_encoder_artifacts import load_image,hex_image
image=load_image(base/'flash_r17/firmware.axf')
expected=bytes(image[address] for address in range(min(image),max(image)+1))
assert (base/'flash_r17/verification/verified_flash.bin').read_bytes()==expected
assert hex_image(base/'flash_r17/firmware.hex')==image
assert profile['ordinary']==dict(kp_per_s=72,max_motor_rpm=5500,ramp_rpm_s=300000,deadband_deg=.0055)
assert 18.52<profile['best_feedback_bandwidth']['bandwidth_3db_hz']<18.53
assert abs(profile['step']['settling_2pct_s']-.055)<1e-6
assert not profile['final_off']['bdtr'][0]&0x8000
for key in ('runtime_fault','foc_fault'):assert profile['final_off'][key]==[0]
assert '.kp_per_s=72.0f' in (root/'software/servo_v1/Servo/Src/output_position.c').read_text(encoding='utf-8')
header=(root/'software/servo_v1/Modules/Encoder/Inc/mpt_t485.h').read_text(encoding='utf-8')
assert '#define OUTPUT_POSITION_PERIOD_MS 1U' in header
symbols=json.loads((base/'r17_final_off/symbols.json').read_text())
assert {k:symbols[k] for k in ('mpt_size','position_size','trajectory_size','frd_size','trace_size')}==dict(mpt_size=108,position_size=56,trajectory_size=84,frd_size=40,trace_size=1024)
for name in ('software/docs/架构说明.md','software/docs/编码器结构体与软件分层说明.md'):
    text=(root/name).read_text(encoding='utf-8')
    for stale in ('200 Hz','2500 rpm','5 ms 查询','5 ms 更新','trace[101]'):
        if stale in text:failures.append(f'Stale current documentation: {name}: {stale}')
result=dict(files=len(names),local_links_checked=links,failures=failures,
            cli_defaults=defaults,firmware_sha256=profile['firmware_sha256'],
            evidence_checked=True,hardware_access=False)
(root/'tmp/position_docs_validation_20261029.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
if failures:raise SystemExit(1)
