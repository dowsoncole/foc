from pathlib import Path
import hashlib, json, shutil, subprocess, zipfile, sys, os

root=Path.cwd()
tag='speed-closed-loop-mt6701-20261026'
release=root/'software/releases'/tag
release.mkdir(parents=True,exist_ok=True)
os.environ['GIT_INDEX_FILE']=str(root/'tmp/mt6701_release.index')
subprocess.run(['git','read-tree','HEAD'],check=True)
project=root/'software/servo_v1'
paths=[]
for directory in ['Core','Drivers','FOC','Modules','Servo','Tests','MDK-ARM/docs']:
    paths.extend(p for p in (project/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.exe','.o','.pyc'])
paths.extend(p for p in project.iterdir() if p.is_file())
paths.extend(p for p in (project/'MDK-ARM').iterdir() if p.suffix in ['.uvprojx','.yml','.s','.ld','.sct'])
scripts=['run_mt6701_speed.py','mt6701_probe.py','commission_mt6701_closed_loop.py','test_highspeed_closed_loop.py','test_encoder_neutral_pwm.py','verify_encoder_artifacts.py','test_highspeed_closed_loop_checks.py','test_mt6701_ssi_driver.py']
paths.extend(root/'software/control_analysis/scripts'/p for p in scripts)
for p in paths:
    out=release/'source'/p.relative_to(root)
    out.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(p,out)
subprocess.run(['git','add','--pathspec-from-file=-','--pathspec-file-nul'],input=b'\0'.join(str(p).encode('utf-8') for p in paths)+b'\0',check=True)
for ext in ['axf','hex','map']:
    out=release/'firmware'/('servo_v1.'+ext)
    out.parent.mkdir(exist_ok=True)
    shutil.copy2(project/'MDK-ARM/servo_v1'/out.name,out)
evidence=release/'evidence';evidence.mkdir(exist_ok=True)
names=['mt6701_unlimited_retry_20261026','mt6701_unlimited_7800_hold_20261026','mt6701_unlimited_7900_hold_20261026']
for name in names:
    shutil.copy2(root/'software/control_analysis/results'/name/'summary.json',evidence/(name+'.json'))
shutil.copy2(root/'software/control_analysis/results'/names[-1]/'flash.bin',release/'firmware/servo_v1.bin')
sys.path.insert(0,str(root/'software/control_analysis/scripts'))
from verify_encoder_artifacts import load_image
image=load_image(release/'firmware/servo_v1.axf')
print(type(image))
expected=json.loads((evidence/(names[-1]+'.json')).read_text())['flash_sha256']
assert hashlib.sha256((release/'firmware/servo_v1.bin').read_bytes()).hexdigest()==expected
doc=f'''# 速度闭环 MT6701 — 2026-10-26 实测版本

版本标签：`{tag}`。文档、源码快照、固件与实测记录随同一个 Git 标签发布；以该标签定位提交，不使用后续 main 的文件替代。本目录 ZIP 是上述文件的配套包，manifest.json 列出文件 SHA-256。

## 最高转速实测

|指令 rpm|实际采样 rpm|保持时间|结果|
|---|---|---|---|
|7800|7775.65～7824.77，平均 7803.02|30.032 秒|通过|
|7900|7840.04～7898.64，平均 7872.90|31.25 秒|通过 ±1% 跟踪检查|
|8000|约 7772～7819|未满足保持条件|跟踪失败，脚本关闭输出|

本轮已验证持续实际转速约 **7873 rpm**，最高采样值 **7899 rpm**。8000 rpm 未通过；这是当前电源、负载、调制和控制参数下的测试结果，不是电机绝对极限。三次试验为不同运行时段，不能将它们视作同一瞬间的连续曲线。测速来自 MT6701 和顺序 RAM 采样，没有独立转速计验证。

CRC、SPI 启动/传输、角度跳变、编码器报警和控制截止时间超限新增计数全部为 0。7900/8000 rpm 附近输出电压饱和，当前表现为电压余量约束。全部试验结束后 MOE=0，驱动输出关闭，运行故障为 0。

## 绑定的固件与配置

- Flash BIN SHA-256：`{expected}`，长度 67328 字节；测试前与板上 Flash 逐字节比对。
- 固件文件：firmware/servo_v1.axf、servo_v1.hex、servo_v1.bin、servo_v1.map。
- MT6701 SSI：PB6 CS、PB3 CLK、PB4 MISO/SO、PB5 dummy MOSI，MODE 高；SPI1 Mode 1，24 位帧，1.3125 MHz，CRC6 校验。
- HSI 内部振荡器经 PLL 得到 168 MHz；编码器参与 10 kHz 控制采样，14 位角度归一化到原有 21 位反馈接口。
- 连续速度运行不设置有限的 Iq 软件幅值上限（FLT_MAX 数值哨兵）；实测 current_reference_limited 始终为 false。I²t 可选模式未启用；保留原有非电流故障保护与主机心跳。
- 当前线性 SVPWM 电压上限 0.56 pu，过调制未启用。更换 PCB 后电流零点 U=1893、V=1892。
- **上电默认仍为电压开环 Ud=1 V、10 Hz、自动输出；速度闭环不会上电自动启动。** 此版本名称表示已验证 MT6701 速度闭环功能，不代表修改了上电运行模式。
- 闭环实测先关闭输出，执行方向检查及固定 1.5 V/3 秒对齐，再启动速度闭环；测试产生的电角偏置是运行时校准值，复位后需要重新对齐。

## 工程与验证

工程入口：source/software/servo_v1/MDK-ARM/servo_v1.uvprojx。保留实测固件，不因打包重新烧录或启动电机。构建环境为 Keil ARM，STM32G4xx_DFP 2.2.0。

本版本前序验证：Keil 构建 0 错误 0 警告；MT6701 SSI 主机测试与速度控制主机测试通过。旧编码器传输测试仍存在 PD2 时序脉冲断言与当前 CRC 错误锁存用途不一致的问题，不能声称全部历史测试通过。

调试脚本位于 source/software/control_analysis/scripts，使用 pyOCD/ST-Link；脚本内保留实测环境路径和探针 ID，换电脑应检查配置。--run 会驱动实际硬件。实测原始 summary.json 在 evidence/，保存完整配置、各阶段采样和最终 OFF 状态。
'''
(release/'README.md').write_text(doc,encoding='utf-8')
manifest={str(p.relative_to(release)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(release.rglob('*')) if p.is_file()}
(release/'manifest.json').write_text(json.dumps({'version':tag,'files':manifest},ensure_ascii=False,indent=2),encoding='utf-8')
archive=release.parent/(tag+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(release.rglob('*')):
        if p.is_file():z.write(p,Path(tag)/p.relative_to(release))
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for rel,digest in manifest.items(): assert hashlib.sha256(z.read(tag+'/'+rel)).hexdigest()==digest
# Source snapshot lives in the ZIP; Git stores normal source paths plus release metadata and binaries.
subprocess.run(['git','add','--',str(archive),str(release/'README.md'),str(release/'manifest.json'),str(release/'firmware'),str(evidence)],check=True)
print('PACKAGED',archive,archive.stat().st_size,'bytes',len(manifest),'verified files')
