from pathlib import Path
s=Path('software/control_analysis/scripts/test_mt6701_ssi_driver.py').read_text()
c=s[s.index('class Probe('):s.index('\np=Probe(')]
Path('software/control_analysis/scripts/mt6701_probe.py').write_text('import os,sys,time,json,subprocess,re,shutil,struct,hashlib\nfrom pathlib import Path\nos.environ["SERVO_PYOCD"]="C:/Users/penghao/.vscode/extensions/arm.vscode-cmsis-debugger-1.10.0-win32-x64/tools/pyocd/pyocd.exe"\nfrom test_highspeed_closed_loop import ProbeRun,base\n'+c)
