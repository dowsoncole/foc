import sys
from pathlib import Path
sys.path.insert(0,str(Path('software/control_analysis/scripts').resolve()))
from mt6701_probe import Probe
import test_highspeed_closed_loop as speed
speed.ProbeRun=Probe
original_check=speed.check_running
def check_unlimited(state,*args,**kwargs):
    original_check(state,*args,**kwargs)
    if state['current_limit_a'] < 3e38 or state['current_reference_limited']:
        raise RuntimeError('Software current reference limit is active; not an unlimited-Iq trial')
speed.check_running=check_unlimited
speed.main()
