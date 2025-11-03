from hexapod import Hexapod, IP
hp = Hexapod(IP)
import automation1 as a1
import time
#from tools.panda import get_pandadata
import sys
sys.path.append('..\ptychoSAXS')
from tools.softglue import sgz_pty, SG, SOFTGLUE_Setup_Error
from tools import struck

s12softglue = sgz_pty()
basename = '12idSGSocket:'
det = SG(basename)

if s12softglue.isConnected:
    s12softglue.set_count_freq(100)
else:
    print("Softglue does not work.")

s12softglue.set_count_freq(10)
s12softglue.ckTime_reset()
s12softglue.memory_clear()
det.fly_ready(0.001, 10, 10)
#struck.mcs_init()
#struck.mcs_ready(5000, 100)
#struck.arm_mcs()

hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[0.01, 0.01], Y_step = 0.000_5, time_per_line=1, pulse_step = 0.000_5, wait=True)
#hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[1, 1], Y_step = 0.05, time_per_line=1, pulse_step = 0.05, wait=True)
hp.run_traj()

s12softglue.flush()

det.FileCaptureOff()
det.Acquire = 0
