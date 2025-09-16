import automation1 as a1
from hexapod import Hexapod, IP
hp = Hexapod(IP)

from pihexapod.gcs import Hexapod, plot_record, IP, WaveGenID
pihp = Hexapod(IP)

import time
#from tools.panda import get_pandadata
import sys
sys.path.append('..\ptychoSAXS')
from tools.softglue import sgz_pty, SG, SOFTGLUE_Setup_Error
from tools import struck

#Delay generator
import tools.dg645 as dg645
from tools.dg645 import DG645_Error
try:
    dg645_12ID = dg645.dg645_12ID.open_from_uri(dg645.ADDRESS_12IDC)
except:
    print("failed to connect DG645. Will not be able to collect detector images")

#dg645_12ID.set_pilatus_fly(0.001)
s12softglue = sgz_pty()
basename = '12idSGSocket:'
det = SG(basename)

if s12softglue.isConnected:
    s12softglue.set_count_freq(100)
else:
    print("Softglue does not work.")

#s12softglue.set_count_freq(10)
#s12softglue.ckTime_reset()
s12softglue.memory_clear()
det.fly_ready(0.001, 10, 10)

#struck.mcs_init()
#struck.mcs_ready(5000, 100)
#struck.arm_mcs()

## Aerotech
#hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[0.01, 0.01], Y_step = 0.000_5, time_per_line=1, pulse_step = 0.000_5, wait=True)
#hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[1, 1], Y_step = 0.05, time_per_line=1, pulse_step = 0.05, wait=True)
#hp.run_traj()

### PI
pihp.set_traj_SNAKE(1, 0, 0.01, 0, 0.01, 0.000_5, 0.000_5)
pihp.run_traj()
s12softglue.flush()

det.FileCaptureOff()
det.Acquire = 0
