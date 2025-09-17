import automation1 as a1
import epics
testequip = "PI"
if testequip == "Aerotech":
    from hexapod import Hexapod, IP
    hp = Hexapod(IP)
    FN = "Aerotech_test"

if testequip == "PI":
    from pihexapod.gcs import Hexapod, plot_record, IP, WaveGenID
    pihp = Hexapod(IP)
    FN = "PI_hexapod_test"

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

dg645_12ID.set_pilatus_fly(60, trigger_source=0)
dg645_12ID.trigger()
s12softglue = sgz_pty()
basename = '12idSGSocket:'

epics.caput("12idSGSocket:HDF1:FilePath", "/home/12id-c")
epics.caput("12idSGSocket:HDF1:FileName", FN)
det = SG(basename)
s12softglue.set_count_freq(10)
#s12softglue.ckTime_reset()
s12softglue.memory_clear()


#struck.mcs_init()
#struck.mcs_ready(5000, 100)
#struck.arm_mcs()

if testequip == "Aerotech":
    ## Aerotech
    hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[0.01, 0.01], Y_step = 0.000_5, time_per_line=1, pulse_step = 0.000_5, wait=True)
    #hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[1, 1], Y_step = 0.05, time_per_line=1, pulse_step = 0.05, wait=True)
    det.fly_ready(0.001, 10, 10)
    time.sleep(0.1)
    hp.run_traj()

if testequip == "PI":
    ### PI
    pihp.set_traj_SNAKE(1, 0, 0.01, 0, 0.01, 0.000_5, 0.000_5)
    det.fly_ready(0.001, 10, 10)
    time.sleep(0.1)
    pihp.run_traj(["X", "Z"])
    isdone = False
    while not isdone:
        p = pihp.isattarget()
        isdone = p["X"]

time.sleep(5)
#s12softglue.flush()

det.FileCaptureOff()
det.Acquire = 0
