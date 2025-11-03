# Aerotech_hexapod
This is to generate part-speed PSO for flyscan. It uses epics, requiring IOC for your hexapod to be running. 
In ephex.py, the names of axes are defined in IOC's motor DESC. For example, 'Z' for m1.DESC.
# how to use
```python
from hexapod import Hexapod, IP
IOC_prefix = 'Your IOC prefix'
hp = Hexapod(IP)
hp.enable_tool() # 'either enable_tool or enable_work is needed whennever the controller power cycled'
hp.get_axes()
hp.get_pos()
hp.get_status()
hp.get_status_all()
hp.get_speed()
```
Before run a fly scan, run two commands below when the automation1 has been restarted.
```python
# Do this at least one time after the controller is reset
hp.fly_conf()
hp.set_pulsestream()
```
Then,
```python
# 1D fly scan
hp.set_traj('X', 0, 1, time=5)
# or for 2D fly scan
hp.set_traj(axis=['X', 'Z'], start=[0,0], final=[0.01, 0.01], Y_step = 0.000_5, time_per_line=1, pulse_step = 0.000_5, wait=True)
hp.run_traj()
```
