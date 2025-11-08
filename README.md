# Aerotech_hexapod
This is to generate part-speed PSO for flyscan. It uses epics, requiring IOC for your hexapod to be running. 
In ephex.py, the names of axes are defined in IOC's motor DESC. For example, 'Z' for m1.DESC.
# how to use
```python
from hexapod import Hexapod, IP, a1
IOC_prefix = 'Your IOC prefix'
hp = Hexapod(IP)
hp.enable_tool() # enabling tool mode. 
hp.get_axes()
hp.get_pos()
hp.get_status()
hp.get_status_all()
hp.get_speed()
```
Before run a fly scan, run two commands below when the automation1 has been restarted.
```python
hp.fly_conf()
hp.set_pulsestream()
```
Then,
```python
hp.set_traj('X', 0, 1, time=5)
hp.run_traj()
```

Save a tuning result or control parameters
```python
a1.save_servo_parameters(hp.controller)
```
will save 'servo_params_ST1.txt'.

Changing tuning parameters to the saved ones.
For example, 'servo_params_ST1_normal.txt' was recorded after easytuning with the hefty rotation stage.
```python
a1.set_servo_parameters(hp.controller, 'servo_params_ST1_normal.txt')
```

If you would like to inspect the parameters
```python
params = a1.load_servo_parameters(hp.controller, 'servo_params_ST1_normal.txt')
```
or
```python
a1.list_servo_parameters(hp.controller)
```