# Aerotech_hexapod
This is to generate part-speed PSO for flyscan. It uses epics, requiring IOC for your hexapod to be running. 
In ephex.py, the names of axes are defined in IOC's motor DESC. For example, 'Z' for m1.DESC.
# how to use
```python
from hexapod import Hexapod, IP
IOC_prefix = 'Your IOC prefix'
hp = Hexapod(IP)
hp.get_axes()
hp.get_pos()
hp.get_status()
hp.get_status_all()
hp.get_speed()
hp.set_traj('X', 0, 1, time=5)
hp.run_traj()
```
