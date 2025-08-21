# Aerotech_hexapod
This is to generate part-speed PSO for flyscan. It uses epics, requiring IOC for your hexapod to be running. The names of axes are defined in IOC's motor DESC. For example, 'Z' for m1.DESC.
# how to use
```python
import AThexapod as hp
IOC_prefix = 'Your IOC prefix'
hp.update_motor_dict()
hp.fly('your axis name (DESC)', 0, 1, time=5)
```
