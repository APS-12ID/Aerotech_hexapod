import automation1 as a1
import threading
import numpy as np
import time
import ast

IP = "at-hex-12id.xray.aps.anl.gov"
BASEPV = '12idhxpAT'
HOST = "at-hex-12id.xray.aps.anl.gov"
period = 0.000_1  # Default period for PSO in seconds
pulse_width = 0.000_1  # Default pulse width in seconds
step_distance = 0.01  # Default distance for PSO in mm
sampling_freq = 1000 # 100Hz 
class Hexapod:
    """A class to use pipython"""
    step_distance = step_distance
    pulse_width = pulse_width
    period = period
    def __init__(self, IPorBasePV=IP):
        _direct_connection_needed = False
        if len(IPorBasePV) == 0: # will use InterfaceSetupDlg
            _direct_connection_needed = True
        else:
            _m = IPorBasePV.split('.')
            if len(_m)>3:
                _direct_connection_needed = True
        if _direct_connection_needed:
            self.controller = a1.Controller.connect(host=IP)
            self.isEPICS = False
        else:
            import ephex as hp
            self.controller = hp
            self.isEPICS = True
            print("C887 is connected through EPICS. Check wth .controller.")

        #self.mycs = UserCS
        self.axes = ['X', 'Y', 'Z', 'A', 'B', 'C']
        self.wave_start = {'X':0, 'Y':0, 'Z':0, 'U':0, 'V':0, 'W':0}
        self.lock = threading.Lock()
        self.command_queue = None
    
    def disconnect(self):
        if self.isEPICS == False:
            self.controller.disconnect()
    
    def connect(self):
        self.controller.connect(host=IP)

    def set_UserDefaultCSname(self, CS):
        self.mycs = CS

    def get_axes(self, startN = 6):
        self.axes = []
        for i, ax in enumerate(self.controller.runtime.parameters.axes):
            if i<startN:
                continue
            self.axes.append(ax.identification.axisname.value)

    def get_pos(self):
        # DriveStatus is a series of bits that can be masked. You will use it to get the axis "enabled" bit.
        # AxisStatus is a series of bits that can be masked. You will use it to get the axis "homed" bit.
        pos = []
        status_item_configuration = a1.StatusItemConfiguration()
        for ax in self.axes:
            status_item_configuration.axis.add(a1.AxisStatusItem.ProgramPosition, ax)
            #status_item_configuration.axis.add(a1.AxisStatusItem.DriveStatus, "X")
            #status_item_configuration.axis.add(a1.AxisStatusItem.AxisStatus, "X")
            #status_item_configuration.axis.add(a1.AxisStatusItem.AxisFault, "X")
        results = self.controller.runtime.status.get_status_items(status_item_configuration)
        for ax in self.axes:
            pos.append(results.axis.get(a1.AxisStatusItem.ProgramPosition, ax).value)
        return pos
    
    def mv(self, **kwargs):
        # Move to absolute position. Speed is set to 10 mm/s
        # Example: mv(X=1, Y=2)
        for key, value in kwargs.items():
            if key in self.axes:
                self.controller.runtime.commands.motion.moveabsolute(key, [value], [10])
    
    def get_status_all(self):
        # DriveStatus is a series of bits that can be masked. You will use it to get the axis "enabled" bit.
        # AxisStatus is a series of bits that can be masked. You will use it to get the axis "homed" bit.
        status_all = {}
        status = {}
        # _status = {'enabled': a1.DriveStatus.Enabled, 
        #         #'inposition': a1.DriveStatus.InPosition, 
        #         #'moving': a1.DriveStatus.MoveActive, 
        #         'homed': a1.AxisStatus.Homed}
        #         #'motiondone' : a1.AxisStatus.MotionDone, 
        #         #'pos': a1.AxisStatusItem.ProgramPosition}
        _status = {}
        AxisStatusItems = a1.AxisStatusItem.__dict__.keys()
        for item in AxisStatusItems:
            if "_" not in item:
                _status[item] = getattr(a1.AxisStatusItem, item)
        status_item_configuration = a1.StatusItemConfiguration()
        for ax in self.axes:
            for key in _status.keys():
                status_item_configuration.axis.add(_status[key], ax)
        results = self.controller.runtime.status.get_status_items(status_item_configuration)
        for ax in self.axes:
            status = {}
            for key in _status.keys():
                s = results.axis.get(_status[key], ax)
                print(key, s)
                try:
                    status[key] = s.value
                except:
                    status[key] = s
                #print(key, _status[key], s.value, ax)
            status_all[ax] = status
        return status_all

    def get_status(self):
        status_all = {}
        status = {}
        AxisStatusItems = ["DriveStatus", "AxisStatus", "AxisFault"]
        _status = {'enabled': [0, a1.DriveStatus.Enabled], 
                'inposition': [0, a1.DriveStatus.InPosition], 
                'moving': [0, a1.DriveStatus.MoveActive], 
                'homed': [1, a1.AxisStatus.Homed],
                'motiondone' : [1, a1.AxisStatus.MotionDone],
                'fault': [2, a1.AxisStatusItem.AxisFault]}
        _status_ = {}
        for item in AxisStatusItems:
            if "_" not in item:
                _status_[item] = getattr(a1.AxisStatusItem, item)
        status_item_configuration = a1.StatusItemConfiguration()
        for ax in self.axes:
            for key in _status_.keys():
                status_item_configuration.axis.add(_status_[key], ax)
        results = self.controller.runtime.status.get_status_items(status_item_configuration)

        for ax in self.axes:
            status_items = []
            for item in AxisStatusItems:
                #print(item, results.axis.get(_status_[item], ax))
                status_items.append(int(results.axis.get(_status_[item], ax).value))
            status = {}
            for key in _status.keys():
                itemindex = _status[key][0]
                itemenum = _status[key][1]
                status[key] = bool(status_items[itemindex] & itemenum)
            status_all[ax] = status
        return status_all

    def isattarget(self, axis=""):
        st = self.get_status()
        if len(axis)==1:
            return st[axis]['inposition']
        axis = self.axes
        status = {}
        for ax in axis:
            if ax in st.keys():
                status[ax] = st[ax]['inposition']
        return status

    def get_speed(self, axis = ""):
        # returns speed in mm/s 
        speed = [0]*len(self.axes)
        for i, ax in enumerate(self.controller.runtime.parameters.axes):
            axisname = ax.identification.axisname.value
            if axisname in self.axes:
                sp = ax.motion.defaultaxisspeed.value
                if axisname == axis:
                    return sp
                speed[self.axes.index(axisname)] = sp
        return speed
    
    def set_speed(self, val):
        # unit of the speed is mm/s 
        for i, ax in enumerate(self.controller.runtime.parameters.axes):
            axisname = ax.identification.axisname.value
            if axisname in self.axes:
                if type(val) == list:
                    ax.motion.defaultaxisspeed.value = val[i]
                else:
                    ax.motion.defaultaxisspeed.value = val

    def enable_all_axes(self):
        self.controller.runtime.commands.motion.enable([0, 1,2,3,4,5,6,7,8,9,10,11])

    def disable_all_axes(self):
        self.controller.runtime.commands.motion.disable([0, 1,2,3,4,5,6,7,8,9,10,11])

    def set_work(self, xoff=0, yoff=0, zoff=0, aoff=0, boff=0, coff=0):
        # disable the real-time hexapod kinematics. 
        ''' After you issue the DisableHexapod() library function, you get full control of each strut. Strut axes are
        named ST1 through ST6. Until you reset or power cycle the controller, it stores all the information
        from previous tool and offset settings.'''
        self.controller.runtime.commands.execute("DisableHexapod()")
        cmd = f"SetBaseToWork({xoff},{yoff},{zoff},{aoff},{boff},{coff})"
        self.controller.runtime.commands.execute(cmd)
    
    def enable_work(self):
        self.controller.runtime.commands.execute("EnableWork()")

    def set_tool(self, tool="Tool1", xoff=0, yoff=0, zoff=0, aoff=0, boff=0, coff=0):
        self.controller.runtime.commands.execute("DisableHexapod()")
        cmd = f'SetToolPoint(1, "{tool}", {xoff},{yoff},{zoff},{aoff},{boff},{coff})'
        self.controller.runtime.commands.execute(cmd)
        cmd = f'ActivateTool("{tool}")'
        self.controller.runtime.commands.execute(cmd)
        self.controller.runtime.commands.motion_setup.setuptasktargetmode(a1.TargetMode.Absolute)
    
    def enable_tool(self):
        self.controller.runtime.commands.execute("EnableTool()")

    def fly_abort(self):
        
        # At this time, end the command queue.
        # You can also call CommandQueue.end_command_queue to abort the command
        # that is currently executing and discard the remaining commands.
        try:
            self.controller.runtime.commands.end_command_queue(self.command_queue)
            self.command_queue = None
        except:
            print("There is no running fly yet. So, will make 'Task3' Idle.")
            self.controller.runtime.tasks[3].program.stop()

    def set_pulsestream(self):
        self.controller.runtime.commands.device.drivepulsestreamoff('ST1') # ST1 is the real axis issuing PSO firing pulses
        self.controller.runtime.commands.execute('DrivePulseStreamConfigure(ST1, [X, Y, Z], [1.0, 1.0, 1.0])') # X/Y/Z are the virtual axes being tracked. ST1 actually fires PSO, scale is 1.0
        self.controller.runtime.commands.device.drivepulsestreamon('ST1') # Turns on ST1's pulse stream input

    def fly_conf(self):
        self.controller.runtime.commands.pso.psoreset('ST1') #Reset all PSO configuration
        #self.controller.runtime.commands.device.drivepulsestreamoff('ST1') # ST1 is the real axis issuing PSO firing pulses
        self.controller.runtime.commands.pso.psodistanceconfigureinputs('ST1', [a1.PsoDistanceInput.iXR3DrivePulseStream]) # Fire based on virtual counts from the iXR3 pulse stream
        self.controller.runtime.commands.pso.psowaveformconfiguremode('ST1', a1.PsoWaveformMode.Pulse)
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedcount('ST1', 1) # 1 pulse (period) per event
        self.controller.runtime.commands.pso.psowaveformon('ST1') # Turn on waveform generator

    def goto_start_pos(self, axis='X'):
        default_speed = 5
        # turn off pso
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1')
        #global motor_dict
        self.controller.runtime.commands.motion.moveabsolute(axis, [self.start_pos], speeds=[default_speed]) # Move axis to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)

    # def set_traj(self, axis = "X", time=5, start = 0, final=5, step_distance=0.01):
    #     self.start_pos = start
    #     self.goto_start_pos(axis)

    #     N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
    #     speed = abs(final-start)/time
    #     print(f"Fly to {final} in {time} seconds with {speed} mm/s.") 
    #     print(f"PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")

    #     self.set_pulses(distance = step_distance, period=period, pulse_width=pulse_width)
    #     self.pulse_number = N_pulses
    #     self.pulse_step = time/N_pulses # pulse time step in seconds
    #     self.scantime = time


    #     self.controller.runtime.commands.pso.psowaveformon('ST1') # Turn on waveform generator
    #     self.controller.runtime.commands.pso.psodistancecounteron('ST1') # Enable the distance counter


    #     # position units : mm
    #     #final = start + distance
    #             # Begin a new command queue on task 1.
    #     if type(self.command_queue) ==type(None):
    #         if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
    #             self.controller.runtime.tasks[3].program.stop()
    #             print("Task 3 was in QueueRunning mode. It is stopped and QueueRunning restarted.")
    #         self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 10, True)
        
    #     if not self.command_queue.status.is_paused:
    #         # First, pause the command queue so you can add all the commands
    #         # before they are executed.
    #         self.command_queue.pause()

    #     # Add all the AeroScript commands that you want to execute.
    #     #self.command_queue.commands.advanced_motion.velocityblendingon()
    #     self.command_queue.commands.pso.psodistanceeventson('ST1') # Turn on PSO
    #     self.command_queue.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
    #     self.command_queue.commands.motion.waitformotiondone(axis)
    #     #self.command_queue.commands.pso.psooutputoff('ST1') # Turn off PSO 
    #     #self.command_queue.commands.advanced_motion.velocityblendingoff()
        

    # def set_traj_SNAKE(self, time_per_line = 5, Xi = -2.5, X_distance=1, Yi = 0, Yf = 1, Y_step = 0.1, pulse_step=0.1):
    #     if type(self.command_queue) ==type(None):
    #         if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
    #             self.controller.runtime.tasks[3].program.stop()
    #             print("Task 3 was in QueueRunning mode. It is stopped and QueueRunning restarted.")
    #         self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 10, True)
        
    #     if not self.command_queue.status.is_paused:
    #         # First, pause the command queue so you can add all the commands
    #         # before they are executed.
    #         self.command_queue.pause()

    #     # Add all the AeroScript commands that you want to execute.
    #     self.command_queue.commands.advanced_motion.velocityblendingon()
    #     N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
    #     speed = abs(final-start)/time
    #     print(f"Fly to {final} in {time} seconds with {speed} mm/s.") 
    #     print(f"PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
    #     self.command_queue.commands.pso.psodistanceeventson('ST1') # Turn on PSO
    #     Y = Yi
    #     direction = np.sign(X_distance)
    #     count = 0
    #     X = Xi
    #     while Y <= Yf:
    #         final = X + (-1)^count*X_distance
    #         self.command_queue.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
    #         self.command_queue.commands.motion.waitformotiondone(axis)
    #         iscw = True
    #         # when Y direction is positive (up direction)
    #         if final > X:
    #             iscw = False
    #         else:
    #             iscw = True
    #         # when Y direction is negative (down direction)
    #         if Y_step < 0:
    #             iscw = not iscw
    #         center = [final, Y + Y_step/2]
    #         self.command_queue.commands.motion.arcmove(["X", "Y"], [final, Y + Y_step], center, speed)
    #         self.command_queue.commands.motion.waitformotiondone(["X", "Y"])
    #         count += 1
    #         Y = Y + Y_step
    #         X = final
    #     self.command_queue.commands.pso.psooutputoff('ST1') # Turn off PSO 
    #     self.command_queue.commands.advanced_motion.velocityblendingoff()

    def check_task_status(self):
        if self.controller.runtime.tasks[2].status.error == 57000:
            print("There was 57000 error on Task2. Resetting the controller")
            self.controller.reset()
            print("Reset done. All axes will be enabled.")
            self.enable_all_axes()
            print("All enabled. Will put WorkMode on, which will home all axes. This may take a while.")
            self.enable_work()
        else:
            print("Task2 is good to go.")

    # def run_traj(self, axis="X",  wait=True):
    #     '''Issue a PSO motion command to the hexapod.
    #     This will move the hexapod to the specified X, Y, Z coordinates
    #     using the configured PSO settings.
    #     '''

    #     # Resume the command queue so that all the commands that you added start
    #     # to execute.
    #     self.command_queue.resume()
    #     #     # Here you can do other things such as more process, get status, etc.
    #     #     # You can do these things because the command queue is executing
    #     #     # commands on the controller and is not blocking your code execution.
    #     if wait:
    #         # Here you wait to make sure that the command queue executes all the commands.
    #         # You must do this before you end the command queue.
    #         # When you end the command queue, this process aborts all motion and commands.
    # #        print("Waiting to be done.")
    #         self.controller.runtime.commands.motion.waitformotiondone(axis)
    #         self.command_queue.wait_for_empty()

    def turn_off_pso(self):
        self.controller.runtime.commands.pso.psooutputoff('ST1') # Turn off PSO 

    def set_pulse(self, axis = 'X', step_distance=0, period=0, pulse_width = 0):

        #self.enable_work()
#        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1')
#        self.controller.runtime.commands.motion.moveabsolute(axis, [0], speeds=[5]) # Move axis to the specified position
#        self.controller.runtime.commands.motion.waitformotiondone(axis)

        if step_distance != 0:
            self.step_distance = step_distance
        if period != 0:
            self.period = period
        if pulse_width != 0:
            self.pulse_width = pulse_width
        #speed = 1
        countsperunit = self.controller.runtime.parameters.axes[axis].units.countsperunit.value
        pulse_distance = round(countsperunit*self.step_distance)
        period_us = self.period*1_000_000
        pulsewidth_us = self.pulse_width*1_000_000
        #print(pulse_distance, ' pulse distance ', pulsewidth_us, ' period')

        self.controller.runtime.commands.pso.psodistanceconfigurefixeddistance('ST1', pulse_distance)
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedtotaltime('ST1', period_us) # usec total time
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedontime('ST1', pulsewidth_us) # usec
        self.controller.runtime.commands.pso.psowaveformapplypulseconfiguration('ST1')# Applies the previous pulse configuration
        self.controller.runtime.commands.pso.psooutputconfigureoutput('ST1', a1.PsoOutputPin.XR3PsoOutput1)
        self.controller.runtime.commands.pso.psooutputconfiguresource('ST1', a1.PsoOutputSource.Waveform) # Use waveform module output as PSO output
        self.controller.runtime.commands.pso.psodistancecounteron('ST1') # Enable the distance counter
        self.controller.runtime.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        
    def run_traj_command_queue(self, axis='X', start=1, final=0, time=1, wait=True):
        N_pulses = int(abs(final - start) / self.step_distance)  # Calculate the number of pulses to fire
        speed = abs(final-start)/time
        self.N_pulses = N_pulses
        self.speed = speed
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1') # Turn on PSO
        self.controller.runtime.commands.motion.moveabsolute(axis, [start], [5]) # Move ST1 to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)
        if type(self.command_queue) ==type(None):
            if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
                self.controller.runtime.tasks[3].program.stop()
        try:
            self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 10, True)
        except:
            pass
        self.command_queue.pause()
        self.command_queue.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        self.command_queue.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
        self.command_queue.commands.motion.waitformotiondone(axis)
        self.command_queue.resume()
        if wait:
            self.controller.runtime.commands.motion.waitformotiondone(axis)
                    
    def run_traj_1D(self, axis='X', start=1, final=0, time=1, wait=True):
        N_pulses = int(abs(final - start) / self.step_distance)  # Calculate the number of pulses to fire
        speed = abs(final-start)/time
        self.N_pulses = N_pulses
        self.speed = speed
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1') # Turn on PSO
        self.controller.runtime.commands.motion.moveabsolute(axis, [start], [5]) # Move ST1 to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)
        self.controller.runtime.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        self.controller.runtime.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
        if wait:
            self.controller.runtime.commands.motion.waitformotiondone(axis)

        
    def set_traj(self, axis=['X', 'Z'], start=[1,0], final=[0, 1], Y_step = 0.01, time_per_line=1, pulse_step = 0.001, wait=True):
        is2D = False
        if type(axis) == type(['X']): # 2D
            xi = start[0]
            xf = final[0]
            yi = start[1]
            yf = final[1]
            defaultspeed = [10, 10]
            N_lines = abs(yf-yi)/abs(Y_step)+1
            is2D = True
        else:
            xf = final
            xi = start
            start = [start]
            defaultspeed = [10]
            N_lines = 1
        X_distance = xf-xi
        speed = abs(X_distance)/time_per_line
        self.speed = speed
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1') # Turn on PSO
        self.controller.runtime.commands.motion.moveabsolute(axis, start, defaultspeed) # Move ST1 to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)

        self.set_pulse(step_distance=pulse_step)

        # data collection started
        if wait:
            num_datapoints = int(sampling_freq*time_per_line*N_lines)
            print(f"Number of data collection points is {num_datapoints}")
            data_config = self.set_datacollection(axis, num_datapoints)
            self.controller.runtime.data_collection.start(a1.DataCollectionMode.Snapshot, data_config)

        if type(self.command_queue) ==type(None):
            if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning or a1.TaskState.QueuePaused:
                self.controller.runtime.tasks[3].program.stop()
        try:
            self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 1000, True)
        except:
            pass
        
        if self.controller.runtime.tasks[3].status.task_state != a1.TaskState.QueuePaused:
            self.command_queue.pause()
        self.controller.runtime.commands.pso.psodistanceeventson('ST1') # Turn on PSO

        # Add all the AeroScript commands that you want to execute.
        self.command_queue.commands.advanced_motion.velocityblendingon()
        X = xi
        if is2D:
            Y = yi
            Y_step = np.sign(yf-yi)*abs(Y_step)
        totaltime = 0
        totaldistance = 0
        count = 0
        while count < N_lines:
            X_step = np.power(-1, count)*X_distance
            Xf = X + X_step
            #print(f"Linear move {count} from [{X, Y}] to [{Xf, Y}]")
            #self.command_queue.commands.motion.moveabsolute(axis, [Xf, Y], [speed, speed]) # Move linearly to the specified position
            if is2D:
                pos2go = [X_step, 0]
            else:
                pos2go = [X_step]
            self.command_queue.commands.motion.movelinear(axis, pos2go, speed) # Move linearly to the specified position
            #self.command_queue.commands.motion.waitformotiondone(axis)
            totaltime += time_per_line
            totaldistance += X_distance
            if is2D:
                iscw = True
                # when Y direction is positive (up direction)
                if X_step > 0:
                    iscw = False
                else:
                    iscw = True
                # when Y direction is negative (down direction)
                if Y_step < 0:
                    iscw = not iscw
                center = [0, Y_step/2] # relative offsets, https://help.aerotech.com/automation1/Content/Concepts/Motion-Functions.htm?Highlight=moveccw
#                print(f"This is center {center}")
#                print(f"Circular move from {[Xf, Y]} to [{Xf}, {Y+Y_step}]")
                if abs(Y-yf)>abs(Y_step)*0.1:
                    if iscw:
                        self.command_queue.commands.motion.movecwbycenter(axis, [0, Y_step], center, speed)
                    else:
                        self.command_queue.commands.motion.moveccwbycenter(axis, [0, Y_step], center, speed)
                    #self.command_queue.commands.motion.waitformotiondone(axis)
                    totaltime += np.pi*Y_step/2/float(speed) # time for half circling = distance / speed. Distance for half cicling = pi*R
                    totaldistance += np.pi*Y_step/2 # distance cicling = 2pi*R/2
                Y = np.round(Y + Y_step, 5)
            count += 1
            X = Xf
        print(f"Total number of lines: {count}")
        print("")
        self.total_scantime = totaltime
        self.total_scandistance = totaldistance
        self.scan_axes = axis
        #N_pulses = int(abs(final - start) / self.step_distance)  # Calculate the number of pulses to fire
        #self.N_pulses = N_pulses
        self.command_queue.commands.pso.psooutputoff('ST1') # Turn off PSO 
        self.command_queue.commands.advanced_motion.velocityblendingoff()
        self.command_queue.commands.motion.waitformotiondone(axis)

    def run_traj(self, wait=True):
        self.command_queue.resume()
        if wait:
            self.controller.runtime.commands.motion.waitformotiondone(self.scan_axes)
            #results = self.controller.runtime.data_collection.get_results(data_config, num_datapoints)
            #return results

    def make_stepscan_arrays(self, Xi = -2.5, Xf=2.5, X_step = 0.1, Yi = 0, Yf = 1, Y_step = 0.1):
        xpos_all = np.array([])
        ypos_all = np.array([])
        for stepN in range(int((Yf-Yi)/Y_step)+1):
            print(stepN, stepN%2)
            if stepN%2 ==0:
                xpos = np.arange(Xi, Xf+X_step, X_step)
            else:
                xpos = np.arange(Xf, Xi-X_step, -X_step)
            Y = Yi + Y_step*stepN
            ypos = np.full(xpos.shape, Y)
            xpos_all = np.concatenate((xpos_all, xpos))
            ypos_all = np.concatenate((ypos_all, ypos))
        return xpos_all, ypos_all
    
    def step_scan_SNAKE(self, Xi, Xf, X_step, Yi, Yf, Y_step, exptime):
        xp, yp = self.make_stepscan_arrays(Xi, Xf, X_step, Yi, Yf, Y_step)
        for x, y in zip(xp, yp):
            self.mv(X=x, Z=y)
            status = False
            while not status:
                time.sleep(0.01)
                try:
                    state = self.isattarget(['X', 'Z'])
                    status = state['X'] and state['Z']
                except:
                    status = False
                time.sleep(0.01)
            print(f"At X={x:.3f}, Z={y:.3f}, wait for {exptime} seconds.")
            time.sleep(exptime)

    def set_datacollection(self, axis, num_points = 1000):
        frequency = a1.DataCollectionFrequency.Frequency1kHz
        data_config = a1.DataCollectionConfiguration(num_points, frequency)
        #Adding the following signals to be collected on the x-axis
        for ax in axis:
            data_config.axis.add(a1.AxisDataSignal.PositionCommand, ax)
            data_config.axis.add(a1.AxisDataSignal.PositionFeedback, ax)
            data_config.axis.add(a1.AxisDataSignal.PositionError, ax)
        #Adding the time signal from the controller.
        data_config.system.add(a1.SystemDataSignal.DataCollectionSampleTime)
        self.data_config = data_config
        self.data_num_points = num_points
        return data_config
    
    def get_datacollection(self, num_points = 0):
        pos = []
        posfb = []
        poserr = []
        if num_points == 0:
            num_points = self.data_num_points
        results = self.controller.runtime.data_collection.get_results(self.data_config, num_points)
        time_array = np.array(results.system.get(a1.SystemDataSignal.DataCollectionSampleTime).points)
        time_array -=time_array[0]
        time_array /=sampling_freq        
        pos.append(time_array)
        posfb.append(time_array)
        for ax in self.scan_axes:
            pos_com = np.array(results.axis.get(a1.AxisDataSignal.PositionCommand, ax).points)
            pos_fbk = np.array(results.axis.get(a1.AxisDataSignal.PositionFeedback, ax).points)
            pos_err = np.array(results.axis.get(a1.AxisDataSignal.PositionError, ax).points)
            pos.append(pos_com)
            posfb.append(pos_fbk)
            poserr.append(pos_err)
        #Storing the time array collected from the controller
        return pos, posfb, poserr

def save_servo_paramters(controller):
    for i, ax in enumerate(controller.runtime.parameters.axes):
        axisname = ax.identification.axisname.value
        if i>0:
            continue
        print(f"Axis {i}: {axisname}")
        print("  Servo Parameters:")
        for param in ax.servo.__dict__.keys():
            #if "_" not in param:
            # if param.startswith("_"):
            #     continue
            try:
                value = getattr(ax.servo, param).value
            except Exception:
                try:
                    value = getattr(ax.servo, param)
                except Exception:
                    value = None
            filename = f"servo_params_{axisname}.txt"
            with open(filename, "a") as fh:
                fh.write(f"{param} : {value}\n")
        print("")


def load_servo_paramters(controller, filename="servo_params_ST1_normal.txt"):
    """
    Load servo parameters from a file produced by save_servo_paramters into a dict.
    Returns: dict mapping parameter name -> parsed Python value.
    """

    params = {}
    try:
        with open(filename, "r") as fh:
            for raw in fh:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line:
                    continue
                key, val = line.split(":", 1)
                key = key.strip()
                val = val.strip()
                # Try a safe literal eval first (handles numbers, lists, tuples, booleans if written as True/False, quoted strings)
                parsed = None
                try:
                    parsed = ast.literal_eval(val)
                except Exception:
                    # Fallbacks for common unquoted forms
                    low = val.lower()
                    if low in ("none", "null"):
                        parsed = None
                    elif low in ("true", "false"):
                        parsed = low == "true"
                    else:
                        # try int then float, else keep raw string
                        try:
                            parsed = int(val)
                        except Exception:
                            try:
                                parsed = float(val)
                            except Exception:
                                parsed = val
                params[key] = parsed
    except FileNotFoundError:
        print(f"File not found: {filename}")
    except Exception as e:
        print(f"Error reading {filename}: {e}")
    return params

def set_servo_paramters(controller, filename = ''):
    isInputAvailable = False
    if len(filename)>1:
        params = load_servo_paramters(controller, filename)
        isInputAvailable = True
    conf = controller.configuration.parameters.get_configuration()
    for i, ax in enumerate(conf.axes):
        #sv = ax.servo
        # if no input, read params from ST1 and set them to other axes
        if isInputAvailable:
            if i==0:
                continue
        if i>5:
            continue
        axisname = ax.identification.axisname.value
        print(f"Axis {i}: {axisname}")
        print("  Servo Parameters:")
        for param in ax.servo.__dict__.keys():
            #if "_" not in param:
            value = getattr(ax.servo, param).value
            if isInputAvailable:
                val0 = params[param]
            else:
                val0 = getattr(conf.axes[0].servo, param).value
            #parts = param.split('__')
            #attr = parts[-1]
            if val0 != value:
                try:
                    getattr(ax.servo, param).value = val0
                except Exception as e:
                    print(f"Could not set {param}: {e}")
                print(f"    {param}: {value} changed to {val0}")
        print("")        
    controller.configuration.parameters.set_configuration(conf)

def list_servo_paramters(controller):
    conf = controller.configuration.parameters.get_configuration()
    for i, ax in enumerate(conf.axes):
        #sv = ax.servo
        if i==0:
            continue
        if i>5:
            continue
        axisname = ax.identification.axisname.value
        print(f"Axis {i}: {axisname}")
        print("  Servo Parameters:")
        for param in ax.servo.__dict__.keys():
            #if "_" not in param:
            value = getattr(ax.servo, param).value
            val0 = getattr(conf.axes[0].servo, param).value
            if val0 != value:
                print(f"    {param}: {value}")
        print("")