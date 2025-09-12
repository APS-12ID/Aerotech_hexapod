import automation1 as a1
import threading
import numpy as np
import time

IP = "at-hex-12id.xray.aps.anl.gov"
BASEPV = '12idhxpAT'
HOST = "at-hex-12id.xray.aps.anl.gov"
period = 0.000_1  # Default period for PSO in seconds
pulse_width = 0.000_1  # Default pulse width in seconds
step_distance = 0.01  # Default distance for PSO in mm

class Hexapod:
    """A class to use pipython"""
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
        if len(axis)>0:
            return st[axis]['inposition']
        axis = self.axes
        status = []
        for ax in axis:
            if ax in st.keys():
                status.append(st[ax]['inposition'])
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
        self.controller.runtime.commands.pso.psodistanceconfigureinputs('ST1', [a1.PsoDistanceInput.iXR3DrivePulseStream]) # Fire based on virtual counts from the iXR3 pulse stream
        self.controller.runtime.commands.pso.psooutputconfigureoutput('ST1', a1.PsoOutputPin.XR3PsoOutput1)
        self.controller.runtime.commands.pso.psooutputconfiguresource('ST1', a1.PsoOutputSource.Waveform) # Use waveform module output as PSO output
        self.controller.runtime.commands.pso.psowaveformconfiguremode('ST1', a1.PsoWaveformMode.Pulse)
        #controller.runtime.commands.pso.psowaveformapplypulseconfiguration('ST1')# Applies the previous pulse configuration
        self.controller.runtime.commands.pso.psowaveformon('ST1') # Turn on waveform generator
        self.controller.runtime.commands.pso.psodistancecounteron('ST1') # Enable the distance counter

    def set_pulses(self, axis = 'X', distance = step_distance, period=period, pulse_width=pulse_width):
        # step_distance : mm
        # period : sec
        # pulse_width : sec
        '''Configure the hexapod for part-speed operation using PSO.'''
        '''period : Periodicity of pulses generated at each distance, us units. When only one pulse is generated, this does not matter'''
        '''Pulse_width : us units'''
        countsperunit = self.controller.runtime.parameters.axes[axis].units.countsperunit.value
        #self.controller.runtime.commands.execute(f'PsoDistanceConfigureFixedDistance(ST1,Round(UnitsToCounts(X, {distance})))') # Fire every 0.01 user units (10 um)
        # In a given period (period), 1 pulse will be generated with a width of pulse_width.
        pulse_distance = round(countsperunit*step_distance)
        period_us = period*1_000_000
        pulsewidth_us = pulse_width*1_000_000
        print(pulse_distance, ' pulse distance ', pulsewidth_us, ' period')

        self.controller.runtime.commands.pso.psodistanceconfigurefixeddistance('ST1', pulse_distance)
        self.controller.runtime.commands.pso.psowaveformconfiguremode('ST1', a1.PsoWaveformMode.Pulse)
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedtotaltime('ST1', period_us) # usec total time
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedontime('ST1', pulsewidth_us) # usec
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedcount('ST1', 1) # 1 pulse (period) per event
        self.controller.runtime.commands.pso.psowaveformapplypulseconfiguration('ST1')# Applies the previous pulse configuration
        #Npulses = int(self.scantime/(period*0.001))
        #self.pulse_positions_index = 

    def goto_start_pos(self, axis='X'):
        default_speed = 5
        # turn off pso
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1')
        #global motor_dict
        self.controller.runtime.commands.motion.moveabsolute(axis, [self.start_pos], speeds=[default_speed]) # Move axis to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)

    def set_traj(self, axis = "X", time=5, start = 0, final=5, step_distance=0.01):
        # position units : mm
        #final = start + distance
                # Begin a new command queue on task 1.
        if type(self.command_queue) ==type(None):
            if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
                self.controller.runtime.tasks[3].program.stop()
                print("Task 3 was in QueueRunning mode. It is stopped and QueueRunning restarted.")
            self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 10, True)
        
        if not self.command_queue.status.is_paused:
            # First, pause the command queue so you can add all the commands
            # before they are executed.
            self.command_queue.pause()

        # Add all the AeroScript commands that you want to execute.
        self.command_queue.commands.advanced_motion.velocityblendingon()
        N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
        speed = abs(final-start)/time
        print(f"Fly to {final} in {time} seconds with {speed} mm/s.") 
        print(f"PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
        self.command_queue.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        self.command_queue.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
        self.command_queue.commands.motion.waitformotiondone(axis)
        self.command_queue.commands.pso.psooutputoff('ST1') # Turn off PSO 
        self.command_queue.commands.advanced_motion.velocityblendingoff()
        
        self.set_pulses(distance = step_distance, period=period, pulse_width=pulse_width)
        self.pulse_number = N_pulses
        self.pulse_step = time/N_pulses # pulse time step in seconds
        self.scantime = time
        self.start_pos = start

    def set_traj_SNAKE(self, time_per_line = 5, Xi = -2.5, X_distance=1, Yi = 0, Yf = 1, Y_step = 0.1, pulse_step=0.1):
        if type(self.command_queue) ==type(None):
            if self.controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
                self.controller.runtime.tasks[3].program.stop()
                print("Task 3 was in QueueRunning mode. It is stopped and QueueRunning restarted.")
            self.command_queue = self.controller.runtime.commands.begin_command_queue("Task 3", 10, True)
        
        if not self.command_queue.status.is_paused:
            # First, pause the command queue so you can add all the commands
            # before they are executed.
            self.command_queue.pause()

        # Add all the AeroScript commands that you want to execute.
        self.command_queue.commands.advanced_motion.velocityblendingon()
        N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
        speed = abs(final-start)/time
        print(f"Fly to {final} in {time} seconds with {speed} mm/s.") 
        print(f"PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
        self.command_queue.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        Y = Yi
        direction = np.sign(X_distance)
        count = 0
        X = Xi
        while Y <= Yf:
            final = X + (-1)^count*X_distance
            self.command_queue.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
            self.command_queue.commands.motion.waitformotiondone(axis)
            iscw = True
            # when Y direction is positive (up direction)
            if final > X:
                iscw = False
            else:
                iscw = True
            # when Y direction is negative (down direction)
            if Y_step < 0:
                iscw = not iscw
            center = [final, Y + Y_step/2]
            self.command_queue.commands.motion.arcmove(["X", "Y"], [final, Y + Y_step], center, speed)
            self.command_queue.commands.motion.waitformotiondone(["X", "Y"])
            count += 1
            Y = Y + Y_step
            X = final
        self.command_queue.commands.pso.psooutputoff('ST1') # Turn off PSO 
        self.command_queue.commands.advanced_motion.velocityblendingoff()

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

    def run_traj(self, axis="X",  wait=True):
        '''Issue a PSO motion command to the hexapod.
        This will move the hexapod to the specified X, Y, Z coordinates
        using the configured PSO settings.
        '''
        self.goto_start_pos(axis)

        # Resume the command queue so that all the commands that you added start
        # to execute.
        self.command_queue.resume()
        #     # Here you can do other things such as more process, get status, etc.
        #     # You can do these things because the command queue is executing
        #     # commands on the controller and is not blocking your code execution.
        if wait:
            # Here you wait to make sure that the command queue executes all the commands.
            # You must do this before you end the command queue.
            # When you end the command queue, this process aborts all motion and commands.
    #        print("Waiting to be done.")
            self.controller.runtime.commands.motion.waitformotiondone(axis)
            self.command_queue.wait_for_empty()

    def turn_off_pso(self):
        self.controller.runtime.commands.pso.psooutputoff('ST1') # Turn off PSO 

    def test(self):

        self.enable_work()
        axis = 'X'
        self.controller.runtime.commands.pso.psodistanceeventsoff('ST1')
        self.controller.runtime.commands.motion.moveabsolute(axis, [0], speeds=[5]) # Move axis to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)

        self.controller.runtime.commands.device.drivepulsestreamoff('ST1') # ST1 is the real axis issuing PSO firing pulses
        #self.controller.runtime.commands.execute('DrivePulseStreamConfigure(ST1, [X, Y, Z], [1.0, 1.0, 1.0])') # X/Y/Z are the virtual axes being tracked. ST1 actually fires PSO, scale is 1.0
        self.controller.runtime.commands.device.drivepulsestreamon('ST1') # Turns on ST1's pulse stream input

        self.controller.runtime.commands.pso.psoreset('ST1') #Reset all PSO configuration
        self.controller.runtime.commands.pso.psodistanceconfigureinputs('ST1', [a1.PsoDistanceInput.iXR3DrivePulseStream]) # Fire based on virtual counts from the iXR3 pulse stream

        final = -5
        start = 0
        N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
        speed = 1
        countsperunit = self.controller.runtime.parameters.axes[axis].units.countsperunit.value
        #self.controller.runtime.commands.execute(f'PsoDistanceConfigureFixedDistance(ST1,Round(UnitsToCounts(X, {distance})))') # Fire every 0.01 user units (10 um)
        # In a given period (period), 1 pulse will be generated with a width of pulse_width.
        pulse_distance = round(countsperunit*step_distance)
        period_us = period*1_000_000
        pulsewidth_us = pulse_width*1_000_000
        print(pulse_distance, ' pulse distance ', pulsewidth_us, ' period')

        self.controller.runtime.commands.pso.psodistanceconfigurefixeddistance('ST1', pulse_distance)
        self.controller.runtime.commands.pso.psowaveformconfiguremode('ST1', a1.PsoWaveformMode.Pulse)
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedtotaltime('ST1', period_us) # usec total time
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedontime('ST1', pulsewidth_us) # usec
        self.controller.runtime.commands.pso.psowaveformconfigurepulsefixedcount('ST1', 1) # 1 pulse (period) per event
        self.controller.runtime.commands.pso.psowaveformapplypulseconfiguration('ST1')# Applies the previous pulse configuration
        self.controller.runtime.commands.pso.psooutputconfigureoutput('ST1', a1.PsoOutputPin.XR3PsoOutput1)
        self.controller.runtime.commands.pso.psooutputconfiguresource('ST1', a1.PsoOutputSource.Waveform) # Use waveform module output as PSO output
        self.controller.runtime.commands.pso.psowaveformon('ST1') # Turn on waveform generator
        self.controller.runtime.commands.pso.psodistancecounteron('ST1') # Enable the distance counter
        self.controller.runtime.commands.pso.psodistanceeventson('ST1') # Turn on PSO
        self.controller.runtime.commands.motion.moveabsolute(axis, [final], [speed]) # Move ST1 to the specified position
        self.controller.runtime.commands.motion.waitformotiondone(axis)
        self.controller.runtime.commands.pso.psooutputoff('ST1') # Turn off PSO 
        
