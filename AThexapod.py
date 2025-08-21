from time import sleep
import epics

command_pv_root = 'ExecuteCommand.VAL'
IOC_prefix = '12idhxpAT:'
command_PV = IOC_prefix + command_pv_root

motor_dict = {"m1": "Z", 
    "m2": "X",
    "m3": "Y",
    "m4": "A",
    "m5": "B",
    "m6": "C"}
period = 0.01  # Default period for PSO in seconds
pulse_width = 10  # Default pulse width in microseconds
step_distance = 0.01  # Default distance for PSO in micrometers
AT_default_motornames = ["X", "Y", "Z", "A", "B", "C"]

def set_IOC_prefix(input_prefix):
    #global command_PV, IOC_prefix
    IOC_prefix = input_prefix
    command_PV = IOC_prefix + command_pv_root

def update_motor_dict():
    '''Update motor_dict so that the PV of the motor is the key and the axis name is the value.'''
    #global motor_dict
    motor_dict = {}
    for key in motor_dict.keys():
        motor_pv = IOC_prefix + key
        axis_name = epics.caget(f"{motor_pv}.DESC")
        #axis_name = axis_name.split(" ")[-1]
        motor_dict[motor_pv] = axis_name

# part-speed PSO configuration for the hexapod
# This code is used to configure the hexapod for part-speed operation
# using the PSO (Pulse Stream Output) feature of the iXR3 controller.
# It sets up a pulse stream that fires at a specified distance and
# configures the waveform for pulse generation.
def fly_pso_configuration(distance = step_distance, period=period, pulse_width=pulse_width):
    '''Configure the hexapod for part-speed operation using PSO.'''
    '''Distance : um units'''
    '''period : Periodicity of pulses generated at each distance, us units. When only one pulse is generated, this does not matter'''
    '''Pulse_width : us units'''
    epics.caput(command_PV, f'DrivePulseStreamOff(ST1)') # ST1 is the real axis issuing PSO firing pulses
    epics.caput(command_PV, f'DrivePulseStreamConfigure(ST1, [X, Y, Z], [1.0, 1.0, 1.0])') # X/Y/Z are the virtual axes being tracked. ST1 actually fires PSO, scale is 1.0
    epics.caput(command_PV, f'DrivePulseStreamOn(ST1)') # Turns on ST1's pulse stream input
    epics.caput(command_PV, f'PsoReset(ST1)') #Reset all PSO configuration
    epics.caput(command_PV, f'PsoDistanceConfigureInputs(ST1,[PsoDistanceInput.iXR3DrivePulseStream])') # Fire based on virtual counts from the iXR3 pulse stream
    epics.caput(command_PV, f'PsoDistanceConfigureFixedDistance(ST1,Round(UnitsToCounts(X, {distance})))') # Fire every 0.01 user units (10 um)
    epics.caput(command_PV, f'PsoWaveformConfigureMode(ST1, PsoWaveformMode.Pulse)') #Sets ST1's waveform to pulse mode
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedTotalTime(ST1, {period})') # 10 usec total time
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedOnTime(ST1, {pulse_width})') # 10 usec total time
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedCount(ST1, 1)') # 1 pulse (period) per event
    epics.caput(command_PV, f'PsoWaveformApplyPulseConfiguration(ST1)') #Applies the previous pulse configuration
    epics.caput(command_PV, f'PsoWaveformOn(ST1)') # Turn on waveform generator
    epics.caput(command_PV, f'PsoOutputConfigureSource(ST1, PsoOutputSource.Waveform)') # Use waveform module output as PSO output
    epics.caput(command_PV, f'PsoDistanceCounterOn(ST1)') # Enable the distance counter

def fly(axis="X", start=0, final=1, time=5):
    '''Issue a PSO motion command to the hexapod.
    This will move the hexapod to the specified X, Y, Z coordinates
    using the configured PSO settings.
    '''
    #global motor_dict
    if axis not in motor_dict.values():
        raise ValueError(f"Axis {axis} is not a valid axis. Valid axes are: {list(motor_dict.values())}")
    n = list(motor_dict.values()).index(axis) + 1  # Get the index of the axis in the motor_dict
    motorpv = list(motor_dict.keys())[n-1]
    axis = AT_default_motornames[n-1]
    print(f"Moving axis to {start} ...")
    epics.caput(command_PV, f'MoveAbsolute({axis}, {start}, 1)') # Move axis to the specified position
    while abs(epics.caget(f'{IOC_prefix}{motorpv}.VAL') - start)>0.001:
        sleep(0.02)
    N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
    print(f" Done. Fly to {final} in {time} seconds. PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
    epics.caput(command_PV, f'PsoDistanceEventsOn(ST1)') # Turn on PSO
    epics.caput(command_PV, f'MoveAbsolute({axis}, {final}, {abs(final-start)/time})') # Move ST1 to the specified position
    sleep(0.5)  # Wait a moment to ensure the move has started
    while not epics.caget(f'{IOC_prefix}{motorpv}.DMOV'):
        sleep(0.02)
    epics.caput(command_PV, f'PsoOutputOff(ST1)') # Turn off PSO 

