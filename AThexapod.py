from time import sleep
import epics

command_pv_root = 'ExecuteCommand.VAL'
IOC_prefix = '12idHXP:'
command_PV = IOC_prefix + command_pv_root

def set_IOC_prefix(input_prefix):
    IOC_prefix = input_prefix
    command_PV = IOC_prefix + command_pv_root

def get_motor_pv(axis):
    '''Return the PV for the specified motor axis.'''
    for n in range(1, 7):
        motorname = epics.caget(IOC_prefix + f"m{n}.DESC")
        if axis.upper() == motorname.upper():
            return IOC_prefix + f"m{n}"
    raise ValueError(f"Axis {axis} not found")

# part-speed PSO configuration for the hexapod
# This code is used to configure the hexapod for part-speed operation
# using the PSO (Pulse Stream Output) feature of the iXR3 controller.
# It sets up a pulse stream that fires at a specified distance and
# configures the waveform for pulse generation.
def flying_hexapod_pso_configuration(distance = 0.01, period=0.01, pulse_width=10):
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

def flying_hexapod_pso_motion(axis="X", start=0, final=1,time=5):
    '''Issue a PSO motion command to the hexapod.
    This will move the hexapod to the specified X, Y, Z coordinates
    using the configured PSO settings.
    '''
    epics.caput(command_PV, f'MoveAbsolute({axis}, {start}, 1)') # Move axis to the specified position
    epics.caput(command_PV, f'PsoDistanceEventsOn(ST1)') # Turn on PSO
    epics.caput(command_PV, f'MoveAbsolute({axis}, {final}, {abs(final-start)/time})') # Move ST1 to the specified position
    motorpv = get_motor_pv(axis)  # Ensure the axis exists
    while epics.caget(f'{motorpv}.RBV') < final:
        sleep(0.1)  # Wait for the motion to complete
    epics.caput(command_PV, f'PsoOutputOff(ST1)') # Turn off PSO 

