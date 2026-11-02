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
    epics.caput(command_PV, f'DrivePulseStreamOff(X)') # X is the real axis issuing PSO firing pulses
    epics.caput(command_PV, f'DrivePulseStreamConfigure(X, [X, Y, Z], [1.0, 1.0, 1.0])') # X/Y/Z are the virtual axes being tracked. X actually fires PSO, scale is 1.0
    epics.caput(command_PV, f'DrivePulseStreamOn(X)') # Turns on X's pulse stream input
    epics.caput(command_PV, f'PsoReset(X)') #Reset all PSO configuration
    epics.caput(command_PV, f'PsoDistanceConfigureInputs(X,[PsoDistanceInput.iXR3DrivePulseStream])') # Fire based on virtual counts from the iXR3 pulse stream
    epics.caput(command_PV, f'PsoDistanceConfigureFixedDistance(X,Round(UnitsToCounts(X, {distance})))') # Fire every 0.01 user units (10 um)
    epics.caput(command_PV, f'PsoWaveformConfigureMode(X, PsoWaveformMode.Pulse)') #Sets X's waveform to pulse mode
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedTotalTime(X, {period})') # 10 usec total time
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedOnTime(X, {pulse_width})') # 10 usec total time
    epics.caput(command_PV, f'PsoWaveformConfigurePulseFixedCount(X, 1)') # 1 pulse (period) per event
    epics.caput(command_PV, f'PsoWaveformApplyPulseConfiguration(X)') #Applies the previous pulse configuration
    epics.caput(command_PV, f'PsoWaveformOn(X)') # Turn on waveform generator
    epics.caput(command_PV, f'PsoOutputConfigureSource(X, PsoOutputSource.Waveform)') # Use waveform module output as PSO output
    epics.caput(command_PV, f'PsoDistanceCounterOn(X)') # Enable the distance counter

def fly(axis="X", start=0, final=1, time=5, wait=True):
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
    print(f"Fly to {final} in {time} seconds. PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
    epics.caput(command_PV, f'PsoDistanceEventsOn(X)') # Turn on PSO
    epics.caput(command_PV, f'MoveAbsolute({axis}, {final}, {abs(final-start)/time})') # Move X to the specified position
    if wait:
        # Wait a moment to ensure the move has started
        while epics.caget(f'{IOC_prefix}{motorpv}.DMOV'):
            sleep(0.02)
        # Wait done
        while epics.caget(f'{IOC_prefix}{motorpv}.DMOV')==0:
            sleep(0.02)
    epics.caput(command_PV, f'PsoOutputOff(X)') # Turn off PSO

# ---------------------------------------------------------------------------
# Center of rotation
# ---------------------------------------------------------------------------
# The IOC has no PVs for the pivot point, so the AeroScript library calls are
# sent through the write-only command PVs:
#   ExecuteCommand       : 256-char waveform
#   ExecuteShortCommand  : 39-char stringout (fallback for short commands)
# Two coordinate systems define where the hexapod rotates about:
#   work : position of the platform center in the base coordinate system
#          (default [0, 0, 230.25, 0, 0, 0])
#   tool : pivot point offset from the platform center (SetToolPoint/ActivateTool)
# Distances are in mm, angles (A, B, C) in degrees.
short_command_PV = IOC_prefix + 'ExecuteShortCommand'
MAX_SHORT_COMMAND = 39
MAX_COMMAND = 256

def send_command(cmd, wait=True):
    '''Send an AeroScript command line to the controller through the IOC.
    Uses ExecuteShortCommand when the command fits in 39 characters,
    otherwise ExecuteCommand (256 characters).'''
    if len(cmd) > MAX_COMMAND:
        raise ValueError(f"Command is {len(cmd)} characters; the limit is {MAX_COMMAND}: {cmd}")
    pv = short_command_PV if len(cmd) <= MAX_SHORT_COMMAND else command_PV
    epics.caput(pv, cmd, wait=wait)

def _pose(x, y, z, a, b, c):
    return ",".join(f"{v:g}" for v in (x, y, z, a, b, c))

def set_work(x=0, y=0, z=230.25, a=0, b=0, c=0):
    '''Define the work coordinate system (platform center in the base system).'''
    send_command("DisableHexapod()")
    send_command(f"SetBaseToWork({_pose(x, y, z, a, b, c)})")
    send_command("EnableWork()")

def set_tool(x=0, y=0, z=0, a=0, b=0, c=0, name="Tool1", index=1):
    '''Define and activate a tool whose origin is the center of rotation,
    given as an offset from the platform center.
    e.g. set_tool(z=50) rotates about a point 50 mm above the platform center.'''
    send_command("DisableHexapod()")
    send_command(f'SetToolPoint({index}, "{name}", {_pose(x, y, z, a, b, c)})')
    send_command(f'ActivateTool("{name}")')
    send_command("EnableTool()")

def set_center_of_rotation(x=0, y=0, z=0, name="Tool1", index=1):
    '''Move the center of rotation to (x, y, z) mm relative to the platform center.'''
    set_tool(x, y, z, 0, 0, 0, name=name, index=index)

# Platform (top plate) center in the bottom base coordinate system at the zero pose.
PLATFORM_HOME = (0.0, 0.0, 230.25)

def get_pose():
    '''Return the current {axis: position} for X, Y, Z, A, B, C from the motor RBVs.'''
    return {axis: epics.caget(f'{IOC_prefix}{motorpv}.RBV') for motorpv, axis in motor_dict.items()}

def base_to_tool_offset(base_xyz, platform_xyz=PLATFORM_HOME):
    '''Offset of a base-frame point from the platform center. Valid when the
    platform has no rotation, i.e. the platform and base axes are parallel.'''
    return tuple(p - c for p, c in zip(base_xyz, platform_xyz))

def set_tcp_from_measurement(x, y, z, check_pose=True, pos_tol=0.05, ang_tol=0.01,
                             name="Tool1", index=1):
    '''Set the center of rotation from a physical measurement.
    x, y, z (mm) are measured from the center of the top plate (the moving
    platform) with the hexapod at its zero pose (X=Y=A=B=C=0, Z=230.25).
    The point is converted to the bottom base coordinate system, and the TCP
    offset is then derived from that base-frame point, so it does not depend on
    where the top plate happens to be afterwards. Returns the base-frame point.
    Note: the RBVs are in the active tool/work system, so reset any active tool
    (reset_center_of_rotation) before relying on check_pose.'''
    if check_pose:
        pose = get_pose()
        off = [abs(pose[ax] - home) for ax, home in zip("XYZ", PLATFORM_HOME)]
        off += [abs(pose[ax]) for ax in "ABC"]
        if any(o > t for o, t in zip(off, [pos_tol]*3 + [ang_tol]*3)):
            raise RuntimeError(f"Hexapod is not at its zero pose {PLATFORM_HOME}: {pose}. "
                               "Move to zero first, or pass check_pose=False.")
    base_xyz = tuple(h + m for h, m in zip(PLATFORM_HOME, (x, y, z)))
    tx, ty, tz = base_to_tool_offset(base_xyz)
    set_tool(tx, ty, tz, name=name, index=index)
    print(f"Center of rotation: base frame {base_xyz}, tool offset ({tx:g}, {ty:g}, {tz:g})")
    return base_xyz

def reset_center_of_rotation(name="Tool1", index=1):
    '''Put the center of rotation back at the platform center.'''
    set_center_of_rotation(0, 0, 0, name=name, index=index)

