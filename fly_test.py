import time
import automation1 as a1
HOST = "at-hex-12id.xray.aps.anl.gov"
period = 0.01  # Default period for PSO in seconds
pulse_width = 10  # Default pulse width in microseconds
step_distance = 0.01  # Default distance for PSO in micrometers
AT_default_motornames = ["X", "Y", "Z", "A", "B", "C"]
command_queue = None
# 1. Connect to the controller
# Replace "localhost" with your controller's IP address if needed.
controller = a1.Controller.connect(host=HOST)
print("Connected to Automation1 controller.")
    # 2. Assign and enable the axis

def set_pulsestream():
    controller.runtime.commands.device.drivepulsestreamoff('ST1') # ST1 is the real axis issuing PSO firing pulses
    controller.runtime.commands.execute('DrivePulseStreamConfigure(ST1, [X, Y, Z], [1.0, 1.0, 1.0])') # X/Y/Z are the virtual axes being tracked. ST1 actually fires PSO, scale is 1.0
    controller.runtime.commands.device.drivepulsestreamon("ST1") # Turns on ST1's pulse stream input

def fly_conf():
    controller.runtime.commands.pso.psoreset("ST1") #Reset all PSO configuration
    controller.runtime.commands.pso.psodistanceconfigureinputs("ST1", [a1.PsoDistanceInput.iXR3DrivePulseStream]) # Fire based on virtual counts from the iXR3 pulse stream
    controller.runtime.commands.pso.psowaveformconfiguremode("ST1", a1.PsoWaveformMode.Pulse)
    controller.runtime.commands.pso.psowaveformconfigurepulsefixedcount("ST1", 1) # 1 pulse (period) per event
    #controller.runtime.commands.pso.psowaveformapplypulseconfiguration("ST1")# Applies the previous pulse configuration
    controller.runtime.commands.pso.psowaveformon("ST1") # Turn on waveform generator
    controller.runtime.commands.pso.psooutputconfiguresource("ST1", a1.PsoOutputSource.Waveform) # Use waveform module output as PSO output
    controller.runtime.commands.pso.psodistancecounteron("ST1") # Enable the distance counter

def fly_set(distance = step_distance, period=period, pulse_width=pulse_width):
    '''Configure the hexapod for part-speed operation using PSO.'''
    '''Distance : um units'''
    '''period : Periodicity of pulses generated at each distance, us units. When only one pulse is generated, this does not matter'''
    '''Pulse_width : us units'''
    controller.runtime.commands.execute(f'PsoDistanceConfigureFixedDistance(ST1,Round(UnitsToCounts(X, {distance})))') # Fire every 0.01 user units (10 um)
    controller.runtime.commands.pso.psowaveformconfigurepulsefixedtotaltime("ST1", period)# 10 usec total time
    controller.runtime.commands.pso.psowaveformconfigurepulsefixedontime("ST1", pulse_width)

def fly_run(axis="X", start=0, final=1, time=5, wait=True):
    '''Issue a PSO motion command to the hexapod.
    This will move the hexapod to the specified X, Y, Z coordinates
    using the configured PSO settings.
    '''
    global command_queue
    
    default_speed = 10
    #global motor_dict
    controller.runtime.commands.motion.moveabsolute(axis, [start], speeds=[default_speed]) # Move axis to the specified position
    controller.runtime.commands.motion.waitformotiondone(axis)
    # Begin a new command queue on task 1.
    if type(command_queue) ==type(None):
        if controller.runtime.tasks[3].status.task_state == a1.TaskState.QueueRunning:
            controller.runtime.tasks[3].program.stop()
            print("Task 3 was in QueueRunning mode. It is stopped and QueueRunning restarted.")
        command_queue = controller.runtime.commands.begin_command_queue("Task 3", 10, True)
    
    if not command_queue.status.is_paused:
        # First, pause the command queue so you can add all the commands
        # before they are executed.
        command_queue.pause()

    # Add all the AeroScript commands that you want to execute.
    command_queue.commands.advanced_motion.velocityblendingon()
    N_pulses = int(abs(final - start) / step_distance)  # Calculate the number of pulses to fire
    print(f"Fly to {final} in {time} seconds. PSO generates pulses every {time/N_pulses} seconds and total {N_pulses} pulses.")
    command_queue.commands.pso.psodistanceeventson("ST1") # Turn on PSO
    command_queue.commands.motion.moveabsolute(axis, [final], [abs(final-start)/time]) # Move ST1 to the specified position
    command_queue.commands.motion.waitformotiondone(axis)
    command_queue.commands.pso.psooutputoff("ST1") # Turn off PSO 
    command_queue.commands.advanced_motion.velocityblendingoff()

    # Resume the command queue so that all the commands that you added start
    # to execute.
    command_queue.resume()
    #     # Here you can do other things such as more process, get status, etc.
    #     # You can do these things because the command queue is executing
    #     # commands on the controller and is not blocking your code execution.
    if wait:
        # Here you wait to make sure that the command queue executes all the commands.
        # You must do this before you end the command queue.
        # When you end the command queue, this process aborts all motion and commands.
#        print("Waiting to be done.")
        controller.runtime.commands.motion.waitformotiondone(axis)
        command_queue.wait_for_empty()

def turn_off_pso():
    controller.runtime.commands.pso.psooutputoff("ST1") # Turn off PSO 

def fly_abort():
    global command_queue
    # At this time, end the command queue.
    # You can also call CommandQueue.end_command_queue to abort the command
    # that is currently executing and discard the remaining commands.
    try:
        controller.runtime.commands.end_command_queue(command_queue)
        command_queue = None
    except:
        print("There is no running fly yet. So, will make 'Task3' Idle.")
        controller.runtime.tasks[3].program.stop()
        command_queue = None


def main():
    fly_run()
    fly_abort()

if __name__ == "__main__":
    main()