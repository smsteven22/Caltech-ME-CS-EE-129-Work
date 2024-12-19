# Imports
import pigpio
import time
import sys
import traceback
import threading
import ctypes
from drive_system import DriveSystem, Direction
from proximity_sensor import ProximitySensor

T_DELAY = 0.3

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False  # Default the state to running the motors
    
    def acquire(self):
        return self.lock.acquire()

    def release(self):
        self.lock.release()
        

def delay_restart(drive, forward):
    if not forward:
        drive.stop()
        # Delay the robot before the sensor reading
        t0 = time.time()
        t = t0
        while t - t0 < T_DELAY:
            t = time.time()
    
    return True

# Define the UI thread's run function.
def runui(shared):
    # Place everything in a try-except block to catch errors and exceptions.
    try:
    # Keep running unless told to stop.
        running = True
        while running:
        # Grab the user input - implicitly sleep while waiting.
            command = input("Pick from the following: go, stop, or quit ")
            command = command.lower()

            # Grab access to the shared data.
            if shared.acquire():
                # Immediately process the command into the shared flags.
                if (command == "go"):
                    # Set the appropriate flag(s)
                    shared.active = True
                elif (command == "stop"):
                    shared.active = False
                elif (command == "quit"):
                    shared.active = False
                    
                    # Tell the loop to stop (do NOT jump out of the protected
                    # if statement without releasing)!
                    running = False
                    
                    # Raise an Exception
                    raise Exception(KeyboardInterrupt)
                else:
                    print("Illegal command ", command)
                
                # Release the shared data.
                shared.release()
    except BaseException as ex:
        print("Ending Run-UI due to exception: %s" % repr(ex))

def runrobot(drive, proximity_sensor, shared):
    try:
        # Default forward and active field to true
        forward = True
        active = True

        while True:
            # Acquire the shared data        
            if shared.acquire():
                active = shared.active
                shared.release()

            
            if active: # Move robot if active
                # Read the latest measurements (having "arrived" in the last 50ms).
                (sensor_1_reading, sensor_2_reading, sensor_3_reading) = proximity_sensor.read()

                if sensor_1_reading > 0.1 and sensor_2_reading < 0.1 and sensor_3_reading > 0.1:
                    forward = False
                    drive.forward(Direction.BACKWARDS)
                elif sensor_1_reading < 0.1 and sensor_2_reading < 0.1 and sensor_3_reading < 0.1:
                    forward = False
                    drive.forward(Direction.BACKWARDS)
                elif sensor_1_reading < 0.1 and sensor_2_reading < 0.1 and sensor_3_reading > 0.1:
                    forward = False
                    drive.forward(Direction.BACKWARDS_TURN_RIGHT)
                elif sensor_1_reading > 0.1 and sensor_2_reading < 0.1 and sensor_3_reading < 0.1:
                    forward = False
                    drive.forward(Direction.BACKWARDS_TURN_LEFT)
                elif sensor_1_reading > 0.2 and sensor_2_reading > 0.2 and sensor_3_reading > 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.STRAIGHT)
                elif sensor_1_reading < 0.2 and sensor_2_reading > 0.2 and sensor_3_reading < 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.STRAIGHT)
                elif sensor_1_reading < 0.2 and sensor_2_reading > 0.2 and sensor_3_reading > 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.HOOK_RIGHT)
                elif sensor_1_reading > 0.2 and sensor_2_reading > 0.2 and sensor_3_reading < 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.HOOK_LEFT)
                elif sensor_1_reading > 0.2 and sensor_2_reading < 0.2 and sensor_3_reading > 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.SPIN_RIGHT)
                elif sensor_1_reading < 0.2 and sensor_2_reading < 0.2 and sensor_3_reading > 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.SPIN_RIGHT)
                elif sensor_1_reading > 0.2 and sensor_2_reading < 0.2 and sensor_3_reading < 0.2:
                    forward = delay_restart(drive, forward)
                    drive.forward(Direction.SPIN_LEFT)
                elif sensor_1_reading < 0.2 and sensor_2_reading < 0.2 and sensor_3_reading < 0.2:
                    forward = False
                    drive.forward(Direction.BACKWARDS)
            else: # Stop robot if inactive
                drive.stop()
    except BaseException as ex:
        print("Ending Run-Robot due to exception: %s" % repr(ex))
        

def main():
    try:
        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")
    
        shared = SharedData()
        proximity_sensor = ProximitySensor(io)
        drive = DriveSystem(io)

        robotthread = threading.Thread(name="RobotThread", target=runrobot,
                                       args=(drive, proximity_sensor, shared))
        robotthread.start()

        # Do the UI work.
        runui(shared)

        # runrobot(drive, proximity_sensor, shared)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
    finally:
        # Send an exception (Keyboard Interrupt) to the worker thread to finish.
        # Then wait to join
        print("Sending exception to robot thread to finish.")
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(robotthread.ident),
            ctypes.py_object(KeyboardInterrupt))
        robotthread.join()
        print("Robot thread returned.")

        print("Shutting down threading.")
        proximity_sensor.shutdown()
        print("Stopping motors and pi.")
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()