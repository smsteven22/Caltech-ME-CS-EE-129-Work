# Imports
import pigpio
import sys
import traceback
import threading
import ctypes
from ros import runros

from robot_system import RobotSystem
from threading_functions import SharedData, runui, runrobot

def main():
    try:
        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")

        robot = RobotSystem(io)

        shared = SharedData()

        # Start the ROS worker thread.
        rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
        rosthread.start()
        print("ROS thread started.")

        robotthread = threading.Thread(name="RobotThread", target=runrobot,
                                       args=(robot, shared))
        robotthread.start()
        print("Robot thread started.")

        # Do the UI work.
        runui(shared)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally: 
        # Send an exception (Keyboard Interrupt) to the worker thread to finish.
        # Then wait to join
        print("Sending exception to robot thread to finish.")
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(robotthread.ident),
            ctypes.py_object(KeyboardInterrupt))
        robotthread.join()
        print("Robot thread returned.")

        print("Sending exception to ROS thread to finish.")
        # End the ROS thread (send the KeyboardInterrupt exception).
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(rosthread.ident), 
            ctypes.py_object(KeyboardInterrupt))
        rosthread.join()
        print("ROS thread returned.")
        
        # Shutdown cleanly.
        print("--- Stopping Motors and Proximity Sensor ---")
        robot.shutdown()
        print("--- Shutting Down Pi ---")
        io.stop()

if __name__ == "__main__":
    main()