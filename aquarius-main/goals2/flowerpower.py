# Imports
import pigpio
import sys
import time
import traceback
import math
import numpy as np
from drive_system import DriveSystem, Direction

def main():
    try:
        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")

        robot = DriveSystem(io, left_ledA=7, left_ledB=8, right_ledA=5, right_ledB=6)

        for direction in [Direction.BACKWARDS, Direction.BACKWARDS_TURN_LEFT, 
                          Direction.BACKWARDS_TURN_RIGHT, Direction.STRAIGHT, 
                          Direction.VEER_LEFT, Direction.VEER_RIGHT,
                          Direction.STEER_LEFT, Direction.STEER_RIGHT,
                          Direction.TURN_LEFT, Direction.TURN_RIGHT,
                          Direction.HOOK_LEFT, Direction.HOOK_RIGHT,
                          Direction.SPIN_LEFT, Direction.SPIN_RIGHT]:
            print(f"Moving in direction: {direction}")
            robot.forward(direction)
            time.sleep(4)
            robot.stop()
            input("Press enter to continue.")
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        print("Finished testing all directions!")
        robot.stop()
        io.stop()

if __name__ == "__main__":
    main()