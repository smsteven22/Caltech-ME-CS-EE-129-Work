import pigpio
import sys
import traceback

from drive_system import DriveSystem, Direction
from line_sensor import LineSensor
from line_following import line_following, pull_forward, turning, Position

def main():
    # Initialize the GPIO and create the objects.
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
    print("GPIO ready...")

    drive = DriveSystem(io, 7, 8, 5, 6)
    sensor = LineSensor(io, 14, 15, 18)

    while True:
        # Run the code in a try-except statement.
        try:
            # Line following code
            print("--- Starting Line Following ---")
            pos = line_following(drive, sensor)
            
            # Drive forward at intersection
            if pos == Position.INTERSECTION:
                print("-- At interssction. Pulling forward --")
                pull_forward(drive)
            elif pos == Position.END_OF_STREET:
                drive.stop()
            
            # Ask for input for direction
            while True:
                direction = input("Turn left or right (L/R) or go straight (S): ")
                if direction == "L" or direction == "R":
                    turning(drive, sensor, direction)
                    break
                elif direction == "S":
                    break
                elif direction == "Q" or direction == "q":
                    # Shutdown cleanly (stopping the motors, then stopping the io).
                    drive.stop()
                    io.stop()
                    return
                else:
                    print("Invalid input. Please enter L or R.")
            
        except BaseException as ex:
            # Report the error, then continue with the normal shutdown.
            print("Ending due to exception: %s" % repr(ex))
            traceback.print_exc()

if __name__ == "__main__":
    main()