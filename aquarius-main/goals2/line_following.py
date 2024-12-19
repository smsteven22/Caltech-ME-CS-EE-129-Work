import pigpio
import sys
import traceback

from ir import IR
from line_sensor import LineSensor
from drive_system import Direction, DriveSystem

class LineFollowing:
    def __init__(self, drive, sensor):
        self.drive = drive
        self.sensor = sensor

        self.prev_reading = None
        self.side = 1 # Starting in the center of the line

    def run(self):
        while True:
            # Read the sensors (using the LineSensor class).
            reading = self.sensor.read()
            prev_reading = self.prev_reading

            # Determine the action (implementing the feedback law table)
            if reading == (0, 1, 0) or reading == (1, 0, 1):
                action = Direction.STRAIGHT
            elif reading == (0, 1, 1):
                action = Direction.TURN_RIGHT
            elif reading == (1, 1, 0):
                action = Direction.TURN_LEFT
            elif reading == (0, 0, 1):
                action = Direction.HOOK_RIGHT
            elif reading == (1, 0, 0):
                action = Direction.HOOK_LEFT
            elif reading == (1, 1, 1):
                action = Direction.SPIN_RIGHT
            elif reading == (0, 0, 0):
                if prev_reading == (0, 1, 1) or prev_reading == (0, 0, 1):
                    self.side = 2
                elif prev_reading == (0, 1, 0) or prev_reading == (1, 0, 1):
                    self.side = 1
                elif prev_reading == (1, 1, 0) or prev_reading == (1, 0, 0):
                    self.side = 0
                
                if self.side == 0:
                    action = Direction.HOOK_LEFT
                elif self.side == 1:
                    action = Direction.SPIN_RIGHT
                elif self.side == 2:
                    action = Direction.HOOK_RIGHT
            else:
                print("The IR sensors are not working as expected.")
                self.drive.stop()
                break

            # Apply the action (using the DriveSystem class)
            self.drive.forward(action)
            self.prev_reading = reading

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
    line_following = LineFollowing(drive, sensor)

    # Run the code in a try-except statement.
    try:
        line_following.run()
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown cleanly (stopping the motors, then stopping the io).
    drive.stop()
    io.stop()

if __name__ == "__main__":
    main()