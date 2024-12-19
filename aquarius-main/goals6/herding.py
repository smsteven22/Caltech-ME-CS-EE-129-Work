# Imports
import pigpio
import time
import traceback
from drive_system import DriveSystem, Direction
from proximity_sensor import ProximitySensor

def herding_behavior(drive, proximity_sensor):
    # Start by triggering.
    proximity_sensor.trigger()
    last_trigger_time = time.time()

    # Loop from here.
    while True:
        # Re-trigger every 50ms.
        now = time.time()
        if now - last_trigger_time > 0.050:
            proximity_sensor.trigger()
            last_trigger_time = now

        # Read the latest measurements (having "arrived" in the last 50ms).
        (sensor_1_reading, sensor_2_reading, sensor_3_reading) = proximity_sensor.read()

        if sensor_1_reading > 0.1 and sensor_2_reading < 0.1 and sensor_3_reading > 0.1:
            drive.forward(Direction.BACKWARDS)
        elif sensor_1_reading < 0.1 and sensor_2_reading < 0.1 and sensor_3_reading < 0.1:
            drive.forward(Direction.BACKWARDS)
        elif sensor_1_reading < 0.1 and sensor_2_reading < 0.1 and sensor_3_reading > 0.1:
            drive.forward(Direction.BACKWARDS_TURN_RIGHT)
        elif sensor_1_reading > 0.1 and sensor_2_reading < 0.1 and sensor_3_reading < 0.1:
            drive.forward(Direction.BACKWARDS_TURN_LEFT)
        elif sensor_1_reading > 0.2 and sensor_2_reading > 0.2 and sensor_3_reading > 0.2:
            drive.forward(Direction.STRAIGHT)
        elif sensor_1_reading < 0.2 and sensor_2_reading > 0.2 and sensor_3_reading < 0.2:
            drive.forward(Direction.STRAIGHT)
        elif sensor_1_reading < 0.2 and sensor_2_reading > 0.2 and sensor_3_reading > 0.2:
            drive.forward(Direction.HOOK_RIGHT)
        elif sensor_1_reading > 0.2 and sensor_2_reading > 0.2 and sensor_3_reading < 0.2:
            drive.forward(Direction.HOOK_LEFT)
        elif sensor_1_reading > 0.2 and sensor_2_reading < 0.2 and sensor_3_reading > 0.2:
            drive.forward(Direction.SPIN_RIGHT)
        elif sensor_1_reading < 0.2 and sensor_2_reading < 0.2 and sensor_3_reading > 0.2:
            drive.forward(Direction.SPIN_RIGHT)
        elif sensor_1_reading > 0.2 and sensor_2_reading < 0.2 and sensor_3_reading < 0.2:
            drive.forward(Direction.SPIN_LEFT)
        elif sensor_1_reading < 0.2 and sensor_2_reading < 0.2 and sensor_3_reading < 0.2:
            drive.forward(Direction.BACKWARDS)

def main():
    try:
        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")
    
        proximity_sensor = ProximitySensor(io)
        drive = DriveSystem(io)

        herding_behavior(drive, proximity_sensor)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        print("Stopping motors and pi.")
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()