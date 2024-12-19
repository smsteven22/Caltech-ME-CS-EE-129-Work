# Imports
import pigpio
import time
import traceback
from drive_system import DriveSystem, Direction
from proximity_sensor import ProximitySensor

target_dist = .30
k_l = -2.1294
c_l = 0.7071
k_r = 2.1294
c_r = 0.7071


def wall_following_discrete(drive, proximity_sensor):
    global target_dist

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
        left_dist = proximity_sensor.read()[0]

        error = left_dist - target_dist

        if abs(error) < .02:
            drive.forward(Direction.STRAIGHT)
        elif abs(error) < .05:
            if (error > 0):
                drive.forward(Direction.VEER_LEFT)
            else:
                drive.forward(Direction.VEER_RIGHT)
        elif abs(error) < .08:
            if (error > 0):
                drive.forward(Direction.STEER_LEFT)
            else:
                drive.forward(Direction.STEER_RIGHT)
        elif abs(error) < .1:
            if (error > 0):
                drive.forward(Direction.TURN_LEFT)
            else:
                drive.forward(Direction.TURN_RIGHT)
        else:
            drive.stop()
    
def wall_following_proportional(drive, proximity_sensor):
    global target_dist

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
        left_dist = proximity_sensor.read()[0]

        error = left_dist - target_dist

        # Calculate the PWM values for left/right sensors
        pwm_left = k_l * error + c_l
        pwm_right = k_r * error + c_r

        pwm_left = min(max(pwm_left, -1), 1)
        pwm_right = min(max(pwm_right, -1), 1)

        drive.pwm(pwm_left, pwm_right)



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

        wall_following_proportional(drive, proximity_sensor)
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