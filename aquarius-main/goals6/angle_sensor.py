# Imports
import pigpio
import traceback
from math import atan2, degrees
from adc import ADC
import matplotlib.pyplot as plt
from drive_system import Direction, DriveSystem

class AngleSensor:
    def __init__(self, adc):
        self.adc = adc
        self.magnetometer_0_readings = []
        self.magnetometer_1_readings = []

    def readangle(self):
        # Read the A/D values (0...255) for both magnetometers
        magnetometer_0 = self.adc.readadc(0)
        magnetometer_1 = self.adc.readadc(1)

        # Scale the values into -1.0...+1.0.
        scaled_reading_0 = (magnetometer_0 - 147.5) / 73.5
        scaled_reading_1 = (magnetometer_1 - 114.5) / 76.0

        # Add magnetometer values to this list
        self.magnetometer_0_readings.append(magnetometer_0)
        self.magnetometer_1_readings.append(magnetometer_1)

        # Use an atan2 function to calculate the angle.
        angle = degrees(atan2(scaled_reading_1, scaled_reading_0))
        return angle

def testing():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
    print("GPIO ready...")
    
    adc = ADC(io)
    angle_sensor = AngleSensor(adc)

    drive = DriveSystem(io, 7, 8, 5, 6)
    drive.forward(Direction.SPIN_LEFT)

    try:
        while True:
            angle = angle_sensor.readangle()
            print("Angle: ", angle)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    print("Stopping motors and pi.")
    drive.stop()
    io.stop()

    min_0 = min(angle_sensor.magnetometer_0_readings)
    max_0 = max(angle_sensor.magnetometer_0_readings)

    min_1 = min(angle_sensor.magnetometer_1_readings)
    max_1 = max(angle_sensor.magnetometer_1_readings)

    print("Magnetometer 0 min: ", min_0)
    print("Magnetometer 0 max: ", max_0)
    print("Magnetometer 0 average ", (max_0 + min_0) / 2)
    print("Magnetometer 0 difference ", (max_0 - min_0) / 2)
    print("Magnetometer 1 min: ", min_1)
    print("Magnetometer 1 max: ", max_1)
    print("Magnetometer 1 average ", (max_1 + min_1) / 2)
    print("Magnetometer 1 difference ", (max_1 - min_1) / 2)

    print("Making Plot!")
    plt.plot(angle_sensor.magnetometer_0_readings, label = "mag_0")
    plt.plot(angle_sensor.magnetometer_1_readings, label = "mag_1")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    testing()