# Imports
import time
import pigpio
import traceback
from ultrasound import Ultrasound

class ProximitySensor:
    def __init__(self, io, pintrig_1=13, pinecho_1=16, pintrig_2=19, pinecho_2=20, pintrig_3=26, pinecho_3=21):
        self.sensor_1 = Ultrasound(io, pintrig_1, pinecho_1)
        self.sensor_2 = Ultrasound(io, pintrig_2, pinecho_2)
        self.sensor_3 = Ultrasound(io, pintrig_3, pinecho_3)

    def trigger(self):
        self.sensor_1.trigger()
        self.sensor_2.trigger()
        self.sensor_3.trigger()

    def read(self):
        return (self.sensor_1.read(), self.sensor_2.read(), self.sensor_3.read())

def test():
    try:
        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")
    
        proximity_sensor = ProximitySensor(io)

        while True:
            # Trigger the ultrasounds and wait 50 ms.
            proximity_sensor.trigger()
            time.sleep(0.050)
            
            # Read/report
            distances = proximity_sensor.read()
            print("Distances = (%6.3fm, %6.3fm, %6.3fm)" % distances)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        print("Stopping pi.")
        io.stop()

if __name__ == "__main__":
    test()