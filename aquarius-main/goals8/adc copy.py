# Imports
import pigpio
import traceback

OUTPUT_PINS = [27, 4]
INPUT_PINS = [17, 9, 10, 11, 12, 22, 23, 24, 25]
LOW = 0
HIGH = 1

class ADC:
    def __init__(self, io):
        self.io = io
        # Set up the modes for the various pins
        for output_pin in OUTPUT_PINS:
            self.io.set_mode(output_pin, pigpio.OUTPUT)

        for pin in INPUT_PINS:
            self.io.set_mode(pin, pigpio.INPUT)

    def readadc(self, address):
        # Step 1: Set LATCH low -- prepare to recieve the address
        self.io.write(OUTPUT_PINS[0], LOW)

        # Step 2: Set ADDRESS -- select which magnetometer to read
        self.io.write(OUTPUT_PINS[1], address)

        # Step 3: Set LATCH high -- A/D chip locks in which signal to read
        self.io.write(OUTPUT_PINS[0], HIGH)

        # Step 4: Set LATCH low -- Start the conversion process
        self.io.write(OUTPUT_PINS[0], LOW)

        # Step 5: Set LATCH high -- Finish the conversion and transfer data to pins
        self.io.write(OUTPUT_PINS[0], HIGH)

        # Step 6: Wait for READY -- Data is now ready to be read
        while not self.io.read(INPUT_PINS[0]):
            continue
        if self.io.read(INPUT_PINS[0]):
            counter = 0
            val = 0
            for pin in INPUT_PINS[1:]:
                data = self.io.read(pin)
                val += data * (2 ** counter)
                counter += 1
        
        return val

def testing():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
    print("GPIO ready...")

    adc = ADC(io)

    min_0 = 255
    max_0 = 0
    min_1 = 255
    max_1 = 0

    try:
        while True:
            magnetometer_0 = adc.readadc(0)
            magnetometer_1 = adc.readadc(1)
            
            if magnetometer_0 < min_0:
                min_0 = magnetometer_0
            if magnetometer_0 > max_0:
                max_0 = magnetometer_0

            if magnetometer_1 < min_1:
                min_1 = magnetometer_1
            if magnetometer_1 > max_1:
                max_1 = magnetometer_1

            print("Magnetometer 0: ", magnetometer_0)
            print("Magnetometer 1: ", magnetometer_1)
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    
    print("Magnetometer 0 min: ", min_0)
    print("Magnetometer 0 max: ", max_0)
    print("Magnetometer 1 min: ", min_1)
    print("Magnetometer 1 max: ", max_1)

if __name__ == "__main__":
    testing()
        
