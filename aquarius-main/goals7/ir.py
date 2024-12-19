# Imports
import pigpio

class IR:
    def __init__(self, io, pin):
        self.io = io
        self.pin = pin

        # Set IR pin as input (reading the sensor).
        self.io.set_mode(self.pin, pigpio.INPUT)

    def read(self):
        return self.io.read(self.pin)
    