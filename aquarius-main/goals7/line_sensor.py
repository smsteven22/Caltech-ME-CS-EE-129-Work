# Imports
import pigpio
from ir import IR

class LineSensor:
    def __init__(self, io, left_pin=14, middle_pin=15, right_pin=18):
        self.left = IR(io, left_pin)
        self.middle = IR(io, middle_pin)
        self.right = IR(io, right_pin)

    def read(self):
        return (self.left.read(), self.middle.read(), self.right.read())
    
    def test(self):
        print("IRs: L %d  M %d  R %d" % self.read())