# Imports
import time
import pigpio
from math import inf

class Ultrasound:
    # Initialization
    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho
        self.trise = 0
        self.prev_trig_time = 0
        self.dist = inf

        self.io.set_mode(pintrig, pigpio.OUTPUT)
        self.io.set_mode(pinecho, pigpio.INPUT)
        
        # Set up the callbacks.
        cbrise = self.io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        cbfall = self.io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)  

    def trigger(self):
        if time.time() - self.prev_trig_time >= 0.05:
            self.io.write(self.pintrig, 1)
            self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        self.trise = ticks

    def falling(self, pin, level, ticks):
        delta_t = ticks - self.trise

        # Add 2^(32) if the difference has rolled over to a negative value
        if delta_t < 0:
            delta_t += 2**32

        # Distance = (Velocity) * (Delta t)
        self.dist = 343 * (delta_t / (10**6) / 2)

    def read(self):
        return self.dist
