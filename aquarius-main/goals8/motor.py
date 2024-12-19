# Imports
import pigpio
import sys
import time
import traceback
import math
import numpy as np

class Motor:
   def __init__(self, leadA, leadB, io):
       # Instantiate variables
       self.leadA = leadA
       self.leadB = leadB
       self.io = io
       self.prev_level = 0


       # Set leadA and leadB to output
       self.io.set_mode(self.leadA, pigpio.OUTPUT)
       self.io.set_mode(self.leadB, pigpio.OUTPUT)


       # Prepare the PWM cycle for leadA and leadB
       self.io.set_PWM_range(self.leadA, 255)
       self.io.set_PWM_range(self.leadB, 255)
       self.io.set_PWM_frequency(self.leadA, 1000)
       self.io.set_PWM_frequency(self.leadB, 1000)


       # Clear pins (if necessary)
       self.stop()




   def setlevel(self, level):
        prev_PWM = int(255 * abs(self.prev_level))
        PWM = int(255 * abs(level))

        if level >= 0:
            self.io.set_PWM_dutycycle(self.leadA, PWM)
            self.io.set_PWM_dutycycle(self.leadB, 0)
        else:
            self.io.set_PWM_dutycycle(self.leadA, 0)
            self.io.set_PWM_dutycycle(self.leadB, PWM)

        # for i in np.linspace(0, 1, 11):
        #     # Change the direction of the voltage depending upon the level
        #     if level >= 0:
        #         self.io.set_PWM_dutycycle(self.leadA, prev_PWM + i * (PWM- prev_PWM))
        #         self.io.set_PWM_dutycycle(self.leadB, 0)
        #     else:
        #         self.io.set_PWM_dutycycle(self.leadA, 0)
        #         self.io.set_PWM_dutycycle(self.leadB, prev_PWM + i * (PWM- prev_PWM))

        # self.prev_level = level      


   def stop(self):
       # Clear all pins
       self.io.set_PWM_dutycycle(self.leadA, 0)
       self.io.set_PWM_dutycycle(self.leadB, 0)


if __name__ == "__main__":
   print("Setting up the GPIO...")
   io = pigpio.pi()
   if not io.connected:
       print("Unable to connection to pigpio daemon!")
       sys.exit(0)
   print("GPIO ready...")
  
   left = Motor(leadA=7, leadB=8, io=io)
   right = Motor(leadA=5, leadB=6, io=io)


   try:
       for i in range(4):
       # Ramp up the speed of the motors to maximum linear velocity
           for j in range(11):
               pwmlevel = 0.1 * j
               left.setlevel(pwmlevel)
               right.setlevel(pwmlevel)
               time.sleep(0.05)
              
           # Move for 1 second at maximum velocity
           time.sleep(2.6)


           # Ramp down the speed of the motors to maximum linear velocity
           for k in range(11):
               pwmlevel = 1 - (k * 0.1)
               left.setlevel(pwmlevel)
               right.setlevel(pwmlevel)
               time.sleep(0.05)


           # Ramp up the speed of the motors to maximum turning velocity
           for l in range(8):
               pwmlevel = 0.1 * l
               left.setlevel(pwmlevel)
               right.setlevel(-1* pwmlevel)
               time.sleep(0.05)
              
           # Turn for one second at maximum turning velocity
           time.sleep(0.2)
                  
           # Ramp down the speed of the motors to maximum linear velocity
           for m in range(8):
               pwmlevel = 1 - (0.1 * m)
               left.setlevel(pwmlevel)
               right.setlevel(-1* pwmlevel)
               time.sleep(0.05)
   except BaseException as ex:
       # Report the error, but continue with the normal shutdown.
       print("Ending due to exception: %s" % repr(ex))
       traceback.print_exc()
  
   left.stop()
   right.stop()
   io.stop()