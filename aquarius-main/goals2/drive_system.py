from motor import Motor
from enum import Enum

class Direction(Enum):
    STRAIGHT = 0
    VEER_LEFT = 1
    VEER_RIGHT = 2
    STEER_LEFT = 3
    STEER_RIGHT = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    HOOK_LEFT = 7
    HOOK_RIGHT = 8
    SPIN_LEFT = 9
    SPIN_RIGHT = 10
    BACKWARDS = 11
    BACKWARDS_TURN_LEFT = 12
    BACKWARDS_TURN_RIGHT = 13
    CORRECT_LEFT = 14
    CORRECT_RIGHT = 15


class DriveSystem:
    def __init__(self, io, left_ledA=7, left_ledB=8, right_ledA=5, right_ledB=6):
        self.left = Motor(leadA=left_ledA, leadB=left_ledB, io=io)
        self.right = Motor(leadA=right_ledA, leadB=right_ledB, io=io)

    # Move robot forward with default direction being straight
    def forward(self, direction=Direction.STRAIGHT):
        if(direction == Direction.STRAIGHT):
            self.left.setlevel(0.78)
            self.right.setlevel(0.78)
        elif(direction == Direction.VEER_RIGHT):
            self.left.setlevel(0.7 + 0.07)
            self.right.setlevel(0.7 - 0.07)
        elif(direction == Direction.VEER_LEFT):
            self.left.setlevel(0.7 - 0.07)
            self.right.setlevel(0.7 + 0.07)
        elif(direction == Direction.STEER_RIGHT):
            self.left.setlevel(0.7 + 0.14)
            self.right.setlevel(0.7 - 0.14)
        elif(direction == Direction.STEER_LEFT):
            self.left.setlevel(0.7 - 0.14)
            self.right.setlevel(0.7 + 0.14)
        elif(direction == Direction.TURN_RIGHT):
            self.left.setlevel(0.7 + 0.21)
            self.right.setlevel(0.7 - 0.21)
        elif(direction == Direction.TURN_LEFT):
            self.left.setlevel(0.7 - 0.21)
            self.right.setlevel(0.7 + 0.21)
        elif(direction == Direction.HOOK_RIGHT):
            self.left.setlevel(0.8)
            self.right.setlevel(0)
        elif(direction == Direction.HOOK_LEFT):
            self.left.setlevel(0)
            self.right.setlevel(0.8)
        elif(direction == Direction.SPIN_RIGHT):
            self.left.setlevel(0.75)
            self.right.setlevel(-.75)
        elif(direction == Direction.SPIN_LEFT):
            self.left.setlevel(-.75)
            self.right.setlevel(0.75)
        elif(direction == Direction.BACKWARDS):
            self.left.setlevel(-0.75)
            self.right.setlevel(-0.75)
        elif(direction == Direction.BACKWARDS_TURN_RIGHT):
            self.left.setlevel(-1 * (0.7 + 0.21))
            self.right.setlevel(-1 * (0.7 - 0.21))
        elif(direction == Direction.BACKWARDS_TURN_LEFT):
            self.left.setlevel(-1 * (0.7 - 0.21))
            self.right.setlevel(-1 * (0.7 + 0.21))
        elif(direction == Direction.CORRECT_LEFT):
            self.left.setlevel(-.7)
            self.right.setlevel(.7)
        elif(direction == Direction.CORRECT_RIGHT):
            self.left.setlevel(0.7)
            self.right.setlevel(-0.7)

    def pwm(self, PWM_L, PWM_R):
        self.left.setlevel(PWM_L)
        self.right.setlevel(PWM_R)


    # Halt the robot in place
    def stop(self):
        self.left.stop()
        self.right.stop()
        

