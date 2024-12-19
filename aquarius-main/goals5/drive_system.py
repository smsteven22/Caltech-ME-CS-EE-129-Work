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

class DriveSystem:
    def __init__(self, io, left_ledA, left_ledB, right_ledA, right_ledB):
        self.left = Motor(leadA=left_ledA, leadB=left_ledB, io=io)
        self.right = Motor(leadA=right_ledA, leadB=right_ledB, io=io)

    # Move robot forward with default direction being straight
    def forward(self, direction=Direction.STRAIGHT):
        if(direction == Direction.STRAIGHT):
            self.left.setlevel(0.77)
            self.right.setlevel(0.77)
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
            self.left.setlevel(0.7)
            self.right.setlevel(-.7)
        elif(direction == Direction.SPIN_LEFT):
            self.left.setlevel(-.7)
            self.right.setlevel(0.7)

    # Halt the robot in place
    def stop(self):
        self.left.stop()
        self.right.stop()
        

