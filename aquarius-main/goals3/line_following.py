import time

from drive_system import Direction
from enum import Enum

T_DETECTOR_SIDE = 0.01
T_DETECTOR = 0.1
T_DETECTOR_TURN = 0.05

UPPER_THRESHOLD = 0.8
LOWER_THRESHOLD = 0.2
INTERSECTION_THRESHOLD = 0.7
TURNING_THRESHOLD = 0.4

T_DRIVE = 0.35
T_STOP = 0.25

class Position(Enum):
    INTERSECTION = 0
    END_OF_STREET = 1

def line_following(drive, sensor):
    t = time.time()
    side_level = 0
    intersection_level = 0
    end_of_street_level = 0
    
    side_output = 0
    intersection_output = 0
    end_of_street_output = 0
    while True:
        tlast = t
        t = time.time()
        dt = t - tlast

        reading = sensor.read()

        # Filter the side reading
        if reading == (0, 1, 0) or reading == (1, 0, 1): # Center
            side_raw = 0
        elif reading == (1, 1, 0): # Slightly off left
            side_raw = 0.5
        elif reading == (1, 0, 0): # Very off left
            side_raw = 1
        elif reading == (0, 1, 1): # Slighhtly off right
            side_raw = -0.5
        elif reading == (0, 0, 1): # Very off right
            side_raw = -1
        else:
            side_raw = side_level

        side_level = side_level + dt/T_DETECTOR_SIDE * (side_raw - side_level)

        if side_level < -UPPER_THRESHOLD:
            side_output = -1
        elif side_level > -LOWER_THRESHOLD and side_level < LOWER_THRESHOLD:
            side_output = 0
        elif side_level > UPPER_THRESHOLD:
            side_output = 1
        
        # Filter the intersection level
        if reading == (1, 1, 1):
            intersection_raw = 1
        else:
            intersection_raw = 0

        intersection_level = intersection_level + dt/T_DETECTOR * (intersection_raw - intersection_level)

        if intersection_level > INTERSECTION_THRESHOLD:
            intersection_output = 1
        elif intersection_level < LOWER_THRESHOLD:
            intersection_output = 0

        # Filter the end of street output
        if reading == (0, 0, 0):
            end_of_street_raw = 1
        else:
            end_of_street_raw = 0

        end_of_street_level = end_of_street_level + dt/T_DETECTOR * (end_of_street_raw - end_of_street_level)

        if end_of_street_level > UPPER_THRESHOLD:
            end_of_street_output = 1
        elif end_of_street_level < LOWER_THRESHOLD:
            end_of_street_output = 0

        # Stop line following if at an intersection or end of street
        if intersection_output == 1:
            return Position.INTERSECTION
        elif end_of_street_output == 1 and side_output == 0:
            return Position.END_OF_STREET

        # Determine the action (implementing the feedback law table) 
        if reading == (0, 1, 0) or reading == (1, 0, 1):
            action = Direction.STRAIGHT
        elif reading == (0, 1, 1):
            action = Direction.STEER_RIGHT
        elif reading == (1, 1, 0):
            action = Direction.STEER_LEFT
        elif reading == (0, 0, 1):
            action = Direction.TURN_RIGHT
        elif reading == (1, 0, 0):
            action = Direction.TURN_LEFT
        elif reading == (0, 0, 0):
            if side_output == 1:
                action = Direction.HOOK_LEFT
            elif side_output == -1:
                action = Direction.HOOK_RIGHT
            elif side_output == 0:
                action = Direction.STRAIGHT

        # Apply the action (using the DriveSystem class)
        drive.forward(action)

def pull_forward(drive):
    # Get the initial time
    t0 = time.time()

    while True:
        t = time.time()
        if t < t0 + T_DRIVE:
            drive.forward(Direction.STRAIGHT)
        else:
            drive.stop()
            break
    
    if t > t0 + T_DRIVE + T_STOP:
        drive.stop()
        return
        
def turning(drive, sensor, direction):
    t0 = time.time()
    t = t0
    turn_level = 1
    turn_output = True
    while True:
        halfway = False
        tlast = t
        t = time.time()
        dt = t - tlast

        reading = sensor.read()

        # Calculate the update to the turn level based on the sensor reading
        if direction == "L":
            if reading == (0, 1, 0):
                if halfway == False: # Stop turning if past halfway
                    turn_raw = 1
                else: # Continue turning otherwise
                    turn_raw = 0
            elif reading == (0, 1, 1):
                turn_raw = 1
            elif reading == (0, 0, 1):
                turn_raw = 1
            elif reading == (0, 0, 0):
                halfway = True
                turn_raw = 1
            elif reading == (1, 0, 0):
                turn_raw = 0.2
            elif reading == (1, 1, 0):
                turn_raw = 0.2
        elif direction == "R":
            if reading == (0, 1, 0):
                if halfway == False: # Stop turning if past halfway
                    turn_raw = 1
                else: # Continue turning otherwise
                    turn_raw = 0
            elif reading == (1, 1, 0):
                turn_raw = 1
            elif reading == (1, 0, 0):
                turn_raw = 1
            elif reading == (0, 0, 0):
                halfway = True
                turn_raw = 1
            elif reading == (0, 0, 1):
                turn_raw = 0.2
            elif reading == (0, 1, 1):
                turn_raw = 0.2
        else:
            print("Invalid direction.")
            drive.stop()
            break

        turn_level = turn_level + dt/T_DETECTOR_TURN * (turn_raw - turn_level)

        # Update the turn detector output
        if turn_level > UPPER_THRESHOLD: # Reached end fo turn
            turn_output = True
        elif turn_level < TURNING_THRESHOLD: # Robot is turning
            turn_output = False

        # Drive the robot
        if turn_output == True:
            if direction == "L":
                drive.forward(Direction.SPIN_LEFT)
            elif direction == "R":
                drive.forward(Direction.SPIN_RIGHT)
        if turn_output == False:
            drive.stop()
            print("Time: ", t-t0)
            break

