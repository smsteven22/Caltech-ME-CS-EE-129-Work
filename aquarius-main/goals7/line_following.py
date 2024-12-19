import time

from drive_system import Direction
from enum import Enum

T_DETECTOR_SIDE = 0.1
T_DETECTOR_END_STREET = 0.1
T_DETECTOR_INTERSECTION = 0.035
T_DETECTOR_TURN = 0.04
T_DETECTOR_ON_ROAD = 0.1

UPPER_THRESHOLD = 0.8
INTERSECTION_THRESHOLD = 0.82
LOWER_THRESHOLD = 0.2
TURNING_THRESHOLD = 0.2

T_DRIVE = 0.25
T_STOP = 0.15

class Position(Enum):
    INTERSECTION = 0
    END_OF_STREET = 1
    BLOCKED = 2

def line_following(drive, sensor, ultrasound=None):
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

        if ultrasound:
            (left, middle, right) = ultrasound.read()
            if middle < 0.125:
                return Position.BLOCKED

        reading = sensor.read()

        # Filter the side reading
        if reading == (0, 1, 0) or reading == (1, 0, 1): # Center
            side_raw = 0
        elif reading == (1, 1, 0): # Slightly off left
            side_raw = 0.7
        elif reading == (1, 0, 0): # Very off left
            side_raw = 1
        elif reading == (0, 1, 1): # Slighhtly off right
            side_raw = -0.7
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

        intersection_level = intersection_level + dt/T_DETECTOR_INTERSECTION * (intersection_raw - intersection_level)

        if intersection_level > INTERSECTION_THRESHOLD:
            intersection_output = 1
        elif intersection_level < LOWER_THRESHOLD:
            intersection_output = 0

        # Filter the end of street output
        if reading == (0, 0, 0):
            end_of_street_raw = 1
        else:
            end_of_street_raw = 0

        end_of_street_level = end_of_street_level + dt/T_DETECTOR_END_STREET * (end_of_street_raw - end_of_street_level)

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
        if reading == (0, 1, 0):
            action = Direction.STRAIGHT
        elif reading == (1, 0, 1):
            action = Direction.STRAIGHT
        elif reading == (0, 1, 1):
            action = Direction.STEER_RIGHT
        elif reading == (1, 1, 0):
            action = Direction.STEER_LEFT
        elif reading == (0, 0, 1):
            action = Direction.TURN_RIGHT
        elif reading == (1, 0, 0):
            action = Direction.TURN_LEFT
        elif reading == (1, 1, 1):
            if intersection_output == 1:
                return Position.INTERSECTION
            else:
                action = Direction.STRAIGHT
        elif reading == (0, 0, 0):
            if side_output == 1:
                action = Direction.HOOK_LEFT
            elif side_output == -1:
                action = Direction.HOOK_RIGHT
            elif side_output == 0:
                action = Direction.STRAIGHT


        # Apply the action (using the DriveSystem class)
        drive.forward(action)

def pull_forward(drive, sensor, line_following=True):
    # Get the initial time
    on_road_level = 1
    on_road_output = 1
    t0 = time.time()
    t = time.time()

    while True:
        tlast = t
        t = time.time()
        dt = t - tlast

        reading = sensor.read()

        if t - t0 < T_DRIVE:
            if line_following == True:
                # Continue line following algoirthm while pulling forward
                reading = sensor.read()
                if reading == (0, 1, 1):
                    action = Direction.STEER_RIGHT
                elif reading == (1, 1, 0):
                    action = Direction.STEER_LEFT
                elif reading == (0, 0, 1):
                    action = Direction.TURN_RIGHT
                elif reading == (1, 0, 0):
                    action = Direction.TURN_LEFT
                else:
                    action = Direction.STRAIGHT
                drive.forward(action)
            else:
                drive.forward(Direction.STRAIGHT)
        elif t - t0 < T_DRIVE + T_STOP:
            drive.stop()
        else:
            break

        # Filter the on road reading
        if reading == (0, 0, 0): # Sensor does not see line
            on_road_raw = 0
        elif reading == (0, 1, 0): # Sensor is on center line
            on_road_raw = 1
        else: # Otherwise, don't adjust reading
            on_road_raw = on_road_level

        # Update the on road filter level
        on_road_level = on_road_level + dt/T_DETECTOR_ON_ROAD * (on_road_raw - on_road_level)

        # Update the on road filter output
        if on_road_level > UPPER_THRESHOLD:
            on_road_output = 1
        elif on_road_level < LOWER_THRESHOLD:
            on_road_output = 0
    
    return bool(on_road_output)
        
def turning(drive, sensor, direction):
    t0 = time.time()
    t = t0
    turn_level = 1
    turn_output = True
    halfway = False
    while True:
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
                turn_raw = 0.5
            elif reading == (1, 1, 0):
                turn_raw = 0.25
            else:
                turn_raw = turn_level
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
                turn_raw = 0.5
            elif reading == (0, 1, 1):
                turn_raw = 0.25
            else:
                turn_raw = turn_level
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
            break

    # Sleep robot for 150ms before turn
    t0 = time.time()
    t = t0
    while t - t0 < T_STOP:
        t = time.time()

    # Align turn with the line
    while True:
        reading = sensor.read()
        if reading == (0, 1, 1) or reading == (0, 0, 1):
            drive.forward(Direction.CORRECT_RIGHT)
        elif reading == (1, 1, 0) or reading == (1, 0, 0):
            drive.forward(Direction.CORRECT_LEFT)
        else:
            drive.stop()
            break

