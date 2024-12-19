# Imports
import pigpio
import sys
import traceback
import numpy as np
import time

from drive_system import DriveSystem, Direction
from line_sensor import LineSensor
from line_following import line_following, pull_forward, turning, Position
from adc import ADC
from angle_sensor import AngleSensor
from map import Map
from intersection import Status
from math import inf, sqrt

# Instantiate global variables
drive = None
sensor = None
angle_sensor = None
robot_map = None
x = 0
y = 0
heading = 0
prev_pos = None
start_heading = 0
starting_angle = 0

T_DELAY = 0.1

def turn_heading(direction, curr_intersection, new_heading):
    global heading

    i = (heading + direction) % 8
    while i != new_heading:
        curr_intersection.streets[i] = Status.NONEXISTENT
        i = (i + direction) % 8

def find_closest_unknown(curr_intersection, to_look):
    direction = heading
    for i in range(5):
        if curr_intersection.streets[direction] != Status.UNKNOWN:
            direction = (direction + to_look) % 8
        else:
            return direction

    return None

def find_closest_unexplored(curr_intersection, to_look):
    direction = heading
    for i in range(5):
        if curr_intersection.streets[direction] != Status.UNEXPLORED:
            direction = (direction + to_look) % 8
        else:
            return direction

    return None

def check_intersection_unknown(curr_intersection):
    for i in range(8):
        if curr_intersection.streets[i] == Status.UNKNOWN:
            return True

    return False

def check_intersection_unexplored(curr_intersection):
    for i in range(8):
        if curr_intersection.streets[i] == Status.UNEXPLORED:
            return True

    return False

def find_closest_unfinished_intersection():
    global robot_map
    global x
    global y
    global heading

    closest_intersection = None
    dist = inf

    for intersection in robot_map.intersections.values():
        if not intersection.is_finished and (intersection.x != x or intersection.y != y):
            new_dist = sqrt((intersection.x - x)**2 + (intersection.y - y)**2)
            if new_dist < dist:
                closest_intersection = intersection
                dist = new_dist

    return closest_intersection

# Returns true, if robot is already at the closest heading signalling break
def find_closest_heading(decision, curr_intersection):
    if decision == "Unknown":
        left_direction = find_closest_unknown(curr_intersection, 1)
        right_direction = find_closest_unknown(curr_intersection, -1)
    elif decision == "Unexplored":
        left_direction = find_closest_unexplored(curr_intersection, 1)
        right_direction = find_closest_unexplored(curr_intersection, -1)

    if left_direction == heading or right_direction == heading:
        return True
    elif left_direction == None:
        execute_turn("R")
    elif right_direction == None:
        execute_turn("L")
    else:
        rounded_left = 8 * np.round(left_direction/8)
        rounded_right = 8 * np.round(right_direction/8)

        if rounded_left <= rounded_right:
            execute_turn("L")
        else:
            execute_turn("R")

def execute_turn(direction):
    global drive
    global sensor
    global angle_sensor
    global robot_map
    global x
    global y
    global heading
    global starting_angle
    global start_heading

    curr_intersection = robot_map.getintersection(x,y)
    turning(drive, sensor, direction)
    drive.stop()

    # Delay the robot before the sensor reading
    t0 = time.time()
    t = t0
    while t - t0 < T_DELAY:
        t = time.time()
    
    end_angle = angle_sensor.readangle()
        
    turn = round(((starting_angle - end_angle) / 45) % 8)

    if turn > 4:
        turn = turn - 8

    new_heading = robot_map.calcturn(start_heading, turn)

    # Update the current intersection variable
    if direction == "R":
        turn_heading(-1, curr_intersection, new_heading)
    elif direction == "L":
        turn_heading(1, curr_intersection, new_heading)

    robot_map.setintersection(x, y, curr_intersection)

    # Update the robot's heading
    heading = new_heading

    # Update map visualization
    if curr_intersection.streets[new_heading] == Status.UNKNOWN:
        curr_intersection.streets[new_heading] = Status.UNEXPLORED

    robot_map.show(x, y, heading)

def execute_uturn():
    global drive
    global sensor
    global robot_map
    global x
    global y
    global heading

    drive.stop()
    t0 = time.time()
    t = t0
    while t - t0 < T_DELAY:
        t = time.time()

    print("-- At end of street. Making a u-turn. ---")

    turning(drive, sensor, "R")

    # Update the robot pose
    heading = robot_map.calcuturn(heading)

    robot_map.show(x, y, heading)

    # Update the current intersection with dead end
    curr_intersection = robot_map.getintersection(x, y)
    curr_intersection.streets[(heading + 4) % 8] = Status.DEADEND

def execute_intersection():
    global drive
    global sensor
    global robot_map
    global x
    global y
    global heading
    global prev_pos
    global start_heading
    global starting_angle

    print("-- At intersection. Pulling forward --")

    # Update previous intersection w/ old pose
    if prev_pos != None:
        prev_intersection = robot_map.getintersection(x, y)
    
    # Updates robot pose
    if prev_pos != Position.END_OF_STREET: # Robot does not change intersection if it made u-turn
        # Update the position of the intersection
        (x,y) = robot_map.calcmove(x, y, heading)

    # Update the current intersection
    curr_intersection = robot_map.getintersection(x, y)

    # Get rid of internal angles
    curr_intersection.streets[(heading - 3) % 8] = Status.NONEXISTENT
    curr_intersection.streets[(heading - 5) % 8] = Status.NONEXISTENT

    # Pull forward, and check if the robot is on the road
    if curr_intersection.streets[heading] == Status.NONEXISTENT:
        is_on_road = pull_forward(drive, sensor, False)
    else:
        is_on_road = pull_forward(drive, sensor, True)

    if prev_pos != Position.END_OF_STREET and prev_pos != None:
        # Connect current intersection to previous intersection
        prev_intersection.streets[heading] = Status.CONNECTED
        curr_intersection.streets[(heading + 4) % 8] = Status.CONNECTED

    # Update current heading value with pull_forward output
    if is_on_road and curr_intersection.streets[heading] == Status.UNKNOWN:
        curr_intersection.streets[heading] = Status.UNEXPLORED
    elif not is_on_road:
        curr_intersection.streets[heading] = Status.NONEXISTENT

    # Get the starting angle and heading for the intersection
    start_heading = heading
    starting_angle = angle_sensor.readangle()

    # Add intersection to dictionary
    robot_map.setintersection(x, y, curr_intersection)
    
    robot_map.show(x, y, heading)

def execute_dijkstra(x_goal, y_goal):
    global prev_pos

    if (x_goal, y_goal) not in robot_map.intersections:
        print("Path is not possible!")
        return
    
    goal = robot_map.getintersection(x_goal, y_goal)
    
    print("--- Starting Dijkstra's Algorithm ---")
    robot_map.dijkstras(goal)
    robot_map.show(x, y, heading)

    print("---- Starting Autonavigating ----")
    while x != x_goal or y != y_goal:
        curr_intersection = robot_map.getintersection(x, y)

        # Check if path is possible
        if curr_intersection.cost == inf:
            print("Path is not possible!")
            break

        # Align heading
        turn_counter = 0
        while heading != curr_intersection.direction:
            if turn_counter > 2:
                break

            if ((curr_intersection.direction - heading) % 8) >= 4:
                execute_turn("R")
            else:
                execute_turn("L")

            turn_counter += 1
    
        # Then go straight                        
        if curr_intersection.streets[heading] != Status.NONEXISTENT and curr_intersection.streets[heading] != Status.DEADEND:
            pos = line_following(drive, sensor)
            if pos == Position.INTERSECTION:
                execute_intersection()
            elif pos == Position.END_OF_STREET:
                execute_uturn()
            prev_pos = pos
    
def main():
    try:
        global drive
        global sensor
        global angle_sensor
        global robot_map
        global x
        global y
        global heading
        global prev_pos
        global start_heading
        global starting_angle

        # Initialize the GPIO and create the objects.
        print("Setting up the GPIO...")
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        print("GPIO ready...")

        drive = DriveSystem(io, 7, 8, 5, 6)
        sensor = LineSensor(io, 14, 15, 18)
        adc = ADC(io)
        angle_sensor = AngleSensor(adc)

        robot_map = Map()

        # Starting pose of the robot
        x = 0
        y = -1
        heading = 0

        user_input = input("Load map? Y/N: ")
        if user_input.upper() == "Y":
            map_name = input("Please provide filename for map to be loaded in: ")
            robot_map = robot_map.load(map_name)
            x = int(input("Please provide the x position of the robot: "))
            y = int(input("Please provide the y position of the robot: "))
            heading = int(input("Please provide the heading value of the robot: "))

        robot_map.show(x, y, heading)

        # Record the position at the previous intersection
        prev_pos = None

        while True:
        # Run the code in a try-except statement.
            # Line following code
            print("--- Starting Line Following ---")
            pos = line_following(drive, sensor)
            
            # Drive forward at intersection
            if pos == Position.INTERSECTION:
                execute_intersection()
                
                print("Starting angle at intersection: ", starting_angle)

                # Ask for input for direction
                while True:
                    print("------------------------------")
                    direction = input("Pick from the following: \nTurn left (L) \nTurn right (R) \nGo straight (S) \nAuto navigate (N) \nAuto explore (E) \nQuit (Q): ")
                    direction = direction.upper()
                    if direction == "L" or direction == "R":
                        execute_turn(direction)
                    elif direction == "S":
                        # Update the current intersection variable
                        curr_intersection = robot_map.getintersection(x, y)
                        if curr_intersection.streets[heading] != Status.NONEXISTENT and curr_intersection.streets[heading] != Status.DEADEND:
                            break 
                    elif direction == "N":
                        print("Please select a goal node.")
                        x_goal = int(input("x-coordinate: "))
                        y_goal = int(input("y-coordinate: "))

                        execute_dijkstra(x_goal, y_goal)
                    elif direction == "E":
                        starting_angle = angle_sensor.readangle()
                        start_heading = heading
                        made_uturn = False

                        while True:
                            while True:
                                if not made_uturn:
                                    curr_intersection = robot_map.getintersection(x, y)
                                    contains_unknown_street = check_intersection_unknown(curr_intersection)
                                    contains_unexplored_street = check_intersection_unexplored(curr_intersection)

                                    if contains_unexplored_street == False and contains_unknown_street == False:
                                        break

                                    for i in range(1): # Allow one turn in auto exploring
                                        if contains_unknown_street:
                                            is_finished = find_closest_heading("Unknown", curr_intersection)
                                            
                                            if is_finished:
                                                break
                                                
                                            contains_unknown_street = check_intersection_unknown(curr_intersection)
                                        elif contains_unexplored_street:
                                            is_finished = find_closest_heading("Unexplored", curr_intersection)

                                            if is_finished:
                                                break

                                            contains_unexplored_street = check_intersection_unexplored(curr_intersection)
                                else:
                                    made_uturn = False
                                
                                pos = line_following(drive, sensor)
                                if pos == Position.INTERSECTION:
                                    execute_intersection()
                                elif pos == Position.END_OF_STREET:
                                    execute_uturn()
                                    made_uturn = True
                                    # pos = line_following(drive, sensor)
                                    # pull_forward(drive, sensor)
                                
                                prev_pos = pos

                            robot_map.unfinished_intersections()
                            next_intersection = find_closest_unfinished_intersection()

                            if next_intersection == None:
                                print("--- Fully explored map ---")
                                break

                            execute_dijkstra(next_intersection.x, next_intersection.y)
                            robot_map.reset_path()
                
                        robot_map.show(x, y, heading)
                    elif direction == "Q":
                        return
                    else:
                        print("Invalid input. Please enter L, R, S, P, E, or Q.")
            elif pos == Position.END_OF_STREET:
                execute_uturn()

            # Replace previous position with current position
            prev_pos = pos
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally: 
        # Stop the motors.
        print("--- Stopping Motors ---")
        drive.stop()

        # Ask if the user wants to save the map.
        save = input("Would you like to save the map? Y/N: ")
        if save.upper() == "Y":
            print("--- Saving the Map ---")
            filename = input("Please input a filename for the map. ")
            robot_map.save(filename)
            print("Robot final pose and heading:")
            print("X: ", x)
            print("Y: ", y)
            print("heading: ", heading)
        
        # Shutdown cleanly.
        print("--- Shutting Down Pi ---")
        io.stop()

if __name__ == "__main__":
    main()