# Imports
import pigpio
import sys
import traceback

from drive_system import DriveSystem, Direction
from line_sensor import LineSensor
from line_following import line_following, pull_forward, turning, Position
from adc import ADC
from angle_sensor import AngleSensor
from map import Map
from intersection import Status
from math import inf

# Instantiate global variables
drive = None
sensor = None
angle_sensor = None
robot_map = None
x = 0
y = 0
heading = 0
prev_pos = None

def turn_heading(direction, curr_intersection, new_heading):
    global heading

    i = (heading + direction) % 8
    while i != new_heading:
        curr_intersection.streets[i] = Status.NONEXISTENT
        i = (i + direction) % 8

def execute_turn(direction):
    global drive
    global sensor
    global angle_sensor
    global robot_map
    global x
    global y
    global heading

    curr_intersection = robot_map.getintersection(x,y)
    beg_angle = angle_sensor.readangle()
    turning(drive, sensor, direction)
    drive.stop()
    end_angle = angle_sensor.readangle()
    turn = round((beg_angle - end_angle) / 45) % 8

    new_heading = robot_map.calcturn(heading, turn)

    # Update the current intersection variable
    if direction == "R":
        turn_heading(-1, curr_intersection, new_heading)
    elif direction == "L":
        turn_heading(1, curr_intersection, new_heading)

    robot_map.setintersection(x, y, curr_intersection)

    # Update the robot pose
    heading = new_heading

    # Update map visualization
    if curr_intersection.streets[heading] == Status.UNKNOWN:
        curr_intersection.streets[heading] = Status.UNEXPLORED

    robot_map.show(x, y, heading)

def execute_uturn():
    global drive
    global sensor
    global robot_map
    global x
    global y
    global heading

    drive.stop()

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

    # Add intersection to dictionary
    robot_map.setintersection(x, y, curr_intersection)
    
    robot_map.show(x, y, heading)
    
   
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
                
                # Ask for input for direction
                while True:
                    print("------------------------------")
                    direction = input("Pick from the following: \nTurn left (L) \nTurn right (R) \nGo straight (S) \nFind optimal path (P) \nQuit (Q): ")
                    direction = direction.upper()
                    if direction == "L" or direction == "R":
                        execute_turn(direction)
                    elif direction == "S":
                        # Update the current intersection variable
                        curr_intersection = robot_map.getintersection(x, y)
                        
                        if curr_intersection.streets[heading] != Status.NONEXISTENT and curr_intersection.streets[heading] != Status.DEADEND:
                            break
                    elif direction == "P":
                        print("Please select a goal node.")
                        x_goal = int(input("x-coordinate: "))
                        y_goal = int(input("y-coordinate: "))
                        
                        if (x_goal, y_goal) not in robot_map.intersections:
                            print("Path is not possible!")
                            continue
                        
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
                    elif direction == "Q":
                        return
                    else:
                        print("Invalid input. Please enter L, R, S, or Q.")
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