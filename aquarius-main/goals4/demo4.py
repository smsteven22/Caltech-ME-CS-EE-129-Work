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

def turn_heading(direction, curr_intersection, new_heading, prev_heading):
    i = (prev_heading + direction) % 8
    while i != new_heading:
        curr_intersection.streets[i] = Status.NONEXISTENT
        i = (i + direction) % 8

def main():
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

    robot_map.show(x, y, heading)

    try:
        # Record the position at the previous intersection
        prev_pos = None

        while True:
        # Run the code in a try-except statement.
            # Line following code
            print("--- Starting Line Following ---")
            pos = line_following(drive, sensor)
            
            # Drive forward at intersection
            if pos == Position.INTERSECTION:
                print("-- At intersection. Pulling forward --")
                is_on_road = pull_forward(drive, sensor)

                # Update previous intersection w/ old pose
                if prev_pos != None:
                    prev_intersection = robot_map.getintersection(x, y)
                
                # Updates robot pose
                if prev_pos != Position.END_OF_STREET: # Robot does not change intersection if it made u-turn
                    # Update the position of the intersection
                    (x,y) = robot_map.calcmove(x, y, heading)

                # Update the current intersection
                curr_intersection = robot_map.getintersection(x, y)

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
                
                # Ask for input for direction
                while True:
                    direction = input("Turn left or right (L/R) or go straight (S): ")
                    if direction == "L" or direction == "R":
                        curr_intersection = robot_map.getintersection(x,y)
                        beg_angle = angle_sensor.readangle()
                        turning(drive, sensor, direction)
                        drive.stop()
                        end_angle = angle_sensor.readangle()
                        turn = round((beg_angle - end_angle) / 45) % 8

                        new_heading = robot_map.calcturn(heading, turn)

                        # Update the current intersection variable
                        if direction == "R":
                            turn_heading(-1, curr_intersection, new_heading, heading)
                        if direction == "L":
                            turn_heading(1, curr_intersection, new_heading, heading)
                        
                        robot_map.setintersection(x, y, curr_intersection)
                        
                        # Update the robot pose
                        heading = new_heading

                        robot_map.show(x, y, heading)
                    elif direction == "S":
                        # Update the current intersection variable
                        curr_intersection = robot_map.getintersection(x, y)
                        robot_map.setintersection(x, y, curr_intersection)

                        break
                    elif direction == "Q" or direction == "q":
                        return
                    else:
                        print("Invalid input. Please enter L or R.")
            elif pos == Position.END_OF_STREET:
                print("-- At end of street. Making a u-turn. ---")
                drive.stop()
                turning(drive, sensor, "L")
                
                # Update the robot pose
                heading = robot_map.calcuturn(heading)

                robot_map.show(x, y, heading)

                # Update the current intersection with dead end
                curr_intersection = robot_map.getintersection(x, y)
                curr_intersection.streets[(heading + 4) % 8] = Status.DEADEND
                robot_map.setintersection(x, y, curr_intersection)
            
            # Replace previous position with current position
            prev_pos = pos
    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally: 
        # Shutdown cleanly (stopping the motors, then stopping the io).
        print("--- Stopping Motors and Pi ---")
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()