# Imports
import pigpio
import sys
import traceback
import numpy as np
import time
import bisect
import math

from drive_system import DriveSystem, Direction
from line_sensor import LineSensor
from line_following import line_following, pull_forward, turning, Position
from adc import ADC
from angle_sensor import AngleSensor
from map import Map, heading_to_dx, heading_to_dy
from intersection import Status
from math import inf, sqrt, pi
from proximity_sensor import ProximitySensor
from intersection import Intersection

T_DELAY = 0.1

class RobotSystem:
    def __init__(self, io):
        self.drive = DriveSystem(io)
        self.sensor = LineSensor(io)
        self.angle_sensor = AngleSensor(ADC(io))
        self.proximity_sensor = ProximitySensor(io)
        
        self.robot_map = Map()
        self.x = 0
        self.y = 0
        self.heading = 0
        
        self.prev_pos = None
        self.start_heading = 0
        self.start_angle = 0

        self.first_intersection = True

        self.next_intersection = None
        self.unreachable_intersections = []
        self.dijkstra_finished = True

        self.directed = False
        self.original_x_goal = 0
        self.original_y_goal = 0
        self.computed_x_goal = 0
        self.computed_y_goal = 0
        
    # Decision Methods
    def turn_heading(self, direction, curr_intersection, new_heading):
        i = (self.heading + direction) % 8
        while i != new_heading:
            curr_intersection.streets[i] = Status.NONEXISTENT
            i = (i + direction) % 8
    
    # Check whther the middle-facing street ahead is blocked
    def check_ahead(self):
        (left, middle, right) = self.proximity_sensor.read()

        if middle <= 0.6:
            return True
        else:
            return False
    
    def mark_blocked(self, x, y, heading):
        is_blocked = self.check_ahead()

        curr_intersection = self.robot_map.getintersection(x, y)
        curr_intersection.blocked[heading] = is_blocked

        if curr_intersection.streets[heading] == Status.CONNECTED:
            dx = heading_to_dx[heading]
            dy = heading_to_dy[heading]

            new_x = x + dx
            new_y = y + dy

            connecting_intersection = self.robot_map.getintersection(new_x, new_y)
            connecting_intersection.blocked[(heading + 4) % 8] = is_blocked

    def find_closest_unknown(self, curr_intersection, to_look):
        direction = self.heading
        # First check for unknown, unblocked streets
        for i in range(5):
            if curr_intersection.streets[direction] != Status.UNKNOWN or curr_intersection.blocked[direction]:
                direction = (direction + to_look) % 8
            else:
                return direction

        return None

    def find_closest_unexplored(self, curr_intersection, to_look):
        direction = self.heading
        for i in range(5):
            if curr_intersection.streets[direction] != Status.UNEXPLORED or curr_intersection.blocked[direction]:
                direction = (direction + to_look) % 8
            else:
                return direction

        return None

    def check_intersection_unknown(self, curr_intersection):
        for i in range(8):
            if curr_intersection.streets[i] == Status.UNKNOWN and not curr_intersection.blocked[i]:
                return True

        return False

    def find_closest_unfinished_intersection(self):
        closest_intersection = None
        dist = inf

        for intersection in self.robot_map.intersections.values():
            if not intersection.is_finished and not (intersection.x, intersection.y) in self.unreachable_intersections and (intersection.x != self.x or intersection.y != self.y):
                new_dist = sqrt((intersection.x - self.x)**2 + (intersection.y - self.y)**2)
                if new_dist < dist:
                    closest_intersection = intersection
                    dist = new_dist

        return closest_intersection

    def check_intersection_unexplored(self, curr_intersection):
        for i in range(8):
            if curr_intersection.streets[i] == Status.UNEXPLORED and not curr_intersection.blocked[i]:
                return True

        return False

    def find_closest_unfinished_intersection_to_goal(self, x_goal, y_goal):
        dist = inf
        closest_intersection = None

        for intersection in self.robot_map.intersections.values():
            if Status.UNEXPLORED in intersection.streets:
                new_dist = math.sqrt((x_goal - intersection.x)**2 + (y_goal - intersection.y)**2)
                if new_dist < dist:
                    dist = new_dist
                    closest_intersection = intersection
        
        return closest_intersection

    # Description: Finds the closest unkown or unexplored heading
    # Return ----
    #  L: turn left
    #  R: turn right
    #  None: at desired heading no action needed
    def find_closest_heading(self, decision, curr_intersection):
        if decision == "Unknown":
            left_direction = self.find_closest_unknown(curr_intersection, 1)
            right_direction = self.find_closest_unknown(curr_intersection, -1)
        elif decision == "Unexplored":
            left_direction = self.find_closest_unexplored(curr_intersection, 1)
            right_direction = self.find_closest_unexplored(curr_intersection, -1)

        if left_direction == self.heading or right_direction == self.heading:
            return "S" # At desired heading, no action needed
        elif left_direction == None:
            return "R"
        elif right_direction == None:
            return "L"
        else:
            rounded_left = 8 * np.round(left_direction/8)
            rounded_right = 8 * np.round(right_direction/8)

            if rounded_left <= rounded_right:
                return "L"
            else:
                return "R"

    def compute_dijkstra(self, x_goal, y_goal):
        if (x_goal, y_goal) not in self.robot_map.intersections:
            self.directed = True
            goal = self.find_closest_unfinished_intersection_to_goal(x_goal, y_goal)
        else:
            goal = self.robot_map.getintersection(x_goal, y_goal)
        
        self.computed_x_goal = goal.x
        self.computed_y_goal = goal.y
        self.original_x_goal = x_goal
        self.original_y_goal = y_goal

        self.robot_map.dijkstras(goal)
        self.dijkstra_finished = False
        self.robot_map.show(self.x, self.y, self.heading)

    def dijkstra_next(self, x_goal, y_goal, turn_counter):
        if turn_counter < 2:
            if self.x != x_goal or self.y != y_goal:
                curr_intersection = self.robot_map.getintersection(self.x, self.y)

                # Check if path is possible
                if curr_intersection.cost == inf:
                    self.robot_map.reset_blockages()
                    self.compute_dijkstra(x_goal, y_goal)

                    curr_intersection = self.robot_map.getintersection(self.x, self.y)

                    if curr_intersection.cost == inf:
                        print("Can't find path")
                        self.unreachable_intersections.append((self.next_intersection.x, self.next_intersection.y))
                        self.dijkstra_finished = False
                        return None # Path is not possible, don't move

                    return self.dijkstra_next(x_goal, y_goal, turn_counter)

                # Align heading
                if self.heading != curr_intersection.direction:
                    if ((curr_intersection.direction - self.heading) % 8) >= 4:
                        return "R"
                    else:
                        return "L"

                # Check if there are any blocked streets. Recalculate dijkstra's if blockages found
                if curr_intersection.blocked[self.heading]:
                    self.robot_map.reset_blockages()
                    self.compute_dijkstra(x_goal, y_goal)
                    self.dijkstra_next(x_goal, y_goal, turn_counter)

                # Then go straight
                if curr_intersection.streets[self.heading] != Status.NONEXISTENT and curr_intersection.streets[self.heading] != Status.DEADEND:
                    return "S"
            else: 
                # Already reached goal, don't move
                self.next_intersection = None
                self.dijkstra_finished = True
                
                if self.directed:
                    return self.directed_explore_next(self.original_x_goal, self.original_y_goal, turn_counter)
                
                self.robot_map.reset_path()
                return None
        else:
            return "S"

    def directed_explore_next(self, x_goal, y_goal, turn_counter):
        if turn_counter < 3:
            if self.x != x_goal or self.y != y_goal:
                x_dist = x_goal - self.x
                y_dist = y_goal - self.y

                choose_street = False
                
                dx = heading_to_dx[self.heading]
                dy = heading_to_dy[self.heading]

                curr_intersection = self.robot_map.getintersection(self.x, self.y)
                self.robot_map.unfinished_intersections()

                if curr_intersection.is_finished:
                    print("Reached a completely explored intersection.")
                    print("x_goal: ", x_goal)
                    print("y_goal: ", y_goal)
                    self.directed = False
                    return self.next_move(0, True, self.original_x_goal, self.original_y_goal)
                if Status.UNKNOWN not in curr_intersection.streets:
                    print("Intersection has no unknown streets. Finding closest unexplored street.")
                    direction = self.find_closest_heading("Unexplored", curr_intersection)

                    if direction != None:
                        return direction
                    elif direction == None:
                        return "S"
                elif ((np.sign(dx) == np.sign(x_dist) or np.sign(dx) == 0) and (np.sign(dy) == np.sign(y_dist) or np.sign(dy) == 0) and not curr_intersection.blocked[self.heading] and curr_intersection.streets[self.heading] != Status.NONEXISTENT):
                    print("Correctly aligned! Going straight.")
                    if curr_intersection.streets[self.heading] == Status.DEADEND:
                        print("Facing dead end! Turning for information.")
                        direction = self.find_closest_heading("Unknown", curr_intersection)

                        if direction != None:
                            return direction
                        elif direction == None:
                            return "S"
                    return "S"
                else:
                    print("Incorrectly aligned! Turning for information.")
                    direction = self.find_closest_heading("Unknown", curr_intersection)

                    if direction != None:
                        return direction
                    elif direction == None:
                        return "S"
            else:
                print("Reached goal!")
                print("x_goal: ", x_goal)
                print("y_goal: ", y_goal)
                print("x original: ", self.original_x_goal)
                print("y original: ", self.original_y_goal)
                print("x computed: ", self.computed_x_goal)
                print("y computed: ", self.computed_y_goal)
                self.original_x_goal = 0
                self.original_y_goal = 0
                self.computed_x_goal = 0
                self.computed_y_goal = 0
                self.directed = False
                self.robot_map.reset_path()
                return None
        else:
            return "S"

    def explore_dijkstra(self, turn_counter):
        self.robot_map.unfinished_intersections()
        if self.next_intersection == None:
            self.next_intersection = self.find_closest_unfinished_intersection()

            if self.next_intersection == None:
                self.dijkstra_finished = True
                return None
            self.compute_dijkstra(self.next_intersection.x, self.next_intersection.y)

        return self.dijkstra_next(self.next_intersection.x, self.next_intersection.y, turn_counter)

    def auto_explore_next(self, turn_counter):
        # Execute dijkstra's even if turn count is >= 1
        if self.first_intersection:
            return "S"

        if self.next_intersection != None:
            direction = self.explore_dijkstra(turn_counter)
            if direction == None:
                if self.dijkstra_finished: # Finished dijkstra path to next intersection continue exploring
                    return self.auto_explore_next(turn_counter)
                else:
                    return None
            else:
                return direction

        if turn_counter < 1:
            if (self.x, self.y) not in self.robot_map.intersections:
                return None
            curr_intersection = self.robot_map.getintersection(self.x, self.y)
            contains_unknown_street = self.check_intersection_unknown(curr_intersection)
            contains_unexplored_street = self.check_intersection_unexplored(curr_intersection)
            self.robot_map.unfinished_intersections()

            if contains_unexplored_street or contains_unknown_street:
                if contains_unknown_street:
                    direction = self.find_closest_heading("Unknown", curr_intersection)
                    
                    if direction != None:
                        return direction
                    elif direction == None:
                        return "S"

                elif contains_unexplored_street:
                    direction = self.find_closest_heading("Unexplored", curr_intersection)

                    if direction != None:
                        return direction
                    elif direction == None:
                        return "S"
            elif self.robot_map.is_finished():
                print("Done!")
                return None
            else:
                direction = self.explore_dijkstra(turn_counter)
                if direction == None:
                    if self.dijkstra_finished: # Finished dijkstra path to next intersection continue exploring
                        return self.auto_explore_next(turn_counter)
                    else:
                        return None
                else:
                    return direction
        else:
            return "S"

        return None

    def next_move(self, turn_counter, goal_changed, x_goal=None, y_goal=None):
        if x_goal is None and y_goal is None: # Pure auto explore
            return self.auto_explore_next(turn_counter)
        else: # goto and directed exploring
            if self.robot_map.is_finished():
                self.robot_map.reset_blockages()
            elif goal_changed:
                self.compute_dijkstra(x_goal, y_goal)
            elif not self.dijkstra_finished:
                return self.dijkstra_next(self.computed_x_goal, self.computed_y_goal, turn_counter)
            elif self.directed:
                return self.directed_explore_next(x_goal, y_goal, turn_counter)

            return self.dijkstra_next(self.computed_x_goal, self.computed_y_goal, turn_counter)

    # Execute Methods
    def execute_turn(self, direction):
        curr_intersection = self.robot_map.getintersection(self.x, self.y)
        turning(self.drive, self.sensor, direction)
        self.drive.stop()

        # Delay the robot before the sensor reading
        t0 = time.time()
        t = t0
        while t - t0 < T_DELAY:
            t = time.time()
        
        end_angle = self.angle_sensor.readangle()
            
        turn = round(((self.start_angle - end_angle) / 45) % 8)

        if turn > 4:
            turn = turn - 8

        new_heading = self.robot_map.calcturn(self.start_heading, turn)

        # Update the current intersection variable
        if direction == "R":
            self.turn_heading(-1, curr_intersection, new_heading)
        elif direction == "L":
            self.turn_heading(1, curr_intersection, new_heading)

        # Check whether the current street is blocked
        self.mark_blocked(self.x, self.y, new_heading)

        self.robot_map.setintersection(self.x, self.y, curr_intersection)

        # Update the robot's heading
        self.heading = new_heading

        # Update map visualization
        if curr_intersection.streets[new_heading] == Status.UNKNOWN or curr_intersection.streets[new_heading] == Status.NONEXISTENT:
            curr_intersection.streets[new_heading] = Status.UNEXPLORED

        self.robot_map.show(self.x, self.y, self.heading)

    def execute_uturn(self, pos):
        self.drive.stop()
        t0 = time.time()
        t = t0
        while t - t0 < T_DELAY:
            t = time.time()

        if pos == Position.BLOCKED:
            self.mark_blocked(self.x, self.y, self.heading)

        turning(self.drive, self.sensor, "R")

        # Update the robot pose
        self.heading = self.robot_map.calcuturn(self.heading)

        # Update the current intersection with dead end
        if pos == Position.END_OF_STREET:
            curr_intersection = self.robot_map.getintersection(self.x, self.y)
            curr_intersection.streets[(self.heading + 4) % 8] = Status.DEADEND 
            
            # Check whether the current street is blocked
            self.mark_blocked(self.x, self.y, (self.heading + 4) % 8)
        
        self.robot_map.show(self.x, self.y, self.heading)

    def execute_intersection(self):
        if self.prev_pos != None and not self.first_intersection:
            prev_intersection = self.robot_map.getintersection(self.x, self.y)

        # Updates robot pose
        if self.prev_pos == Position.INTERSECTION: # Only update the position if an intersection has been reached
            # Update the position of the intersection
            (self.x,self.y) = self.robot_map.calcmove(self.x, self.y, self.heading)
            # Get rid of internal angles
            # Update the current intersection
            
            curr_intersection = self.robot_map.getintersection(self.x, self.y)
            
            curr_intersection.streets[(self.heading - 3) % 8] = Status.NONEXISTENT
            curr_intersection.streets[(self.heading - 5) % 8] = Status.NONEXISTENT
        
        curr_intersection = self.robot_map.getintersection(self.x, self.y)

        # Pull forward, and check if the robot is on the road
        if curr_intersection.streets[self.heading] == Status.NONEXISTENT:
            is_on_road = pull_forward(self.drive, self.sensor, False)
        else:
            is_on_road = pull_forward(self.drive, self.sensor, True)

        if self.prev_pos == Position.INTERSECTION and self.prev_pos != None and not self.first_intersection:
            # Connect current intersection to previous intersection
            prev_intersection.streets[self.heading] = Status.CONNECTED
            curr_intersection.streets[(self.heading + 4) % 8] = Status.CONNECTED

        # Update current heading value with pull_forward output
        if is_on_road and curr_intersection.streets[self.heading] == Status.UNKNOWN:
            curr_intersection.streets[self.heading] = Status.UNEXPLORED
        elif not is_on_road:
            curr_intersection.streets[self.heading] = Status.NONEXISTENT

        # Check whether the current street is blocked
        self.mark_blocked(self.x, self.y, self.heading)

        # Get the starting angle and heading for the intersection
        self.start_heading = self.heading
        self.start_angle = self.angle_sensor.readangle()

        # Add intersection to dictionary
        self.robot_map.setintersection(self.x, self.y, curr_intersection)

        if self.first_intersection:
            if curr_intersection.streets[(self.heading + 4) % 8] != Status.CONNECTED:
                curr_intersection.streets[(self.heading + 4) % 8] = Status.UNEXPLORED

            if self.prev_pos == Position.END_OF_STREET:
                curr_intersection.streets[(self.heading + 4) % 8] = Status.DEADEND
            
            self.first_intersection = False
        
        self.robot_map.show(self.x, self.y, self.heading)
    
    def execute_straight(self):
        if (self.x, self.y) in self.robot_map.intersections:
            curr_intersection = self.robot_map.getintersection(self.x, self.y)
        else:
            curr_intersection = None

        # Check that you can go straight down the current intersection
        if curr_intersection == None or (curr_intersection.streets[self.heading] != Status.NONEXISTENT and curr_intersection.streets[self.heading] != Status.DEADEND 
            and curr_intersection.blocked[self.heading] == False):
            pos = line_following(self.drive, self.sensor, self.proximity_sensor)
            if pos == Position.INTERSECTION:
                self.drive.stop()
                self.prev_pos = pos
                self.execute_intersection()
            elif pos == Position.END_OF_STREET or pos == Position.BLOCKED:
                self.execute_uturn(pos)
                self.prev_pos = pos
                new_pos = line_following(self.drive, self.sensor, self.proximity_sensor)
                if new_pos == Position.INTERSECTION:
                    self.execute_intersection()
                elif new_pos == Position.BLOCKED:
                    self.drive.stop()
                    while True:
                        (left, middle, right) = self.proximity_sensor.read()
                        if middle > 0.35:
                            break
                    line_following(self.drive, self.sensor, self.proximity_sensor)
                    self.execute_intersection()

    # Stop the robot and shutdown the ultrasound trigger thread
    def shutdown(self):
        self.drive.stop()
        self.proximity_sensor.shutdown()
    