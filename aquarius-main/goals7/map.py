import matplotlib.pyplot as plt
from intersection import Status, Intersection
from math import cos, sin, radians, inf, sqrt
from enum import Enum
import pickle
import bisect

class Heading(Enum):
    NORTH = 0
    NORTHWEST = 1
    WEST = 2
    SOUTHWEST = 3
    SOUTH = 4
    SOUTHEAST = 5
    EAST = 6
    NORTHEAST = 7

heading_to_dx = {0: 0,
                 1: -1,
                 2: -1,
                 3: -1,
                 4: 0,
                 5: 1,
                 6: 1,
                 7: 1}

heading_to_dy = {0: 1, 
                 1: 1, 
                 2: 0, 
                 3: -1, 
                 4: -1, 
                 5: -1, 
                 6: 0, 
                 7: 1}

status_colors = {Status.UNKNOWN: 'black',
                 Status.NONEXISTENT: 'lightgray',
                 Status.UNEXPLORED: 'blue',
                 Status.DEADEND: 'red',
                 Status.CONNECTED: 'green'}

class Map:
    def __init__(self):
        # Start with an empty dictionary
        self.intersections = {}
        self.axes = None
        
    # Show the x/y/heading of the current robot pose.
    def show(self, xpose, ypose, hpose):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-4.5, 4.5)
        plt.gca().set_ylim(-4.5, 4.5)
        plt.gca().set_aspect('equal')

        # Show all the possible locations.
        for x in range(-4, 5):
            for y in range(-4, 5):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)
        
        # PUT ALL THE ACTUAL DRAWING FUNCTIONS HERE!
        dx = 0.5 * cos(radians(hpose * 45 + 90))
        dy = 0.5 * sin(radians(hpose * 45 + 90))
        xbase = xpose - dx/2
        ybase = ypose - dy/2
        plt.arrow(xbase, ybase, dx, dy,
                    width=0.2,
                    head_width=0.3,
                    head_length=0.1,
                    color='magenta')
        
        for (x,y) in self.intersections:
            intersection = self.intersections[(x, y)]
            for h in range(8):
                xto = 0.5 * heading_to_dx[h]
                yto = 0.5 * heading_to_dy[h]
                color = status_colors[intersection.streets[h]]    
                if intersection.blocked[h]:  
                    plt.plot([x, x + xto], [y, y + yto], color=color, linestyle=":")
                else:
                    plt.plot([x, x + xto], [y, y + yto], color=color, linestyle="solid")
            optimal_heading = intersection.direction
            if optimal_heading is not None:
                dx = heading_to_dx[optimal_heading]
                dy = heading_to_dy[optimal_heading]
                plt.arrow(x, y, 0.5 * dx, 0.5 * dy,
                            width=0.1,
                            head_width=0.25,
                            head_length=0.1,
                            color='orange')
        
        # Show the graph and continue.
        plt.savefig("map.png")

    # Grab the specified intersection
    def getintersection(self, x, y):
        # Create only if intersection hasn't been used before.
        if (x, y) not in self.intersections:
            self.intersections[(x,y)] = Intersection(x,y,inf)
        # Return the intersection
        return self.intersections[(x,y)]

    def setintersection(self, x, y, intersection):
        self.intersections[(x, y)] = intersection

    def unfinished_intersections(self):
        for intersection in self.intersections.values():
            intersection.unfinished_intersection()

    def calcmove(self, xold, yold, heading):
        dx = heading_to_dx[heading]
        dy = heading_to_dy[heading]
        return (xold + dx, yold + dy)

    def calcturn(self, oldheading, turn):
        return (oldheading + turn) % 8

    def calcuturn(self, oldheading):
        return self.calcturn(oldheading, 4)
    
    def save(self, filename):
         # Save the map to file.
        print("Saving the map to %s..." % filename)
        with open(filename, 'wb') as file:
            pickle.dump(self, file)

    def load(self, filename):
        print("Loading the map from %s..." % filename)
        with open(filename, 'rb') as file:
            robot_map = pickle.load(file)
        return robot_map

    def is_finished(self):
        for intersection in self.intersections.values():
            if intersection.is_finished == False:
                return False

        return True

    def reset_blockages(self):
        for intersection in self.intersections.values():
            intersection.clear_blockages()

    def reset_path(self):
        for intersection in self.intersections.values():
            intersection.cost = inf
            intersection.direction = None

    def find_closest_neighbor(self, node, to_look):        
        heading = (node.direction + to_look) % 8
        count = 0
        while node.streets[heading] != Status.CONNECTED and node.streets[heading] != Status.UNEXPLORED:
            if count > 8:
                return None
            heading = (heading + to_look) % 8
            count += 1

        return heading

    def dijkstras(self, goal):
        # Reset the path. Move all nodes to "air": Mark all costs as infinity, all directions as None
        self.reset_path()
        
        # Initialize. Place goal onDeck with a zero cost, retaining the None direction
        goal.cost = 0
        onDeck = [goal]
        
        # While the onDeck queue is not empty, loop:
        while len(onDeck) > 0:

            # Pop the current node off the front of the onDeck queue (lowest onDeck cost).
            current = onDeck.pop(0)

            for heading in range(8):
                # Check if the given heading is connected to the current intersection
                if (current.streets[heading] == Status.CONNECTED or current.streets[heading] == Status.UNEXPLORED) and current.blocked[heading] == False:
                    dx = heading_to_dx[heading]
                    dy = heading_to_dy[heading]
                
                    if (current.x + dx, current.y + dy) not in self.intersections.keys():
                        continue
                    neighbor = self.getintersection(current.x + dx, current.y + dy)
                    new_cost = current.cost + sqrt(dx**2 + dy**2)
                    
                    # If found something with lower cost:
                    if new_cost < neighbor.cost:
                        if neighbor in onDeck:
                            # Temporarily remove neighbor from the onDeck queue (might need to be re-sorted)
                            onDeck.remove(neighbor)
                        
                        # Save the new cost and direction in the neighbor
                        neighbor.cost = new_cost
                        neighbor.direction = (heading + 4) % 8
                        
                        # Insert the neighbor into the onDeck queue in the right oreder.
                        bisect.insort(onDeck, neighbor)
                    # Update the interesection variable with new costs for neighbor
                    self.setintersection(current.x + dx, current.y + dy, neighbor)         