import matplotlib.pyplot as plt
from intersection import Intersection, Status
from math import cos, sin, radians
from enum import Enum

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
        

    # Show the x/y/heading of the current robot pose.
    def show(self, xpose, ypose, hpose):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect('equal')

        # Show all the possible locations.
        for x in range(-3, 4):
            for y in range(-3, 4):
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
            for h in range(8):
                intersection = self.intersections[(x,y)]
                xto = 0.5 * heading_to_dx[h]
                yto = 0.5 * heading_to_dy[h]
                color = status_colors[intersection.streets[h]]      
                plt.plot([x, x + xto], [y, y + yto], color=color)
        
        # Show the graph and continue.
        plt.pause(0.001)

    # Grab the specified intersection
    def getintersection(self, x, y):
        # Create only if intersection hasn't been used before.
        if (x, y) not in self.intersections:
            self.intersections[(x,y)] = Intersection(x,y)
        # Return the intersection
        return self.intersections[(x,y)]

    def setintersection(self, x, y, intersection):
        self.intersections[(x, y)] = intersection

    def calcmove(self, xold, yold, heading):
        dx = heading_to_dx[heading]
        dy = heading_to_dy[heading]
        return (xold + dx, yold + dy)

    def calcturn(self, oldheading, turn):
        return (oldheading + turn) % 8

    def calcuturn(self, oldheading):
        return self.calcturn(oldheading, 4)
