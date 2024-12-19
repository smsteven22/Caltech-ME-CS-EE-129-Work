# Imports
from enum import Enum
from math import inf

class Status(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

class Intersection:
    def __init__(self, x=0, y=0, cost=inf, direction=None):
        self.x = x
        self.y = y
        self.streets = []
        self.cost = cost
        self.direction = direction
        self.is_finished = False
        
        for i in range(8):
            self.streets.append(Status.UNKNOWN)

    # Define the "less-than" to enable sorting by cost
    def __lt__(self, other):
        return self.cost < other.cost

    def unfinished_intersection(self):
        for street in self.streets:
            if street == Status.UNKNOWN or street == Status.UNEXPLORED:
                self.is_finished = False
                return
        
        self.is_finished = True
