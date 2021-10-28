import numpy as np
import time

"""
Movable object within cell-grid.
Moves towards defined targets.
Is unable to move towards obstacles.
"""
class Pedestrian:

    """
    Creates a pedestrian with given speed on a given cell.

    @param cell: Cell to spawn to.
    @param speed: Movement speed of pedestrian.
    """
    def __init__(self, cell, speed):
        self.cell = cell
        self.speed = speed
        self.last_movement_timestamp = time.time()
        self.next_movement_timestamp = time.time() + 2

    """
    Checks if planned cell is reachable until the current time.

    @param planned_cell: The cell which movement is planned to.
    @return: If movement is legal given elapsed time since last movement.
    
    """
    def time_to_move_to(self, planned_cell):
        planned_distance = np.linalg.norm(self.cell.loc - planned_cell.loc)
        self.next_movement_timestamp = self.last_movement_timestamp + planned_distance / self.speed
        return time.time() >= self.next_movement_timestamp

    """
    Moves object to cell considering the time to execute the movement.

    @param cell: Cell to move to, if possible within elapsed time since last movement.
    """
    def move_in_time(self, cell):
        if self.time_to_move_to(cell):
            self.last_movement_timestamp = time.time()
            self.cell = cell

    """
    Determines the cost of neighboring fields depending on other pedestrians.

    @param pedestrians: Other pedestrians which influence the cost of neighboring fields.
    @param r_max: Parameter for distribution of cost around other pedestrians.
    """
    def calc_pedestrian_cost(self, pedestrians, r_max=2):
        neighbor_pedestrian_cost = np.zeros(len(self.cell.get_avail_neighbors()))
        for i, neighbor_cell in enumerate(self.cell.get_avail_neighbors()):
            for p in pedestrians:
                if p is not self:
                    dist = np.linalg.norm(neighbor_cell.loc - p.cell.loc)
                    if dist == 0:
                        neighbor_pedestrian_cost[i] += 100000
                    elif dist < r_max:
                        neighbor_pedestrian_cost[i] += np.exp(1/(dist**2 - r_max**2))
        return neighbor_pedestrian_cost

"""
Object which pedestrians want to reach.
"""
class Target:

    """
    Create a target at given cell.

    @param cell: Cell on which to spawn target.
    """
    def __init__(self, cell):
        self.cell = cell

"""
Object which pedestrians cannot pass.
"""
class Obstacle:

    """
    Create an obstacle at given cell.

    @param cell: Cell on which to spawn obstacle.
    """
    def __init__(self, cell):
        self.cell = cell