import numpy as np
import time

'''
Movable object within cell-grid.
Moves towards defined targets.
Is unable to move towards obstacles.
'''
class Pedestrian:

    def __init__(self, cell, speed):
        self.cell = cell
        self.speed = speed
        self.last_movement_timestamp = time.time()
        self.next_movement_timestamp = time.time() + 5

    '''
    Checks if planned cell is reachable until the current time
    '''
    def time_to_move_to(self, planned_cell):
        time_scaling = 1
        planned_distance = np.linalg.norm(self.cell.loc - planned_cell.loc)
        self.next_movement_timestamp = self.last_movement_timestamp + planned_distance / self.speed * time_scaling
        return time.time() >= self.next_movement_timestamp

    '''
    Moves object to cell
    '''
    def move(self, cell):
        if self.time_to_move_to(cell):
            self.last_movement_timestamp = time.time()
            self.cell = cell

    '''
    Determines the cost of neighboring fields depending on other pedestrians
    '''
    def calc_pedestrian_cost(self, pedestrians):
        r_max = 1
        neighbor_pedestrian_cost = np.zeros(len(self.cell.get_avail_neighbors()))
        for i, neighbor_cell in enumerate(self.cell.get_avail_neighbors()):
            for p in pedestrians:
                if p is not self:
                    dist = np.linalg.norm(neighbor_cell.loc - p.cell.loc)
                    neighbor_pedestrian_cost[i] += np.exp(1/(dist**2 - r_max**2))
        return neighbor_pedestrian_cost


class Target:

    def __init__(self, cell):
        self.cell = cell

class Obstacle:

    def __init__(self, cell):
        self.cell = cell