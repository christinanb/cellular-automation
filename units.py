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
    @param max_steps: Amount of steps to take.
        -1 if no step-restriction is to be applied.
    """
    def __init__(self, cell, speed, max_steps,i):
        self.cell = cell
        self.speed = speed
        self.steps_left = max_steps
        self.identity= i
        self.last_movement_timestamp = time.time() + 2
        self.first_movement_timestamp = self.last_movement_timestamp
        self.next_movement_timestamp = None

        # Used for calculation of speed with density of an area
        self.enter_time = 0
        self.exit_time = 0


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
        if self.steps_left != 0 and self.time_to_move_to(cell):
            self.last_movement_timestamp = time.time()
            self.cell = cell
            self.steps_left -= 1

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

"""
Object which pedestrians pass for measuring a certain characteristic.
"""
class Point:

    """
    Create a measuring point at given cell.

    @param cell: Cell on which to spawn obstacle.
    """
    def __init__(self, cell):
        self.cell = cell

"""
Area created from two coordinates to measure its density.
"""
class Area:

    """
    Creates an area with two coordinates.

    @param top_left: Cell on which the desired area starts from the top left.
    @param bottom_right: Cell on which the desired area ends from the bottom right.
    @param density: The area's density
    """
    def __init__(self, top_left, bottom_right):
        self.top_left = top_left
        self.bottom_right = bottom_right

        self.range_x = np.array([top_left.loc[0], bottom_right.loc[0]])
        self.range_y = np.array([top_left.loc[1], bottom_right.loc[1]])

        self.pedestrians = []
        self.area = (self.range_x[1] - self.range_x[0]) * (self.range_y[0] - self.range_y[1])
        self.density = len(self.pedestrians) / self.area

        self.coordinates = []

    """
    Checks if a pedestrian is inside the area by verifying the x coordinates.

    @param pedestrian: Pedestrian that we want to check.
    """
    def is_inside(self, pedestrian):
        if pedestrian.cell.loc[0] == self.range_x[0] and pedestrian not in self.pedestrians:
                self.pedestrians.append(pedestrian)
                pedestrian.enter_time = time.time()
                self.update()
        elif pedestrian.cell.loc[0] == self.range_x[1] and pedestrian in self.pedestrians:
            self.pedestrians.remove(pedestrian)
            pedestrian.exit_time = time.time()
            self.calculate_speed(pedestrian)
            self.update()

    """
    Updates the density of the area.
    """
    def update(self):
        self.density = len(self.pedestrians) / self.area
    
    """
    Calculates the new speed of a pedestrian depending on the area's length and the elapsed time.

    @param pedestrian: Pedestrian that we want to check speed.
    """
    def calculate_speed(self, pedestrian):
        elapsed_time = abs(pedestrian.exit_time - pedestrian.enter_time)
        if elapsed_time != 0:
            length = abs(self.range_x[1] - self.range_x[0])
            speed = (length + 1) / elapsed_time
            self.coordinates.append([speed, self.density])
