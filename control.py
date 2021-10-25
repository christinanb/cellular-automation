import numpy as np
from environment import *
from units import *
from visual import FieldVisual

'''
Controls the movement of all pedestrians in the Field.
'''
class PedestrianController:

    def __init__(self, width, height, pedestrians_loc, targets_loc, obstacles_loc, speed, n_timesteps, devour=False):
        self.field = Field(width, height)
        self.pedestrians = [Pedestrian(self.field.cells[x, y], speed) for (x, y) in pedestrians_loc]
        self.targets = [Target(self.field.cells[x, y]) for (x, y) in targets_loc]
        self.obstacles = [Obstacle(self.field.cells[x, y]) for (x, y) in obstacles_loc]
        self.speed = speed
        self.n_timesteps = n_timesteps
        self.timestep = 0
        self.devour = devour

        self.field_visual = FieldVisual(width, height)
    
    '''
    Initialize the costs of targets and obstacles since these values do not change within the course of a simulation.
    '''
    def init_costs(self):
        obstacle_cost = 10000

        for cell in self.field.cells.flatten():
            for target in self.targets:
                cell.static_cost += self.cost_function(np.linalg.norm(cell.loc - target.cell.loc))
            for obstacle in self.obstacles:
                obstacle.cell.static_cost += obstacle_cost

    '''
    The cost-function used for the targets.
    '''
    def cost_function(self, dist):
        return dist

    def run(self):
        while True:
            self.update()

    def update(self):
        for p in self.pedestrians:
            optimal_neighbor = self.find_optimal_neighbor(p)
            p.move(optimal_neighbor)
        self.timestep += 1
        self.field_visual.draw_update(self.pedestrians, self.obstacles, self.targets)

    '''
    Find the best reachable neighbor according to the calculated cost-functions
    '''
    def find_optimal_neighbor(self, p):
        avail_neighbor_pedestrian_costs = p.calc_pedestrian_cost(self.pedestrians)
        min_neighbor_cost = None
        optimal_neighbor = None
        for i, neighbor in enumerate(p.cell.get_avail_neighbors()):
            neighbor_cost = neighbor.static_cost + avail_neighbor_pedestrian_costs[i]
            if min_neighbor_cost is None or neighbor_cost < min_neighbor_cost:
                min_neighbor_cost = neighbor_cost
                optimal_neighbor = neighbor
        return optimal_neighbor

# circle starting-point, exercise 4
controller = PedestrianController(50, 50, [(5, 25), (25, 5), (33, 7),
                                    (7, 33), (10, 12)], [(25, 25)], [(20, 23), (20, 24), (20, 25)], 1.0, 100)
controller.init_costs()
controller.run()

