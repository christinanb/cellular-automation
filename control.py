import numpy as np
from environment import *
from units import *
from visual import *

"""
Controls the state of the Field.
"""
class PedestrianController:

    """
    Initialize the state of the Field.

    @param width: x-dimension of the Field.
    @param height: y-dimension of the Field.
    @param pedestrians_loc: Starting-Locations of the pedestrians.
                            (x, y): x in [1, width], y in [1, height].
    @param targets_loc: Locations of the targets. 
                            (x, y): x in [1, width], y in [1, height].
    @param obstacles_loc: Locations of the obstacles. 
                            (x, y): x in [1, width], y in [1, height].
    """
    def __init__(self, width, height, pedestrians_loc, targets_loc, obstacles_loc, speed, max_timesteps, devour=False, dijkstra=False, verbose_visualization=False):
        self.field = Field(width, height)
        self.pedestrians = [Pedestrian(self.field.cells[x, y], speed) for (x, y) in pedestrians_loc]
        self.targets = [Target(self.field.cells[x, y]) for (x, y) in targets_loc]
        self.obstacles = [Obstacle(self.field.cells[x, y]) for (x, y) in obstacles_loc]
        self.speed = speed
        self.max_timesteps = max_timesteps
        self.devour = devour
        self.target_cost_calculation = CostUpdate.dijkstra if dijkstra else CostUpdate.distance

        self.field_visual = FieldVisual(width, height, verbose_visualization)
    
    """
    Initialize the costs of targets and obstacles since these values do not change within the course of a simulation.
    Calculate targettt-related costs according to selected target cost-calculation.
    """
    def init_costs(self):
        obstacle_cost = self.field.width * self.field.height * 2

        for obstacle in self.obstacles:
            obstacle.cell.static_cost = obstacle_cost

        for target in self.targets:
            self.target_cost_calculation(target.cell, self.field)

    """
    Run the simulation.
    """
    def run(self):
        while True:
            self._update()

    """
    Update the simulation once.
    This method is to be executed per time frame only by the method run().
    """
    def _update(self):
        remove_pedestrians = []
        for p in self.pedestrians:
            optimal_neighbor = self.find_optimal_neighbor(p)
            p.move_in_time(optimal_neighbor)
            # devour pedestrians who have reached a target
            if p.cell in [t.cell for t in self.targets]:
                print("Elapsed time:", time.time() - p.first_movement_timestamp,  "s")
                if self.devour:
                    remove_pedestrians.append(p)
        if len(remove_pedestrians) > 0:
            self.pedestrians = [p for p in self.pedestrians if p not in remove_pedestrians]
        self.field_visual.draw_update(self.field, self.pedestrians, self.obstacles, self.targets)

    """
    Find the best reachable neighbor for pedestrian p according to the calculated cost-functions.

    @param pedestrian: Pedestrian to find optimal neighbor-cell for next movement.
    """
    def find_optimal_neighbor(self, pedestrian):
        avail_neighbor_pedestrian_costs = pedestrian.calc_pedestrian_cost(self.pedestrians)
        min_neighbor_cost = None
        optimal_neighbor = None
        for i, neighbor in enumerate(pedestrian.cell.get_avail_neighbors()):
            neighbor_cost = neighbor.static_cost + avail_neighbor_pedestrian_costs[i]
            if min_neighbor_cost is None or neighbor_cost < min_neighbor_cost:
                min_neighbor_cost = neighbor_cost
                optimal_neighbor = neighbor
        return optimal_neighbor

"""
Specifies various strategies on how to calculate the cost of targets.
Assigns them in-place.
"""
class CostUpdate:

    """
    Calculates the distance-cost purely depending on the euclidian distance from the target cell.

    @param target: Cell to calculate cost from.
    @param field: Field to operate on.
    """
    def distance(target: Cell, field):
        for cell in field.cells.flatten():
            cell.static_cost += np.linalg.norm(cell.loc - target.loc)

    """
    Calculates the distance-cost according to the number of steps needed to reach the target.

    @param target: Cell to calculate cost from.
    @param field: Field to operate on.
    """
    def dijkstra(target: Cell, field):
        visited_cells = [target]
        border_cells = [(target, 1)]
        while len(border_cells) > 0:
            new_border_cells = []
            for cell, steps in border_cells:
                for n in cell.get_avail_neighbors():
                    if n not in new_border_cells and n.static_cost == 0:
                        visited_cells.append(n)
                        travel_cost = 1
                        if not any(n.loc == cell.loc):
                            travel_cost = np.sqrt(2)
                        n.static_cost = steps + travel_cost
                        new_border_cells.append((n, steps + travel_cost))
            border_cells = new_border_cells
            steps += 1



# circle starting-point, exercise 4
controller = PedestrianController(50, 50, [(5, 25)],#, (25, 5), (33, 7), (7, 33), (10, 12)], 
                                    [(25, 25)], [(20, 23), (20, 24), (20, 25), 
                                    (20, 26), (20, 27)], 3.0, 10000, dijkstra=True, devour=True, verbose_visualization=True)
controller.init_costs()
controller.run()

