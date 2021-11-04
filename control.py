import numpy as np
from environment import *
from units import *
import visual
import sys

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
    @param points_loc: Locations of the measuring points. 
                            (x, y): x in [1, width], y in [1, height].
    """
    def __init__(self, width, height, pedestrians_loc, targets_loc, obstacles_loc, points_loc, speed, max_timesteps, devour=False, dijkstra=False, verbose_visualization=False, visualization=True, end_on_reached_targets=False):
        self.field = Field(width, height)
        
        if pedestrians_loc is None:
            self.pedestrians=[]
        else:
            if len(speed) == 1:
                self.pedestrians = [Pedestrian(self.field.cells[x, y], speed[0], max_timesteps,i) for i, (x, y) in enumerate(pedestrians_loc)]
            else:
                self.pedestrians = [Pedestrian(self.field.cells[x, y], speed[i], max_timesteps, i) for i, (x, y) in enumerate(pedestrians_loc)]
        if targets_loc is None:
            self.targets=[]
        else:
            self.targets = [Target(self.field.cells[x, y]) for (x, y) in targets_loc]
        if obstacles_loc is None:
            self.obstacles=[]
        else:
            self.obstacles = [Obstacle(self.field.cells[x, y]) for (x, y) in obstacles_loc]
        if points_loc is None:
            self.points=[]
        else:
            self.points = [Point(self.field.cells[x, y]) for (x, y) in points_loc]
        
        self.speed = sum(speed)/len(speed)
        self.max_timesteps = max_timesteps
        self.devour = devour
        self.dijkstra = dijkstra
        self.target_cost_calculation = CostUpdate.dijkstra if dijkstra else CostUpdate.distance
        self.visualization=visualization
        if self.visualization is True:
            self.field_visual = visual.FieldVisual(width, height, verbose_visualization)
        self.elapsed_time= None
        self.sim_running=True
        self.end_on_reached_targets = end_on_reached_targets
        self.finishing_times = []
        
        # Varaibles used to measure the density of areas
        self.areas = []
        self.with_density = False
        self.start_time = 0
        self.passed_point = False

    """
    Initialize the costs of targets and obstacles since these values do not change within the course of a simulation.
    Calculate target-related costs according to selected target cost-calculation.
    """
    def init_costs(self):
        obstacle_cost = 1000000000

        for obstacle in self.obstacles:
            obstacle.cell.static_cost = obstacle_cost
        
        for p in self.points:
            p.cell.static_cost = obstacle_cost

        for target in self.targets:
            self.target_cost_calculation(target.cell, self.field)

    """
    Run the simulation.
    """
    def run(self):
        if self.visualization is True:
             while self.field_visual.is_running:
                self._update()
        else :
            print('simulation running with no visualization')
            while self.sim_running is True:
                self._update()

    """
    Update the simulation once.
    This method is to be executed per time frame only by the method run().
    """
    def _update(self):
        remove_pedestrians = []
        for p in self.pedestrians:
            # checks if pedestrian passed a measuring point
            if not self.passed_point and p.cell.loc[0] in [pt.cell.loc[0] for pt in self.points]:
                self.start_time = time.time()
                self.passed_point = True
            # checks if pedestrian is in an area
            for a in self.areas:
                a.is_inside(p)
            # loops pedestrians to the left if density is being calculated
            if self.with_density and p.cell.loc[0] in [t.cell.loc[0]-2 for t in self.targets]:
                self.pedestrians.append(Pedestrian(self.field.cells[1, p.cell.loc[1]], p.speed, p.steps_left, p.identity))
                remove_pedestrians.append(p)
                #p.cell.loc[0] = 1
            if not p.cell in [t.cell for t in self.targets]:
                optimal_neighbor = self.find_optimal_neighbor(p)
                p.move_in_time(optimal_neighbor)
                # devour pedestrians who have reached a target
                if p.cell in [t.cell for t in self.targets]:
                    finishing_time = time.time() - p.first_movement_timestamp
                    print("Elapsed time:", finishing_time,  "s for pedestrian #", p.identity)
                    self.finishing_times.append(finishing_time)
                    if self.devour:
                        remove_pedestrians.append(p)
        if len(remove_pedestrians) > 0:
            self.pedestrians = [p for p in self.pedestrians if p not in remove_pedestrians]
        
        if self.visualization is True:
            self.field_visual.draw_update(self.field, self.pedestrians, self.obstacles, self.targets, self.points)
        if len(remove_pedestrians) > len(self.pedestrians):
            self.sim_running = False

        if self.end_on_reached_targets and not self.pedestrians:
            self.field_visual.is_running = False
        
        # After x number of seconds have passed, it terminates the program. Depends on the density value.
        if ((time.time() - self.start_time) >= 20) and self.with_density and self.passed_point:
            self.sim_running = False
            if self.visualization:
                self.field_visual.is_running = False
            
    """
    Sets the measuring areas when calculating the density of the simulation.

    @param: areas: List with coordinates for the specific areas.
    """
    def set_areas(self, areas):
        self.areas = [Area(self.field.cells[area[0][0], area[0][1]], self.field.cells[area[1][0], area[1][1]]) for area in areas]
        self.with_density = True

    """
    Returns the coordinates (speed, density) from each measuring area.

    @return: List with coordinates for speed and density
    """
    def get_coordinates(self):
        coordinates = []
        for a in self.areas:
            coordinates.append(a.coordinates)
        return coordinates

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
            if cell.static_cost < 1000000000:
                distance = np.linalg.norm(cell.loc - target.loc)
                if distance > 0:
                    costs = np.array([cell.static_cost, distance])
                    cell.static_cost = np.min(costs[costs > 0])

    """
    Calculates the distance-cost according to the number of steps needed to reach the target.

    @param target: Cell to calculate cost from.
    @param field: Field to operate on.
    """
    def dijkstra(target: Cell, field):
        visited_cells = [target]
        border_cells = [(target, 1)]
        target.static_cost = 1
        while len(border_cells) > 0:
            new_border_cells = []
            new_border_cells_costs = []
            for cell, parent_cost in border_cells:
                for n in cell.get_avail_neighbors():
                    if not n in visited_cells and n.static_cost < 1000000000:
                        travel_cost = np.linalg.norm(n.loc - cell.loc)
                        costs = np.array([n.static_cost, parent_cost + travel_cost])

                        n.static_cost = np.min(costs[costs > 0]) # if a neighbor offers less cost, update cost to minimum
                        if n in new_border_cells:
                            new_border_cells_costs[new_border_cells.index(n)] = n.static_cost
                        else:
                            new_border_cells.append(n)
                            new_border_cells_costs.append(n.static_cost)
            visited_cells.extend(new_border_cells)
            border_cells = list(zip(new_border_cells, new_border_cells_costs))


