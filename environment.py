import numpy as np

"""
A cell represents a single spot for an object (pedestrian, obstacle, ...) to be in.
A cell has never-changing neighbors within a grid of cells.
"""
class Cell:
    
    """
    Initializes a cell at a certain location within a grid of given dimensions.

    @param x: x-coordinate of the cell within a grid of cells.
    @param y: y-coordinate of the cell within a grid of cells.
    @param field_width: x-dimension of the encapsulating grid of cells.
    @param field_height: y-dimension of the encapsulating grid of cells.
    @param is_border: the valid grid of cells is surrounded by a border. Cells denoted as is_border are not passable.
    """
    def __init__(self, x:int, y:int, field_width:int, field_height:int, is_border=False):
        self.loc = np.array([x, y])
        self.is_border = self.is_border_index(field_width, field_height)
        self.static_cost = 0 # gets calculated once for all targets and obstacles
        self.neighbors = None
        self.avail_neighbors = None

    """
    Returns neighbors which are usable within the cell-grid. Border-cells are thereby excluded.
    Neighbors have to be computed and set externally. (Depends on grid-usage and movement-possibilities).
    """
    def get_avail_neighbors(self):
        if self.avail_neighbors is None:
            self.avail_neighbors = [self] + self.neighbors
            self.avail_neighbors = [n for n in self.avail_neighbors if not n.is_border]
        return self.avail_neighbors

    """
    Determines if this cell is in the border-region.

    @param field_width: x-dimension of the encapsulating grid of cells.
    @param field_height: y-dimension of the encapsulating grid of cells.
    """
    def is_border_index(self, field_width, field_height):
        x, y = self.loc
        return (x in [0, field_width-1] or y in [0, field_height-1])

"""
A grid of inter-connected cells, surrounded by non-passable border-cells.
"""
class Field:

    """
    Initializes a grid of cells of given dimensions.

    @param width: x-dimension of the grid of cells.
    @param height: y-dimension of the grid of cells.
    """
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.init_cell_grid()

    """
    Populate cells and interconnect them to form a grid of cells.
    """
    def init_cell_grid(self):
        # construct grid of cells
        self.cells = np.array([Cell(x, y, self.width, self.height) 
            for x in range(self.width+1) for y in range(self.height+1)]).reshape(self.width+1, self.height+1)

        # inter-connect each cell with its neighbors
        for x in range(1, self.width-1):
            for y in range(1, self.height-1):
                self.cells[x, y].neighbors = [self.cells[x, y-1], self.cells[x+1, y-1], self.cells[x+1, y], self.cells[x+1, y+1], 
                self.cells[x, y+1], self.cells[x-1, y+1], self.cells[x-1, y], self.cells[x-1, y-1]]
