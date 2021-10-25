import numpy as np

'''
A cell represents a single spot for an object (pedestrian, obstacle, ...) to be in.
A cell has never-changing neighbors within a grid of cells.
'''
class Cell:
    
    '''
    x: x-coordinate of the cell within a grid of cells
    y: y-coordinate of the cell within a grid of cells
    is_border: the valid grid of cells is surrounded by a border. Cells denoted as is_border are not passable.
    '''
    def __init__(self, x:int, y:int, is_border=False):
        self.loc = np.array([x, y])
        self.is_border = is_border
        self.static_cost = 0 # gets calculated once for all targets and obstacles
        self.neighbors = None
        self.avail_neighbors = None

    '''
    Returns neighbors which are usable within the cell-grid. Border-cells are thereby excluded.
    '''
    def get_avail_neighbors(self):
        if self.avail_neighbors is None:
            self.avail_neighbors = [self] + self.neighbors
            self.avail_neighbors = [n for n in self.avail_neighbors if not n.is_border]
        return self.avail_neighbors

    '''
    Given the dimensions of the cell-grid, returns if a cell-index is in the border-region.
    '''
    def is_border_index(x, y, width, height):
        return (x in [0, width-1] or y in [0, height-1])

'''
A grid of inter-connected cells, surrounded by non-passable border-cells.
'''
class Field:

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.cells = np.array([Cell(x, y, Cell.is_border_index(x, y, width, height)) 
            for x in range(width+1) for y in range(height+1)]).reshape(self.width+1, self.height+1)

        for x in range(1, width-1):
            for y in range(1, height-1):
                self.cells[x, y].neighbors = [self.cells[x, y-1], self.cells[x+1, y-1], self.cells[x+1, y], self.cells[x+1, y+1], 
                self.cells[x, y+1], self.cells[x-1, y+1], self.cells[x-1, y], self.cells[x-1, y-1]]
