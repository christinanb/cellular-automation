import matplotlib.pyplot as plt
import numpy as np
import pygame 

"""
Visualization of the Field and its objects.
"""
class FieldVisual:

    """
    Visualization of field of given dimensions.

    @param width: Amount of cells in horizontal direction.
    @param height: Amount of cells in vertical direction:
    @param box_size: Visualization-size per cell.
    """
    def __init__(self, width, height, box_size=10):
        self.width = width
        self.height = height
        self.box_size = box_size

        pygame.init()
        self.screen = pygame.display.set_mode((width*box_size, height*box_size))
        self.screen.fill((0, 0, 0))
        self.draw_update()

    """
    Draw the current state of the Field and its objects.

    @param pedestrians: List of pedestrians on the Field which to visualize.
    @param obstacles: List of obstacles on the Field which to visualize.
    @param targets: List of targets on the Field which to visualize.
    """
    def draw_update(self, pedestrians=[], obstacles=[], targets=[]):
        self.screen.fill((0, 0, 0))
        for p in pedestrians:
            x = p.cell.loc[0]*self.box_size
            y = p.cell.loc[1]*self.box_size
            rect = pygame.Rect(x, y, self.box_size, self.box_size)
            pygame.draw.rect(self.screen, (0, 200, 200), rect, 0)

        for o in obstacles:
            x = o.cell.loc[0]*self.box_size
            y = o.cell.loc[1]*self.box_size
            rect = pygame.Rect(x, y, self.box_size, self.box_size)
            pygame.draw.rect(self.screen, (150, 150, 150), rect, 0)

        for t in targets:
            x = t.cell.loc[0]*self.box_size
            y = t.cell.loc[1]*self.box_size
            rect = pygame.Rect(x, y, self.box_size, self.box_size)
            pygame.draw.rect(self.screen, (200, 0, 0), rect, 1)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        pygame.display.update()
    
    """
    TODO: Delete this!
    Only for debugging!

    Visualizes the static cost of the field.
    """
    def visDebug(self, field):
            self.screen.fill((0, 0, 0))
            for c in field.cells.flatten():
                x = c.loc[0]*self.box_size
                y = c.loc[1]*self.box_size
                rect = pygame.Rect(x, y, self.box_size, self.box_size)
                pygame.draw.rect(self.screen, (0, int(c.static_cost*10), 200), rect, 0)
            pygame.display.update()



