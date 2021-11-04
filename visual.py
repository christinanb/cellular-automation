import matplotlib.pyplot as plt
import numpy as np
import pygame 
import sys

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
    def __init__(self, width, height, visualize_cost=False, box_size=10):
        self.width = width
        self.height = height
        self.visualize_cost = visualize_cost
        self.box_size = box_size
        self.is_running = True

        pygame.init()
        self.screen = pygame.display.set_mode((width*box_size, height*box_size))
        self.screen.fill((0, 0, 0))

    """
    Draw the current state of the Field and its objects.

    @param pedestrians: List of pedestrians on the Field which to visualize.
    @param obstacles: List of obstacles on the Field which to visualize.
    @param targets: List of targets on the Field which to visualize.
    @param visualize_cost: If field costs should be visualized as well.
    """
    def draw_update(self, field, pedestrians=[], obstacles=[], targets=[], points=[]):
        self.screen.fill((0, 0, 0))

        if self.visualize_cost:
            for c in field.cells.flatten():
                neighbor_x = c.loc[0]*self.box_size
                neighbor_y = c.loc[1]*self.box_size
                rect = pygame.Rect(neighbor_x, neighbor_y, self.box_size, self.box_size)
                pygame.draw.rect(self.screen, (0, min([int(c.static_cost*10), 255]), 200), rect, 0)

        for p in pedestrians:
            x = p.cell.loc[0]*self.box_size
            y = p.cell.loc[1]*self.box_size
            if self.visualize_cost:
                for n in p.cell.get_avail_neighbors():
                    neighbor_x = p.cell.loc[0]*self.box_size
                    neighbor_y = p.cell.loc[1]*self.box_size
                    rect = pygame.Rect(neighbor_x, neighbor_y, self.box_size, self.box_size)
                    pygame.draw.rect(self.screen, (0, min([int(c.static_cost*10), 255]), 200), rect, 0)
            rect = pygame.Rect(x, y, self.box_size, self.box_size)
            pygame.draw.rect(self.screen, (200, 0, 200), rect, 0)

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
        
        for p in points:
            x = p.cell.loc[0]*self.box_size
            y = p.cell.loc[1]*self.box_size
            rect = pygame.Rect(x, y, self.box_size, self.box_size)
            pygame.draw.rect(self.screen, (0, 51, 51), rect, 0)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.is_running = False
                break
        try:
            pygame.display.update()
        except:
            pygame.quit()
            self.is_running = False

