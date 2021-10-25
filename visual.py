import matplotlib.pyplot as plt
import numpy as np
import pygame 

'''
Visualization of the Field and its objects
'''
class FieldVisual:

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.box_size = 10

        pygame.init()
        self.screen = pygame.display.set_mode((self.width*self.box_size, self.height*self.box_size))
        self.screen.fill((0, 0, 0))
        self.draw_update()

    '''
    Draw the current state of the Field and its objects
    '''
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
        




