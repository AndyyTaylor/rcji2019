
import pygame
import numpy as np

from config import COLORS, ROBOT_WIDTH, ROBOT_SPEED


class Robot:

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.vel = np.array([0, 0])
    
    def update(self):
        self.x += self.vel[0] * ROBOT_SPEED
        self.y += self.vel[1] * ROBOT_SPEED

    def render(self, screen: pygame.Surface):
        pygame.draw.circle(screen, COLORS.BLUE, (self.x, self.y), ROBOT_WIDTH)
    
    def set_x_vel(self, val):
        self.vel[0] = val
    
    def set_y_vel(self, val):
        self.vel[1] = val
    
    def get_x_vel(self):
        return self.vel[0]
    
    def get_y_vel(self):
        return self.vel[1]
