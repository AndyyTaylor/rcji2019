
import pygame
import numpy as np

from config import COLORS, ROBOT, ROBOT_WIDTH, ROBOT_SPEED, LIDAR_RENDER_LENGTH


class Robot:

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.vel = np.array([0, 0])

        self.lidar_heading = 0

    def update(self):
        self.x += self.vel[0] * ROBOT_SPEED
        self.y += self.vel[1] * ROBOT_SPEED

        self.lidar_heading += (2 * np.pi / 180) * 0.9

        if self.lidar_heading >= 2 * np.pi:
            self.lidar_heading -= 2 * np.pi

    def render(self, screen: pygame.Surface):
        pygame.draw.circle(screen, COLORS.BLUE, (int(self.x), int(self.y)), ROBOT.RADIUS)

        lidar_pos = self.get_lidar_pos()

        pygame.draw.line(screen, COLORS.GREEN, lidar_pos[0], lidar_pos[1])

    def get_lidar_pos(self, x=None, y=None):
        if x is None or y is None:
            x = self.x
            y = self.y

        lidar_x = int(np.cos(self.lidar_heading) * LIDAR_RENDER_LENGTH + x)
        lidar_y = int(np.sin(self.lidar_heading) * LIDAR_RENDER_LENGTH + y)

        return [(x, y), (lidar_x, lidar_y)]

    def set_x_vel(self, val):
        self.vel[0] = val

    def set_y_vel(self, val):
        self.vel[1] = val

    def get_x_vel(self):
        return self.vel[0]

    def get_y_vel(self):
        return self.vel[1]
