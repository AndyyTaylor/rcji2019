
import pygame

from config import COLORS, WALL_WIDTH


class Point:

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def get_pos(self):
        return (self.x, self.y)


class Wall:

    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2

    def render(self, screen: pygame.Surface):
        pygame.draw.line(screen, COLORS.BLACK, self.p1.get_pos(), self.p2.get_pos(), WALL_WIDTH)


class Environment:

    def __init__(self):
        self.walls = [
            Wall(Point(200, 200), Point(200, 600)),
            Wall(Point(200, 600), Point(600, 600)),
            Wall(Point(600, 600), Point(600, 200))
        ]

    def render(self, screen: pygame.Surface):
        for wall in self.walls:
            wall.render(screen)
