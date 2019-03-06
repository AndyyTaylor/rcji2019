
import pygame
import numpy as np

from config import COLORS, FIELD


class Wall:
    def __init__(self, x, y, x2, y2, solid=True):
        self.x = x
        self.y = y
        self.x2 = x2
        self.y2 = y2
        self.solid = solid

        if solid:
            default_width = FIELD.WALL_WIDTH
        else:
            default_width = FIELD.LINE_WIDTH

        self.w = abs(x - x2) or default_width
        self.h = abs(y - y2) or default_width

    def render(self, screen: pygame.Surface):
        if self.solid:
            color = COLORS.BLACK
        else:
            color = COLORS.GRAY

        pygame.draw.rect(screen, color, self.get_int_rect())

    def get_int_rect(self):
        return (int(self.x), int(self.y), int(self.w), int(self.h))


class Field:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.walls = []
        self.generate_walls()
        self.generate_field_lines()

    def update(self):
        pass

    def render(self, screen):
        field_surf = pygame.Surface((FIELD.WIDTH + FIELD.WALL_WIDTH * 2, FIELD.HEIGHT + FIELD.WALL_WIDTH * 2))  # pylint: disable=too-many-function-args
        field_surf.fill(COLORS.WHITE)

        self.render_walls(field_surf)

        screen.blit(field_surf, (self.x, self.y))

    def render_walls(self, surf):
        for wall in self.walls:
            wall.render(surf)

    def generate_walls(self):
        top_left = (0, 0)
        top_right = (FIELD.WIDTH, 0)
        bottom_left = (0, FIELD.HEIGHT)
        bottom_right = (FIELD.WIDTH, FIELD.HEIGHT)

        self.walls.append(Wall(0, 0, FIELD.WIDTH - FIELD.WALL_WIDTH, FIELD.WALL_WIDTH))
        self.walls.append(Wall(0, 0, FIELD.WALL_WIDTH, FIELD.HEIGHT))
        self.walls.append(Wall(FIELD.WIDTH - FIELD.WALL_WIDTH, 0, FIELD.WIDTH, FIELD.HEIGHT))
        self.walls.append(Wall(0, FIELD.HEIGHT - FIELD.WALL_WIDTH, FIELD.WIDTH, FIELD.HEIGHT))

    def generate_field_lines(self):
        top_left = (FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP)
        top_right = (FIELD.WIDTH - FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP)
        bottom_left = (FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP)
        bottom_right = (FIELD.WIDTH - FIELD.BOUNDARY_GAP,
                        FIELD.HEIGHT - FIELD.BOUNDARY_GAP)

        self.walls.append(Wall(FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.WIDTH - FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, False))
        self.walls.append(Wall(FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP), False))
        # self.walls.append(Wall(*top_left, *bottom_left, False))
        # self.walls.append(Wall(*top_right, *bottom_right, False))
        # self.walls.append(Wall(*bottom_left, *bottom_right, False))
