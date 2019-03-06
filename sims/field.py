
import math
import pygame
import numpy as np

from config import COLORS, FIELD, LIDAR


class Wall:
    def __init__(self, x, y, x2, y2, default_width=FIELD.WALL_WIDTH, solid=True):
        self.x = x
        self.y = y
        self.x2 = x2
        self.y2 = y2
        self.solid = solid

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

    def get_points(self):
        return (self.x, self.y, self.x2, self.y2)


class Field:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.walls = []
        self.generate_walls()
        self.generate_goals()
        self.generate_field_lines()

        self.pings = []
        self.landmarks = []
        self.render_lines = []

    def update(self, robot):
        lidar_endpoints = robot.get_lidar_pos()
        lidar_heading = robot.lidar_heading

        closest_ping = None
        closest_dist = 9999
        for wall in self.walls:
            if not wall.solid:
                continue

            found, x, y = self.find_intersection(wall, lidar_endpoints, lidar_heading)

            if found:
                dist = self.distance(*lidar_endpoints[0], x, y)

                if dist < closest_dist:
                    closest_dist = dist
                    closest_ping = (x, y)

        if closest_ping is not None and closest_ping not in self.pings:
            self.pings.append(closest_ping)

        self.analyse_pings()

        while len(self.pings) > 20:
            self.pings.pop(0)

    def analyse_pings(self):
        if len(self.pings) < 4:
            return

        recent_pings = self.pings[-4:]

        angle1 = self.angle_between_points(*recent_pings[0], *recent_pings[1])
        angle2 = self.angle_between_points(*recent_pings[-2], *recent_pings[-1])

        diff = abs(angle1 - angle2)
        if diff > math.pi:
            diff -= math.pi

        max_allowed_dist = math.tan(math.radians(5)) * FIELD.WIDTH
        max_dist = 0
        for i in range(len(recent_pings) - 1):
            dist = self.distance(*recent_pings[i], *recent_pings[i + 1])
            if dist > max_dist:
                max_dist = dist

        if abs(diff - math.pi / 2) < 0.1 and max_dist <= max_allowed_dist:
            self.landmarks.append(self.midpoint(*recent_pings[1], *recent_pings[2]))
            self.render_lines.append(recent_pings)

    def angle_between_points(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def midpoint(self, x1, y1, x2, y2):
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def find_intersection(self, wall, endpoints, heading):
        x, y = self.point_intersects_line(*wall.get_points(), *endpoints[0], *endpoints[1])

        minx = min(wall.x, wall.x2)
        maxx = max(wall.x, wall.x2)
        miny = min(wall.y, wall.y2)
        maxy = max(wall.y, wall.y2)

        found = True
        if x < minx or x > maxx or y < miny or y > maxy:
            found = False
        else:
            intersect_heading = math.atan2(y - endpoints[0][1], x - endpoints[0][0])

            if intersect_heading < 0:
                intersect_heading += math.pi * 2

            if abs(intersect_heading - heading) > 1:
                found = False

        return (found, x, y)

    def render(self, screen):
        field_surf = pygame.Surface((FIELD.WIDTH + FIELD.WALL_WIDTH * 2, FIELD.HEIGHT + FIELD.WALL_WIDTH * 2))  # pylint: disable=too-many-function-args
        field_surf.fill(COLORS.WHITE)

        self.render_walls(field_surf)

        for ping in self.pings:
            pygame.draw.circle(field_surf, COLORS.BLUE, (int(ping[0]), int(ping[1])), LIDAR.PING_RADIUS)

        for landmark in self.landmarks:
            pygame.draw.circle(field_surf, COLORS.GREEN, (int(landmark[0]), int(landmark[1])), LIDAR.PING_RADIUS)

        # for line in self.render_lines:
        #     for i in range(len(line) - 1):
        #         pygame.draw.line(field_surf, COLORS.RED, (int(line[i][0]), int(line[i][1])), (int(line[i + 1][0]), int(line[i + 1][1])))

        screen.blit(field_surf, (self.x, self.y))

    def render_walls(self, surf):
        for wall in self.walls:
            wall.render(surf)

    def generate_walls(self):
        self.walls.append(Wall(0, 0, FIELD.WIDTH, 0))
        self.walls.append(Wall(0, 0, FIELD.WALL_WIDTH, FIELD.HEIGHT))
        self.walls.append(Wall(FIELD.WIDTH - FIELD.WALL_WIDTH, 0, FIELD.WIDTH - FIELD.WALL_WIDTH, FIELD.HEIGHT))
        self.walls.append(Wall(0, FIELD.HEIGHT - FIELD.WALL_WIDTH, FIELD.WIDTH, FIELD.HEIGHT - FIELD.WALL_WIDTH))

    def generate_goals(self):
        goal_x = int(FIELD.WIDTH / 2 - FIELD.GOAL_WIDTH / 2)

        self.walls.append(Wall(goal_x, FIELD.BOUNDARY_GAP - FIELD.GOAL_DEPTH, goal_x + FIELD.GOAL_WIDTH, FIELD.BOUNDARY_GAP - FIELD.GOAL_DEPTH, FIELD.GOAL_WALL_WIDTH))
        self.walls.append(Wall(goal_x, FIELD.BOUNDARY_GAP - FIELD.GOAL_DEPTH, goal_x, FIELD.BOUNDARY_GAP, FIELD.GOAL_WALL_WIDTH))
        self.walls.append(Wall(goal_x + FIELD.GOAL_WIDTH, FIELD.BOUNDARY_GAP - FIELD.GOAL_DEPTH, goal_x + FIELD.GOAL_WIDTH, FIELD.BOUNDARY_GAP, FIELD.GOAL_WALL_WIDTH))

        self.walls.append(Wall(goal_x, FIELD.HEIGHT - FIELD.BOUNDARY_GAP + FIELD.GOAL_DEPTH - FIELD.GOAL_WALL_WIDTH, goal_x + FIELD.GOAL_WIDTH, FIELD.HEIGHT - FIELD.BOUNDARY_GAP + FIELD.GOAL_DEPTH - FIELD.GOAL_WALL_WIDTH, FIELD.GOAL_WALL_WIDTH))
        self.walls.append(Wall(goal_x, FIELD.HEIGHT - FIELD.BOUNDARY_GAP, goal_x, FIELD.HEIGHT - FIELD.BOUNDARY_GAP + FIELD.GOAL_DEPTH, FIELD.GOAL_WALL_WIDTH))
        self.walls.append(Wall(goal_x + FIELD.GOAL_WIDTH, FIELD.HEIGHT - FIELD.BOUNDARY_GAP, goal_x + FIELD.GOAL_WIDTH, FIELD.HEIGHT - FIELD.BOUNDARY_GAP + FIELD.GOAL_DEPTH, FIELD.GOAL_WALL_WIDTH))
        # self.walls.append(Wall(goal_x + FIELD.GOAL_WIDTH, FIELD.BOUNDARY_GAP - FIELD.GOAL_DEPTH, goal_x + FIELD.GOAL_WIDTH, FIELD.BOUNDARY_GAP, FIELD.GOAL_WALL_WIDTH))

    def generate_field_lines(self):
        self.walls.append(Wall(FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.WIDTH - FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.LINE_WIDTH, False))
        self.walls.append(Wall(FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP, FIELD.LINE_WIDTH,  False))
        self.walls.append(Wall(FIELD.WIDTH - FIELD.BOUNDARY_GAP - FIELD.LINE_WIDTH, FIELD.BOUNDARY_GAP, FIELD.WIDTH - FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP, FIELD.LINE_WIDTH, False))
        self.walls.append(Wall(FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP - FIELD.LINE_WIDTH, FIELD.WIDTH - FIELD.BOUNDARY_GAP, FIELD.HEIGHT - FIELD.BOUNDARY_GAP, FIELD.LINE_WIDTH, False))

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def point_intersects_line(self, x1, y1, x2, y2, x3, y3, x4, y4):
        intersect_x = ( (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4) ) / ( (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) or 0.0001)
        intersect_y = ( (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4) ) / ( (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) or 0.0001)

        return (intersect_x, intersect_y)
