
import pygame
import math
import numpy as np

from config import SCREEN_WIDTH, SCREEN_HEIGHT, COLORS, FIELD, WALL_WIDTH, LIDAR_PING_RADIUS, FIELD_MARGIN


class Point:

    def __init__(self, *args):
        if len(args) == 1:
            self.x = args[0][0]
            self.y = args[0][1]
        else:
            self.x = args[0]
            self.y = args[1]

    def get_pos(self):
        return (self.x, self.y)

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y


class Wall:

    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2

    def render(self, screen: pygame.Surface):
        pygame.draw.rect(screen, COLORS.BLACK, (*self.p1.get_pos(), *np.add(np.subtract(self.p2.get_pos(), self.p1.get_pos()), (10, 10))))
        # pygame.draw.line(screen, COLORS.BLACK, self.p1.get_pos(), self.p2.get_pos(), WALL_WIDTH)


class Environment:

    def __init__(self):
        self.create_walls()

        self.lidar_pings = []
        self.landmarks = []

    def update(self, robot):
        lidar_pos = robot.get_lidar_pos()

        pings = []
        for wall in self.walls:
            x, y = self.findIntersection(wall, lidar_pos, robot.lidar_heading)
            if x != -1 and y != -1:
                pings.append((x, y))

        closest = 99999
        best_pos = None
        for ping in pings:
            dist = math.sqrt((ping[0] - lidar_pos[0][0]) ** 2 + (ping[1] - lidar_pos[0][1]) ** 2)
            if dist < closest:
                closest = dist
                best_pos = ping

        if best_pos is not None:
            self.lidar_pings.append(best_pos)

        if len(self.lidar_pings) >= 5:
            analysis_pings = self.lidar_pings[-5:]

            a1 = self.angleBetweenTwoPoints(analysis_pings[0], analysis_pings[1])
            a2 = self.angleBetweenTwoPoints(analysis_pings[3], analysis_pings[4])

            diff = abs(a1 - a2)
            if diff > math.pi:
                diff -= math.pi

            max_dist = 0
            for i in range(len(analysis_pings) - 1):
                dist = math.sqrt((analysis_pings[i][0] - analysis_pings[i + 1][0]) ** 2 + (analysis_pings[i][1] - analysis_pings[i + 1][1]) ** 2)
                if dist > max_dist:
                    max_dist = dist

            if abs(diff - math.pi / 2) < 0.1 and len(set(analysis_pings)) == len(analysis_pings) and max_dist < 50:
                self.landmarks.append(analysis_pings)


    def render(self, screen: pygame.Surface):
        self.render_field(screen)

        for wall in self.walls:
            wall.render(screen)

        for x, y in self.lidar_pings:
            pygame.draw.circle(screen, COLORS.BLUE, (int(x), int(y)), LIDAR_PING_RADIUS)

        for landmark in self.landmarks:
            for i in range(len(landmark) - 1):
                pygame.draw.line(screen, COLORS.RED, landmark[i], landmark[i + 1], 3)

            pygame.draw.circle(screen, COLORS.GREEN, (int(landmark[2][0]), int(landmark[2][1])), 5)

        # while len(self.landmarks) > 10:
        #     self.landmarks.pop(0)

        while len(self.lidar_pings) > 10:
            self.lidar_pings.pop(0)

    def render_field(self, screen):
        x_margin = (SCREEN_WIDTH - FIELD.WIDTH) // 2 + FIELD_MARGIN
        y_margin = (SCREEN_HEIGHT - FIELD.HEIGHT) // 2 + FIELD_MARGIN

        field_surf = pygame.Surface((FIELD.WIDTH + FIELD.WALL_WIDTH * 2 + FIELD_MARGIN * 2,
                                     FIELD.HEIGHT + FIELD.WALL_WIDTH * 2 + FIELD_MARGIN * 2))
        field_surf.fill(COLORS.WHITE)
        self.render_walls(field_surf)
        screen.blit(field_surf, (x_margin, y_margin))

    def render_walls(self, screen):
        for wall in self.walls:
            wall.render(screen)

    def create_walls(self):
        self.walls = []

        top_left = (0, 0)
        top_right = (FIELD.WIDTH, 0)
        bottom_left = (0, FIELD.HEIGHT + FIELD.WALL_WIDTH)
        bottom_right = (FIELD.WIDTH + FIELD.WALL_WIDTH, FIELD.HEIGHT + FIELD.WALL_WIDTH)

        self.walls.append(Wall(Point(top_left), Point(top_right)))
        self.walls.append(Wall(Point(top_left), Point(bottom_left)))
        self.walls.append(Wall(Point(top_right), Point(bottom_right)))
        self.walls.append(Wall(Point(bottom_left), Point(bottom_right)))

    def findIntersection(self, wall: Wall, lidar_pos, lidar_heading):
        x, y = self.findIntersectionWiki(*wall.p1.get_pos(), *wall.p2.get_pos(), *lidar_pos[0], *lidar_pos[1])

        if x > max(wall.p1.get_x(), wall.p2.get_x()) or x < min(wall.p1.get_x(), wall.p2.get_x()):
            x = -1

        if y > max(wall.p1.get_y(), wall.p2.get_y()) or y < min(wall.p1.get_y(), wall.p2.get_y()):
            y = -1

        intersect_heading = math.atan2(y - lidar_pos[0][1], x - lidar_pos[0][0])
        if (intersect_heading < 0):
            intersect_heading += math.pi * 2

        if abs(intersect_heading - lidar_heading) > 1:
            x, y = -1, -1

        return (x, y)

    def angleBetweenTwoPoints(self, point1, point2):
        return math.atan2(point2[1] - point1[1], point2[0] - point1[0])

    def findIntersectionWiki(self, x1,y1,x2,y2,x3,y3,x4,y4):  # From wikipedia
        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) or 0.0001 )
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) or 0.0001)
        return (px, py)
