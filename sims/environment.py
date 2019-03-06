
import pygame
import math

from config import COLORS, WALL_WIDTH, LIDAR_PING_RADIUS


class Point:

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

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
        pygame.draw.line(screen, COLORS.BLACK, self.p1.get_pos(), self.p2.get_pos(), WALL_WIDTH)


class Environment:

    def __init__(self):
        self.walls = [
            Wall(Point(200, 200), Point(200, 600)),
            Wall(Point(200, 600), Point(600, 600)),
            Wall(Point(600, 600), Point(600, 200)),
            Wall(Point(600, 200), Point(200, 200)),
            Wall(Point(350, 200), Point(350, 250)),
            Wall(Point(450, 200), Point(450, 250)),
            Wall(Point(350, 600), Point(350, 550)),
            Wall(Point(450, 600), Point(450, 550))
        ]

        self.lidar_pings = []
        self.landmarks = []
    
    def update(self, robot):
        lidar_pos = robot.get_lidar_pos()

        pings = []
        for wall in self.walls:
            x, y = self.findIntersection(wall, lidar_pos, robot.lidar_heading)
            if x and y:
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

    def findIntersection(self, wall: Wall, lidar_pos, lidar_heading):
        x, y = self.findIntersectionWiki(*wall.p1.get_pos(), *wall.p2.get_pos(), *lidar_pos[0], *lidar_pos[1])

        if x > max(wall.p1.get_x(), wall.p2.get_x()) or x < min(wall.p1.get_x(), wall.p2.get_x()):
            x = False

        if y > max(wall.p1.get_y(), wall.p2.get_y()) or y < min(wall.p1.get_y(), wall.p2.get_y()):
            y = False

        intersect_heading = math.atan2(y - lidar_pos[0][1], x - lidar_pos[0][0])
        if (intersect_heading < 0):
            intersect_heading += math.pi * 2

        if abs(intersect_heading - lidar_heading) > 1:
            x, y = False, False

        return (x, y)
    
    def angleBetweenTwoPoints(self, point1, point2):
        return math.atan2(point2[1] - point1[1], point2[0] - point1[0])
    
    # pylint: disable=all
    def findIntersectionWiki(self, x1,y1,x2,y2,x3,y3,x4,y4):  # From wikipedia  
        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) or 0.0001 ) 
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) or 0.0001)
        return (px, py)
    # pylint: enable=all
