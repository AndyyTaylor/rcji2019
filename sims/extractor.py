
import math
import numpy as np

from config import FIELD


class Extractor:
    def __init__(self):
        self.points = []
        self.landmarks = []

        self.line_p1 = None
        self.line_p2 = None
        self.line_p3 = None

        self.render_lines = []

    def add_point(self, point):
        if point not in self.points:
            self.points.append(point)

        if len(self.points) > 4:
            self.points.pop(0)

    def extract_landmarks(self):
        self.extract_corners()
        self.extract_lines()

    def extract_lines(self):
        if len(self.points) < 3:
            return

        if self.line_p1 is None or self.line_p2 is None:
            self.line_p1 = self.points[0]
            self.line_p2 = self.points[1]
            self.line_p3 = self.points[2]

        angle1 = self.angle_between_points(*self.line_p1, *self.line_p2)
        angle2 = self.angle_between_points(*self.line_p1, *self.points[-1])

        min_dist = FIELD.WIDTH

        if abs(angle1 - angle2) > 0.01 and abs(angle1 - angle1) < math.radians(15):
            if self.distance(*self.line_p1, *self.line_p3) > min_dist:
                self.render_lines.append([self.line_p1, self.line_p2, self.line_p3])
                self.landmarks.append(self.midpoint(*self.line_p1, *self.line_p3))
            self.line_p1 = None
            self.line_p2 = None
            self.line_p3 = None

        self.line_p3 = self.points[-1]

    def extract_corners(self):
        if len(self.points) < 4:
            return

        angle1 = self.angle_between_points(*self.points[0], *self.points[1])
        angle2 = self.angle_between_points(*self.points[-2], *self.points[-1])

        diff = abs(angle1 - angle2)
        if diff > math.pi:
            diff -= math.pi

        max_allowed_dist = math.tan(math.radians(6)) * FIELD.WIDTH
        max_dist = 0
        for i in range(len(self.points) - 1):
            dist = self.distance(*self.points[i], *self.points[i + 1])
            if dist > max_dist:
                max_dist = dist

        if abs(diff - math.pi / 2) < 0.1 and max_dist <= max_allowed_dist:
            self.render_lines.append(self.points.copy());
            self.landmarks.append(self.midpoint(*self.points[1], *self.points[2]))

    def angle_between_points(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def midpoint(self, x1, y1, x2, y2):
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
