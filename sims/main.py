
import pygame

from robot import Robot
from environment import Environment
from config import SCREEN_WIDTH, SCREEN_HEIGHT, COLORS

# To Be Done
# [ ] FastSlam2.0
# [ ] ICP
# [ ] Graph Based SLAM
# [ ] Circle Fitting for Robot Detection

# pylint: disable=invalid-name

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

environment = Environment()
robot = Robot(480, 480)

running = True

# pylint: enable=invalid-name

while running:
    screen.fill(COLORS.WHITE)

    robot.update()

    environment.render(screen)
    robot.render(screen)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                running = False
            elif event.key == pygame.K_RIGHT:
                robot.set_x_vel(1)
            elif event.key == pygame.K_LEFT:
                robot.set_x_vel(-1)
            elif event.key == pygame.K_UP:
                robot.set_y_vel(-1)
            elif event.key == pygame.K_DOWN:
                robot.set_y_vel(1)
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_RIGHT:
                robot.set_x_vel(min(robot.get_x_vel(), 0))
            elif event.key == pygame.K_LEFT:
                robot.set_x_vel(max(robot.get_x_vel(), 0))
            elif event.key == pygame.K_UP:
                robot.set_y_vel(max(robot.get_y_vel(), 0))
            elif event.key == pygame.K_DOWN:
                robot.set_y_vel(min(robot.get_y_vel(), 0))

    pygame.display.update()
