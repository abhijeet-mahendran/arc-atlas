import math
import pygame # type: ignore

from rover_api import rover # type: ignore

WIDTH = 1000
HEIGHT = 1000
PIXELS_PER_METER = 100

SIM_DT = 0.1 # simulation time step dt
V_R = 8.0 # right wheel linear velocity
V_L = 4.0 # left wheel linear velocity
L = 2.0 # rover track width

def world_to_screen(x, y):
    screen_x = WIDTH / 2 + x * PIXELS_PER_METER
    screen_y = HEIGHT / 2 - y * PIXELS_PER_METER

    return int(screen_x), int(screen_y)

def rotate_point(x, y, theta):
    rotated_x = x * math.cos(theta) - y * math.sin(theta)
    rotated_y = x * math.sin(theta) + y * math.cos(theta)

    return rotated_x, rotated_y

def rover_polygon(state):

    local_points = [
        (0.45, 0.0),
        (-0.30, 0.25),
        (-0.30, -0.25)
    ]

    screen_points = []

    for local_x, local_y in local_points:

        rotated_x, rotated_y = rotate_point(
            local_x,
            local_y,
            state.theta
        )

        world_x = state.x + rotated_x
        world_y = state.y + rotated_y

        screen_point = world_to_screen(
            world_x,
            world_y
        )

        screen_points.append(screen_point)

    return screen_points

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Arc Atlas")

clock = pygame.time.Clock()

rover.rover_init(L)

running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    rover.rover_step(V_R, V_L, SIM_DT)

    state = rover.rover_get_state()

    screen.fill((25, 25, 30))

    points = rover_polygon(state)

    pygame.draw.polygon(
        screen,
        (220, 220, 220),
        points
    )

    pygame.display.flip()

    clock.tick(60)

pygame.quit()