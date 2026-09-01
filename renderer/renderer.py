import math
from collections import deque

import pygame  # type: ignore

from rover_api import rover  # type: ignore


WIDTH = 1000
HEIGHT = 1000
PIXELS_PER_METER = 100

SIM_DT = 0.5
REFERENCE_SIM_DT = 0.2

V_R = 4.0
V_L = 2.0
L = 1.0
V_C = (V_R + V_L) / 2
OMEGA = (V_R - V_L) / L

TARGET_FPS = 60

# How often a historical rover pose is saved
SHADOW_INTERVAL = 0.5

# Prevent history from consuming unlimited memory
MAX_PATH_POINTS = 10000
MAX_SHADOWS = 200

CAMERA_FOLLOW_SPEED = 5.0


def world_to_screen(
    x: float,
    y: float,
    camera_x: float,
    camera_y: float,
) -> tuple[int, int]:
    screen_x = (
        WIDTH / 2
        + (x - camera_x) * PIXELS_PER_METER
    )

    screen_y = (
        HEIGHT / 2
        - (y - camera_y) * PIXELS_PER_METER
    )

    return int(screen_x), int(screen_y)

def rotate_point(
    x: float,
    y: float,
    theta: float,
) -> tuple[float, float]:
    rotated_x = (
        x * math.cos(theta)
        - y * math.sin(theta)
    )

    rotated_y = (
        x * math.sin(theta)
        + y * math.cos(theta)
    )

    return rotated_x, rotated_y

def exact_position(t, x0, y0, theta0, v_C, omega):
    x_exact = x0 + (v_C/omega)*(math.sin(theta0 + omega*t) - math.sin(theta0))
    y_exact = y0 - (v_C/omega)*(math.cos(theta0 + omega*t) - math.cos(theta0))
    return x_exact, y_exact

def position_error(x_exact, y_exact, x_state, y_state):
    x_error = x_state - x_exact
    y_error = y_state - y_exact

    pos_error = math.sqrt(
        x_error * x_error +
        y_error * y_error
    )

    return x_error, y_error, pos_error
    
def rover_polygon(
    x: float,
    y: float,
    theta: float,
    camera_x: float,
    camera_y: float,
) -> list[tuple[int, int]]:
    local_points = [
        (0.45, 0.0),     # front
        (-0.30, 0.25),   # rear-left
        (-0.30, -0.25),  # rear-right
    ]

    screen_points = []

    for local_x, local_y in local_points:
        rotated_x, rotated_y = rotate_point(
            local_x,
            local_y,
            theta,
        )

        world_x = x + rotated_x
        world_y = y + rotated_y

        screen_points.append(
            world_to_screen(
                world_x,
                world_y,
                camera_x,
                camera_y,
            )
        )

    return screen_points

def draw_path(
    screen: pygame.Surface,
    path_history: deque,
    camera_x: float,
    camera_y: float,
    color,
) -> None:
    if len(path_history) < 2:
        return

    screen_points = [
        world_to_screen(x, y, camera_x, camera_y)
        for x, y in path_history
    ]

    pygame.draw.lines(
        screen,
        color,
        False,
        screen_points,
        3,
    )


def draw_shadows(
    screen: pygame.Surface,
    shadow_history: deque,
    camera_x: float,
    camera_y: float,
) -> None:
    # Separate transparent drawing surface
    shadow_surface = pygame.Surface(
        (WIDTH, HEIGHT),
        pygame.SRCALPHA,
    )

    number_of_shadows = len(shadow_history)

    for index, (x, y, theta) in enumerate(shadow_history):
        # Older poses are dimmer; newer poses are brighter
        age_fraction = (
            (index + 1) / number_of_shadows
            if number_of_shadows > 0
            else 1.0
        )

        alpha = int(30 + 120 * age_fraction)

        points = rover_polygon(
            x,
            y,
            theta,
            camera_x,
            camera_y,
        )

        pygame.draw.polygon(
            shadow_surface,
            (150, 180, 220, alpha),
            points,
            width=2,
        )

    screen.blit(shadow_surface, (0, 0))


def draw_grid(
    screen: pygame.Surface,
    camera_x: float,
    camera_y: float,
) -> None:
    grid_spacing_pixels = PIXELS_PER_METER

    camera_screen_x = (
        WIDTH / 2
        - camera_x * PIXELS_PER_METER
    )

    camera_screen_y = (
        HEIGHT / 2
        + camera_y * PIXELS_PER_METER
    )

    first_vertical = int(
        camera_screen_x % grid_spacing_pixels
    )

    first_horizontal = int(
        camera_screen_y % grid_spacing_pixels
    )

    for screen_x in range(
        first_vertical,
        WIDTH,
        grid_spacing_pixels,
    ):
        pygame.draw.line(
            screen,
            (45, 45, 52),
            (screen_x, 0),
            (screen_x, HEIGHT),
            1,
        )

    for screen_y in range(
        first_horizontal,
        HEIGHT,
        grid_spacing_pixels,
    ):
        pygame.draw.line(
            screen,
            (45, 45, 52),
            (0, screen_y),
            (WIDTH, screen_y),
            1,
        )


def reset_simulation():
    rover.rover_init(L)

    initial_state = rover.rover_get_state()

    path_history = deque(
        [(initial_state.x, initial_state.y)],
        maxlen=MAX_PATH_POINTS,
    )

    reference_history = deque(
        [(initial_state.x, initial_state.y)],
        maxlen=MAX_PATH_POINTS,
    )
    shadow_history = deque(
        [
            (
                initial_state.x,
                initial_state.y,
                initial_state.theta,
            )
        ],
        maxlen=MAX_SHADOWS,
    )

    return initial_state, path_history, reference_history, shadow_history


pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Arc Atlas")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 28)

state, path_history, reference_history, shadow_history = reset_simulation()

camera_x = state.x
camera_y = state.y

camera_follow = True
display_mode = "path"

accumulator = 0.0
shadow_timer = 0.0
sim_time = 0.0

x_error = 0.0
y_error = 0.0
pos_error = 0.0
pos_error_max = 0.0

running = True

while running:
    frame_time = clock.tick(TARGET_FPS) / 1000.0

    # Prevent an enormous catch-up loop after pausing,
    # dragging the window, or hitting a breakpoint
    frame_time = min(frame_time, 0.25)

    accumulator += frame_time

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_TAB:
                if display_mode == "path":
                    display_mode = "shadows"
                else:
                    display_mode = "path"

            elif event.key == pygame.K_f:
                camera_follow = not camera_follow

            elif event.key == pygame.K_c:
                path_history.clear()
                shadow_history.clear()
                reference_history.clear()

            elif event.key == pygame.K_r:
                (
                    state,
                    path_history,
                    reference_history,
                    shadow_history,
                ) = reset_simulation()

                camera_x = state.x
                camera_y = state.y

                accumulator = 0.0
                shadow_timer = 0.0
                sim_time = 0.0

                x_error = 0.0
                y_error = 0.0
                pos_error = 0.0
                pos_error_max = 0.0

    while accumulator >= SIM_DT:
        # C performs every physical calculation
        rover.rover_step(V_R, V_L, SIM_DT)
        sim_time += SIM_DT

        state = rover.rover_get_state()

        # Record the C-calculated centre position
        path_history.append(
            (state.x, state.y)
        )

        exact_x, exact_y = exact_position(
            sim_time,
            0.0,
            0.0,
            0.0,
            V_C,
            OMEGA
        )

        reference_history.append(
            (exact_x, exact_y)
        )

        x_error, y_error, pos_error = position_error(
            exact_x,
            exact_y,
            state.x,
            state.y
        )

        pos_error_max = max(pos_error, pos_error_max)

        # Save rover poses less frequently than path points
        shadow_timer += SIM_DT

        if shadow_timer >= SHADOW_INTERVAL:
            shadow_history.append(
                (
                    state.x,
                    state.y,
                    state.theta,
                )
            )

            shadow_timer -= SHADOW_INTERVAL

        accumulator -= SIM_DT

    # Smooth visual camera following
    # This changes only the viewpoint, never the rover
    if camera_follow:
        follow_amount = min(
            1.0,
            CAMERA_FOLLOW_SPEED * frame_time,
        )

        camera_x += (
            state.x - camera_x
        ) * follow_amount

        camera_y += (
            state.y - camera_y
        ) * follow_amount

    screen.fill((25, 25, 30))

    draw_grid(screen, camera_x, camera_y)

    if display_mode == "path":
        draw_path(
            screen,
            path_history,
            camera_x,
            camera_y,
            (90, 170, 255),
        )

        draw_path(
            screen,
            reference_history,
            camera_x,
            camera_y,
            (255, 150, 100),
        )
    else:
        draw_shadows(
            screen,
            shadow_history,
            camera_x,
            camera_y,
        )

    current_rover_points = rover_polygon(
        state.x,
        state.y,
        state.theta,
        camera_x,
        camera_y,
    )

    pygame.draw.polygon(
        screen,
        (230, 230, 235),
        current_rover_points,
    )

    technical_lines = [
        "SIMULATION",
        f"Mode: {display_mode}",
        f"Time: {sim_time:.2f} s",
        f"dt: {SIM_DT:.3f} s",
        "",
        "STATE",
        f"x: {state.x:.3f} m",
        f"y: {state.y:.3f} m",
        f"theta: {state.theta:.3f} rad",
        "",
        "ERROR",
        f"X error: {x_error:+.3e} m",
        f"Y error: {y_error:+.3e} m",
        f"Position error: {pos_error:.3e} m",
        f"Maximum Position error: {pos_error_max:.3e} m",
    ]

    control_lines = [
        "CONTROLS",
        "TAB  Path / Shadows",
        f"F    Camera follow ({camera_follow})",
        "C    Clear history",
        "R    Reset simulation",
    ]

    for index, line in enumerate(technical_lines):
        text_surface = font.render(
            line,
            True,
            (225, 225, 230),
        )

        screen.blit(
            text_surface,
            (20, 20 + index * 27),
        )

    for index, line in enumerate(control_lines):
        text_surface = font.render(
            line,
            True,
            (225, 225, 230),
        )

        screen.blit(
            text_surface,
            (WIDTH - 260, 20 + index * 27),
        )

    pygame.display.flip()

pygame.quit()