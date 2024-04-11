import numpy as np
import pygame
import math
from Robot import Robot
from Robot2 import Robot2
from Wall import Wall

# Initialize Pygame
pygame.init()

# Set up the display
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Simulation")

# Setup font for distance labels
pygame.font.init()  # Initialize font module
font = pygame.font.SysFont(None, 18)  # Create a font object from the system fonts

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Clock to control the frame rate
clock = pygame.time.Clock()

# Limit frames per second
delta_ms = clock.tick(60)  # Get the number of milliseconds since last frame
delta_t = delta_ms / 1000.0 * 5  # Convert milliseconds to seconds

robot = Robot2(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 100, 12, 200)
walls = [
    Wall(100, 100, 200, 10),  # Horizontal wall
    Wall(300, 200, 10, 150)  # Vertical wall
]

SPEED = delta_ms

import pygame
import numpy as np

def detect_collision(robot, walls, return_distance=False):
    robot_radius = robot.get_l() / 2
    robot_center = np.array(robot.get_position())
    for wall in walls:
        closest_x = max(wall.rect.left, min(robot_center[0], wall.rect.right))
        closest_y = max(wall.rect.top, min(robot_center[1], wall.rect.bottom))
        distance = np.sqrt((closest_x - robot_center[0])**2 + (closest_y - robot_center[1])**2)

        if distance <= robot_radius:
            if return_distance:
                return True, wall, robot_center - np.array([closest_x, closest_y])
            return True, wall
    return (False, None) + ((None,) if return_distance else ())


def backtrack_to_collision(robot, wall, delta_t, initial_velocity):
    collision_time = delta_t
    time_step = delta_t / 10  # Start with 10% of the delta time

    while collision_time > 0:
        # Move the robot back slightly in time
        new_x = robot._x - initial_velocity[0] * time_step
        new_y = robot._y - initial_velocity[1] * time_step
        robot.set_position(new_x, new_y)

        # Check if it's still colliding
        if not detect_collision(robot, [wall])[0]:
            # If not colliding, step forward slightly to find approximate collision point
            new_x = robot._x + initial_velocity[0] * time_step
            new_y = robot._y + initial_velocity[1] * time_step
            robot.set_position(new_x, new_y)
            collision_time -= time_step
            break

        collision_time -= time_step

    return collision_time


def get_velocity_components(robot, delta_t):
    # Calculate effective velocity based on wheel speeds and orientation
    velocity = (robot._vr + robot._vl) / 2
    vx = velocity * np.cos(robot._theta)
    vy = velocity * np.sin(robot._theta)
    return vx * delta_t, vy * delta_t  # Return the displacement components

def get_wall_normal(robot, wall):
    robot_center = np.array(robot.get_position())
    if wall.rect.width > wall.rect.height:  # Horizontal wall
        # Normal points up if robot is below the wall (y decreases as you go up)
        return np.array([0, -1]) if robot_center[1] > wall.rect.centery else np.array([0, 1])
    else:  # Vertical wall
        # Normal points left if robot is to the right of the wall
        return np.array([-1, 0]) if robot_center[0] > wall.rect.centerx else np.array([1, 0])


def handle_collision(robot, wall):
    normal = get_wall_normal(robot, wall)
    robot_radius = robot.get_l() / 2

    if normal[1] == 0:  # Collision with vertical wall
        offset_x = (wall.rect.left - robot_radius if normal[0] > 0 else wall.rect.right + robot_radius)
        robot.set_position(offset_x, robot.get_position()[1])
    else:  # Collision with horizontal wall
        offset_y = (wall.rect.top - robot_radius if normal[1] > 0 else wall.rect.bottom + robot_radius)
        robot.set_position(robot.get_position()[0], offset_y)

    # Reset velocities to zero to stop further movement into the wall
    robot._vl = 0
    robot._vr = 0



running = True
while running:

    collision, wall = detect_collision(robot, walls)
    if collision:
        handle_collision(robot, wall)

    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    key = pygame.key.get_pressed()

    # Handle combined key presses first
    if key[pygame.K_a] and key[pygame.K_z]:
        robot.set_velocity((-SPEED, -SPEED))
    elif key[pygame.K_LEFT] and key[pygame.K_RIGHT]:
        robot.set_velocity((SPEED, SPEED))
    else:
        # Handle individual key presses
        left_speed = -SPEED if key[pygame.K_a] else SPEED if key[pygame.K_LEFT] else 0
        right_speed = -SPEED if key[pygame.K_z] else SPEED if key[pygame.K_RIGHT] else 0

        # Apply new wheel speeds if any key is pressed, otherwise stop
        if key[pygame.K_LEFT] or key[pygame.K_RIGHT] or key[pygame.K_a] or key[pygame.K_z]:
            robot.set_velocity((left_speed, right_speed))
        else:
            robot.set_velocity((0, 0))

    # Update the robot's position based on current wheel speeds
    robot.update(delta_t)
    robot.detect_walls(walls)

    # Clear the screen
    screen.fill(WHITE)

    RADIUS = robot.get_l() / 2
    # draw wheels
    angle = robot.get_orientation() + 2 * math.pi / 4
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, BLACK, (xx, yy), 10)
    angle = angle + math.pi
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, BLACK, (xx, yy), 10)

    # Draw the robot as a circle
    pygame.draw.circle(screen, RED, (int(robot.get_position()[0]), int(robot.get_position()[1])), robot.get_l() / 2)

    for wall in walls:
        wall.draw(screen)  # Draw each wall

    # Draw the robot's orientation
    end_x = robot.get_position()[0] + robot.get_l() / 2 * math.cos(robot.get_orientation())
    end_y = robot.get_position()[1] - robot.get_l() / 2 * math.sin(robot.get_orientation())
    pygame.draw.line(screen, BLACK, (robot.get_position()[0], robot.get_position()[1]), (end_x, end_y), 5)

    # draw ICC
    if robot.get_velocity()[0] != robot.get_velocity()[1]:
        pygame.draw.circle(screen, BLUE, (int(robot.get_icc()[0]), int(robot.get_icc()[1])), 5)

    for i in range(robot.get_num_sensors()):
        s = robot.get_sensors()[i]
        sensor_angle = robot.get_orientation() + i * 2 * math.pi / len(robot.get_sensors())
        xx = s.get_position()[0] + s.get_radius() * math.cos(sensor_angle)
        yy = s.get_position()[1] - s.get_radius() * math.sin(sensor_angle)

        #pygame.draw.line(screen, GREEN, robot.get_position(), robot.get_sensors()[i].get_position())
        pygame.draw.line(screen, GREEN, robot.get_position(), (xx,yy))

        dist_text = font.render(f"{robot.get_sensors()[i].get_dist()}", True, BLACK)
        text_x = robot.get_sensors()[i].get_position()[0] + 30 * math.cos(sensor_angle)
        text_y = robot.get_sensors()[i].get_position()[1] - 30 * math.sin(sensor_angle)
        screen.blit(dist_text, (text_x, text_y))

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()

