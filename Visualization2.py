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

robot = Robot2(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 50, 12, 200)

walls = [
    Wall(100, 100, 600, 20),
    Wall(100, 120, 20, 360),
    Wall(100, 460, 620, 20),
    Wall(700, 120, 20, 360),
    Wall(200, 120, 20, 260),
    Wall(300, 200, 20, 200),
    Wall(400, 120, 20, 200),
    Wall(500, 200, 20, 200),
    Wall(600, 120, 20, 260),
    Wall(300, 380, 400, 20)
]

SPEED = delta_ms


def detect_collision(robot, walls, return_distance=False):
    robot_radius = robot.get_l() / 2
    robot_center = np.array(robot.get_position())
    for wall in walls:
        closest_x = max(wall.rect.left, min(robot_center[0], wall.rect.right))
        closest_y = max(wall.rect.top, min(robot_center[1], wall.rect.bottom))
        distance = np.sqrt((closest_x - robot_center[0]) ** 2 + (closest_y - robot_center[1]) ** 2)

        if distance <= robot_radius:
            if return_distance:
                return True, wall, robot_center - np.array([closest_x, closest_y])
            return True, wall
    return (False, None) + ((None,) if return_distance else ())


def get_wall_vectors(wall):
    """ Returns the unit normal and tangent vectors to the wall, adjusted for Pygame's coordinate system. """
    if wall.rect.width > wall.rect.height:  # Horizontal wall
        normal = np.array([0, 1]) if robot.get_position()[1] > wall.rect.centery else np.array([0, -1])
        tangent = np.array([1, 0])
    else:  # Vertical wall
        normal = np.array([1, 0]) if robot.get_position()[0] < wall.rect.centerx else np.array([-1, 0])
        tangent = np.array([0, 1])
    return normal, tangent



def decompose_velocity(velocity, normal, tangent):
    """ Decomposes the velocity into components parallel and perpendicular to the wall. """
    V = np.array(velocity)
    V_parallel = np.dot(V, tangent) * tangent
    V_perpendicular = np.dot(V, normal) * normal
    return V_parallel, V_perpendicular


def handle_collision(robot, wall):
    """Adjusts robot's velocity upon collision to stop movement into the wall."""
    normal, tangent = get_wall_vectors(wall)
    vx, vy = robot.get_velocity_components()  # Ensure this method returns the current velocity components
    V_parallel, V_perpendicular = decompose_velocity((vx, vy), normal, tangent)

    # Set the perpendicular velocity to zero
    new_velocity = V_parallel  # Only the component parallel to the wall is retained
    robot.set_velocity(new_velocity)


def reposition_robot(robot, wall, normal):
    """Repositions robot to prevent it from intersecting the wall, considering Pygame's inverted y-axis."""
    robot_radius = robot.get_l() / 2
    robot_center = np.array(robot.get_position())

    if normal[0] != 0:  # Vertical wall
        new_x = wall.rect.left - robot_radius if normal[0] > 0 else wall.rect.right + robot_radius
        robot.set_position(new_x, robot_center[1])

    if normal[1] != 0:  # Horizontal wall
        new_y = wall.rect.top - robot_radius if normal[1] < 0 else wall.rect.bottom + robot_radius
        robot.set_position(robot_center[0], new_y)


running = True
while running:

    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Check for collision
    collision, wall = detect_collision(robot, walls)
    if collision:
        handle_collision(robot, wall)
        normal, _ = get_wall_vectors(wall)
        reposition_robot(robot, wall, normal)

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

    # Display wheels
    RADIUS = robot.get_l() / 2
    angle = robot.get_orientation() + 2 * math.pi / 4
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, BLACK, (xx, yy), robot.get_l()/10)
    angle = angle + math.pi
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, BLACK, (xx, yy), robot.get_l()/10)

    # Display Robot
    pygame.draw.circle(screen, RED, (int(robot.get_position()[0]), int(robot.get_position()[1])), robot.get_l() / 2)

    # Display Walls
    for wall in walls:
        wall.draw(screen)

    # Display Orientation
    end_x = robot.get_position()[0] + robot.get_l() / 2 * math.cos(robot.get_orientation())
    end_y = robot.get_position()[1] - robot.get_l() / 2 * math.sin(robot.get_orientation())
    pygame.draw.line(screen, BLACK, (robot.get_position()[0], robot.get_position()[1]), (end_x, end_y), 5)

    # Display ICC
    if robot.get_velocity()[0] != robot.get_velocity()[1]:
        pygame.draw.circle(screen, BLUE, (int(robot.get_icc()[0]), int(robot.get_icc()[1])), 5)

    for i in range(robot.get_num_sensors()):
        s = robot.get_sensors()[i]
        sensor_angle = s.get_angle()

        # Display sensor lines
        if s.get_dist() < s.get_radius():
            pygame.draw.line(screen, GREEN, robot.get_position(), s.get_point_closest_wall())
        else:
            pygame.draw.line(screen, GREEN, robot.get_position(), robot.get_sensors()[i].get_position())

        # Display distance label of each sensor
        dist_text = font.render(f"{int(robot.get_sensors()[i].get_dist())}", True, BLACK)
        text_x = robot.get_sensors()[i].get_position()[0] + 30 * math.cos(sensor_angle)
        text_y = robot.get_sensors()[i].get_position()[1] - 30 * math.sin(sensor_angle)
        screen.blit(dist_text, (text_x, text_y))

    # Display Wheels speed
    angle = robot.get_orientation() + 2 * math.pi / 4
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle) / 2)
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle) / 2)
    speedLeft_text = font.render(f"{int(robot.get_velocity()[0])}", True, BLACK)
    screen.blit(speedLeft_text, (xx, yy))
    angle = angle + math.pi
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle) / 2)
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle) / 2)
    speedRight_text = font.render(f"{int(robot.get_velocity()[1])}", True, BLACK)
    screen.blit(speedRight_text, (xx, yy))

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()
