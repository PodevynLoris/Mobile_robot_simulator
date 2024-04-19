import numpy as np
import pygame
import math
from Robot2 import Robot2
from Wall import Wall

# Initialize Pygame
pygame.init()

# Set up the display
SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 800
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
PURPLE = (201, 52, 235)
GREY = (113, 108, 115)
DARK_YELLOW = (112, 82, 0)
WHITE = (255, 255, 255)

# Clock to control the frame rate
clock = pygame.time.Clock()

ROBOT_SPEED_PER_SECOND = 10

# Limit frames per second
delta_ms = clock.tick(60)  # Caps the frame rate at 60 FPS and measures frame duration
delta_t = delta_ms / 1000.0  # Converts milliseconds to seconds
SPEED = delta_ms

# Creation Robot2 object
robot = Robot2(SCREEN_WIDTH // 2, (SCREEN_HEIGHT // 2) - 40, 0, 60, 12, 200)

walls = [
    Wall(0, 0, 1200, 2),  # Top boundary
    Wall(0, 580, 1200, 2),  # Bottom boundary
    Wall(0, 0, 2, 600),  # Left boundary
    Wall(1180, 0, 2, 600),  # Right boundary

    # Internal Walls
    Wall(100, 0, 2, 400),  # Vertical wall from top stopping before the bottom
    Wall(200, 200, 2, 380),  # Vertical wall starting from somewhat top to bottom
    Wall(300, 0, 2, 180),  # Short vertical wall from top
    Wall(300, 280, 2, 300),  # Vertical wall starting from mid to bottom
    Wall(400, 400, 800, 2),  # Horizontal wall near the bottom
    Wall(1000, 0, 2, 380),  # Vertical wall near the right side

    # More Complex Elements
    # Wall(500, 180, 2, 300),  # Vertical barrier in the middle
    # Wall(600, 0, 2, 180),  # Vertical small wall from the top
    # Wall(800, 420, 380, 2),  # Horizontal wall
    # Wall(900, 220, 2, 200),  # Vertical wall from mid to near bottom
    # Wall(700, 200, 200, 2),  # Horizontal wall in the middle
    # Wall(400, 20, 2, 180),  # Vertical small wall from the top inside
    # Wall(400, 580, 600, 2),  # Horizontal wall near the bottom from left
    Wall(800, 180, 200, 2),  # Horizontal wall from mid to near right
]


# METHOD to detect collision between robot and walls
def detect_collision(robot, walls):
    robot_radius = robot.get_l() / 2 # radius of robot
    robot_center = np.array(robot.get_position()) # position of center of robot
    colliding_walls = [] # array that will contain walls that are colliding with robot

    for wall in walls: # loop through every walls
        closest_x = max(wall.rect.left, min(robot_center[0], wall.rect.right)) # closest x coordinates
        closest_y = max(wall.rect.top, min(robot_center[1], wall.rect.bottom)) # closest y coordinates
        distance = np.sqrt((closest_x - robot_center[0]) ** 2 + (closest_y - robot_center[1]) ** 2)

        if distance <= robot_radius: # if distance <= radius, then it is colliding
            colliding_walls.append(wall)

    if len(colliding_walls) > 0:
        return True, colliding_walls # if array is not empty, return true, else, return false
    else:
        return False, colliding_walls


running = True
while running:

    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Constants
    ACCELERATION = ROBOT_SPEED_PER_SECOND * 15  # constant of acceleration of robot
    DECELERATION = ACCELERATION  # deceleration symmetrical to acceleration
    right_wheel_velocity = robot.get_velocity()[1]
    left_wheel_velocity = robot.get_velocity()[0]
    MAX_SPEED = 300 # max speed of a wheel

    # Right wheel control
    key = pygame.key.get_pressed()
    if key[pygame.K_RIGHT]:
        new_velocity = right_wheel_velocity + ACCELERATION * delta_t
        if new_velocity <= MAX_SPEED:
            right_wheel_velocity = new_velocity
        else:
            right_wheel_velocity = MAX_SPEED
    elif key[pygame.K_z]:
        new_velocity = right_wheel_velocity - DECELERATION * delta_t
        if new_velocity >= -MAX_SPEED:
            right_wheel_velocity = new_velocity
        else:
            right_wheel_velocity = -MAX_SPEED
    else:
        if right_wheel_velocity != 0:
            if abs(right_wheel_velocity) - DECELERATION * delta_t > 0:
                right_wheel_velocity -= DECELERATION * delta_t * np.sign(right_wheel_velocity)
            else:
                right_wheel_velocity = 0

    # Left wheel control
    if key[pygame.K_LEFT]:
        new_velocity = left_wheel_velocity + ACCELERATION * delta_t
        if new_velocity <= MAX_SPEED:
            left_wheel_velocity = new_velocity
        else:
            left_wheel_velocity = MAX_SPEED
    elif key[pygame.K_a]:
        new_velocity = left_wheel_velocity - DECELERATION * delta_t
        if new_velocity >= -MAX_SPEED:
            left_wheel_velocity = new_velocity
        else:
            left_wheel_velocity = -MAX_SPEED
    else:
        if left_wheel_velocity != 0:
            if abs(left_wheel_velocity) - DECELERATION * delta_t > 0:
                left_wheel_velocity -= DECELERATION * delta_t * np.sign(left_wheel_velocity)
            else:
                left_wheel_velocity = 0

    # Set updated velocities to the robot
    robot.set_velocity((left_wheel_velocity, right_wheel_velocity))

    # Collision check
    coll, wall = detect_collision(robot, walls)
    if coll:
        robot.updatewall(wall, delta_t)
        robot.detect_walls(walls)
    else:
        robot.update(delta_t)
        robot.detect_walls(walls)

    # Clear the screen
    screen.fill((15, 1, 48))

    # Display wheels
    RADIUS = robot.get_l() / 2
    angle = robot.get_orientation() + 2 * math.pi / 4
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, GREY, (xx, yy), robot.get_l() / 10)
    angle = angle + math.pi
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle))
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle))
    pygame.draw.circle(screen, GREY, (xx, yy), robot.get_l() / 10)

    # Display Robot
    pygame.draw.circle(screen, PURPLE, (int(robot.get_position()[0]), int(robot.get_position()[1])), robot.get_l() / 2)

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
        dist_text = font.render(f"{int(robot.get_sensors()[i].get_dist())}", True, WHITE)
        text_x = robot.get_sensors()[i].get_position()[0] + 30 * math.cos(sensor_angle) - int(robot.get_l() / 8)
        text_y = robot.get_sensors()[i].get_position()[1] - 30 * math.sin(sensor_angle)
        screen.blit(dist_text, (text_x, text_y))

    # Display Wheels speed
    angle = robot.get_orientation() + 2 * math.pi / 4
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle) / 2)
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle) / 2)
    speedLeft_text = font.render(f"{int(robot.get_velocity()[0])}", True, WHITE)
    screen.blit(speedLeft_text, (xx, yy))
    angle = angle + math.pi
    xx = int(robot.get_position()[0] + RADIUS * math.cos(angle) / 2)
    yy = int(robot.get_position()[1] - RADIUS * math.sin(angle) / 2)
    speedRight_text = font.render(f"{int(robot.get_velocity()[1])}", True, WHITE)
    screen.blit(speedRight_text, (xx, yy))

    # Display Data
    font = pygame.font.SysFont(None, 26)  # Create a font object from the system fonts
    velocityX_text = font.render("Velocity X: " + str(int(robot.get_velocity_components()[0])), True, RED)
    screen.blit(velocityX_text, (10, 10))

    velocityX_text = font.render("Velocity Y: " + str(int(robot.get_velocity_components()[1])), True, RED)
    screen.blit(velocityX_text, (10, 30))

    velocityLeft_text = font.render("Velocity Left Wheel: " + format((robot.get_velocity()[0]), ".2f"), True, RED)
    screen.blit(velocityLeft_text, (10, 50))

    velocityLeft_text = font.render("Velocity Right Wheel: " + format((robot.get_velocity()[1]), ".2f"), True, RED)
    screen.blit(velocityLeft_text, (10, 70))

    R_text = font.render("R: " + str(int(robot.get_R())), True, BLUE)
    screen.blit(R_text, (10, 90))

    ICCX_text = font.render("ICC X : " + str(int(robot.get_icc()[0])), True, BLUE)
    screen.blit(ICCX_text, (10, 110))

    ICCX_text = font.render("ICC Y : " + str(int(robot.get_icc()[1])), True, BLUE)
    screen.blit(ICCX_text, (10, 130))

    tetha_text = font.render("TETHA: " + str(int(math.degrees(robot.get_orientation()))), True, BLUE)
    screen.blit(tetha_text, (10, 150))

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()
