import pygame
import math
from Robot import Robot

# Initialize Pygame
pygame.init()

# Set up the display
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Simulation")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

RADIUS = 50
SPEED = 2
# Initialize the Robot at the center of the screen, facing right (0 radians)
robot = Robot(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 1, 1, 0.05)  # Smaller dt for smoother simulation

# Clock to control the frame rate
clock = pygame.time.Clock()

# Number of sensors
DISTANCE = 200  # Constant distance to obstacles
NUM_SENSORS = 12
SENSOR_ANGLE = 2 * math.pi / NUM_SENSORS

# Setup font for distance labels
pygame.font.init()  # Initialize font module
font = pygame.font.SysFont(None, 18)  # Create a font object from the system fonts

# Flag to keep the game running
running = True

while running:
    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    key = pygame.key.get_pressed()

    # Handle combined key presses first
    if key[pygame.K_a] and key[pygame.K_z]:
        robot.set_wheel_speeds((-SPEED, -SPEED))
    elif key[pygame.K_LEFT] and key[pygame.K_RIGHT]:
        robot.set_wheel_speeds((SPEED, SPEED))
    else:
        # Handle individual key presses
        left_speed = -SPEED if key[pygame.K_a] else SPEED if key[pygame.K_LEFT] else 0
        right_speed = -SPEED if key[pygame.K_z] else SPEED if key[pygame.K_RIGHT] else 0

        # Apply new wheel speeds if any key is pressed, otherwise stop
        if key[pygame.K_LEFT] or key[pygame.K_RIGHT] or key[pygame.K_a] or key[pygame.K_z]:
            robot.set_wheel_speeds((left_speed, right_speed))
        else:
            robot.set_wheel_speeds((0, 0))
    # Update the robot's position based on current wheel speeds
    robot.update_position(SCREEN_WIDTH, SCREEN_HEIGHT)

    # Clear the screen
    screen.fill(WHITE)

    # Draw the robot as a circle
    pygame.draw.circle(screen, RED, (int(robot.position[0]), int(robot.position[1])), RADIUS)

    # Draw the robot's orientation
    end_x = robot.position[0] + RADIUS * math.cos(robot.orientation)
    end_y = robot.position[1] - RADIUS * math.sin(robot.orientation)
    pygame.draw.line(screen, BLACK, (robot.position[0], robot.position[1]), (end_x, end_y), 2)

    for i in range(NUM_SENSORS):
        angle = robot.orientation + i * SENSOR_ANGLE
        end_x = robot.position[0] + RADIUS * math.cos(angle)
        end_y = robot.position[1] - RADIUS * math.sin(angle)
        pygame.draw.line(screen, GREEN, robot.position, (end_x, end_y), 1)

        # Display the distance value
        dist_text = font.render(f"{DISTANCE}", True, BLACK)
        text_x = robot.position[0] + RADIUS * math.cos(angle)  # Adjust text position a bit beyond the line end
        text_y = robot.position[1] - RADIUS * math.sin(angle)
        screen.blit(dist_text, (text_x, text_y))

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()
