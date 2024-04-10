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

# Initialize the Robot at the center of the screen, facing right (0 radians)
robot = Robot(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 1, 1, 0.05)  # Smaller dt for smoother simulation

# Clock to control the frame rate
clock = pygame.time.Clock()

# Flag to keep the game running
running = True

while running:
    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Key is pressed
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                # Increment left wheel speed
                current_speeds = robot.wheel_speeds
                robot.set_wheel_speeds((current_speeds[0] + 1.0, current_speeds[1]))
            elif event.key == pygame.K_RIGHT:
                # Decrement left wheel speed
                current_speeds = robot.wheel_speeds
                robot.set_wheel_speeds((current_speeds[0], current_speeds[1] + 1.0))

            elif event.key == pygame.K_a:
                # Decrement left wheel speed
                current_speeds = robot.wheel_speeds
                robot.set_wheel_speeds((current_speeds[0] - 1.0, current_speeds[1]))

            elif event.key == pygame.K_z:
                # Decrement right wheel speed
                current_speeds = robot.wheel_speeds
                robot.set_wheel_speeds((current_speeds[0] , current_speeds[1]-1.0))

    # Update the robot's position based on current wheel speeds
    robot.update_position()

    # Clear the screen
    screen.fill(WHITE)

    # Draw the robot as a circle
    pygame.draw.circle(screen, RED, (int(robot.position[0]), int(robot.position[1])), 20)

    # Draw the robot's orientation
    end_x = robot.position[0] + 20 * math.cos(robot.orientation)
    end_y = robot.position[1] + 20 * math.sin(robot.orientation)
    pygame.draw.line(screen, BLACK, (robot.position[0], robot.position[1]), (end_x, end_y), 2)

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()
