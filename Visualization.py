import pygame

from Robot import Robot

pygame.init()
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Initialize the Robot
robot = Robot(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 1, 1, 0.1)  # Start at center of the screen
# Clock to control the frame rate
clock = pygame.time.Clock()



running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

 # Key is pressed
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                # Activate/adjust left wheel speed
                robot.set_wheel_speeds(-1, robot.v_r)
            if event.key == pygame.K_RIGHT:
                # Activate/adjust right wheel speed
                robot.set_wheel_speeds(robot.v_l, -1)
            if event.key == pygame.K_UP:
                # Increase forward speed
                robot.set_wheel_speeds(robot.v_l + 0.5, robot.v_r + 0.5)
            if event.key == pygame.K_DOWN:
                # Decrease speed (or reverse)
                robot.set_wheel_speeds(robot.v_l - 0.5, robot.v_r - 0.5)

        # Key is released
        if event.type == pygame.KEYUP:
            if event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP, pygame.K_DOWN):
                # Stop the robot when keys are released
                robot.set_wheel_speeds(0, 0)

    # Update the robot's position
    robot.update_position()

    # Fill the screen with white
    screen.fill(WHITE)

    # Draw the robot as a circle
    pygame.draw.circle(screen, RED, (int(robot._x), int(robot._y)), 10)

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()