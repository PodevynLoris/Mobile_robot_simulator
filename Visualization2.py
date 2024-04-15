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

ROBOT_SPEED_PER_SECOND = 10
# Limit frames per second
delta_ms = clock.tick(60)  # Caps the frame rate at 60 FPS and measures frame duration
delta_t = delta_ms / 1000.0  # Converts milliseconds to seconds


robot = Robot2(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, 0, 80, 12, 200)

walls = [

    Wall(600, 120, 20, 260),
    Wall(300, 380, 400, 20)
]
'''Wall(100, 100, 600, 20),
  Wall(100, 120, 20, 360),
  Wall(100, 460, 620, 20),
  Wall(700, 120, 20, 360),
  Wall(200, 120, 20, 260),
  Wall(300, 200, 20, 200),
  Wall(400, 120, 20, 200),
  Wall(500, 200, 20, 200),'''
<<<<<<< Updated upstream

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


=======
  
>>>>>>> Stashed changes
running = True
while running:

    # Handle each event in the event queue
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False



    key = pygame.key.get_pressed()

    if key[pygame.K_RIGHT]:
        robot.set_velocity((0, ROBOT_SPEED_PER_SECOND * delta_t), True)
    if key[pygame.K_LEFT]:
        robot.set_velocity((ROBOT_SPEED_PER_SECOND * delta_t, 0), True)
    if key[pygame.K_z]:
        robot.set_velocity((0, -ROBOT_SPEED_PER_SECOND * delta_t), True)
    if key[pygame.K_a]:
        robot.set_velocity((-ROBOT_SPEED_PER_SECOND * delta_t, 0), True)
    # # Handle combined key presses first
    # if key[pygame.K_a] and key[pygame.K_z]:
    #     robot.set_velocity((-SPEED, -SPEED))
    # elif key[pygame.K_LEFT] and key[pygame.K_RIGHT]:
    #     robot.set_velocity((SPEED, SPEED))
    # else:
    #     # Handle individual key presses
    #     left_speed = -SPEED if key[pygame.K_a] else SPEED if key[pygame.K_LEFT] else 0
    #     right_speed = -SPEED if key[pygame.K_z] else SPEED if key[pygame.K_RIGHT] else 0

    #     # Apply new wheel speeds if any key is pressed, otherwise stop
    #     if key[pygame.K_LEFT] or key[pygame.K_RIGHT] or key[pygame.K_a] or key[pygame.K_z]:
    #         robot.set_velocity((left_speed, right_speed))
    #     else:
    #         robot.set_velocity((0, 0))

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



    # Display Data
    font = pygame.font.SysFont(None, 26)  # Create a font object from the system fonts
    velocityX_text = font.render("Velocity X: " + str(int(robot.get_velocity_components()[0])), True, RED)
    screen.blit(velocityX_text, (10,10))

    velocityX_text = font.render("Velocity Y: " + str(int(robot.get_velocity_components()[1])), True, RED)
    screen.blit(velocityX_text, (10,30))

    velocityLeft_text = font.render("Velocity Left Wheel: " + str(int(robot.get_velocity()[0])), True, RED)
    screen.blit(velocityLeft_text, (10,50))

    velocityLeft_text = font.render("Velocity Right Wheel: " + str(int(robot.get_velocity()[1])), True, RED)
    screen.blit(velocityLeft_text, (10, 70))

    R_text = font.render("R: " + str(int(robot.get_R())), True, BLUE)
    screen.blit(R_text, (10,90))

    ICCX_text = font.render("ICC X : " + str(int(robot.get_icc()[0])), True, BLUE)
    screen.blit(ICCX_text, (10,110))

    ICCX_text = font.render("ICC Y : " + str(int(robot.get_icc()[1])), True, BLUE)
    screen.blit(ICCX_text, (10, 130))

    tetha_text = font.render("TETHA: "+ str(int(math.degrees(robot.get_orientation()))), True, BLUE)
    screen.blit(tetha_text, (10,150))

    # Update the display
    pygame.display.flip()

    # Limit frames per second
    clock.tick(60)

# Quit Pygame
pygame.quit()
