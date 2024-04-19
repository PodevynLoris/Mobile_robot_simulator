# utils.py
import numpy as np

def get_wall_vectors(wall, robot_position):
    if wall.rect.width > wall.rect.height:
        normal = np.array([0, 1]) if robot_position[1] > wall.rect.centery else np.array([0, -1])
        tangent = np.array([1, 0])
    else:
        normal = np.array([1, 0]) if robot_position[0] < wall.rect.centerx else np.array([-1, 0])
        tangent = np.array([0, 1])
    return normal, tangent
