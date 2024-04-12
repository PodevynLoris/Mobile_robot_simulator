import math


class Sensor:

    def __init__(self, x, y, radius, angle, num_sensors, robot, id):
        self._x = x
        self._y = y
        self._radius = radius
        self._angle = angle
        self._num_sensors = num_sensors
        self._distance = radius
        self._robot = robot
        self._id = id

        self._point_closest_wall_x = None
        self._point_closest_wall_y = None

    def get_position(self):
        return self._x, self._y

    def get_radius(self):
        return self._radius

    def get_dist(self):
        return self._distance

    def get_angle(self):
        return self._angle

    def set_angle(self, angle):
        self._angle =angle

    def get_num_sensors(self):
        return self._num_sensors

    def get_point_closest_wall(self):
        return self._point_closest_wall_x, self._point_closest_wall_y

    def set_position(self,x,y):
        self._x, self._y = x,y

    def detect_wall(self, walls):

        """Check if the sensor's line intersects with any walls and measure the distance."""
        end_x = self._x + self._radius * math.cos(self._angle)  # End point of the sensor's line
        end_y = self._y - self._radius * math.sin(self._angle)
        sensor_line = (self._x, self._y), (end_x, end_y)

        closest_distance = self._radius  # Initialize with infinity

        for wall in walls:
            intersection_points = wall.rect.clipline(sensor_line)
            if intersection_points:
                # Calculate distances to each intersection point, return the minimum
                for point in intersection_points:
                    distance = math.sqrt((point[0] - self._x) ** 2 + (point[1] - self._y) ** 2)
                    if distance < closest_distance:
                        closest_distance = distance
                        self._point_closest_wall_x = point[0]
                        self._point_closest_wall_y = point[1]

        if closest_distance < float('inf'):
            self._distance = closest_distance
            return True  # Wall detected, return distance
        self._distance = self._radius
        return False # No wall detected
