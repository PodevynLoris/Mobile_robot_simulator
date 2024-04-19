import math


class Sensor:

    def __init__(self, x, y, radius, angle, num_sensors, robot, id):
        self.x = x  # x coordinate of sensor position
        self.y = y  # y coordinate of sensor position
        self.radius = radius  # length of sensor detection
        self.angle = angle  # angle betwwen sensor line and orientation of robot
        self.num_sensors = num_sensors  # number of sensors of robot
        self.distance = radius  # distance between sensor and clossest wall on line detection
        self.robot = robot  # robot
        self.id = id  # id of sensor

        self.point_closest_wall_x = None  # x coordinate of closest wall point
        self.point_closest_wall_y = None  # y coordinate of closest wall point

    def get_position(self):
        return self.x, self.y

    def get_radius(self):
        return self.radius

    def get_dist(self):
        return self.distance

    def get_angle(self):
        return self.angle

    def set_angle(self, angle):
        self.angle = angle

    def get_num_sensors(self):
        return self.num_sensors

    def get_point_closest_wall(self):
        return self.point_closest_wall_x, self.point_closest_wall_y

    def set_position(self, x, y):
        self.x, self.y = x, y

    def detect_wall(self, walls):

        # Check if the sensor's line intersects with any walls and measure the distance
        end_x = self.x + self.radius * math.cos(self.angle)  # End point of the sensor's line
        end_y = self.y - self.radius * math.sin(self.angle)
        sensor_line = (self.x, self.y), (end_x, end_y)

        closest_distance = self.radius  # Initialize with length of detection

        for wall in walls:
            intersection_points = wall.rect.clipline(sensor_line)
            if intersection_points:
                # Calculate distances to each intersection point, return the minimum
                for point in intersection_points:
                    distance = math.sqrt((point[0] - self.x) ** 2 + (point[1] - self.y) ** 2)
                    if distance < closest_distance:
                        closest_distance = distance
                        self.point_closest_wall_x = point[0]
                        self.point_closest_wall_y = point[1]

        if closest_distance < self.radius:
            self.distance = closest_distance
            return True  # Wall detected, return true
        else:
            self.distance = self.radius
            return False  # No wall detected, return false
