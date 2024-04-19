import math

import numpy as np

from Sensor import Sensor
from utils import get_wall_vectors


class Robot2:
    def __init__(self, x, y, theta, l, num_sensors, radius_sensor):
        self.x = x # x coordinates of center of robot
        self.y = y # y coordinates of center of robot
        self.theta = theta # angle of orientation of robot
        self.l = l # diameter of robot
        self.vl = 0 # velocity of left wheel
        self.vr = 0 # velocity of right wheel
        self.iccx = 0 # x coordinates of Instantaneous Center of Curvature
        self.iccy = 0 # y coordinates of  Instantaneous Center of Curvature
        self.num_sensors = num_sensors # number of sensors of robot
        self.radius_sensor = radius_sensor # length of sensor detection
        self.sensors = [] # array that contains sensors object
        self.R = 0 # signed distance from the ICC to the midpoint between the wheels
        self.vx = 0 # velocity on x-axis
        self.vy = 0 # velocity on y-axis

        # Initialization od sensors
        self._SENSOR_ANGLE = 2 * math.pi / num_sensors
        for i in range(num_sensors):
            angle = self.get_orientation() + i * self._SENSOR_ANGLE
            x_sensor = self.get_position()[0] + self.get_l() / 2 * math.cos(angle)
            y_sensor = self.get_position()[1] - self.get_l() / 2 * math.sin(angle)
            self.sensors.append(Sensor(x_sensor, y_sensor, radius_sensor, angle, num_sensors, self, i))

    def get_position(self):
        return self.x, self.y

    def get_num_sensors(self):
        return self.num_sensors

    def get_sensors(self):
        return self.sensors

    def get_SENSOR_ANGLE(self):
        return self._SENSOR_ANGLE

    def get_orientation(self):
        return self.theta

    def get_velocity(self):
        return self.vl, self.vr

    def get_velocity_components(self):

        return self.vx, self.vy

    def get_l(self):
        return self.l

    def get_icc(self):
        return self.iccx, self.iccy

    def set_velocity(self, velocity, increment=False):
        if increment:
            # Increment the current velocities
            self.vl += velocity[0]
            self.vr += velocity[1]
        else:
            # Set the velocities directly
            self.vl, self.vr = velocity

    def update(self, delta_t):
        # Calculate angular velocity
        w = (self._vr - self._vl) / self._l

        if abs(self._vr - self._vl) > 1e-10:  # Avoid division by zero with a small threshold
            # Compute radius to ICC (Instantaneous Center of Curvature)
            R = (self._l / 2) * ((self._vl + self._vr) / (self._vr - self._vl))

            # Compute the ICC (Instantaneous Center of Curvature) coordinates
            ICCx = self._x - R * math.sin(self._theta)
            ICCy = self._y + R * math.cos(self._theta)

            # Compute the rotation matrix
            rotation_matrix = np.array([
                [np.cos(w * delta_t), -np.sin(w * delta_t), 0],
                [np.sin(w * delta_t),  np.cos(w * delta_t), 0],
                [0,                    0,                    1]
            ])

            # Compute the position vector relative to ICC
            position_vector = np.array([self._x - ICCx, self._y - ICCy, 1])

            # Compute the rotated position
            rotated_position = rotation_matrix @ position_vector
            final_position = rotated_position + np.array([ICCx, ICCy, 0])

            # Update position
            self._x = final_position[0]
            self._y = final_position[1]

        else:  # When vr and vl are very close, the robot is effectively moving in a straight line
            self._x += self._vr * delta_t * math.cos(self._theta)
            self._y += self._vr * delta_t * math.sin(self._theta)

        # Update the orientation
        self._theta += w * delta_t

        # Normalize the orientation to the range [0, 2*pi)
        self._theta %= 2 * math.pi

        V = (self.vr + self.vl) / 2  # Translational velocity
        self.vx = V * math.cos(self.theta)  # Velocity component in the x-direction
        self.vy = V * math.sin(self.theta)  # Velocity component in the y-direction (
        w = (self.vr - self.vl) / self.l  # Angular velocity

        # If both wheel have same velocity, go straight (no rotation)
        if self.vr == self.vl:
            # Move straight, no rotation
            self.x = self.x + self.vr * np.cos(self.theta) * delta_t
            # Invert the y-axis change
            self.y = self.y - self.vr * np.sin(self.theta) * delta_t
            self.iccx = 0
            self.iccy = 0
        # If both wheel do not have same velocity -> rotation adapted to y-axis increasing downward
        else:
            self.R = (self.l / 2) * ((self.vr + self.vl) / (self.vr - self.vl))
            ICC = [self.x - (self.R * np.sin(self.theta)), self.y - (self.R * np.cos(self.theta))]
            self.iccx = ICC[0]
            self.iccy = ICC[1]

            rotation_matrix = np.array([
                [np.cos(w * delta_t), np.sin(w * delta_t), 0],
                [-np.sin(w * delta_t), np.cos(w * delta_t), 0],
                [0, 0, 1]
            ])

            icc_matrix = np.array([
                [self.x - self.iccx],
                [self.y - self.iccy],
                [self.theta]

            ])

            icc2_matrix = np.array([
                [self.iccx],
                [self.iccy],
                [w * delta_t]
            ])

            multiplication_matrix = rotation_matrix @ icc_matrix + icc2_matrix
            self.x = multiplication_matrix[0, 0]
            self.y = multiplication_matrix[1, 0]
            self.theta = multiplication_matrix[2, 0] % (2 * math.pi)

        self.update_sensors()

    def set_position(self, x, y):
        self.x = int(x)
        self.y = int(y)

    def detect_walls(self, walls):
        for sensor in self.sensors:
            sensor.detect_wall(walls)

    def get_R(self):
        return self.R

    def update_sensors(self):
        for i in range(self.num_sensors):
            SENSOR_ANGLE = 2 * math.pi / self.num_sensors
            angle = self.get_orientation() + i * SENSOR_ANGLE
            x_sensor = self.get_position()[0] + self.get_l() / 2 * math.cos(angle)
            y_sensor = self.get_position()[1] - self.get_l() / 2 * math.sin(angle)

            self.sensors[i].set_position(x_sensor, y_sensor)
            self.sensors[i].set_angle(angle)

    def updatewall(self, walls, delta_t):

        V = (self.vr + self.vl) / 2  # Translational velocity
        self.vx = 0  # Velocity component in the x-direction
        self.vy = 0  # Velocity component in the y-direction

        w = (self.vr - self.vl) / self.l  # angular velocity in radians per second
        potential_x = self.x
        potential_y = self.y

        if self.vr == self.vl:  # If moving straight
            # Move straight, no rotation
            potential_x += self.vr * delta_t * math.cos(self.theta)
            # Invert the y-axis change
            potential_y -= self.vr * delta_t * math.sin(self.theta)  # Negate the change in y
        else:

            self.R = (self.l / 2) * ((self.vr + self.vl) / (self.vr - self.vl))
            ICC = [self.x - (self.R * np.sin(self.theta)), self.y - (self.R * np.cos(self.theta))]
            self.iccx = ICC[0]
            self.iccy = ICC[1]

            rotation_matrix = np.array([
                [np.cos(w * delta_t), np.sin(w * delta_t), 0],
                [-np.sin(w * delta_t), np.cos(w * delta_t), 0],
                [0, 0, 1]
            ])

            icc_matrix = np.array([
                [self.x - self.iccx],
                [self.y - self.iccy],
                [self.theta]

            ])

            icc2_matrix = np.array([
                [self.iccx],
                [self.iccy],
                [w * delta_t]
            ])

            multiplication_matrix = rotation_matrix @ icc_matrix + icc2_matrix
            potential_x = multiplication_matrix[0, 0]
            potential_y = multiplication_matrix[1, 0]

        h = False
        v = False
        norm1, tan1 = 0, 0
        norm2, tan2 = 0, 0

        for i in range(len(walls)):
            ww = walls[i]
            index = 0
            normal, tangent = get_wall_vectors(ww, [self.x, self.y])
            if tangent[0] == 0 and tangent[1] == 1:  # Vertical wall
                v = True
                index = i
                norm1, tan1 = get_wall_vectors(ww, [self.x, self.y])

            if tangent[0] == 1 and tangent[1] == 0:
                h = True
                index = i
                norm2, tan2 = get_wall_vectors(ww, [self.x, self.y])

        # Initialize adjustments
        adjust_x, adjust_y = self.x, self.y

        # Vertical wall logic
        if v and h == False:  # Vertical wall
            adjust_y = potential_y
            self.vy = V * math.sin(self.theta)
            normal, tangent = get_wall_vectors(walls[index], [self.x, self.y])
            if (normal[0] == 1 and potential_x + self.l / 2 < ww.rect.left) or (
                    normal[0] == -1 and potential_x - self.l / 2 > ww.rect.right):
                adjust_x = potential_x
                self.vx = V * math.cos(self.theta)

                # Horizontal wall logic
        if h and v == False:  # Horizontal wall
            adjust_x = potential_x
            self.vx = V * math.cos(self.theta)
            normal, tangent = get_wall_vectors(walls[index], [self.x, self.y])
            if (normal[1] == 1 and potential_y - self.l / 2 > ww.rect.bottom) or (
                    normal[1] == -1 and potential_y + self.l / 2 < ww.rect.top):
                adjust_y = potential_y
                self.vy = V * math.sin(self.theta)

                # Corner logic
        if h and v:
            if (norm1[0] == 1 and potential_x + self.l / 2 < ww.rect.left) or (
                    norm1[0] == -1 and potential_x - self.l / 2 > ww.rect.right):
                adjust_x = potential_x
                self.vx = V * math.cos(self.theta)
            if (norm2[1] == 1 and potential_y - self.l / 2 > ww.rect.bottom) or (
                    norm2[1] == -1 and potential_y + self.l / 2 < ww.rect.top):
                adjust_y = potential_y
                self.vy = V * math.sin(self.theta)

                # Apply position adjustments
        self.x, self.y = adjust_x, adjust_y
        self.theta += w * delta_t  # Update orientation

        # Update sensor positions
        self.update_sensors()
