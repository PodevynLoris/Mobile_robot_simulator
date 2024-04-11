import math

import numpy as np

from Sensor import Sensor


class Robot2:
    def __init__(self, x, y, theta, l, num_sensors, radius_sensor):
        self._x = x
        self._y = y
        self._theta = theta
        self._l = l
        self._vl = 0
        self._vr = 0
        self._iccx = 0
        self._iccy = 0
        self._num_sensors = num_sensors
        self._radius_sensor = radius_sensor
        self._sensors = []

        self._SENSOR_ANGLE = 2 * math.pi / num_sensors
        for i in range(num_sensors):
            angle = self.get_orientation() + i * self._SENSOR_ANGLE
            x_sensor = self.get_position()[0] + self.get_l()/2 * math.cos(angle)
            y_sensor = self.get_position()[1] - self.get_l()/2 * math.sin(angle)
            self._sensors.append(Sensor(x_sensor, y_sensor, radius_sensor, angle, num_sensors, self, i))

    def get_position(self):
        return self._x, self._y

    def get_num_sensors(self):
        return self._num_sensors

    def get_sensors(self):
        return self._sensors

    def get_SENSOR_ANGLE(self):
        return self._SENSOR_ANGLE

    def get_orientation(self):
        return self._theta

    def get_velocity(self):
        return self._vl, self._vr

    def get_l(self):
        return self._l

    def get_icc(self):
        return self._iccx, self._iccy

    def set_velocity(self, velocity):
        self._vl, self._vr = velocity

    def update(self, delta_t):
        w = (self._vr - self._vl) / self._l  # angular velocity in radians per second

        if self._vr == self._vl:  # If moving straight
            # Move straight, no rotation
            self._x += self._vr * delta_t * math.cos(self._theta)
            # Invert the y-axis change
            self._y -= self._vr * delta_t * math.sin(self._theta)  # Negate the change in y
        else:
            # Compute radius to ICC
            R = (self._l / 2) * ((self._vl + self._vr) / (self._vr - self._vl))
            ICCx = self._x - R * math.sin(self._theta)
            ICCy = self._y + R * math.cos(self._theta)  # Correct ICC calculation remains the same
            self._iccx = ICCx
            self._iccy = ICCy

            # Rotation matrix for rotation about ICC
            rotation_matrix = np.array([
                [np.cos(w * delta_t), -np.sin(w * delta_t), 0],
                [np.sin(w * delta_t), np.cos(w * delta_t), 0],
                [0, 0, 1]
            ])

            # Position vector relative to ICC
            position_vector = np.array([self._x - ICCx, self._y - ICCy, 1])

            # Perform rotation
            rotated_position = rotation_matrix @ position_vector
            final_position = rotated_position + np.array([ICCx, ICCy, 0])

            # Update position
            self._x = final_position[0]
            self._y = final_position[1]  # No need to negate y here because it's handled in rotation

        self._theta += w * delta_t  # Ensure any theta adjustment is in radians

        for i in range(self._num_sensors):
            SENSOR_ANGLE = 2 * math.pi / self._num_sensors
            angle = self.get_orientation() + i * SENSOR_ANGLE
            x_sensor = self.get_position()[0] + self.get_l() / 2 * math.cos(angle)
            y_sensor = self.get_position()[1] - self.get_l() / 2 * math.sin(angle)

            self._sensors[i].set_position(x_sensor,y_sensor) #= Sensor(x_sensor, y_sensor, self._radius_sensor, angle, self._num_sensors)


    def set_position(self,x,y):
        self._x = int(x)
        self._y = int(y)

    def detect_walls(self, walls):
        for sensor in self._sensors:
            sensor.detect_wall(walls)
               #print(f"Wall detected by sensor at angle {sensor.get_angle()} radians")