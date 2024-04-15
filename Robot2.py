import math

import numpy as np

from Sensor import Sensor
from utils import get_wall_vectors


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
        self._R = 0

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

    def get_velocity_components(self):
        """Compute the velocity components v_x and v_y based on the current orientation and wheel velocities."""
        V = (self._vr + self._vl) / 2  # Translational velocity
        v_x = V * math.cos(self._theta)  # Velocity component in the x-direction
        v_y = V * math.sin(self._theta)  # Velocity component in the y-direction
        return v_x, v_y

    def get_l(self):
        return self._l

    def get_icc(self):
        return self._iccx, self._iccy

    def set_velocity(self, velocity, increment=False):
        if increment:
            # Increment the current velocities
            self._vl += velocity[0]
            self._vr += velocity[1]
        else:
            # Set the velocities directly
            self._vl, self._vr = velocity

    # def increment_speed(self, increment):
    #     # Retrieve the current velocities
    #     current_left_speed, current_right_speed = self.get_velocity()
        
    #     # Increment the speeds
    #     new_left_speed = current_left_speed + increment
    #     new_right_speed = current_right_speed + increment
        
    #     # Set the new velocities
    #     self.set_velocity((new_left_speed, new_right_speed))
 
        
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


        w = (self._vr - self._vl) / self._l  # angular velocity in radians per second

        if self._vr == self._vl:  # If moving straight
            # Move straight, no rotation
            self._x += self._vr * delta_t * math.cos(self._theta)
            # Invert the y-axis change
            self._y -= self._vr * delta_t * math.sin(self._theta)  # Negate the change in y
        else:
            # Compute radius to ICC
            self._R = (self._l / 2) * ((self._vl + self._vr) / (self._vr - self._vl))
            ICCx = self._x - self._R * math.sin(self._theta)
            ICCy = self._y + self._R * math.cos(self._theta)  # Correct ICC calculation remains the same
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
            self._sensors[i].set_angle(angle)


    def set_position(self,x,y):
        self._x = int(x)
        self._y = int(y)

    def detect_walls(self, walls):
        for sensor in self._sensors:
            sensor.detect_wall(walls)
               #print(f"Wall detected by sensor at angle {sensor.get_angle()} radians")

    def get_R(self):
        return self._R

    def updatewall(self, wall, delta_t):

        w = (self._vr - self._vl) / self._l  # angular velocity in radians per second

        wall = get_wall_vectors(wall,[self._x, self._y])
        print(wall)
        if wall[1] == 0 and (wall[0]==1 or wall[0]==-1):  # vertical
            self._y -= self._vr * delta_t * math.sin(self._theta)  # Negate the change in y
            self._theta += w * delta_t  # Ensure any theta adjustment is in radians
            print(self._x,self._y)
        if wall[0]==1 and (wall[1]==-1 or wall[1]==1): # horizontal
            self._x += self._vr * delta_t * math.cos(self._theta)
            self._theta += w * delta_t  # Ensure any theta adjustment is in radians
            print(self._x, self._y)

        for i in range(self._num_sensors):
            SENSOR_ANGLE = 2 * math.pi / self._num_sensors
            angle = self.get_orientation() + i * SENSOR_ANGLE
            x_sensor = self.get_position()[0] + self.get_l() / 2 * math.cos(angle)
            y_sensor = self.get_position()[1] - self.get_l() / 2 * math.sin(angle)

            self._sensors[i].set_position(x_sensor,
                                          y_sensor)  # = Sensor(x_sensor, y_sensor, self._radius_sensor, angle, self._num_sensors)
            self._sensors[i].set_angle(angle)









