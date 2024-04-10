import math

class Robot:
    def __init__(self, x=0, y=0, theta=0, r=1, l=1, dt=0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.r = r
        self.l = l
        self.v_l = 0
        self.v_r = 0
        self.dt = dt

    def set_wheel_speeds(self, v_l, v_r):
        self.v_l = v_l
        self.v_r = v_r

    def update_position(self):
        # Calculate linear and angular velocities
        v = (self.r / 2) * (self.v_l + self.v_r)
        omega = (self.r / self.l) * (self.v_r - self.v_l)
        
        # Update robot's orientation
        self.theta += omega * self.dt
        self.theta = self.theta % (2 * math.pi)  # Keep theta within [0, 2*pi]
        
        # Update robot's position
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt

# Example usage
robot = Robot()
robot.set_wheel_speeds(1, 1)  # Move forward
robot.update_position()


# Position (x, y): The robot's current position in the 2D space.
# Orientation (theta): The robot's current orientation (angle) in radians, relative to a global reference, typically the positive direction of the x-axis.
# Wheel Radius (r): Radius of each wheel. This is important for converting wheel rotations into linear movement of the robot.
# Wheel Distance (l): The distance between the two wheels. This impacts the robot's turning capability.
# Left Wheel Speed (v_l): Speed of the left wheel. Positive for forward, negative for backward.
# Right Wheel Speed (v_r): Speed of the right wheel. Positive for forward, negative for backward.
# Time Step (dt): The discrete time step for the simulation. This is crucial for updating the robot's position and orientation over time.
