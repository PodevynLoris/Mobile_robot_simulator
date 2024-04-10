import math

class Robot:
    def __init__(self, x=0, y=0, theta=0, r=1, l=1, dt=0.1):
        self._x = x
        self._y = y
        self._theta = theta
        self._r = r
        self._l = l
        self._v_l = 0
        self._v_r = 0
        self._dt = dt

    # Getters
    @property
    def position(self):
        return (self._x, self._y)

    @property
    def orientation(self):
        return self._theta

    @property
    def wheel_speeds(self):
        return (self._v_l, self._v_r)

    # Setters
    
    def wheel_speeds(self, speeds):
        self._v_l, self._v_r = speeds

    def update_position(self):
        v = (self._r / 2) * (self._v_l + self._v_r)
        omega = (self._r / self._l) * (self._v_r - self._v_l)

        # Update robot's orientation
        self._theta += omega * self._dt
        self._theta = self._theta % (2 * math.pi)  # Keep theta within [0, 2*pi]
        
        # Update robot's position
        self._x += v * math.cos(self._theta) * self._dt
        self._y += v * math.sin(self._theta) * self._dt

# Example usage
robot = Robot()
robot.wheel_speeds = (1, 1)  # Move forward
robot.update_position()

# Accessing position and orientation
print(f"Robot Position: {robot.position}")
print(f"Robot Orientation: {robot.orientation} radians")
