"""
sim/robot.py - the robot model used by every simulation in this project.

This file describes ONE robot's BODY: its wheels, its motor limits and how it
moves. It contains no "brain" (no decisions). The brain lives elsewhere and
only ever asks the body: "please move with forward speed v and turn rate omega".

Keeping body and brain separate matters later:
  - the same body is reused by every mission (cover, gather, draw, push ...)
  - on the real robot, this file's maths becomes ESP32 firmware.

Units used everywhere in this project:
  distance  -> metres (m)
  angle     -> radians (rad)      0 = facing +x (right), pi/2 = facing +y (up)
  speed     -> metres per second (m/s)
  turn rate -> radians per second (rad/s)
  time      -> seconds (s)
"""

import math


class DiffDriveRobot:
    """A two-wheeled (differential drive) robot.

    Top view:            L = left wheel, R = right wheel
       +-----------+     . = robot centre (this is the point (x, y))
       |  L  .  R  |     wheel_base = distance between the two wheels
       +-----------+

    The robot has no steering wheel. It turns by spinning its two wheels
    at DIFFERENT speeds - that difference is why it is called "differential".
    """

    def __init__(self, x, y, theta,
                 wheel_base=0.08,          # 8 cm between wheels (a guess until the chassis is designed)
                 max_wheel_speed=0.15,     # software speed limit per wheel, in m/s
                 limit_mode="scale"):      # how to obey that limit: "scale" (correct) or "clamp" (experiment 4)
        # --- Pose: where the robot is and which way it faces ---
        self.x = x              # position along the arena's x-axis (m)
        self.y = y              # position along the arena's y-axis (m)
        self.theta = theta      # heading (rad)

        # --- Physical properties of the robot ---
        self.wheel_base = wheel_base
        self.max_wheel_speed = max_wheel_speed
        self.limit_mode = limit_mode

        # --- Current wheel speeds (m/s). Positive = wheel rolls forward. ---
        # The robot stores WHEEL speeds, exactly like the real hardware,
        # because motors only understand wheel speeds.
        self.v_left = 0.0
        self.v_right = 0.0

    # ------------------------------------------------------------------
    # 1. Brain's wish -> wheel speeds  (called once per loop by the brain)
    # ------------------------------------------------------------------
    def set_velocity(self, v, omega):
        """Turn the brain's request (v, omega) into wheel speeds the motors can do.

        v     : desired forward speed of the robot centre (m/s)
        omega : desired turning rate (rad/s). Positive = turn LEFT (anticlockwise).
        """
        # Inverse kinematics: (v, omega) -> (v_left, v_right).
        # To turn left, the right wheel must go faster than the left one.
        # Each wheel sits wheel_base/2 away from the centre, so the turn
        # adds omega * wheel_base/2 to one wheel and removes it from the other.
        v_right = v + omega * self.wheel_base / 2
        v_left = v - omega * self.wheel_base / 2

        # Motors have a top speed. How fast is the fastest wheel being asked to go?
        # abs() ignores direction, so a wheel spinning backwards too fast is caught too.
        fastest = max(abs(v_left), abs(v_right))

        if fastest > self.max_wheel_speed:
            if self.limit_mode == "scale":
                # CORRECT method: slow BOTH wheels by the same factor.
                # The ratio between the wheels stays the same, so the turning
                # radius (and therefore the path) stays the same - the robot
                # simply travels along that path more slowly.
                scale = self.max_wheel_speed / fastest     # a number below 1
                v_left *= scale                            # same as v_left = v_left * scale
                v_right *= scale
            elif self.limit_mode == "clamp":
                # WRONG method (kept only for experiment 4): cut each wheel at
                # the limit separately. The difference between the wheels
                # shrinks, so the robot turns LESS than the brain asked for.
                v_left = max(-self.max_wheel_speed, min(v_left, self.max_wheel_speed))
                v_right = max(-self.max_wheel_speed, min(v_right, self.max_wheel_speed))

        # Store the final wheel speeds. step() will use them.
        self.v_left = v_left
        self.v_right = v_right

    # ------------------------------------------------------------------
    # 2. Wheel speeds -> how the body actually moves
    # ------------------------------------------------------------------
    def body_velocity(self):
        """Forward kinematics: wheel speeds -> (v, omega) of the robot centre.

        v     = average of the two wheels
        omega = difference of the wheels, divided by the wheel base
                (a wider robot turns more slowly for the same difference)
        """
        v = (self.v_right + self.v_left) / 2
        omega = (self.v_right - self.v_left) / self.wheel_base
        return v, omega          # a function can return two values at once

    # ------------------------------------------------------------------
    # 3. Move forward in time by one small step
    # ------------------------------------------------------------------
    def step(self, dt):
        """Advance the robot by dt seconds (Euler integration).

        For one short step we pretend the robot drives in a straight line
        along its current heading, then turns. Small dt = accurate.
        """
        v, omega = self.body_velocity()               # what the wheels are doing right now
        self.x += v * math.cos(self.theta) * dt       # cos splits speed into its x part
        self.y += v * math.sin(self.theta) * dt       # sin splits speed into its y part
        self.theta += omega * dt                      # heading changes by turn rate x time