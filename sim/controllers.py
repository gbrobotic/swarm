"""
sim/controllers.py - robot BRAINS (decision-making).

A controller looks at what the robot knows (its pose and a goal) and decides
(v, omega). It never moves the robot itself - that is the body's job
(sim/robot.py). Keeping brain and body separate means:
  - the same brain can drive a simulated robot or, later, a real one
  - the brain only uses what a sensor could tell it, never "secret" true values

See the diagrams 1_go_to_goal_geometry.png and 3_angle_wrapping.png.
"""

import math


def wrap_angle(angle):
    """Return the same direction as `angle`, but expressed between -pi and +pi.

    Example: -340 degrees and +20 degrees point the same way. Without wrapping,
    a robot told to turn -340 degrees spins almost a full circle the long way.

    How it works: sin and cos of an angle do not change if you add 360 degrees,
    and atan2 always answers with an angle between -pi and +pi.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def go_to_goal(x, y, theta, goal_x, goal_y,
               k_v=0.5,            # forward gain: how strongly distance turns into speed
               k_omega=3.0,        # turning gain: how strongly heading error turns into turn rate
               v_max=0.12,         # never ask for more forward speed than this (m/s)
               tolerance=0.02):    # "close enough" distance (m): stop when within 2 cm
    """Proportional (P) controller that steers a robot towards (goal_x, goal_y).

    Inputs  : the robot's pose (x, y, theta) as reported by its sensor, and the goal.
    Returns : (v, omega) - the speed and turn rate to request from the body.

    The idea of proportional control:
        the bigger the error, the bigger the correction.
    Far from the goal -> drive fast. Facing the wrong way -> turn hard.
    As the errors shrink, the corrections shrink, and the robot settles smoothly.
    """
    # STEP 1 - Vector from the robot to the goal.
    dx = goal_x - x
    dy = goal_y - y

    # STEP 2 - How far away is the goal?  d = sqrt(dx^2 + dy^2)
    # TODO: compute d (hint: math.hypot(dx, dy) does exactly this)
    d = math.hypot(dx, dy)

    # STEP 3 - Close enough? Then stop. Without this the robot would keep
    # making tiny corrections forever and jitter around the goal.
    if d < tolerance:
        return 0.0, 0.0

    # STEP 4 - Which direction is the goal?  phi = atan2(dy, dx)
    # atan2 (not atan!) handles all four quadrants correctly.
    # TODO: compute phi
    phi = math.atan2(dy, dx)

    # STEP 5 - Heading error: how far must the robot still turn?
    # Use wrap_angle() so the robot always turns the short way.
    # TODO: compute alpha
    alpha = wrap_angle(phi - theta)
    # alpha =  phi - theta
    # STEP 6 - Proportional control laws.
    # TODO: v = k_v * d, but never more than v_max   (hint: min(a, b))
    # TODO: omega = k_omega * alpha
    v = min(k_v * d, v_max)
    omega = k_omega * alpha
    v = v * max(0.0, math.cos(alpha))
    return v, omega
