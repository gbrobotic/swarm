"""
sim/sensors.py - simulated sensors.

A real sensor never reports the exact truth. Our robots will get their pose
from an overhead camera, and every camera reading is a little bit wrong,
in a slightly different way each time. This file simulates that.

Rule of the project from now on: the BRAIN only ever sees sensor readings,
never robot.x / robot.y / robot.theta directly. Only the world (the main loop)
and the drawing code are allowed to look at the true pose.

See 5_gaussian_noise.png.
"""

import math
import random


class NoisyPoseSensor:
    """Simulates an overhead camera that measures (x, y, theta) with random error.

    The error of each reading is drawn from a Gaussian (normal) distribution:
      - mean 0          -> the sensor is not biased: errors are equally often + and -
      - standard deviation sigma -> how big the errors typically are.
        About 68% of readings are within 1 sigma of the truth,
        about 95% within 2 sigma.
    """

    def __init__(self, sigma_pos=0.01, sigma_theta_deg=2.0, seed=None):
        self.sigma_pos = sigma_pos                          # position noise (m). 0.01 = 1 cm
        self.sigma_theta = math.radians(sigma_theta_deg)    # heading noise, stored in radians
        # A private random-number generator. Giving it a seed makes the
        # "random" numbers repeat exactly every run -> reproducible experiments.
        # seed=None means different random numbers every run.
        self.rng = random.Random(seed)

    def read(self, robot):
        """Return one noisy measurement (x_meas, y_meas, theta_meas) of the robot's pose.

        self.rng.gauss(0.0, s) returns a random number from a Gaussian with
        mean 0 and standard deviation s. Add one to each true value.
        """
        # TODO: x_meas = the true x plus Gaussian noise with standard deviation self.sigma_pos
        # TODO: y_meas = the same for y (draw a NEW random number, do not reuse x's)
        # TODO: theta_meas = the true theta plus Gaussian noise with standard deviation self.sigma_theta
        x_meas = robot.x + self.rng.gauss(0.0, self.sigma_pos)
        y_meas = robot.y + self.rng.gauss(0.0, self.sigma_pos)
        theta_meas = robot.theta + self.rng.gauss(0.0, self.sigma_theta)
        return x_meas, y_meas, theta_meas
