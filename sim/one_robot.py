import math
import matplotlib.pyplot as plt


class Robot:
    def __init__(self, x, y, theta):
        self.x = x            # position in metres
        self.y = y
        self.theta = theta    # heading in radians (0 = facing +x)
        self.v = 0.0          # forward speed in m/s
        self.omega = 0.0      # turning rate in rad/s

    def sense(self):
        # No sensors yet. Later: distances, neighbours' messages, light levels.
        return None

    def think(self, observation):
        # The robot's "brain": decide speed and turning rate.
        self.v = 0.10         # 10 cm per second
        self.omega = 0.0      # no turning: drive straight

    def act(self, dt):
        # Carry out the decision for one small time step.
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt


ARENA = 2.4   # arena side in metres (your 8 x 8 ft floor)
DT = 0.05     # time step in seconds (20 updates per second)
STEPS = 400   # 400 x 0.05 s = 20 seconds of simulated time

robot = Robot(x=0.3, y=0.3, theta=math.radians(30))
trail_x, trail_y = [], []

fig, ax = plt.subplots()
for step in range(STEPS):
    obs = robot.sense()       # SENSE
    robot.think(obs)          # THINK
    robot.act(DT)             # ACT

    trail_x.append(robot.x)
    trail_y.append(robot.y)

    # --- drawing only; this is not part of the robot ---
    ax.clear()
    ax.set_xlim(0, ARENA)
    ax.set_ylim(0, ARENA)
    ax.set_aspect("equal")
    ax.plot(trail_x, trail_y, linewidth=1)                             # path so far
    ax.add_patch(plt.Circle((robot.x, robot.y), 0.05, fill=False))     # 5 cm body
    hx = robot.x + 0.08 * math.cos(robot.theta)
    hy = robot.y + 0.08 * math.sin(robot.theta)
    ax.plot([robot.x, hx], [robot.y, hy])                              # heading line
    ax.set_title(f"t = {step * DT:.1f} s")
    plt.pause(0.001)

plt.show()