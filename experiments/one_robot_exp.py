# experiment 1
# Turning. In think(), set self.omega = 0.5.
# What shape will the path be? # my guess is a circle 
# What size? Hint: think about how far the robot travels in one full turn. # circumference = 2 * pi * r, where r is the radius of the circle.
# The robot travels at a speed of v, and it takes time T to complete one full turn, where T = 2 * pi / omega. 
# Therefore, the distance traveled in one full turn is v * T = v * (2 * pi / omega). Setting this equal to the circumference gives us 2 * pi * r = v * (2 * pi / omega), which simplifies to r = v / omega.
# Then derive a formula for the radius in terms of v and ω.
# r = v / omega






import math
import matplotlib.pyplot as plt


class Robot:
    def __init__(self, x, y, theta):           # Initialize the robot's state
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
        if self.x < WALL_MARGIN or self.x > ARENA - WALL_MARGIN or self.y < WALL_MARGIN or self.y > ARENA - WALL_MARGIN:
            self.omega = 1.0      # turn if close to wall
        else:
            self.omega = 0.0      # no turning: drive straight

    def act(self, dt):
        # Carry out the decision for one small time step.
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt


ARENA = 2.4   # arena side in metres (your 8 x 8 ft floor)
DT = 0.05  # time step in seconds (20 updates per second)
STEPS = 1200  # 1200x0.05 = 60 seconds of simulated time
WALL_MARGIN = 0.15
# robot = Robot(x=0.3, y=0.3, theta=math.radians(30))  # Initialize a robot at position (0.3, 0.3) with a heading of 30 degrees
robot = Robot(x=1.2, y=1.2, theta=0)  # Initialize a robot at position (1.2, 1.2) with a heading of 0 degrees
trail_x, trail_y = [], [] # Lists to store the robot's path for plotting

fig, ax = plt.subplots() # Create a new figure and axes for plotting
for step in range(STEPS):
    obs = robot.sense()       # SENSE
    robot.think(obs)          # THINK
    robot.act(DT)             # ACT

    trail_x.append(robot.x) # Append the current x position to the trail for plotting
    trail_y.append(robot.y) # Append the current y position to the trail for plotting

    # --- drawing only; this is not part of the robot ---
    ax.clear()
    ax.set_xlim(0, ARENA) # Set the x-axis limits of the plot to match the arena size
    ax.set_ylim(0, ARENA) # Set the y-axis limits of the plot to match the arena size   
    ax.set_aspect("equal") 
    ax.plot(trail_x, trail_y, linewidth=1)                             # path so far
    ax.add_patch(plt.Circle((robot.x, robot.y), 0.05, fill=False))     # 5 cm body
    hx = robot.x + 0.08 * math.cos(robot.theta) # Calculate the x-coordinate of the heading line's endpoint
    hy = robot.y + 0.08 * math.sin(robot.theta) # Calculate the y-coordinate of the heading line's endpoint
    ax.plot([robot.x, hx], [robot.y, hy])                             # heading line
    ax.set_title(f"t = {step * DT:.1f} s")
    plt.pause(0.001)

plt.show()



#experiment 2 
# Time step. Keep omega = 0.5. Try DT = 0.5 (with STEPS = 40), then DT = 0.01 (with STEPS = 2000).
# What happens to the circle in each case, and why? #my guess is that the circle will be more accurate with a smaller time step, as the robot's position will be updated more frequently, resulting in a smoother path. With a larger time step, the robot's position will be updated less frequently, resulting in a more jagged path that may not accurately represent a circle. This shows something important about how all simulations approximate reality.
# This shows something important about how all simulations approximate reality.
# the smaller the time step, the more accurate the simulation will be, but it will also take longer to run. The larger the time step, the less accurate the simulation will be, but it will run faster. This is a trade-off that must be considered when designing simulations.

# experiment 3
# Your first behavior. Right now the robot drives out of the arena. Change think() so that if the robot is within 15 cm of any wall, it turns (some nonzero omega); otherwise it drives straight.
# Run it for 60 seconds.
# Does it stay inside? What happens in corners?
# Congratulations: that’s a reactive behavior, the simplest kind of robot intelligence, and the starting point for exploration in Phase 2.



# One last challenge to close Phase 0

# Make your robot escape. Start it heading straight at a wall, for example Robot(x=1.2, y=1.2, theta=0), which heads right.
# Predict first: using r = v / ω, how far past the turning line will it go, and will it leave the arena? my guess is that it will go 0.2 m past the turning line, and it will leave the arena.
# Run it and check.
# Fix it without changing WALL_MARGIN. You have two levers: v and ω. Which one would you change, and what are the trade-offs? (Think about what a real robot can and can’t do.)
#my guess is that I would change v, as a slower speed would allow the robot to turn more quickly and avoid hitting the wall. The trade-off is that a slower speed would also mean that the robot would take longer to reach its destination.
# i also changed omega to 1.0, as a faster turning rate would allow the robot to turn more quickly and avoid hitting the wall. The trade-off is that a faster turning rate would also mean that the robot would take longer to reach its destination.