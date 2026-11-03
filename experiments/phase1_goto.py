"""
experiments/phase1_goto.py - Phase 1, Part 1B: click anywhere, the robot drives there.

HOW TO RUN (from the project folder):
    python -m experiments.phase1_goto

LEFT-CLICK inside the arena to set a new goal. Close the window to stop.
"""

import math
import matplotlib.pyplot as plt

from sim.robot import DiffDriveRobot      # the BODY
from sim.controllers import go_to_goal    # the BRAIN


# ======================================================================
# SETTINGS - change these for the experiments
# ======================================================================
ARENA = 2.4          # arena side (m)
BODY_RADIUS = 0.05   # robot body radius (m), for drawing only
DT = 0.05            # time step (s) - 20 brain decisions per second
DRAW_EVERY = 3       # redraw every 3 steps

K_V = 0.5            # forward gain   (experiment 2 keeps this fixed)
K_OMEGA = 1.0      # turning gain   (experiment 2: try 0.5, 3.0, 50.0)


# ======================================================================
# CREATE THE ROBOT AND THE SHARED STATE
# ======================================================================
robot = DiffDriveRobot(x=1.2, y=1.2, theta=0.0)

# The click function below must be able to CHANGE the goal. A dictionary is
# a simple container both the main loop and the click function can see.
state = {"goal": None,          # (x, y) of the current goal, or None = no goal yet
         "start_time": 0.0,     # simulated time when this goal was set
         "reached": True}       # has the current goal been reached?

trail_x, trail_y = [robot.x], [robot.y]
sim_time = 0.0                  # simulated seconds since the start


def on_click(event):
    """Matplotlib calls this function by itself whenever the mouse is clicked.

    A function that a library calls for you when something happens is
    called a CALLBACK. `event` carries the details of the click.
    """
    if event.inaxes is not ax or event.button != 1:   # ignore clicks outside the plot / not left button
        return
    # Keep the goal inside the arena (a click on the border could land outside).
    gx = min(max(event.xdata, BODY_RADIUS), ARENA - BODY_RADIUS)
    gy = min(max(event.ydata, BODY_RADIUS), ARENA - BODY_RADIUS)
    state["goal"] = (gx, gy)
    state["start_time"] = sim_time
    state["reached"] = False
    trail_x.clear(); trail_y.clear()                   # start a fresh path for this goal
    print(f"New goal: ({gx:.2f}, {gy:.2f})")


fig, ax = plt.subplots()
fig.canvas.mpl_connect("button_press_event", on_click)   # "when clicked, call on_click"


# ======================================================================
# MAIN LOOP - runs until you close the window
# ======================================================================
step = 0
while plt.fignum_exists(fig.number):        # True while the window is open

    # ---------------- SENSE: what does the robot know? ----------------
    # Phase 1B: a perfect sensor - the robot knows its exact pose.
    # Phase 1C will add noise right here.
    x_meas, y_meas, theta_meas = robot.x, robot.y, robot.theta

    # ---------------- THINK: the brain decides ----------------
    if state["goal"] is None:
        v, omega = 0.0, 0.0                 # no goal yet: stay still
    else:
        gx, gy = state["goal"]
        v, omega = go_to_goal(x_meas, y_meas, theta_meas, gx, gy,
                              k_v=K_V, k_omega=K_OMEGA)
        # The controller returns (0, 0) when it is close enough: goal reached.
        if v == 0.0 and omega == 0.0 and not state["reached"]:
            state["reached"] = True
            print(f"Reached goal in {sim_time - state['start_time']:.1f} s")

    # ---------------- ACT: the body moves ----------------
    robot.set_velocity(v, omega)
    robot.step(DT)
    sim_time += DT
    trail_x.append(robot.x)
    trail_y.append(robot.y)

    # ---------------- DRAWING ONLY ----------------
    if step % DRAW_EVERY == 0:
        ax.clear()
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect("equal")
        ax.add_patch(plt.Rectangle((0, 0), ARENA, ARENA, fill=False, linewidth=2))
        ax.plot(trail_x, trail_y, linewidth=1)
        if state["goal"] is not None:
            ax.plot(*state["goal"], marker="*", markersize=15, color="orange")  # * unpacks (x, y)
        ax.add_patch(plt.Circle((robot.x, robot.y), BODY_RADIUS, fill=False))
        ax.plot([robot.x, robot.x + 0.08 * math.cos(robot.theta)],
                [robot.y, robot.y + 0.08 * math.sin(robot.theta)])
        ax.set_title(f"t = {sim_time:.1f} s   K_omega = {K_OMEGA}   (left-click to set a goal)")
        plt.pause(0.001)
    step += 1


# ======================================================================
# EXPERIMENT NOTES
# ======================================================================
# Exp 1 - path shape: Click a goal to the upper-left of the robot. Is the path a straight line or a curve? Why? (Hint: what is the robot doing with v and ω at the same time?)  curve
# Exp 2 - K_OMEGA = 0.5 / 3.0 / 50.0, prediction and result: Change K_OMEGA at the top of phase1_goto.py and try 0.5, then 3.0, then 50.0, always clicking the same goal. Predict each one first. For 50, watch the heading line very closely. What would that do to a real robot’s motors? # my prediction is lower K_OMEGA will result in a slower turning rate and a more gradual curve, while higher K_OMEGA will result in a faster turning rate and a sharper curve. 
# Exp 3 - goal behind the robot: Set K_OMEGA = 1.0, wait until the robot stops, then click directly behind it. What does the path look like? Then fix it by adding one line in go_to_goal,  # my guess path is curve, fix: ...
# Exp 4 - wrapping removed:  the robot will turn the long way around to reach the goal, instead of the short way.  and rotating back and forth, oscillating around the goal. 