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
K_OMEGA = 3.0        # turning gain   (experiment 2: tried 0.5, 3.0, 50.0 -> 3.0 is best)


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
# Format for every experiment:
#   Setup     : what I changed and what I clicked (so I can repeat it)
#   Predicted : what I expected BEFORE running (never edit this afterwards)
#   Observed  : only what I actually saw / measured
#   Why       : my explanation of the observation
# Start: robot at (1.2, 1.2) facing right. Same goal for Exp 1 and 2: (0.6, 1.9).
#
# Exp 1 - path shape
#   Setup     : K_OMEGA = 3.0, before the cos line was added, goal upper-left (0.6, 1.9)
#   Predicted : a curve
#   Observed  : smooth curve; 0.95 m driven vs 0.92 m straight line; 11.4 s
#   Why       : the robot drives (v) and turns (omega) at the same time.
#               Any motion with both v and omega is a curve, not a straight line.
#
# Exp 2 - turning gain K_OMEGA (with the cos line), same goal (0.6, 1.9)
#   Predicted : low gain = gentle, wide curve; high gain = sharp turn;
#               at 50 it would overshoot the goal and oscillate around it
#   Observed  : 0.5  -> wide curve, 0.98 m, 13.4 s, slowest
#               3.0  -> nearly straight, 0.91 m, 11.2 s, fastest
#               50.0 -> reached the goal (0.90 m, 13.8 s) but the heading line
#                       twitched left-right the whole way: turn direction flipped
#                       262 times in 275 steps; total rotation 1657 deg instead of ~133
#   Why       : the gain sets how hard the robot corrects. Too low = lazy,
#               wide path. Too high = each 0.05 s step turns PAST the goal
#               direction, the error flips sign, it turns back -> "chattering".
#               It is the HEADING that oscillates, not the position around the goal.
#               On real motors: ~20 direction reversals per second = current
#               spikes, heat, gear wear, wasted battery, buzzing.
#               Lesson: there is a best gain; higher is not always better.
#
# Exp 3 - goal directly behind the robot
#   Setup     : K_OMEGA = 1.0, goal behind the robot at (0.4, 1.2)
#   Predicted : a curve
#   Observed  : without the cos line -> swings out wide, 0.99 m for a 0.80 m distance
#               with the cos line    -> turns first, then drives, 0.81 m
#   Why       : v only depended on distance, not on direction, so the robot drove
#               forward (away from the goal) while turning. max(0, cos(alpha))
#               gives 0 speed when the goal is beside or behind -> turn first, then go.
#
# Exp 4 - angle wrapping removed (alpha = phi - theta)
#   Setup     : first goal (0.4, 1.2) so the robot ends facing left (183 deg),
#               then goal (0.2, 1.0) below-left (direction -135 deg)
#   Predicted : turns the long way; maybe oscillates around the goal
#   Observed  : without wrapping -> spun 328 deg clockwise (long way), 6.9 s
#               with wrapping    -> turned 46 deg anticlockwise (short way), 5.5 s
#               no oscillation in either case
#   Why       : -135 - 183 = -318 deg. Same direction as +42 deg, but the
#               controller does not know that, so it turns -318 deg.
#               wrap_angle() converts -318 deg to +42 deg -> short way.