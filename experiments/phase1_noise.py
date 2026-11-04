"""
experiments/phase1_noise.py - Phase 1, Part 1C: go-to-goal with a NOISY sensor.

Same as phase1_goto.py, but the brain no longer sees the true pose. It sees
readings from a simulated overhead camera that is wrong by a few cm each time.

HOW TO RUN (from the project folder):
    python -m experiments.phase1_noise

LEFT-CLICK inside the arena to set a goal. Close the window to stop.
On screen:  black circle = where the robot REALLY is
            red dots     = where the sensor SAYS it is (last 30 readings)
"""

import math
import matplotlib.pyplot as plt

from sim.robot import DiffDriveRobot      # BODY
from sim.controllers import go_to_goal    # BRAIN
from sim.sensors import NoisyPoseSensor   # SENSOR (new)


# ======================================================================
# SETTINGS
# ======================================================================
ARENA = 2.4
BODY_RADIUS = 0.05
DT = 0.05
DRAW_EVERY = 3

K_V = 0.5
K_OMEGA = 3.0

SIGMA_POS = 0.01     # position noise, standard deviation (m). Experiments: 0.0, 0.01, 0.03, 0.05
SIGMA_THETA_DEG = 2.0     # heading noise, standard deviation (degrees)
SEED = None                # fixed seed = same "random" noise every run. None = different each run.
SHOW_READINGS = 30        # how many recent sensor readings to draw as red dots


# ======================================================================
# CREATE THE ROBOT, THE SENSOR AND THE SHARED STATE
# ======================================================================
robot = DiffDriveRobot(x=1.2, y=1.2, theta=0.0)
sensor = NoisyPoseSensor(sigma_pos=SIGMA_POS, sigma_theta_deg=SIGMA_THETA_DEG, seed=SEED)

state = {"goal": None,
         "start_time": 0.0,
         "reached": True,
         "restarts": 0,          # how many times the robot stopped at the goal and then started moving again
         "was_stopped": True}    # was the robot stopped in the previous step?

trail_x, trail_y = [robot.x], [robot.y]
meas_x, meas_y = [], []         # recent sensor readings, for drawing
sim_time = 0.0


def on_click(event):
    """Callback: set a new goal where the user left-clicks."""
    if event.inaxes is not ax or event.button != 1:
        return
    gx = min(max(event.xdata, BODY_RADIUS), ARENA - BODY_RADIUS)
    gy = min(max(event.ydata, BODY_RADIUS), ARENA - BODY_RADIUS)
    state.update(goal=(gx, gy), start_time=sim_time, reached=False, restarts=0)
    trail_x.clear(); trail_y.clear()
    print(f"New goal: ({gx:.2f}, {gy:.2f})")


fig, ax = plt.subplots()
fig.canvas.mpl_connect("button_press_event", on_click)


# ======================================================================
# MAIN LOOP
# ======================================================================
step = 0
while plt.fignum_exists(fig.number):

    # ---------------- SENSE: the brain only gets a noisy reading ----------------
    x_meas, y_meas, theta_meas = sensor.read(robot)
    meas_x.append(x_meas); meas_y.append(y_meas)
    del meas_x[:-SHOW_READINGS], meas_y[:-SHOW_READINGS]   # keep only the last SHOW_READINGS

    # ---------------- THINK ----------------
    if state["goal"] is None:
        v, omega = 0.0, 0.0
    else:
        gx, gy = state["goal"]
        v, omega = go_to_goal(x_meas, y_meas, theta_meas, gx, gy, k_v=K_V, k_omega=K_OMEGA)

        stopped = (v == 0.0 and omega == 0.0)
        # The robot THINKS it has arrived. How far away is it REALLY?
        if stopped and not state["reached"]:
            state["reached"] = True
            true_error = math.hypot(gx - robot.x, gy - robot.y)
            print(f"Robot thinks it reached the goal after {sim_time - state['start_time']:.1f} s"
                  f" - true distance to goal: {true_error * 100:.1f} cm")
        # It was stopped, and now a new noisy reading makes it move again: a restart.
        if state["reached"] and state["was_stopped"] and not stopped:
            state["restarts"] += 1
        state["was_stopped"] = stopped

    # ---------------- ACT ----------------
    robot.set_velocity(v, omega)
    robot.step(DT)
    sim_time += DT
    trail_x.append(robot.x); trail_y.append(robot.y)

    # ---------------- DRAWING ONLY (allowed to see the truth) ----------------
    if step % DRAW_EVERY == 0:
        ax.clear()
        ax.set_xlim(0, ARENA); ax.set_ylim(0, ARENA); ax.set_aspect("equal")
        ax.add_patch(plt.Rectangle((0, 0), ARENA, ARENA, fill=False, linewidth=2))
        ax.plot(trail_x, trail_y, linewidth=1)
        ax.plot(meas_x, meas_y, ".", color="red", markersize=3)              # what the sensor says
        if state["goal"] is not None:
            ax.plot(*state["goal"], marker="*", markersize=15, color="orange")
            err_cm = math.hypot(state["goal"][0] - robot.x, state["goal"][1] - robot.y) * 100
            info = f"true error {err_cm:.1f} cm   restarts {state['restarts']}"
        else:
            info = "left-click to set a goal"
        ax.add_patch(plt.Circle((robot.x, robot.y), BODY_RADIUS, fill=False))  # where it really is
        ax.plot([robot.x, robot.x + 0.08 * math.cos(robot.theta)],
                [robot.y, robot.y + 0.08 * math.sin(robot.theta)])
        ax.set_title(f"t = {sim_time:.1f} s   noise {SIGMA_POS * 100:.0f} cm / {SIGMA_THETA_DEG:.0f} deg\n{info}",
                     fontsize=10)
        plt.pause(0.001)
    step += 1


# ======================================================================
# EXPERIMENT NOTES  (Setup / Predicted / Observed / Why - write them yourself)
# ======================================================================
# Exp 1 - small noise
#   Setup     : SIGMA_POS = 0.01, SIGMA_THETA_DEG = 2.0
#   Predicted : reach the goal its starts turning leftand right 
#   Observed  : red dots are scattered around the true position, but the robot still reaches the goal. 
#   Why       : for small no
#
# Exp 2 - bigger noise
#   Setup     : sigma_pos= 0.03, sigma_pos = 0.05
#   Predicted : 4cm , 6cm 
#   Observed  : for 0.01 true distance is 2.7 cm, for 0.3 true distance is 3.6 cm , for 0.05 true distance is 5.4 cm 
#   Why       : near the goal, the robot is more likely to think it has reached the goal when it hasn't, and then it will start moving again. The bigger the noise, the more likely this is to happen. The robot will keep oscillating around the goal, never really reaching it.
#
# Exp 3 - seeds
#   Setup     :SEED = 1, SEED = 2, SEED = None.
#   Predicted : seed 1 and 2 will give different results but their pattern same for respectively , None will give different results each time.
#   Observed  : trajectories are different for different seeds, but the overall pattern is the same. For None, the trajectory is different each time.
#   Why       : fixed seeds give the same "random" noise each time, so the robot will behave the same way each time. Different seeds give different "random" noise, so the robot will behave differently each time. None gives different "random" noise each time, so the robot will behave differently each time. this helps im research.
