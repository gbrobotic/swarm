"""
experiments/phase1_diffdrive.py - Phase 1, Part 1A experiments.

One differential-drive robot drives straight and turns when it gets close
to a wall. Used for experiments 1-4 (wheel puzzles, scaling, escape test,
scale vs clamp).

HOW TO RUN (from the project folder, NOT from inside experiments/):
    source .venv/bin/activate
    python -m experiments.phase1_diffdrive

"-m" makes Python start from the project folder, so "from sim.robot import ..."
can find the sim/ folder. Running "python experiments/phase1_diffdrive.py"
instead gives ModuleNotFoundError.
"""

import math
import matplotlib.pyplot as plt

from sim.robot import DiffDriveRobot   # the robot body lives in ONE place: sim/robot.py


# ======================================================================
# SETTINGS - change these to run each experiment. Nothing else needs editing.
# ======================================================================
ARENA = 2.4          # arena side (m) - the 8 x 8 ft floor
WALL_MARGIN = 0.15   # start turning when the robot centre is this close to a wall (m)
BODY_RADIUS = 0.05   # robot body radius (m), used to check if the body hits a wall

DT = 0.05            # time step (s): 20 updates per simulated second
STEPS = 1200         # 1200 x 0.05 s = 60 seconds of simulated time
DRAW_EVERY = 5       # redraw the plot every 5 steps (drawing is slow; physics is not)

FORWARD_SPEED = 0.10     # the brain's requested forward speed v (m/s). Experiment 4: try 0.15
TURN_RATE = 2.0          # the brain's requested turn rate near walls (rad/s)
LIMIT_MODE = "scale"     # "scale" (correct) or "clamp" (experiment 4)

# Experiment 1 (wheel puzzles): put two wheel speeds here, e.g. (0.05, 0.10),
# to bypass the brain and drive the wheels directly. Use None to use the brain.
MANUAL_WHEELS = None     # (v_left, v_right) in m/s, or None


# ======================================================================
# CREATE THE ROBOT
# ======================================================================
robot = DiffDriveRobot(x=1.2, y=1.2, theta=0.0,      # arena centre, facing right (+x)
                       limit_mode=LIMIT_MODE)

trail_x, trail_y = [], []    # every position the robot has visited, for drawing its path
printed_turn = False         # have we already printed the wheel speeds during a turn?
escaped = False              # did the robot's body ever cross a wall?


# ======================================================================
# MAIN LOOP - "the world". It advances time; the brain decides; the body moves.
# ======================================================================
fig, ax = plt.subplots()

for step in range(STEPS):

    # ---------------- BRAIN: decide what to do ----------------
    if MANUAL_WHEELS is not None:
        # Experiment 1: no brain, set the wheels directly.
        robot.v_left, robot.v_right = MANUAL_WHEELS
        near_wall = False
    else:
        # Is the robot centre inside the "danger zone" near any of the 4 walls?
        near_wall = (robot.x < WALL_MARGIN or robot.x > ARENA - WALL_MARGIN
                     or robot.y < WALL_MARGIN or robot.y > ARENA - WALL_MARGIN)
        if near_wall:
            robot.set_velocity(FORWARD_SPEED, TURN_RATE)   # turn away
        else:
            robot.set_velocity(FORWARD_SPEED, 0.0)         # drive straight

    # Experiment 2: print the wheel speeds ONCE, the first time the robot turns.
    # (Printing every step would flood the terminal with 1200 lines.)
    if near_wall and not printed_turn:
        v, omega = robot.body_velocity()                   # what the robot REALLY does
        print(f"Asked for : v = {FORWARD_SPEED:.4f} m/s, omega = {TURN_RATE:.3f} rad/s, "
              f"r = {FORWARD_SPEED / TURN_RATE:.4f} m")
        print(f"Wheels    : v_left = {robot.v_left:.4f}, v_right = {robot.v_right:.4f} m/s")
        print(f"Got       : v = {v:.4f} m/s, omega = {omega:.3f} rad/s, r = {v / omega:.4f} m")
        printed_turn = True

    # ---------------- BODY: move for one time step ----------------
    robot.step(DT)
    trail_x.append(robot.x)
    trail_y.append(robot.y)

    # ---------------- CHECK: did the body cross a wall? ----------------
    # The body is a circle, so its edge is BODY_RADIUS away from the centre.
    if (robot.x - BODY_RADIUS < 0 or robot.x + BODY_RADIUS > ARENA
            or robot.y - BODY_RADIUS < 0 or robot.y + BODY_RADIUS > ARENA):
        escaped = True

    # ---------------- DRAWING ONLY (not part of the robot) ----------------
    if step % DRAW_EVERY == 0:          # % = remainder: true on steps 0, 5, 10, ...
        ax.clear()
        ax.set_xlim(-0.1, ARENA + 0.1)  # a little extra space so escapes are visible
        ax.set_ylim(-0.1, ARENA + 0.1)
        ax.set_aspect("equal")          # 1 m on x looks the same length as 1 m on y

        # Arena walls (solid) and the turning line (dashed).
        ax.add_patch(plt.Rectangle((0, 0), ARENA, ARENA, fill=False, linewidth=2))
        ax.add_patch(plt.Rectangle((WALL_MARGIN, WALL_MARGIN),
                                   ARENA - 2 * WALL_MARGIN, ARENA - 2 * WALL_MARGIN,
                                   fill=False, linestyle="--", linewidth=0.8))

        ax.plot(trail_x, trail_y, linewidth=1)                                  # path so far
        ax.add_patch(plt.Circle((robot.x, robot.y), BODY_RADIUS, fill=False))   # body
        hx = robot.x + 0.08 * math.cos(robot.theta)                             # heading line end
        hy = robot.y + 0.08 * math.sin(robot.theta)
        ax.plot([robot.x, hx], [robot.y, hy])

        status = "ESCAPED" if escaped else "inside"
        ax.set_title(f"t = {step * DT:.1f} s   mode = {LIMIT_MODE}   v = {FORWARD_SPEED}   [{status}]")
        plt.pause(0.001)                # show the frame

# ======================================================================
# RESULT
# ======================================================================
print("Result:", "ESCAPED - the body crossed a wall" if escaped else "stayed inside the arena")
plt.show()                              # keep the window open at the end


# ======================================================================
# EXPERIMENT NOTES
# ======================================================================
# Format: Setup / Predicted / Observed / Why  (see phase1_goto.py)
#
# Exp 1 - wheel puzzles
#   Setup     : MANUAL_WHEELS = (v_left, v_right)
#   Predicted : (0.10, 0.10)  -> straight line
#               (-0.05, 0.05) -> spins in place, r = 0, omega = 1.25 rad/s
#               (0.05, 0.10)  -> v = 0.75, omega = 0.625, r = 1.2 m   (decimal slip)
#   Observed  : straight line; spin in place; circle of radius 0.12 m (24 cm across)
#   Why       : v = (0.10 + 0.05) / 2 = 0.075 m/s, omega = 0.05 / 0.08 = 0.625 rad/s,
#               r = 0.075 / 0.625 = 0.12 m. Sanity check: one wheel twice as fast
#               as the other is a SHARP turn, so 1.2 m could not be right.
#
# Exp 2 - motor limit scaling
#   Setup     : default settings (v = 0.10, omega = 2.0, mode "scale")
#   Predicted : v = 0.0833, omega = 2.0, r = 0.0417 m
#   Observed  : wheels 0.0167 / 0.1500 m/s -> v = 0.0833 m/s, omega = 1.667 rad/s,
#               r = 0.0500 m (exactly the radius that was asked for)
#   Why       : scaling both wheels by 0.833 scales v AND omega by 0.833,
#               so r = v / omega does not change. Same path, just slower.
#
# Exp 3 - escape test with motor limits
#   Setup     : default settings
#   Predicted : escapes, because the turning radius is too small
#   Observed  : stayed inside the arena
#   Why       : space available = 2.35 - 2.25 = 0.10 m, radius needed = 0.05 m.
#               A SMALL turning radius needs LESS space -> it is good near a wall.
#
# Exp 4 - scale vs clamp
#   Setup     : FORWARD_SPEED = 0.15, LIMIT_MODE "scale" then "clamp"
#   Predicted : clamp lets it turn away more easily
#   Observed  : scale -> r = 0.075 m, stayed inside
#               clamp -> omega dropped from 2.0 to 1.0, r = 0.110 m, ESCAPED
#               (at v = 0.10 the two methods were almost the same: r 0.050 vs 0.052)
#   Why       : clamping one wheel shrinks the DIFFERENCE between the wheels,
#               so the robot turns less and the radius grows. 0.110 m > 0.10 m space.
#               It only matters near top speed, where there is no headroom to turn.