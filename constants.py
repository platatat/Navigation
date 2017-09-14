# constants.py
import math

"""Constants used in nav code"""

# properties of the p struct in matlab
G = 9.81
L = 1.02
B = 0.3
H = 0.9
# trail is zero
C = 0 

# gains
K1 = 71.
K2 = 21.
K3 = -20.

# time
TIMESTEP = 1.0/100
PAUSE = TIMESTEP

# used in converting output of nav to steerD
MAX_STEER = math.pi/6.0

# Generic math
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

## Nav algo constants

# Normal controller
MAX_ACCEPTABLE_ANGLE_DIFF = math.pi / 4.0
PID_DIST_GAIN = 1.0
PID_ANGLE_GAIN = 1.5
PID_D_DIST_GAIN = 0.2
PID_D_ANG_GAIN = 0.3
PID_I_DIST_GAIN = 0
TURN_LOOKAHEAD_DIST = 7
NEXT_TURN_GAIN = 1

# Lookahead
BASE_LOOKAHEAD = 3
LOOKAHEAD_ANGLE_GAIN = 4
MIN_TURN_RADIUS = 7.0 # determined with the simulation by setting steering angle to MAX_STEER

## Visualization constants
ANIM_INTERVAL = 5 # timer control for the drawing/animation loop
