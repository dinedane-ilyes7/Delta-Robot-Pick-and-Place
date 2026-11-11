"""Every dimension of the robot in one place (mm). Geometry comes from config/robot.yaml;
the rest is the v2 mechanical layout. Arm-local frame: origin on the arm axis at radius R,
x radial outward, y along the axis (right-handed with z up), z up. Theta positive = arm down."""
import math
import os

import yaml

from . import gt2

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = yaml.safe_load(open(os.path.join(ROOT, "config", "robot.yaml")))
G = CFG["geometry"]
R, r, L1, L2, W = (1000 * G[k] for k in ("R", "r", "L1", "L2", "w"))
PHI = G["phi_deg"]
TH_MIN, TH_MAX = G["theta_min_deg"], G["theta_max_deg"]
TCP = 1000 * G["tcp_offset"]
CELL = CFG["cell"]

# ---------------------------------------------------------------- drive: GT2 20T -> 120T, 10 mm belt, 150 teeth
N_SMALL, N_BIG, N_BELT = 20, 120, 150
C_BELT = gt2.solve_centre_distance(N_BIG, N_SMALL, N_BELT)    # 72.94 mm, motor axis above the arm axis
Z_MOTOR = C_BELT
BELT_W = 10.0
TEETH_Y = (34.0, 45.0)                 # tooth zone of both pulleys
BELT_Y = (34.5, 44.5)

# ---------------------------------------------------------------- axial stack along the arm axis (y)
HUB_Y = 8.0                            # hub y -8..8
BRG_Y = 20.0                           # 608 bearings centred at y = +-20, 7 wide -> 16.5..23.5
BRG_W = 7.0
CHEEK = (15.0, 25.0)                   # cheeks |y| 15..25
SHOULDER_D = 18.0                      # bearing shoulder bore in the cheek, |y| 23.5..25
SPACER_IN = (HUB_Y, BRG_Y - BRG_W / 2)          # 8..16.5
SPACER_POS = (BRG_Y + BRG_W / 2, 26.5)          # +y outer spacer 23.5..26.5
SPACER_NEG = (-26.0, -(BRG_Y + BRG_W / 2))      # -y outer spacer -26..-23.5
COLLAR_Y = (-34.0, -26.0)
SHAFT_Y = (-38.0, 46.0)                # 84 mm, 8 h6
SHAFT_FLATS = ((-9.0, 9.0), (26.5, 46.0))
PULLEY_HUB_Y = (26.5, 46.0)
MAG_HOLDER_Y = (-43.5, -34.0)          # holder sleeve on the shaft end -38..-34
MAGNET_Y = (-43.5, -41.0)              # 3 mm clear of the steel shaft end
CHIP_FACE_Y = -45.0                    # 1.5 mm air gap
POST_XZ = ((-9.5, -9.5), (9.5, -9.5))  # AS5600 board posts
BOARD_Y = -46.5                        # board front face (posts end here)

# motor side
MPULLEY_Y0 = TEETH_Y[0]                # 20T teeth start; hub ends at 53
MPLATE_Y = (53.5, 57.5)
MOTOR_FACE_Y = 57.5
MOTOR_SLOT = 2.5                       # +-2.5 mm along z for belt tension

# bracket
Z_PLATE = 100.0                        # base plate underside
FLANGE_Z = (92.0, 100.0)
FLANGE_X = (-34.0, 34.0)
FLANGE_Y = (-25.0, 57.5)
CHEEK_RC = 24.0                        # cheek outline radius around the axis
CHEEK_TOP_X = 30.0
BACKWALL_X = (-30.0, -26.0)
GUSSET_X = 30.0                        # gussets at |x| 30..34
FLANGE_HOLES = ((-27.0, -20.0), (27.0, -20.0), (-27.0, 50.0), (27.0, 50.0))

# hub, stops
TAIL_L, TAIL_H = 22.0, 6.0             # tail to x=-22, half thickness 6
STOP_R = 15.0                          # contact point distance on the tail
STOP_PIN_D = 6.0

# arm tubes and sockets
TUBE_UP = (12.0, 10.0)
TUBE_UP_X = (15.0, 150.0)              # 135 long
HUB_SOCKET_X = (15.0, 40.0)            # bore bottom, mouth
ELBOW_SOCKET_X = (123.0, 150.0)        # mouth, bottom
ELBOW_BLOCK = dict(x=(150.0, 176.0), y=11.0, z=6.0)   # y = stud face; the block body is +-LUG_CORE
TUBE_LO = (6.0, 4.0)
ROD_SOCKET = (9.0, 23.0)               # rod-end socket, z' from bore bottom to mouth
ROD_TUBE_Z = (ROD_SOCKET[0], L2 - ROD_SOCKET[0])

# ball joints
BALL_D = 8.0
STUD_FACE = 9.0                        # hex bearing face 9 mm from the ball centre
CUP_RIM = 1.5                          # cup rim plane 1.5 mm past the ball centre
SPRING_POST = (-8.0, 15.0)             # (x', z') of the spring pin: on the side away from the upper arm
SPRING_PIN_D = 3.0

# platform
PLAT_LUG = dict(u=(23.0, 37.0), t=11.0, z=5.5)
LUG_CORE = 8.0                         # lug / elbow block body |t| <= 8; a dia 10 spot-face boss carries the stud to 11
PLAT_HUB = dict(d=20.0, z=(-5.5, 4.0))
BASE_PLATE = dict(size=340.0, t=6.0, corner=15.0, centre_hole=40.0)
FRAME_RAIL_Y = 150.0                   # 2020 rails carrying the plate, along x at y = +-150
