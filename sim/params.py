"""
RiverExtract sorting stage -- parameters.

Every geometric number below is taken from the printable CAD
(RiverExtract_Sorter.scad / assembled Fusion model) or from the corrected
gate (hub moved to the plate's top edge).  Every timing number of the
as-built controller is taken from riverextract_lite.py.  Units: mm, s, rad,
kg, N, unless a name says otherwise.
"""
import math

# ---------------------------------------------------------------- bridge ---
CLEAR_OPENING = 270.0            # mm, between the inner faces of the posts
POST_INNER = CLEAR_OPENING / 2   # 135 mm from the belt centreline
POST_X = 32.0                    # mm, post column depth along the belt
POST_TOTAL_H = 140.0             # mm, post height (foot + column)
BEAM = (135.0, 30.0, 6.0)        # mm, each beam half
CENTER_MOUNT = (90.0, 70.0, 6.0) # mm
NEMA17_BOLT_SPACING = 31.0       # mm (2 x 15.5)

# ------------------------------------------------------------------ gate ---
PLATE_L = 200.0                  # mm, along the belt at 0 deg
PLATE_H = 80.0                   # mm, vertical
PLATE_T = 5.0                    # mm
R_HALF = PLATE_L / 2             # 100 mm, half length (pivot at mid-length)
R_SWING = math.hypot(R_HALF, PLATE_T / 2)   # 100.03 mm, outer-corner radius
HUB_D, HUB_H, BORE_D = 18.0, 20.0, 5.4      # mm, hub on the top edge
FLARE_H = (HUB_D - PLATE_T) / 2             # 6.5 mm, 45-deg transition
CLEAT_H = 50.0                   # mm, belt cleat height (stated)
RUN_CLEAR = 5.0                  # mm, running clearance over cleat tips
GATE_LOW = CLEAT_H + RUN_CLEAR   # 55 mm, lower edge of the gate above belt
GATE_HIGH = GATE_LOW + PLATE_H   # 135 mm

RHO_PLA = 1240.0                 # kg/m^3 (fully dense PLA, upper bound)

# --------------------------------------------------------- route poses ---
TH_REST = math.radians(45.0)     # rest pose = OTHER route (rear-left)
TH_ORG = math.radians(-45.0)     # organic pose (rear-right)
BAND = R_HALF * math.sin(math.radians(45.0))   # 70.71 mm capture half-band

# --------------------------------------------------------------- motor ---
STEP = math.radians(1.8)         # full step (MS1..MS3 pulled low)
STEPS_90 = 50

# ------------------------------------------- proposed motion profiles ---
TAU_STRIKE = 0.100               # N m, inertial torque budget, strike
TAU_ORG = 0.050                  # N m, organic pre-positioning
TAU_RET = 0.025                  # N m, gentle return (accel limit)
OMEGA_RET = 3.0                  # rad/s, gentle-return speed cap
DWELL = 0.05                     # s, dwell at the end of a strike

# ----------------------------------------- as-built firmware (lite.py) ---
STEP_DELAY = 0.002               # s -> 4 ms period -> 250 step/s
STEP_DELAY_FAST = 0.001          # s -> 2 ms period -> 500 step/s
F_SLOW = 1.0 / (2 * STEP_DELAY)  # 250 step/s
F_FAST = 1.0 / (2 * STEP_DELAY_FAST)   # 500 step/s
HOLD_ASBUILT = 0.25              # s
PLASTIC_SWING_DELAY = 0.3        # s
T_PACE = 0.6                     # s, sleep per detection cycle
BIN_FULL_COUNT = 3
FULL_RESET_SECONDS = 5.0
CONF_THRESH = 0.55
YOLO_IMGSZ = 320
FRAME_W, FRAME_H = 480, 360

# ---------------------------------------------------------------- camera ---
F_CAM = 30.0                     # frame/s (C270, 720p/30 fps class)

G_MM = 9810.0                    # mm/s^2
