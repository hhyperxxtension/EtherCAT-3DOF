import pysoem

ECAT_ADAPTER_NAME = "Realtek PCIe GbE Family Controller"

EC_CYCLE_TIME = 0.004  # 4 ms cycle

# CiA 402 object dictionary
CONTROLWORD = 0x6040
STATUSWORD = 0x6041
MODE_OP = 0x6060
MODE_OP_DISPLAY = 0x6061
TARGET_VELOCITY = 0x60FF
VELOCITY_ACTUAL = 0x606C
TORQUE_ACTUAL = 0x6077
MAX_TORQUE = 0x6072            # CiA 402 max torque — NOT enforced by Y7 in CSV/CSP
# The Y7 mode block diagrams (CSV/CSP/PP/PV) use 0x60E0/0x60E1 as the active
# torque-limit reference, NOT 0x6072. Both are ‰ of rated torque, default
# 8000 (=800%) — which is why writing only 0x6072 left torque uncapped.
POS_TORQUE_LIMIT = 0x60E0      # forward max torque limit (‰), the effective cap
NEG_TORQUE_LIMIT = 0x60E1      # reverse max torque limit (‰), the effective cap

# Safety torque limit applied per drive at bring-up (‰ of rated torque, same
# units as 0x6077). NOTE observed running torque under csv_multi was ~200-260
# (20-26%) just to move the unloaded arm, so a cap below that will stall the
# motor / raise a torque-limit fault (A.9B5 / Err4). Keep it ABOVE operating
# torque — e.g. 500 (50%) gives headroom while still catching a jam/runaway.
MAX_TORQUE_PERMILLE = 350
PROFILE_ACCEL = 0x6083
PROFILE_DECEL = 0x6084
MAX_MOTOR_SPEED = 0x6080

# Brake objects
BRAKE_OFF_DELAY = 0x2506
BRAKE_SPEED_LEVEL = 0x2507
BRAKE_ON_DELAY = 0x2508

# Controlword commands
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OP = 0x000F
CW_DISABLE_VOLTAGE = 0x0000
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080

# Statusword masks (CiA 402 state machine)
SW_READY_TO_SWITCH_ON = 0x0031  # bits 0,4,5 set
SW_SWITCHED_ON = 0x0033         # bits 0,1,4,5 set
SW_OPERATION_ENABLED = 0x0037   # bits 0,1,2,4,5 set
SW_FAULT = 0x0008               # bit 3 set
SW_MASK = 0x006F                # bits 0-6

# Modes of operation
MODE_PROFILE_VELOCITY = 3
MODE_CYCLIC_SYNC_POSITION = 8
MODE_CYCLIC_SYNC_VELOCITY = 9

# Cyclic-sync-position safety: software slew-rate cap on how fast the cycle
# thread walks the commanded target toward its goal (motor-side RPM). The
# drive's own 0x6080 is the hard cap; this keeps bring-up/jog moves gentle.
CSP_JOG_RPM = 40.0
# Position actual / target objects (CSP)
POSITION_ACTUAL = 0x6064
TARGET_POSITION = 0x607A
INTERP_TIME_PERIOD = 0x60C2

# Speed limits
SPEED_MIN_RPM = 0
SPEED_MAX_RPM = 4500

# Brake default params
BRAKE_RELEASE_SPEED_RPM = 10
BRAKE_OFF_DELAY_MS = 200
BRAKE_ON_DELAY_MS = 500

# Acceleration default (rpm/s)
DEFAULT_ACCEL_RPM = 3000
DEFAULT_DECEL_RPM = 3000

# ─────────────────────────────────────────────────────────────────────────
# Robot geometry — planar 2DOF (shoulder + forearm), vertical plane.
#
# Frame: origin at the shoulder joint axis. X horizontal (forward +),
# Z vertical (up +). The arm moves in the X-Z plane.
#   q[0] = shoulder angle, measured from +X axis, CCW positive.
#   q[1] = elbow angle, relative to link 1, CCW positive.
# The end effector sits at the tip of link 2.
#
# NOTE: all values below are PLACEHOLDERS — replace with real numbers from
# the CAD model / harmonic-drive datasheets before running on hardware.
# ─────────────────────────────────────────────────────────────────────────

# Link lengths in MILLIMETRES: (L1 shoulder->elbow, L2 elbow->end effector).
# All Cartesian quantities (targets, workspace, FK/IK output) are in mm.
# Measured: axis1->axis2 = 200 mm, axis2->end-effector = 281 mm.
# LINK_LENGTHS = (200.0, 281.0)
LINK_LENGTHS = (200.0, 351.0)


# Which IK branch to use: "up" (elbow above the shoulder-EE line) or "down".
# With JOINT_DIR mirrored (-1,-1) the physical bend is the opposite of the
# math branch, so "down" makes the elbow physically point UP (away from the
# table). Toggle at runtime with the `elbow` command if needed.
ELBOW_CONFIG = "down"

# Software joint limits (radians), (min, max) per joint. TODO: from mechanics.
JOINT_LIMITS = (
    (-2.094, 2.094),   # q0 shoulder: ±120°
    (-2.618, 2.618),   # q1 elbow:    ±150°
)

# ── Joint ↔ motor mapping ────────────────────────────────────────────────
# A joint angle is delivered to the motor through a harmonic drive:
#   motor_turns = joint_angle / (2π) * GEAR_RATIO
# and the drive counts encoder increments per motor turn.
# Conversion (see joint_map.py):
#   counts = JOINT_DIR * (joint_angle - JOINT_ZERO_RAD) / (2π)
#            * GEAR_RATIO * ENCODER_COUNTS_PER_REV
#
# Verify ENCODER_COUNTS_PER_REV against the drive: the Y7 may already apply
# its own gear/feed-constant scaling (0x6091/0x6092) to position units.

# Effective motor->joint reduction (motor revs per joint rev). The harmonic
# drives are nameplated 100:1 / 200:1, but a `deg` jog test on hardware
# (2026-06-01) showed the joint turning exactly 2x the commanded angle on BOTH
# axes, while `rev` (motor-shaft revolutions) was exact. So the true reduction
# is half the nameplate — likely an uncompensated 2:1 stage. These calibrated
# values make `deg`/Cartesian moves correct:
GEAR_RATIO = (50.0, 100.0)   # calibrated; nameplate was 100/200
# Counts the drive's object dictionary uses per *motor* revolution for
# position (0x607A / 0x6064). The Y7 electronic gear (0x6091) reads 1:1,
# so these are raw motor-side counts — but NOT the physical encoder bits:
#   - Motor datasheet: encoder is 20-bit (2^20 = 1048576 inc/rev physical).
#   - Hand-tuned VELOCITY_FACTOR (7.26e-6, speed_test.py) implies the drive
#     scales velocity as if 2^23 counts/rev (137741 raw/RPM ≈ 2^23/60),
#     i.e. it interpolates the 20-bit encoder up to 23 bits internally.
# So the OD position unit is most likely 2^23, NOT 2^20. CONFIRM with one
# controlled revolution: read 0x6064, command +8388608 in CSP, check the
# motor shaft turns exactly one revolution; correct this constant if not.
ENCODER_COUNTS_PER_REV = 1 << 23       # 8388608 — CONFIRM by 1-rev test
JOINT_DIR = (-1, -1)                   # both flipped: hardware +cmd was CW, so
                                       # the math frame's +Z came out inverted;
                                       # negating both joints mirrors z back up.
JOINT_ZERO_RAD = (0.0, 0.0)            # joint angle at home; set by teaching home

# ── Motion defaults (joint space) ────────────────────────────────────────
# Max joint speed / acceleration used by the trapezoidal PTP planner.
JOINT_VEL_MAX = 1.0                    # rad/s  TODO: tune
JOINT_ACC_MAX = 2.0                    # rad/s² TODO: tune
