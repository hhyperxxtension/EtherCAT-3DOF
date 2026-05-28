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
PROFILE_ACCEL = 0x6083
PROFILE_DECEL = 0x6084
MAX_MOTOR_SPEED = 0x6080
GEAR_RATIO = 0x6091  # :1 numerator (motor revs), :2 denominator (shaft revs)

# Encoder — Y7 motor code "D" = 23-bit absolute; use 17 for code "A"
ENCODER_BITS = 23
ENCODER_COUNTS_PER_REV = 1 << ENCODER_BITS

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
MODE_CYCLIC_SYNC_VELOCITY = 9

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
