"""Hardware and motion settings for the Halloween skull."""

# LD2450 UART. Connect radar TX to ESP32 RX; radar RX is optional unless
# sending configuration commands to the sensor.
RADAR_UART_ID = 2
RADAR_TX_PIN = 17
RADAR_RX_PIN = 16
RADAR_BAUDRATE = 256000

# BTT TMC2209 V1.3 STEP/DIR control.
PAN_STEP_PIN = 25
PAN_DIR_PIN = 26
PAN_ENABLE_PIN = 27
TILT_STEP_PIN = 18
TILT_DIR_PIN = 19
TILT_ENABLE_PIN = 21
ENABLE_ACTIVE_LOW = True

# Pan UART uses the BTT V1.3 factory shared-RX/UART pad. Put a 1 kOhm resistor
# between TX and the shared bus; RX connects directly to the bus.
TMC_UART_RX_PIN = 22
TMC_UART_TX_PIN = 23
TMC_UART_ID = 1
TMC_UART_BAUDRATE = 57600
PAN_DIAG_PIN = 32
TILT_DIAG_PIN = 33

# Initial unloaded pan-motor commissioning current. The 17HE19-2004S is rated
# 2 A/phase; this starts conservatively at about 0.77 A RMS run / 0.39 A hold.
PAN_RUN_CURRENT_MA = 770
PAN_HOLD_CURRENT_MA = 390
TMC_SENSE_RESISTOR_OHMS = 0.11
TMC_VSENSE_VOLTS = 0.325

NEOPIXEL_PIN = 13
NEOPIXEL_COUNT = 2
EYE_RED = (255, 0, 0)
EYE_OFF = (0, 0, 0)

# A typical NEMA17 is 200 full steps/revolution. MICROSTEPS is the software
# scale only; the BTT driver's actual resolution must be verified/configured.
FULL_STEPS_PER_REVOLUTION = 200
MICROSTEPS = 16
# Set true only after configuring/verifying the driver's actual resolution.
DRIVER_MICROSTEPS_CONFIRMED = False
MAX_STEP_RATE = 400

# Angles are relative to the forward-facing, level skull pose. With the current
# radar mounting, +X is toward the sensor's left. Positive yaw points to +X;
# positive pitch points upward.
PAN_MIN_DEG = -60.0
PAN_MAX_DEG = 60.0
TILT_MIN_DEG = -35.0
TILT_MAX_DEG = 45.0
PARK_YAW_DEG = 0.0
PARK_PITCH_DEG = -25.0

# Geometry in millimetres, in the radar frame: X is lateral, Y is forward,
# and Z is up. With the current mounting, +X is toward the sensor's left.
# Tilt-axis offsets are measured in the pan assembly's local frame
# (right, forward, up) and rotate with the pan axis.
PAN_PIVOT_FROM_SENSOR_X_MM = 0.0
PAN_PIVOT_FROM_SENSOR_Y_MM = 0.0
PAN_PIVOT_FROM_SENSOR_Z_MM = 0.0
TILT_PIVOT_FROM_PAN_X_MM = 0.0
TILT_PIVOT_FROM_PAN_Y_MM = 0.0
TILT_PIVOT_FROM_PAN_Z_MM = 0.0

# The LD2450 reports no target height. Aim at a fixed height above the floor.
# Starter values assume the radar is 1.0 m above floor and aim at 1.2 m.
SENSOR_HEIGHT_ABOVE_FLOOR_MM = 1000
TARGET_HEIGHT_ABOVE_FLOOR_MM = 1200
MIN_TARGET_RANGE_MM = 300
MAX_TARGET_RANGE_MM = 6000
TARGET_LOST_MS = 1200
TRACKING_UPDATE_MS = 40

# "assume" skips homing. Physically place axes at this pose before power-up.
# Other modes stay locked out until their homing code is implemented.
POSITION_MODE = "assume"
STARTUP_PAN_DEG = PARK_YAW_DEG
STARTUP_TILT_DEG = PARK_PITCH_DEG