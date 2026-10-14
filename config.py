"""Hardware and motion settings for the two Halloween skull trackers."""

# LD2450 UART. Connect radar TX to ESP32 RX; radar RX is optional unless
# sending configuration commands to the sensor.
RADAR_UART_ID = 2
RADAR_TX_PIN = 17
RADAR_RX_PIN = 16
RADAR_BAUDRATE = 256000

# BTT TMC2209 V1.3 STEP/DIR control. Skull 2's pan motor reuses the former
# tilt driver (STEP 18 / DIR 19 / EN 21, UART address 1).
ENABLE_ACTIVE_LOW = True

# Shared TMC UART: BTT V1.3 factory shared-RX/UART pad. Put a 1 kOhm resistor
# between TX and the shared bus; RX connects directly to the bus.
TMC_UART_RX_PIN = 22
TMC_UART_TX_PIN = 23
TMC_UART_ID = 1
TMC_UART_BAUDRATE = 115200

# Two skull trackers. Each entry is one pan motor plus one NeoPixel eye pair.
SKULLS = (
    {
        "name": "Skull 1",
        "step_pin": 25,
        "dir_pin": 26,
        "enable_pin": 27,
        "eyes_pin": 13,
        "tmc_address": 0,
        "direction_inverted": False,
        "run_current_ma": 770,
        "hold_current_ma": 390,
    },
    {
        "name": "Skull 2",
        "step_pin": 18,
        "dir_pin": 19,
        "enable_pin": 21,
        "eyes_pin": 14,
        "tmc_address": 1,
        "direction_inverted": False,
        "run_current_ma": 770,
        "hold_current_ma": 390,
    },
)

NEOPIXEL_COUNT = 2
EYE_RED = (255, 0, 0)
EYE_OFF = (0, 0, 0)

TMC_SENSE_RESISTOR_OHMS = 0.11
TMC_VSENSE_VOLTS = 0.325

# A typical NEMA17 is 200 full steps/revolution. MICROSTEPS is configured on
# the TMC by startup code and used here for angle-to-pulse conversion.
FULL_STEPS_PER_REVOLUTION = 200
# 1/8 doubles shaft speed versus 1/16 for the same pulse rate (0.225 deg/pulse).
MICROSTEPS = 8
# ESP32 bit-banged STEP loop, not the motor, is the practical speed ceiling;
# 1600 pps (~360 deg/s at 1/8) leaves headroom for reliable pulse timing.
MAX_STEP_RATE = 1600
MIN_TRACKING_STEP_RATE = 50
# Start/stop hysteresis: nothing moves until the smoothed error reaches the
# start threshold, then it corrects until the error falls below the stop
# threshold. Start is sized above LD2450 noise (+/-2 degrees).
TRACKING_DEADBAND_START_DEG = 2.5
TRACKING_DEADBAND_STOP_DEG = 1.5
TRACKING_FULL_SPEED_ERROR_DEG = 12.0
TRACKING_ERROR_SPEED_EXPONENT = 1.4
TRACKING_FEEDFORWARD_GAIN = 0.65
TRACKING_STEP_RATE_SLEW_PPS_S = 4800
TRACKING_RATE_FILTER_ALPHA = 0.35
TRACKING_TARGET_SMOOTHING_MS = 250
MAX_TRACKING_BEARING_RATE_DEG_S = 45.0

# Pan angles are relative to each skull's forward-facing pose. With the current
# radar mounting, +X is toward the sensor's left. Positive yaw points to +X.
PAN_MIN_DEG = -60.0
PAN_MAX_DEG = 60.0
PARK_YAW_DEG = 0.0

# Pan pivot position relative to the radar, in millimetres, X lateral and
# Y forward. Zero placeholders until measured for the physical prop layout.
PAN_PIVOT_FROM_SENSOR_X_MM = 0.0
PAN_PIVOT_FROM_SENSOR_Y_MM = 0.0

MIN_TARGET_RANGE_MM = 300
MAX_TARGET_RANGE_MM = 6000
TARGET_LOST_MS = 1200
TRACKING_UPDATE_MS = 40

# How skulls pick targets from the radar frame:
#   "nearest_two" - skull 1 follows the nearest valid target, skull 2 the
#                   next-nearest. Either idles when fewer targets exist.
#   "fixed_slot"  - skull 1 always follows radar slot 1, skull 2 slot 2;
#                   an empty slot parks that skull.
TARGET_ASSIGNMENT_MODE = "fixed_slot"

# "assume" skips homing. Physically place each pan axis at this angle before
# power-up. Other modes stay locked out until homing code is implemented.
POSITION_MODE = "assume"
STARTUP_PAN_DEG = PARK_YAW_DEG
