"""Hardware and motion settings for the multi-target prop trackers."""

# LD2450 UART. Connect radar TX to ESP32 RX; radar RX is optional unless
# sending configuration commands to the sensor.
RADAR_UART_ID = 2               # Hardware UART peripheral ID on ESP32
RADAR_TX_PIN = 17               # ESP32 TX pin to radar RX (optional)
RADAR_RX_PIN = 16               # ESP32 RX pin from radar TX (primary data stream)
RADAR_BAUDRATE = 256000         # LD2450 factory baud rate

# BTT TMC2209 V1.3 STEP/DIR control. Tracker 2's pan motor reuses STEP 18 / DIR 19 / EN 21.
# Active-low enable: True pulls EN low (0 V) to enable motor coils.
ENABLE_ACTIVE_LOW = True

# Shared TMC UART: BTT V1.3 factory shared-RX/UART pad. Put a 1 kOhm resistor
# between TX and the shared bus; RX connects directly to the bus.
TMC_UART_RX_PIN = 22            # ESP32 RX directly connected to shared PDN_UART line
TMC_UART_TX_PIN = 23            # ESP32 TX through 1 kOhm resistor to shared line
TMC_UART_ID = 1                 # Hardware UART peripheral ID on ESP32
TMC_UART_BAUDRATE = 115200      # Baud rate for TMC2209 register reads/writes

# Pan angles are relative to each tracker's forward-facing pose. With the current
# radar mounting, +X is toward the sensor's left. Positive yaw points to +X.
MIN_YAW_DEG = -90.0             # Default mechanical/software pan limit to the right (deg)
MAX_YAW_DEG = 90.0              # Default mechanical/software pan limit to the left (deg)
PARK_YAW_DEG = 0.0              # Default resting/parked yaw angle (deg, facing center)

# Two prop/gimbal trackers. Each entry is one pan motor plus one NeoPixel indicator pair.
# Per-motor / per-tracker pivot offsets from the radar sensor:
#   pan_pivot_x_mm: Lateral offset of pan pivot from radar (+X = sensor left)
#   pan_pivot_y_mm: Forward offset of pan pivot from radar (+Y = forward)
#   pan_min_deg: Mechanical/software pan limit to the right (deg)
#   pan_max_deg: Mechanical/software pan limit to the left (deg)
#
# Note on Z offset:
#   There is no Z (vertical) offset variable because the LD2450 is a 2D planar
#   radar (detecting only X and Y coordinates in the horizontal detection plane),
#   and each tracker is currently configured for 1-axis horizontal pan (yaw) rotation.
#   In horizontal planar tracking, the line-of-sight yaw angle is determined solely
#   by relative X and Y positions: atan2(dx, dy). Any vertical (Z) elevation
#   difference between the radar plane and the pan pivot has zero geometric effect
#   on the yaw angle. (A Z offset would only be relevant if 2-axis pan/tilt
#   mechanisms were implemented to compute pitch/elevation angles).
TRACKERS = (
    {
        "name": "Tracker 1",
        "step_pin": 25,
        "dir_pin": 26,
        "enable_pin": 27,
        "eyes_pin": 13,
        "tmc_address": 0,
        "direction_inverted": False,
        "run_current_ma": 770,
        "hold_current_ma": 390,
        "pan_pivot_x_mm": 0.0,
        "pan_pivot_y_mm": 0.0,
        "pan_min_deg": MIN_YAW_DEG,
        "pan_max_deg": MAX_YAW_DEG,
    },
    {
        "name": "Tracker 2",
        "step_pin": 18,
        "dir_pin": 19,
        "enable_pin": 21,
        "eyes_pin": 14,
        "tmc_address": 1,
        "direction_inverted": False,
        "run_current_ma": 770,
        "hold_current_ma": 390,
        "pan_pivot_x_mm": 0.0,
        "pan_pivot_y_mm": 0.0,
        "pan_min_deg": MIN_YAW_DEG,
        "pan_max_deg": MAX_YAW_DEG,
    },
)

# NeoPixel WS2812B eye indicators
NEOPIXEL_COUNT = 2              # Number of LEDs per tracker (one per eye/indicator socket)
EYE_RED = (255, 0, 0)          # Eye color when actively tracking a target
EYE_OFF = (0, 0, 0)            # Eye color when sleeping / idle / target lost

# TMC2209 driver sensing parameters (used to compute current scale registers)
TMC_SENSE_RESISTOR_OHMS = 0.11  # Current sense resistor on BTT V1.3 (R110 = 0.11 Ohm)
TMC_VSENSE_VOLTS = 0.325        # Internal current sense reference voltage (standard mode)

# A typical NEMA17 is 200 full steps/revolution. MICROSTEPS is configured on
# the TMC by startup code and used here for angle-to-pulse conversion.
FULL_STEPS_PER_REVOLUTION = 200
# 1/16 microstepping yields 3,200 pulses/rev (0.1125 deg/pulse) for whisper-quiet
# motion with resonance-free coil micro-stepping.
MICROSTEPS = 16
# Hardware RMT generates step pulses with zero CPU bit-banging overhead.
# 6400 pps (~720 deg/s or 2.0 rev/s at 1/16) provides rapid tracking response.
MAX_STEP_RATE = 6400
MIN_TRACKING_STEP_RATE = 35     # Floor step rate while moving (avoids crawl/resonance)

# Micro-hysteresis deadband: 2 microsteps (~0.225 deg at 1/16 microstepping).
# Instead of the old 2.5 deg ratchet, a small 2-microstep deadband prevents
# hunting over a single rounding digit when standing still.
# Once moving beyond 2 microsteps, tracking is smooth and continuous to target.
TRACKING_DEADBAND_START_DEG = 0.225
TRACKING_DEADBAND_STOP_DEG = 0.0

# Speed shaping and feed-forward parameters:
TRACKING_FULL_SPEED_ERROR_DEG = 12.0     # Pointing error (deg) at which speed reaches MAX_STEP_RATE
TRACKING_ERROR_SPEED_EXPONENT = 1.4       # Exponent for error-to-speed curve (>1 = gentle near target)
TRACKING_FEEDFORWARD_GAIN = 0.85          # Target velocity feed-forward gain (pacing moving targets)
TRACKING_STEP_RATE_SLEW_PPS_S = 14400     # Max acceleration/deceleration rate (pulses/s^2)
TRACKING_RATE_FILTER_ALPHA = 0.35         # Low-pass filter weight for target angular rate estimation
TRACKING_TARGET_SMOOTHING_MS = 150        # Time constant (ms) for exponential target angle smoothing
MAX_TRACKING_BEARING_RATE_DEG_S = 90.0    # Clamp on target angular speed to reject sensor noise jumps

# TODO testing out these for snappier motion tuning (all or some subset at a time)
# TRACKING_TARGET_SMOOTHING_MS = 150  (350)      # Cuts lag by more than half (~150 ms vs 350 ms)
# MAX_TRACKING_BEARING_RATE_DEG_S = 90.0 (45)   # Allows tracking fast swings up close
# TRACKING_FEEDFORWARD_GAIN = 0.85 (0.65)          # Closer to 1:1 speed matching

# Motion Shaping & Smoothing Toggles:
# Set all to False for pure direct tracking. Turn them back on one at a time
# to evaluate their individual effect on tracking feel and smoothness.
ENABLE_TARGET_SMOOTHING = True  # Exponential smoothing of radar target angle
ENABLE_DEADBAND = True           # Start/stop hysteresis threshold
ENABLE_SPEED_SHAPING = True      # Error-proportional speed curve & feedforward
ENABLE_RATE_SLEW = True          # Slew rate limiter on acceleration/deceleration

# Constant tracking speed used when ENABLE_SPEED_SHAPING is False
# (800 pps at 1/16 microstepping is 90 deg/s; smooth and quiet)
TRACKING_BASE_STEP_RATE = 800

# Radar target filtering and tracking loop timing:
MIN_TARGET_RANGE_MM = 300       # Minimum distance (mm) from radar to consider target valid
MAX_TARGET_RANGE_MM = 6000      # Maximum detection range (mm) for prop interaction
TARGET_LOST_MS = 1200           # Duration (ms) without detection before returning to park
TRACKING_UPDATE_MS = 20         # Interval (ms) between tracking angle/speed updates (50 Hz)

# ESP32 Hardware RMT stepper pulse parameters:
RMT_RESOLUTION_HZ = 1000000     # 1 MHz clock (1 microsecond resolution)
RMT_PULSE_WIDTH_US = 10         # STEP high pulse duration (microseconds)
RMT_BURST_SLICE_MS = 15         # Non-blocking slice duration (ms) per RMT pulse burst

# How trackers pick targets from the radar frame:
#   "nearest_two" - tracker 1 follows the nearest valid target, tracker 2 the
#                   next-nearest. Either idles when fewer targets exist.
#   "fixed_slot"  - tracker 1 always follows radar slot 1, tracker 2 slot 2;
#                   an empty slot parks that tracker.
TARGET_ASSIGNMENT_MODE = "fixed_slot"

# "assume" skips homing. Physically place each pan axis at this angle before
# power-up. Other modes stay locked out until homing code is implemented.
POSITION_MODE = "assume"
STARTUP_PAN_DEG = PARK_YAW_DEG  # Initial pan angle assumed at MCU power-up
