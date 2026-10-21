"""LD2450-tracked multi-target tracker controller for ESP32 MicroPython.

Each prop is a Tracker: one yaw stepper axis plus one NeoPixel indicator pair.
Tracker 1 follows the nearest valid radar target, tracker 2 the next-nearest.
"""

import time
from machine import Pin, UART
import neopixel

import config
from aiming import clamp, solve_yaw
from ld2450 import LD2450Reader
from stepper import StepperAxis
from tmc2209 import REG_GLOBALSCALER, REG_IOIN, TMC2209
from tracking import (assign_targets, choose_two_targets, slew_step_rate,
                      smooth_toward_target, tracking_step_rate,
                      update_deadband)


def set_eyes(eyes, color):
    """Set both NeoPixel eye LEDs to the specified RGB color tuple and commit."""
    for index in range(config.NEOPIXEL_COUNT):
        eyes[index] = color
    eyes.write()


def find_tmc_drivers():
    """Open the shared TMC UART and require every tracker's driver to reply.

    The project uses a single shared half-duplex UART bus for all TMC2209 drivers.
    Each driver is distinguished by its hardware address pins (MS1/MS2). This
    function verifies physical connectivity and valid silicon revision (0x21)
    for each driver before allowing motion.
    """
    addresses = [tracker["tmc_address"] for tracker in config.TRACKERS]
    if len(set(addresses)) != len(addresses):
        raise ValueError("each tracker must have a distinct TMC UART address")

    uart = UART(config.TMC_UART_ID, baudrate=config.TMC_UART_BAUDRATE,
                bits=8, parity=None, stop=1,
                tx=Pin(config.TMC_UART_TX_PIN),
                rx=Pin(config.TMC_UART_RX_PIN))
    time.sleep_ms(50)
    drivers = {}
    for tracker in config.TRACKERS:
        driver = TMC2209(uart, slave_address=tracker["tmc_address"])
        try:
            ioin = driver.read_register(REG_IOIN)
        except OSError as error:
            uart.deinit()
            raise OSError("{} driver at address {} did not reply: {}".format(
                tracker["name"], tracker["tmc_address"], error))
        if (ioin >> 24) != 0x21:
            uart.deinit()
            raise OSError("unexpected {} TMC2209 version: IOIN=0x{:08x}".
                          format(tracker["name"], ioin))
        drivers[tracker["name"]] = driver
        print("{} TMC2209 replied at address {}; IOIN=0x{:08x}".
              format(tracker["name"], tracker["tmc_address"], ioin))
    return uart, drivers


def configure_driver(driver, name, run_current_ma, hold_current_ma):
    """Configure a TMC2209 over UART for digital current and microstepping.

    Steps performed:
      1. GCONF: Switch from analog VREF scaling to digital UART register control.
      2. GLOBALSCALER: Set to 0 (full 256/256 scale).
      3. IHOLD_IRUN: Set operating run current (RMS) and standby hold current (RMS).
         Verified via IFCNT (interface transmission counter).
      4. CHOPCONF: Set MRES for 1/16 microstepping (3,200 steps/rev).
    """
    gconf = driver.configure_uart_mode()
    scaler_ifcnt = driver.write_register_confirmed(REG_GLOBALSCALER, 0)
    run_cs, hold_cs, current_register, current_ifcnt = (
        driver.configure_current(
            run_current_ma,
            hold_current_ma,
            config.TMC_SENSE_RESISTOR_OHMS,
            config.TMC_VSENSE_VOLTS,
        )
    )
    chopconf = driver.configure_microsteps(config.MICROSTEPS)
    print("{} GCONF=0x{:08x}; GLOBALSCALER IFCNT={}".
          format(name, gconf, scaler_ifcnt))
    print("{} IHOLD_IRUN=0x{:08x} accepted (IRUN={}, IHOLD={}, IFCNT={})".
          format(name, current_register, run_cs, hold_cs, current_ifcnt))
    print("{} CHOPCONF=0x{:08x}; {} microsteps".
          format(name, chopconf, config.MICROSTEPS))


class Tracker:
    """Controls one tracker prop: yaw stepper axis, status LEDs, and tracking pipeline.

    A Tracker encapsulates all motion, LED state, target filtering, and position
    history for a single tracker. Each instance maintains:
      - Stepper motor control via StepperAxis (hardware RMT pulse generation).
      - NeoPixel eye/indicator color state (red when tracking, dark when idle/parked).
      - Target filtering, angular velocity estimation, and exponential smoothing.
      - Dual-threshold deadband hysteresis and velocity slew rate limiting.
    """

    def __init__(self, tracker_cfg):
        self.name = tracker_cfg["name"]

        # --- Hardware Pins and Actuators ---
        # Direct GPIO pin controls passed to the StepperAxis:
        self.step_pin = Pin(tracker_cfg["step_pin"], Pin.OUT, value=0)
        self.direction_pin = Pin(tracker_cfg["dir_pin"], Pin.OUT, value=0)
        self.enable_pin = Pin(tracker_cfg["enable_pin"], Pin.OUT, value=1)

        # Eye/status illumination (WS2812B NeoPixel chain):
        self.eyes = neopixel.NeoPixel(Pin(tracker_cfg["eyes_pin"]),
                                      config.NEOPIXEL_COUNT)
        set_eyes(self.eyes, config.EYE_OFF)

        # Prop geometry: position of this motor's yaw axis relative to the radar.
        # (+X = sensor left, +Y = forward into detection zone)
        self.yaw_pivot_x_mm = tracker_cfg.get(
            "yaw_pivot_x_mm", tracker_cfg.get("pan_pivot_x_mm", 0.0))
        self.yaw_pivot_y_mm = tracker_cfg.get(
            "yaw_pivot_y_mm", tracker_cfg.get("pan_pivot_y_mm", 0.0))
        self.yaw_min_deg = tracker_cfg.get(
            "yaw_min_deg", tracker_cfg.get(
                "pan_min_deg", getattr(config, "MIN_YAW_DEG", -90.0)))
        self.yaw_max_deg = tracker_cfg.get(
            "yaw_max_deg", tracker_cfg.get(
                "pan_max_deg", getattr(config, "MAX_YAW_DEG", 90.0)))

        # Motion axis with ESP32 hardware RMT pulse generation:
        steps_per_revolution = (config.FULL_STEPS_PER_REVOLUTION *
                                config.MICROSTEPS)
        self.axis = StepperAxis(
            self.step_pin,
            self.direction_pin,
            self.enable_pin,
            steps_per_revolution,
            self.yaw_min_deg,
            self.yaw_max_deg,
            config.MAX_STEP_RATE,
            config.ENABLE_ACTIVE_LOW,
            getattr(config, "STARTUP_YAW_DEG", config.PARK_YAW_DEG),
            tracker_cfg["direction_inverted"],
            rmt_resolution_hz=getattr(config, "RMT_RESOLUTION_HZ", 1000000),
            pulse_width_us=getattr(config, "RMT_PULSE_WIDTH_US", 10),
            burst_slice_ms=getattr(config, "RMT_BURST_SLICE_MS", 15),
        )

        # --- Motion & Alertness State Flags ---
        # self.moving: True when yaw motor is actively commanded to step toward target_yaw.
        # Managed by the deadband hysteresis (starts moving if error >= START_DEG,
        # stops when error <= STOP_DEG or exact target step reached).
        self.moving = False

        # self.awake: True when a target is actively tracked (eyes illuminated red).
        # False when in parked/sleep mode (eyes off, target lost for > TARGET_LOST_MS).
        self.awake = False

        # self.reversing: True when motion direction has flipped before coming to a stop.
        # When reversing, the tracker decelerates to MIN_TRACKING_STEP_RATE first
        # to avoid jerking and step loss.
        self.reversing = False
        self.current_motion_dir = 0

        # --- Target Tracking & History ---
        # self.current_target: (x_mm, y_mm, speed_cm_s, resolution_mm) tuple of
        # currently tracked target. Retained across brief radar dropouts until TARGET_LOST_MS.
        self.current_target = None

        # self.last_target_ms: System timestamp (ms) when a valid target was last received.
        # Used to evaluate timeout against config.TARGET_LOST_MS.
        self.last_target_ms = None

        # --- Angular Velocity (Bearing Rate) Feed-Forward ---
        # self.last_bearing_yaw: Previous raw target yaw (deg) for calculating delta-yaw.
        self.last_bearing_yaw = None

        # self.last_bearing_ms: Timestamp (ms) of previous bearing sample for delta-time.
        self.last_bearing_ms = None

        # self.filtered_yaw_rate: Low-pass filtered angular speed of target (deg/s).
        # Used by tracking_step_rate to add feed-forward velocity pacing.
        self.filtered_yaw_rate = 0.0

        # --- Angle Smoothing & Command Generation ---
        # self.last_raw_yaw: Most recent instantaneous geometric yaw angle (deg)
        # calculated directly from radar X/Y coordinates and clamped to min/max limits.
        self.last_raw_yaw = None

        # self.smoothed_yaw: Exponentially filtered yaw angle (deg) that glides smoothly
        # between radar measurements, eliminating stepping jitter from raw radar noise.
        self.smoothed_yaw = None

        # self.target_yaw: Final commanded yaw angle (deg) sent to the stepper axis.
        # Equals smoothed_yaw while tracking, or PARK_YAW_DEG when idle/lost.
        self.target_yaw = config.PARK_YAW_DEG

        # self.desired_rate: Un-slewed instantaneous step rate (pulses/s) calculated from
        # current pointing error and target bearing speed.
        self.desired_rate = config.MIN_TRACKING_STEP_RATE

    def on_radar_frame(self, target, now_ms):
        """Record this tracker's assigned target from the latest radar frame.

        Parameters:
          target: (x_mm, y_mm, speed_cm_s, resolution_mm) tuple from the radar,
                  or None if no target is assigned to this tracker.
          now_ms: Monotonic millisecond timestamp of the radar frame.

        Processing:
          1. Updates current_target and refreshes last_target_ms (postponing timeout).
          2. Solves the geometric line-of-sight yaw angle from radar coordinates.
          3. Estimates target angular velocity (bearing rate in deg/s) and applies
             a low-pass filter to provide feed-forward velocity pacing.
        """
        if target is None:
            self.last_raw_yaw = None
            return
        self.current_target = target
        self.last_target_ms = now_ms

        # Compute geometric yaw angle to the target
        yaw = solve_yaw(target[0], target[1],
                        self.yaw_pivot_x_mm, self.yaw_pivot_y_mm)
        yaw = clamp(yaw, self.yaw_min_deg, self.yaw_max_deg)
        self.last_raw_yaw = yaw

        # Calculate target angular speed (bearing rate in deg/s) for feed-forward
        if self.last_bearing_ms is not None and self.last_bearing_yaw is not None:
            bearing_elapsed_ms = time.ticks_diff(now_ms, self.last_bearing_ms)
            if 0 < bearing_elapsed_ms <= 250:
                yaw_rate = min(
                    abs(yaw - self.last_bearing_yaw) * 1000.0 /
                    bearing_elapsed_ms,
                    config.MAX_TRACKING_BEARING_RATE_DEG_S)
                alpha = config.TRACKING_RATE_FILTER_ALPHA
                self.filtered_yaw_rate += alpha * (
                    yaw_rate - self.filtered_yaw_rate)
            else:
                self.filtered_yaw_rate = 0.0
        self.last_bearing_yaw = yaw
        self.last_bearing_ms = now_ms

    def update(self, now_ms, elapsed_ms):
        """Execute one tracking update step (called on radar frames and 50 Hz timer).

        Parameters:
          now_ms: Monotonic millisecond timestamp.
          elapsed_ms: Milliseconds elapsed since the previous update step.

        Lifecycle and motion pipeline:
          1. Target Timeout: If no target received for > TARGET_LOST_MS, transition
             to sleep/park mode, turn eye LEDs dark, and command PARK_YAW_DEG.
          2. Target Tracking: If target is active, turn eye LEDs red, smooth raw
             yaw with an exponential filter (if enabled) to eliminate radar noise.
          3. Deadband Hysteresis: Apply start/stop error thresholds to prevent
             hunting/vibration when standing nearly still.
          4. Speed Shaping & Slew: If moving, compute velocity-profiled step rate
             (E^1.4 + feedforward). Slew acceleration/deceleration to protect against
             missed steps, and decelerate before reversing direction.
          5. Coil Power: Keep stepper driver coils continuously energized while
             running so holding torque locks position against wind or mass imbalance.
             When stationary, TMC2209 automatically drops current to IHOLD.
        """
        expired = (self.last_target_ms is None or
                   time.ticks_diff(now_ms, self.last_target_ms) >
                   config.TARGET_LOST_MS)
        if expired:
            # Target has timed out: park facing forward with eyes dark
            self.current_target = None
            self.last_raw_yaw = None
            self.last_bearing_yaw = None
            self.last_bearing_ms = None
            self.filtered_yaw_rate = 0.0
            self.smoothed_yaw = config.PARK_YAW_DEG
            self.target_yaw = config.PARK_YAW_DEG
            if self.awake:
                set_eyes(self.eyes, config.EYE_OFF)
                self.awake = False
                print("{}: target lost; returning to park.".format(self.name))
        else:
            # Active target: determine target yaw and ensure eyes are illuminated
            if self.current_target is None:
                return
            x_mm, y_mm, speed_cm_s, resolution_mm = self.current_target
            target_yaw = self.last_raw_yaw
            if target_yaw is None:
                target_yaw = solve_yaw(
                    x_mm, y_mm,
                    self.yaw_pivot_x_mm, self.yaw_pivot_y_mm)
                target_yaw = clamp(target_yaw, self.yaw_min_deg,
                                   self.yaw_max_deg)
                self.last_raw_yaw = target_yaw

            if not self.awake:
                set_eyes(self.eyes, config.EYE_RED)
                self.awake = True
                print("{}: target acquired x={} y={} mm; yaw={:.1f} deg".
                      format(self.name, x_mm, y_mm, target_yaw))
                # Seed smoother directly with the initial angle to avoid lag on wake
                if getattr(config, "ENABLE_TARGET_SMOOTHING", True):
                    self.smoothed_yaw = target_yaw

            # Apply low-pass exponential angle smoothing if enabled
            if getattr(config, "ENABLE_TARGET_SMOOTHING", True):
                if self.smoothed_yaw is None:
                    self.smoothed_yaw = target_yaw
                else:
                    self.smoothed_yaw = smooth_toward_target(
                        self.smoothed_yaw, target_yaw, elapsed_ms,
                        config.TRACKING_TARGET_SMOOTHING_MS)
                self.target_yaw = self.smoothed_yaw
            else:
                self.smoothed_yaw = target_yaw
                self.target_yaw = target_yaw

        # Calculate pointing error in degrees
        current_degrees = self.axis.position_steps / self.axis.steps_per_degree
        error_degrees = self.target_yaw - current_degrees

        # Evaluate deadband hysteresis to decide whether motor should move
        if getattr(config, "ENABLE_DEADBAND", True):
            self.moving = update_deadband(
                abs(error_degrees),
                self.moving,
                config.TRACKING_DEADBAND_START_DEG,
                config.TRACKING_DEADBAND_STOP_DEG,
            )
            if self.axis.position_steps == round(self.target_yaw * self.axis.steps_per_degree):
                self.moving = False
        else:
            # Without deadband: move whenever position differs from target
            self.moving = (self.axis.position_steps !=
                           round(self.target_yaw * self.axis.steps_per_degree))

        if self.moving:
            # Calculate desired step rate via error-shaping curve + feed-forward
            if getattr(config, "ENABLE_SPEED_SHAPING", True):
                desired_rate = tracking_step_rate(
                    error_degrees,
                    self.filtered_yaw_rate,
                    config.MIN_TRACKING_STEP_RATE,
                    config.MAX_STEP_RATE,
                    config.TRACKING_FULL_SPEED_ERROR_DEG,
                    config.TRACKING_DEADBAND_STOP_DEG,
                    config.TRACKING_ERROR_SPEED_EXPONENT,
                    self.axis.steps_per_degree,
                    config.TRACKING_FEEDFORWARD_GAIN,
                    getattr(config, "TRACKING_FEEDFORWARD_MIN_ERROR_DEG", 3.0),
                )
            else:
                desired_rate = getattr(config, "TRACKING_BASE_STEP_RATE", 400)
            self.desired_rate = desired_rate

            # Apply acceleration/deceleration slew rate limiting
            if getattr(config, "ENABLE_RATE_SLEW", True):
                desired_dir = 1 if error_degrees > 0 else (-1 if error_degrees < 0 else 0)
                reversing = (self.current_motion_dir != 0 and
                             desired_dir != 0 and
                             self.current_motion_dir != desired_dir)
                self.reversing = reversing
                if reversing and self.axis.step_rate > config.MIN_TRACKING_STEP_RATE:
                    # Decelerate to minimum speed before executing reverse
                    rate = slew_step_rate(self.axis.step_rate,
                                          config.MIN_TRACKING_STEP_RATE,
                                          elapsed_ms,
                                          config.TRACKING_STEP_RATE_SLEW_PPS_S)
                    self.axis.set_step_rate(rate)
                    self.axis.set_target_degrees(
                        (self.axis.position_steps / self.axis.steps_per_degree) +
                        self.current_motion_dir)
                else:
                    # Reversal brake complete or continuing in same direction
                    self.current_motion_dir = desired_dir
                    rate = slew_step_rate(self.axis.step_rate, desired_rate,
                                          elapsed_ms,
                                          config.TRACKING_STEP_RATE_SLEW_PPS_S)
                    self.axis.set_step_rate(rate)
                    self.axis.set_target_degrees(self.target_yaw)
            else:
                self.reversing = False
                self.current_motion_dir = 1 if error_degrees > 0 else (-1 if error_degrees < 0 else 0)
                self.axis.set_step_rate(desired_rate)
                self.axis.set_target_degrees(self.target_yaw)
        else:
            # Not moving: hold current position and reset rate to minimum floor
            self.desired_rate = config.MIN_TRACKING_STEP_RATE
            self.reversing = False
            self.current_motion_dir = 0
            self.axis.set_step_rate(config.MIN_TRACKING_STEP_RATE)
            if getattr(config, "ENABLE_DEADBAND", True):
                self.axis.target_steps = self.axis.position_steps
            else:
                self.axis.set_target_degrees(self.target_yaw)

        # Coil energization: keep coils engaged continuously. TMC2209 automatically drops
        # to silent IHOLD current when stationary, locking position against wind/imbalance.
        self.axis.set_enabled(True)

    def run(self):
        """Dispatch hardware RMT pulse burst slice or bit-bang fallback step."""
        self.axis.run()

    def shutdown(self):
        """Release hardware peripherals, turn off eye LEDs, and disable coils."""
        self.axis.deinit()
        set_eyes(self.eyes, config.EYE_OFF)


def main():
    """Main program entry point: initialize hardware and run real-time tracking loop."""
    if config.POSITION_MODE != "assume":
        raise RuntimeError("selected position mode is not implemented")
    print("Startup pose: every tracker yaw at {} deg".format(
        getattr(config, "STARTUP_YAW_DEG", config.PARK_YAW_DEG)))

    trackers = [Tracker(tracker_cfg) for tracker_cfg in config.TRACKERS]
    tmc_uart = None
    radar_uart = None
    try:
        # 1. Discover and initialize both TMC2209 stepper drivers via shared UART
        tmc_uart, drivers = find_tmc_drivers()
        print("All TMC2209s found at {} baud; configuring before motion.".
              format(config.TMC_UART_BAUDRATE))
        for tracker_cfg in config.TRACKERS:
            configure_driver(drivers[tracker_cfg["name"]], tracker_cfg["name"],
                             tracker_cfg["run_current_ma"], tracker_cfg["hold_current_ma"])

        # Enable coils now that drivers are configured with digital currents;
        # coils remain energized continuously (holding at IHOLD when stationary) to lock position.
        for tracker in trackers:
            tracker.axis.set_enabled(True)

        # 2. Open LD2450 radar UART and initialize streaming reader
        radar_uart = UART(config.RADAR_UART_ID, baudrate=config.RADAR_BAUDRATE,
                          bits=8, parity=None, stop=1,
                          tx=config.RADAR_TX_PIN, rx=config.RADAR_RX_PIN)
        radar = LD2450Reader(radar_uart)

        # Print active configuration summary
        features = []
        if getattr(config, "ENABLE_TARGET_SMOOTHING", False):
            features.append("smooth:{}ms".format(config.TRACKING_TARGET_SMOOTHING_MS))
        if getattr(config, "ENABLE_DEADBAND", False):
            features.append("deadband:{:.2f}/{:.2f}deg".format(
                config.TRACKING_DEADBAND_START_DEG,
                config.TRACKING_DEADBAND_STOP_DEG))
        if getattr(config, "ENABLE_SPEED_SHAPING", False):
            features.append("speed-shaping:{}-{}pps".format(
                config.MIN_TRACKING_STEP_RATE, config.MAX_STEP_RATE))
        else:
            features.append("base-speed:{}pps".format(
                getattr(config, "TRACKING_BASE_STEP_RATE", 400)))
        if getattr(config, "ENABLE_RATE_SLEW", False):
            features.append("slew:{}pps/s".format(
                config.TRACKING_STEP_RATE_SLEW_PPS_S))
        active_str = ", ".join(features) if features else "pure direct baseline"
        print("Trackers running: 1/{} microsteps ({})".format(
            config.MICROSTEPS, active_str))

        last_update_ms = time.ticks_ms()

        # --- Main Real-Time Control Loop ---
        while True:
            # (A) Poll radar for new decoded frames (~10 Hz)
            targets = radar.poll()
            now_ms = time.ticks_ms()
            if targets is not None:
                # Assign radar targets to trackers (fixed_slot or nearest)
                targetless_behavior = getattr(
                    config, "TARGETLESS_BEHAVIOR",
                    getattr(config, "TARGETLESS_BEHAVIOUR", "rest"))
                assigned = assign_targets(targets,
                                          len(trackers),
                                          config.MIN_TARGET_RANGE_MM,
                                          config.MAX_TARGET_RANGE_MM,
                                          config.TARGET_ASSIGNMENT_MODE,
                                          targetless_behavior)
                for tracker, target in zip(trackers, assigned):
                    tracker.on_radar_frame(target, now_ms)

                # Run tracking pipeline on radar frame arrival
                update_elapsed_ms = max(1, time.ticks_diff(now_ms, last_update_ms))
                last_update_ms = now_ms
                for tracker in trackers:
                    tracker.update(now_ms, update_elapsed_ms)

            # (B) Periodic tracking update at 50 Hz (every TRACKING_UPDATE_MS = 20 ms)
            update_elapsed_ms = time.ticks_diff(now_ms, last_update_ms)
            if update_elapsed_ms >= config.TRACKING_UPDATE_MS:
                last_update_ms = now_ms
                for tracker in trackers:
                    tracker.update(now_ms, update_elapsed_ms)

            # (C) Service stepper motors (dispatches next hardware RMT burst slice)
            for tracker in trackers:
                tracker.run()

            # (D) Cooperative yield: RMT hardware generates pulses autonomously,
            # so sleep 1 ms to prevent CPU starvation, service WiFi/REPL, and keep ESP32 cool.
            time.sleep_ms(1)
    finally:
        for tracker in trackers:
            tracker.shutdown()
        if radar_uart is not None:
            radar_uart.deinit()
        if tmc_uart is not None:
            tmc_uart.deinit()
        print("Trackers stopped; all drivers disabled and eyes off.")


main()
