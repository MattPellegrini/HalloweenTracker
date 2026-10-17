"""LD2450-tracked two-skull controller for ESP32 MicroPython.

Each skull is a Tracker: one pan stepper axis plus one NeoPixel eye pair.
Skull 1 follows the nearest valid radar target, skull 2 the next-nearest.
"""

import time
from machine import Pin, UART
import neopixel

import config
from aiming import clamp, solve_angles
from ld2450 import LD2450Reader
from stepper import StepperAxis
from tmc2209 import REG_GLOBALSCALER, REG_IOIN, TMC2209
from tracking import (choose_two_targets, slew_step_rate,
                      smooth_toward_target, tracking_step_rate,
                      update_deadband)


def set_eyes(eyes, color):
    for index in range(config.NEOPIXEL_COUNT):
        eyes[index] = color
    eyes.write()


def find_tmc_drivers():
    """Open the shared TMC UART and require every skull's driver to reply."""
    addresses = [skull["tmc_address"] for skull in config.SKULLS]
    if len(set(addresses)) != len(addresses):
        raise ValueError("each skull must have a distinct TMC UART address")

    uart = UART(config.TMC_UART_ID, baudrate=config.TMC_UART_BAUDRATE,
                bits=8, parity=None, stop=1,
                tx=Pin(config.TMC_UART_TX_PIN),
                rx=Pin(config.TMC_UART_RX_PIN))
    time.sleep_ms(50)
    drivers = {}
    for skull in config.SKULLS:
        driver = TMC2209(uart, slave_address=skull["tmc_address"])
        try:
            ioin = driver.read_register(REG_IOIN)
        except OSError as error:
            uart.deinit()
            raise OSError("{} driver at address {} did not reply: {}".format(
                skull["name"], skull["tmc_address"], error))
        if (ioin >> 24) != 0x21:
            uart.deinit()
            raise OSError("unexpected {} TMC2209 version: IOIN=0x{:08x}".
                          format(skull["name"], ioin))
        drivers[skull["name"]] = driver
        print("{} TMC2209 replied at address {}; IOIN=0x{:08x}".
              format(skull["name"], skull["tmc_address"], ioin))
    return uart, drivers


def configure_driver(driver, name, run_current_ma, hold_current_ma):
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
    """One skull: pan axis, eye LEDs, and independent target-tracking state."""

    def __init__(self, skull):
        self.name = skull["name"]
        self.step_pin = Pin(skull["step_pin"], Pin.OUT, value=0)
        self.direction_pin = Pin(skull["dir_pin"], Pin.OUT, value=0)
        self.enable_pin = Pin(skull["enable_pin"], Pin.OUT, value=1)
        self.eyes = neopixel.NeoPixel(Pin(skull["eyes_pin"]),
                                      config.NEOPIXEL_COUNT)
        set_eyes(self.eyes, config.EYE_OFF)
        steps_per_revolution = (config.FULL_STEPS_PER_REVOLUTION *
                                config.MICROSTEPS)
        self.axis = StepperAxis(
            self.step_pin,
            self.direction_pin,
            self.enable_pin,
            steps_per_revolution,
            config.PAN_MIN_DEG,
            config.PAN_MAX_DEG,
            config.MAX_STEP_RATE,
            config.ENABLE_ACTIVE_LOW,
            config.STARTUP_PAN_DEG,
            skull["direction_inverted"],
        )
        self.moving = False
        self.awake = False
        self.reversing = False
        self.current_target = None
        self.last_assigned_target = None
        self.last_target_ms = None
        self.last_bearing_yaw = None
        self.last_bearing_ms = None
        self.filtered_yaw_rate = 0.0
        self.smoothed_yaw = None
        self.target_yaw = config.PARK_YAW_DEG
        self.desired_rate = config.MIN_TRACKING_STEP_RATE
        self.last_raw_yaw = None
        self.last_frame_steps = self.axis.position_steps

    def on_radar_frame(self, target, now_ms):
        """Record this skull's assigned target from the latest radar frame."""
        self.last_assigned_target = target
        if target is None:
            self.last_raw_yaw = None
            return
        self.current_target = target
        self.last_target_ms = now_ms
        yaw, _ = solve_angles(target[0], target[1], 0,
                              (config.PAN_PIVOT_FROM_SENSOR_X_MM,
                               config.PAN_PIVOT_FROM_SENSOR_Y_MM, 0),
                              (0, 0, 0))
        yaw = clamp(yaw, config.PAN_MIN_DEG, config.PAN_MAX_DEG)
        self.last_raw_yaw = yaw

        bearing_elapsed_ms = 0
        if self.last_bearing_ms is not None and self.last_bearing_yaw is not None:
            bearing_elapsed_ms = time.ticks_diff(now_ms, self.last_bearing_ms)
            if bearing_elapsed_ms > 0:
                yaw_rate = min(
                    abs(yaw - self.last_bearing_yaw) * 1000.0 /
                    bearing_elapsed_ms,
                    config.MAX_TRACKING_BEARING_RATE_DEG_S)
                alpha = config.TRACKING_RATE_FILTER_ALPHA
                self.filtered_yaw_rate += alpha * (
                    yaw_rate - self.filtered_yaw_rate)
        self.last_bearing_yaw = yaw
        self.last_bearing_ms = now_ms

    def update(self, now_ms, elapsed_ms):
        expired = (self.last_target_ms is None or
                   time.ticks_diff(now_ms, self.last_target_ms) >
                   config.TARGET_LOST_MS)
        if expired:
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
            x_mm, y_mm, speed_cm_s, resolution_mm = self.current_target
            target_yaw = self.last_raw_yaw
            if target_yaw is None:
                target_yaw, _ = solve_angles(
                    x_mm, y_mm, 0,
                    (config.PAN_PIVOT_FROM_SENSOR_X_MM,
                     config.PAN_PIVOT_FROM_SENSOR_Y_MM, 0),
                    (0, 0, 0))
                target_yaw = clamp(target_yaw, config.PAN_MIN_DEG,
                                   config.PAN_MAX_DEG)
                self.last_raw_yaw = target_yaw

            if not self.awake:
                set_eyes(self.eyes, config.EYE_RED)
                self.awake = True
                print("{}: target acquired x={} y={} mm; yaw={:.1f} deg".
                      format(self.name, x_mm, y_mm, target_yaw))
                if getattr(config, "ENABLE_TARGET_SMOOTHING", True):
                    self.smoothed_yaw = target_yaw

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

        current_degrees = self.axis.position_steps / self.axis.steps_per_degree
        error_degrees = self.target_yaw - current_degrees

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
                )
            else:
                desired_rate = getattr(config, "TRACKING_BASE_STEP_RATE", 400)
            self.desired_rate = desired_rate

            if getattr(config, "ENABLE_RATE_SLEW", True):
                previous_difference = (self.axis.target_steps -
                                       self.axis.position_steps)
                reversing = (previous_difference != 0 and
                             previous_difference * error_degrees < 0)
                self.reversing = reversing
                if reversing and self.axis.step_rate > config.MIN_TRACKING_STEP_RATE:
                    rate = slew_step_rate(self.axis.step_rate,
                                          config.MIN_TRACKING_STEP_RATE,
                                          elapsed_ms,
                                          config.TRACKING_STEP_RATE_SLEW_PPS_S)
                    self.axis.set_step_rate(rate)
                    self.axis.set_target_degrees(current_degrees)
                else:
                    rate = slew_step_rate(self.axis.step_rate, desired_rate,
                                          elapsed_ms,
                                          config.TRACKING_STEP_RATE_SLEW_PPS_S)
                    self.axis.set_step_rate(rate)
                    self.axis.set_target_degrees(self.target_yaw)
            else:
                self.reversing = False
                self.axis.set_step_rate(desired_rate)
                self.axis.set_target_degrees(self.target_yaw)
        else:
            self.desired_rate = config.MIN_TRACKING_STEP_RATE
            self.reversing = False
            self.axis.set_step_rate(config.MIN_TRACKING_STEP_RATE)
            if getattr(config, "ENABLE_DEADBAND", True):
                self.axis.set_target_degrees(current_degrees)
            else:
                self.axis.set_target_degrees(self.target_yaw)

        if self.awake or self.moving:
            self.axis.set_enabled(True)
        else:
            self.axis.set_enabled(False)

    def format_frame_log(self, prefix, delta_steps, delta_deg):
        """Format a single-line status log for this frame."""
        if self.last_assigned_target is not None:
            x_mm, y_mm, speed_cm_s, _ = self.last_assigned_target
            tgt_str = "tgt=({:4d},{:4d},{:3d})".format(
                x_mm, y_mm, speed_cm_s)
            raw_str = "raw={:+5.1f}".format(self.last_raw_yaw)
        else:
            tgt_str = "tgt=None          "
            raw_str = "raw= None"

        sm_val = (self.smoothed_yaw if self.smoothed_yaw is not None
                  else config.PARK_YAW_DEG)
        sm_str = "sm={:+5.1f}".format(sm_val)

        cur_deg = self.axis.position_steps / self.axis.steps_per_degree
        pos_str = "pos={:+5.1f}".format(cur_deg)

        err_deg = self.target_yaw - cur_deg
        err_str = "err={:+5.1f}".format(err_deg)

        step_str = "step={:+4d} ({:+5.1f}deg)".format(int(delta_steps), delta_deg)
        rate_str = "pps={:4d}".format(int(self.axis.step_rate))
        brg_str = "brg={:4.1f}".format(self.filtered_yaw_rate)

        if self.awake:
            if self.last_assigned_target is not None:
                state = ("REV" if self.reversing
                         else ("TRK" if self.moving else "HOLD"))
            else:
                state = "COAST" if self.moving else "IDLE"
        else:
            state = "PARK" if self.moving else "SLEEP"

        return "{} {}: {} {} {} {} {} {} {} {} [{:5s}]".format(
            prefix, self.name, tgt_str, raw_str, sm_str, pos_str,
            err_str, step_str, rate_str, brg_str, state)

    def run(self):
        self.axis.run()

    def shutdown(self):
        self.step_pin.value(0)
        self.axis.set_enabled(False)
        set_eyes(self.eyes, config.EYE_OFF)


def main():
    if config.POSITION_MODE != "assume":
        raise RuntimeError("selected position mode is not implemented")
    print("Startup pose: every skull pan at {} deg".format(
        config.STARTUP_PAN_DEG))

    trackers = [Tracker(skull) for skull in config.SKULLS]
    tmc_uart = None
    radar_uart = None
    try:
        tmc_uart, drivers = find_tmc_drivers()
        print("All TMC2209s found at {} baud; configuring before motion.".
              format(config.TMC_UART_BAUDRATE))
        for skull in config.SKULLS:
            configure_driver(drivers[skull["name"]], skull["name"],
                             skull["run_current_ma"], skull["hold_current_ma"])

        radar_uart = UART(config.RADAR_UART_ID, baudrate=config.RADAR_BAUDRATE,
                          bits=8, parity=None, stop=1,
                          tx=config.RADAR_TX_PIN, rx=config.RADAR_RX_PIN)
        radar = LD2450Reader(radar_uart)

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
        if config.ENABLE_FRAME_LOGGING:
            print("Log columns: [#frame +dt_ms L:loops] Skull: tgt=(x,y,spd) "
                  "raw=deg sm=deg pos=deg err=deg step=N (deg) pps=rate brg=deg/s [STATE]")

        radar_frame_count = 0
        last_frame_ms = time.ticks_ms()
        last_update_ms = time.ticks_ms()
        loop_iterations = 0

        while True:
            loop_iterations += 1
            targets = radar.poll()
            now_ms = time.ticks_ms()
            if targets is not None:
                radar_frame_count += 1
                frame_dt_ms = time.ticks_diff(now_ms, last_frame_ms)
                last_frame_ms = now_ms

                # Record motor motion since previous radar frame
                moves = []
                for tracker in trackers:
                    steps = (tracker.axis.position_steps -
                             tracker.last_frame_steps)
                    tracker.last_frame_steps = tracker.axis.position_steps
                    deg = steps / tracker.axis.steps_per_degree
                    moves.append((steps, deg))

                assigned = choose_two_targets(targets,
                                              config.MIN_TARGET_RANGE_MM,
                                              config.MAX_TARGET_RANGE_MM,
                                              config.TARGET_ASSIGNMENT_MODE)
                for tracker, target in zip(trackers, assigned):
                    tracker.on_radar_frame(target, now_ms)

                update_elapsed_ms = time.ticks_diff(now_ms, last_update_ms)
                last_update_ms = now_ms
                for tracker in trackers:
                    tracker.update(now_ms, update_elapsed_ms)

                if config.ENABLE_FRAME_LOGGING:
                    decimation = getattr(config, "FRAME_LOG_DECIMATION", 1)
                    if decimation <= 1 or (radar_frame_count % decimation) == 0:
                        prefix = "[#{:04d} +{:3d}ms L:{:2d}]".format(
                            radar_frame_count, frame_dt_ms, loop_iterations)
                        for tracker, (steps, deg) in zip(trackers, moves):
                            print(tracker.format_frame_log(prefix, steps, deg))
                loop_iterations = 0

            update_elapsed_ms = time.ticks_diff(now_ms, last_update_ms)
            if update_elapsed_ms >= config.TRACKING_UPDATE_MS:
                last_update_ms = now_ms
                for tracker in trackers:
                    tracker.update(now_ms, update_elapsed_ms)

            for tracker in trackers:
                tracker.run()

            any_motion = any(
                tracker.moving
                or tracker.axis.position_steps != tracker.axis.target_steps
                or tracker.axis.step_high
                for tracker in trackers
            )
            if not any_motion:
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
