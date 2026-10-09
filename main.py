"""LD2450-tracked Halloween skull controller for ESP32 MicroPython."""

import time
from machine import Pin, UART
import neopixel

import config
from aiming import clamp, solve_angles
from ld2450 import LD2450Reader
from stepper import StepperAxis
from tracking import TargetSelector


def make_axis(step_pin, direction_pin, enable_pin, minimum, maximum,
              initial_degrees):
    steps_per_revolution = (config.FULL_STEPS_PER_REVOLUTION *
                            config.MICROSTEPS)
    return StepperAxis(
        Pin(step_pin, Pin.OUT),
        Pin(direction_pin, Pin.OUT),
        Pin(enable_pin, Pin.OUT),
        steps_per_revolution,
        minimum,
        maximum,
        config.MAX_STEP_RATE,
        config.ENABLE_ACTIVE_LOW,
        initial_degrees,
    )


def set_eyes(eyes, color):
    for index in range(config.NEOPIXEL_COUNT):
        eyes[index] = color
    eyes.write()


def main():
    pan = make_axis(config.PAN_STEP_PIN, config.PAN_DIR_PIN,
                    config.PAN_ENABLE_PIN, config.PAN_MIN_DEG,
                    config.PAN_MAX_DEG, config.STARTUP_PAN_DEG)
    tilt = make_axis(config.TILT_STEP_PIN, config.TILT_DIR_PIN,
                     config.TILT_ENABLE_PIN, config.TILT_MIN_DEG,
                     config.TILT_MAX_DEG, config.STARTUP_TILT_DEG)
    eyes = neopixel.NeoPixel(Pin(config.NEOPIXEL_PIN), config.NEOPIXEL_COUNT)
    set_eyes(eyes, config.EYE_OFF)

    if (config.POSITION_MODE != "assume" or
            not config.DRIVER_MICROSTEPS_CONFIRMED):
        if config.POSITION_MODE != "assume":
            print("Motion disabled: selected homing mode is not implemented")
        else:
            print("Motion disabled: verify driver microsteps in config.py")
        while True:
            pan.run()
            tilt.run()
            time.sleep_ms(10)

    uart = UART(config.RADAR_UART_ID, baudrate=config.RADAR_BAUDRATE,
                bits=8, parity=None, stop=1,
                tx=config.RADAR_TX_PIN, rx=config.RADAR_RX_PIN)
    radar = LD2450Reader(uart)
    selector = TargetSelector(config.MIN_TARGET_RANGE_MM,
                              config.MAX_TARGET_RANGE_MM)
    pan_pivot = (config.PAN_PIVOT_FROM_SENSOR_X_MM,
                 config.PAN_PIVOT_FROM_SENSOR_Y_MM,
                 config.PAN_PIVOT_FROM_SENSOR_Z_MM)
    tilt_offset = (config.TILT_PIVOT_FROM_PAN_X_MM,
                   config.TILT_PIVOT_FROM_PAN_Y_MM,
                   config.TILT_PIVOT_FROM_PAN_Z_MM)
    current_target = None
    last_target_ms = None
    last_update_ms = time.ticks_ms()
    awake = False

    while True:
        frame_targets = radar.poll()
        now_ms = time.ticks_ms()
        if frame_targets is not None:
            selected = selector.choose(frame_targets)
            if selected is not None:
                current_target = selected
                last_target_ms = now_ms

        if time.ticks_diff(now_ms, last_update_ms) >= config.TRACKING_UPDATE_MS:
            last_update_ms = now_ms
            target_expired = (last_target_ms is None or
                              time.ticks_diff(now_ms, last_target_ms) >
                              config.TARGET_LOST_MS)
            if target_expired:
                current_target = None
                selector.previous = None

            if current_target is None:
                pan.set_target_degrees(config.PARK_YAW_DEG)
                tilt.set_target_degrees(config.PARK_PITCH_DEG)
                if awake:
                    set_eyes(eyes, config.EYE_OFF)
                    awake = False
            else:
                x_mm, y_mm, speed_cm_s, resolution_mm = current_target
                target_z = (config.TARGET_HEIGHT_ABOVE_FLOOR_MM -
                            config.SENSOR_HEIGHT_ABOVE_FLOOR_MM)
                yaw, pitch = solve_angles(x_mm, y_mm, target_z,
                                          pan_pivot, tilt_offset)
                pan.set_target_degrees(clamp(yaw, config.PAN_MIN_DEG,
                                             config.PAN_MAX_DEG))
                tilt.set_target_degrees(clamp(pitch, config.TILT_MIN_DEG,
                                              config.TILT_MAX_DEG))
                if not awake:
                    set_eyes(eyes, config.EYE_RED)
                    awake = True

        pan.run()
        tilt.run()
        time.sleep_ms(1)


main()