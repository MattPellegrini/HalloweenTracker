"""No-load pan motor jog; moves about 36 degrees each way, then disables."""

import time
from machine import Pin

import config


TEST_STEP_COUNT = 320
TEST_STEP_PERIOD_US = 10000


def move_steps(step_pin, direction_pin, direction, count):
    direction_pin.value(direction)
    time.sleep_ms(20)
    for _ in range(count):
        step_pin.value(1)
        time.sleep_us(TEST_STEP_PERIOD_US // 2)
        step_pin.value(0)
        time.sleep_us(TEST_STEP_PERIOD_US // 2)


def main():
    step_pin = Pin(config.PAN_STEP_PIN, Pin.OUT, value=0)
    direction_pin = Pin(config.PAN_DIR_PIN, Pin.OUT, value=0)
    enable_pin = Pin(config.PAN_ENABLE_PIN, Pin.OUT, value=1)

    print("Pan jog: 320 pulses each way, about 36 degrees at 1/16 steps")
    print("Speed: about 100 pulses/second")
    print("Keep the bare motor secure and its shaft clear.")
    try:
        enable_pin.value(0)
        time.sleep_ms(100)
        move_steps(step_pin, direction_pin, 0, TEST_STEP_COUNT)
        time.sleep_ms(300)
        move_steps(step_pin, direction_pin, 1, TEST_STEP_COUNT)
    finally:
        step_pin.value(0)
        enable_pin.value(1)
    print("Pan jog complete; driver disabled.")


main()