"""Verify pan STEP/DIR reach TMC2209 inputs while motor outputs are disabled."""

import time
from machine import Pin, UART

import config
from tmc2209 import REG_IOIN, TMC2209


UART_BAUDRATE = 57600
IOIN_STEP_BIT = 1 << 7
IOIN_DIR_BIT = 1 << 9


def sample(driver, label):
    value = driver.read_register(REG_IOIN)
    step = 1 if value & IOIN_STEP_BIT else 0
    direction = 1 if value & IOIN_DIR_BIT else 0
    print("{}: STEP={} DIR={} IOIN=0x{:08x}".format(
        label, step, direction, value))
    return step, direction


def main():
    uart = UART(config.TMC_UART_ID,
                baudrate=UART_BAUDRATE,
                bits=8,
                parity=None,
                stop=1,
                tx=Pin(config.TMC_UART_TX_PIN),
                rx=Pin(config.TMC_UART_RX_PIN))
    driver = TMC2209(uart, slave_address=0)
    step_pin = Pin(config.PAN_STEP_PIN, Pin.OUT, value=0)
    direction_pin = Pin(config.PAN_DIR_PIN, Pin.OUT, value=0)
    enable_pin = Pin(config.PAN_ENABLE_PIN, Pin.OUT, value=1)

    sample(driver, "baseline, disabled")
    step_pin.value(1)
    time.sleep_ms(5)
    sample(driver, "STEP high, disabled")
    step_pin.value(0)
    time.sleep_ms(5)
    sample(driver, "STEP low, disabled")
    direction_pin.value(1)
    time.sleep_ms(5)
    sample(driver, "DIR high, disabled")
    direction_pin.value(0)
    time.sleep_ms(5)
    sample(driver, "DIR low, disabled")
    enable_pin.value(1)
    print("Driver remained disabled; no steps were issued.")


main()