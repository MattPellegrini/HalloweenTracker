"""Check pan-driver enable/current status without issuing STEP pulses."""

import time
from machine import Pin, UART

import config
from tmc2209 import (REG_DRV_STATUS, REG_IOIN, TMC2209)


UART_BAUDRATE = 57600
IOIN_ENN_BIT = 1 << 0
DRV_STATUS_CS_ACTUAL_MASK = 0x1f << 16
DRV_STATUS_STST_BIT = 1 << 31


def read_status(driver, label):
    ioin = driver.read_register(REG_IOIN)
    status = driver.read_register(REG_DRV_STATUS)
    enabled_pin = not bool(ioin & IOIN_ENN_BIT)
    current_scale = (status & DRV_STATUS_CS_ACTUAL_MASK) >> 16
    standstill = bool(status & DRV_STATUS_STST_BIT)
    print("{}: ENN={} driver_enabled={} cs_actual={} standstill={} "
          "IOIN=0x{:08x} DRV_STATUS=0x{:08x}".format(
              label, bool(ioin & IOIN_ENN_BIT), enabled_pin, current_scale,
              standstill, ioin, status))


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

    read_status(driver, "disabled")
    enable_pin.value(0)
    time.sleep_ms(500)
    read_status(driver, "enabled")
    enable_pin.value(1)
    time.sleep_ms(20)
    read_status(driver, "disabled_again")
    step_pin.value(0)
    direction_pin.value(0)
    print("No STEP pulses were sent.")


main()