"""LD2450-only diagnostic for ESP32 MicroPython; no motor pins are used."""

import time
from machine import UART

import config
from ld2450 import LD2450Reader


def main():
    uart = UART(config.RADAR_UART_ID, baudrate=config.RADAR_BAUDRATE,
                bits=8, parity=None, stop=1,
                tx=config.RADAR_TX_PIN, rx=config.RADAR_RX_PIN)
    radar = LD2450Reader(uart)
    frames = 0
    latest_targets = None
    last_report_ms = time.ticks_ms()

    print("LD2450 test: UART{} RX=GPIO{} TX=GPIO{} baud={}".format(
        config.RADAR_UART_ID, config.RADAR_RX_PIN, config.RADAR_TX_PIN,
        config.RADAR_BAUDRATE))

    while True:
        targets = radar.poll()
        if targets is not None:
            frames += 1
            latest_targets = targets

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_report_ms) >= 1000:
            if frames == 0:
                print("No valid frames yet; check radar power, TX-to-GPIO16, "
                      "GND, and 256000 baud")
            else:
                print("frames={} target_count={} targets={}".format(
                    frames, len(latest_targets), latest_targets))
            last_report_ms = now_ms

        time.sleep_ms(1)


main()