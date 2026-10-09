"""Configure and verify the pan BTT TMC2209; this script never steps a motor."""

from machine import Pin, UART

import config
from tmc2209 import (REG_GLOBALSCALER, REG_IOIN, TMC2209)


def main():
    driver = None
    uart = None
    baudrates = []
    for candidate_baudrate in (config.TMC_UART_BAUDRATE, 115200, 57600,
                               38400, 19200, 9600):
        if candidate_baudrate not in baudrates:
            baudrates.append(candidate_baudrate)

    for baudrate in baudrates:
        if uart is not None:
            uart.deinit()
        uart = UART(config.TMC_UART_ID,
                    baudrate=baudrate,
                    bits=8,
                    parity=None,
                    stop=1,
                    tx=Pin(config.TMC_UART_TX_PIN),
                    rx=Pin(config.TMC_UART_RX_PIN))
        print("Probing TMC UART at {} baud".format(baudrate))
        for address in range(4):
            candidate = TMC2209(uart, slave_address=address)
            try:
                ioin = candidate.read_register(REG_IOIN)
            except OSError as error:
                print("  No reply at address {}: {}".format(address, error))
            else:
                driver = candidate
                print("TMC2209 UART replied at {} baud, address {}; "
                      "IOIN=0x{:08x}".format(baudrate, address, ioin))
                break
        if driver is not None:
            break

    if driver is None:
        raise OSError("TMC2209 did not reply at any tested baud/address; "
                      "no settings were written")

    gconf = driver.configure_uart_mode()
    print("Verified GCONF=0x{:08x} for UART-controlled current and microsteps".
          format(gconf))

    # GLOBALSCALER=0 means full scale (256) on the TMC2209.
    scaler_ifcnt = driver.write_register_confirmed(REG_GLOBALSCALER, 0)
    print("GLOBALSCALER write accepted; IFCNT={}".format(scaler_ifcnt))

    run_cs, hold_cs, current_register, current_ifcnt = driver.configure_current(
        config.PAN_RUN_CURRENT_MA,
        config.PAN_HOLD_CURRENT_MA,
        config.TMC_SENSE_RESISTOR_OHMS,
        config.TMC_VSENSE_VOLTS,
    )
    chopconf = driver.configure_microsteps(config.MICROSTEPS)
    print("IHOLD_IRUN write accepted by IFCNT; requested value=0x{:08x} "
          "(IRUN={}, IHOLD={}, IFCNT={})".format(
              current_register, run_cs, hold_cs, current_ifcnt))
    print("Verified CHOPCONF=0x{:08x}; MRES configured for {} microsteps".format(
        chopconf, config.MICROSTEPS))
    print("No motor motion was commanded. Power-cycle resets these settings;")
    print("do not run the skull controller until startup config is integrated.")


main()