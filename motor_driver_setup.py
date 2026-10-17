"""Configure and verify TMC2209 drivers for each skull; this script never steps a motor."""

import time
from machine import Pin, UART

import config
from tmc2209 import REG_GLOBALSCALER, REG_IOIN, TMC2209


def configure_skull_driver(driver, skull):
    name = skull["name"]
    address = skull["tmc_address"]

    ioin = driver.read_register(REG_IOIN)
    version = ioin >> 24
    if version != 0x21:
        raise OSError(
            "Unexpected {} TMC2209 version: IOIN=0x{:08x} (expected version 0x21)".format(
                name, ioin
            )
        )
    print("\n[{}] TMC2209 replied at address {}; IOIN=0x{:08x}".format(name, address, ioin))

    gconf = driver.configure_uart_mode()
    print("  Verified GCONF=0x{:08x} (UART current & microstep mode)".format(gconf))

    # GLOBALSCALER=0 means full scale (256) on the TMC2209.
    scaler_ifcnt = driver.write_register_confirmed(REG_GLOBALSCALER, 0)
    print("  GLOBALSCALER write accepted; IFCNT={}".format(scaler_ifcnt))

    run_cs, hold_cs, current_register, current_ifcnt = driver.configure_current(
        skull["run_current_ma"],
        skull["hold_current_ma"],
        config.TMC_SENSE_RESISTOR_OHMS,
        config.TMC_VSENSE_VOLTS,
    )
    print(
        "  IHOLD_IRUN=0x{:08x} accepted (IRUN={}, IHOLD={}, IFCNT={}) -> {} mA run / {} mA hold".format(
            current_register,
            run_cs,
            hold_cs,
            current_ifcnt,
            skull["run_current_ma"],
            skull["hold_current_ma"],
        )
    )

    chopconf = driver.configure_microsteps(config.MICROSTEPS)
    print(
        "  Verified CHOPCONF=0x{:08x}; MRES configured for 1/{} microsteps".format(
            chopconf, config.MICROSTEPS
        )
    )


def main():
    uart = None
    try:
        uart = UART(
            config.TMC_UART_ID,
            baudrate=config.TMC_UART_BAUDRATE,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(config.TMC_UART_TX_PIN),
            rx=Pin(config.TMC_UART_RX_PIN),
        )
        time.sleep_ms(50)
        print(
            "Connected to TMC UART (ID {}) at {} baud".format(
                config.TMC_UART_ID, config.TMC_UART_BAUDRATE
            )
        )
        print("Configuring {} skull driver(s)...".format(len(config.SKULLS)))

        configured = 0
        for skull in config.SKULLS:
            name = skull["name"]
            address = skull.get("tmc_address")
            if address is None:
                print("Skipping {}: no tmc_address defined".format(name))
                continue

            driver = TMC2209(uart, slave_address=address)
            try:
                configure_skull_driver(driver, skull)
                configured += 1
            except OSError as error:
                print(
                    "\n[{}] Failed to communicate with driver at address {}: {}".format(
                        name, address, error
                    )
                )

        print(
            "\nSetup summary: {}/{} driver(s) configured successfully.".format(
                configured, len(config.SKULLS)
            )
        )
        print("No motor motion was commanded. Power-cycle resets volatile settings;")
        print(
            "run this before manual motor tests or run main.py (which configures drivers automatically)."
        )
    finally:
        if uart is not None:
            uart.deinit()


main()
