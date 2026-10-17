"""No-load motor test; verifies signal wiring and driver status, spins each skull motor:
1) 360 degrees forward and back at constant test speed,
2) 30 degrees forward and back at minimum tracking speed,
3) 360 degrees forward and back with slewed acceleration and deceleration
   abiding by config (MIN_TRACKING_STEP_RATE -> MAX_STEP_RATE -> MIN_TRACKING_STEP_RATE via TRACKING_STEP_RATE_SLEW_PPS_S).

Incorporates STEP/DIR line testing (via IOIN register readback while disabled)
and driver enable/current/standstill status checks before and after motion.
Pass '--check-only' or '--signals-only' on the command line to skip physical motion.
"""

import math
import sys
import time
from machine import Pin

import config
from tmc2209 import REG_DRV_STATUS, REG_GLOBALSCALER, REG_IOIN, TMC2209


TEST_STEP_PERIOD_US = 1000
STEPS_PER_REVOLUTION = config.FULL_STEPS_PER_REVOLUTION * config.MICROSTEPS
MIN_SPEED_STEP_PERIOD_US = int(1000000 / config.MIN_TRACKING_STEP_RATE)
STEPS_30_DEG = round(STEPS_PER_REVOLUTION * 30.0 / 360.0)

IOIN_ENN_BIT = 1 << 0
IOIN_STEP_BIT = 1 << 7
IOIN_DIR_BIT = 1 << 9
DRV_STATUS_CS_ACTUAL_MASK = 0x1F << 16
DRV_STATUS_STST_BIT = 1 << 31


def configure_tmc_drivers():
    """Attempt to configure TMC2209 drivers via shared UART if available."""
    try:
        from machine import UART
    except ImportError:
        return None, {}

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
    except Exception as error:
        print("TMC UART open failed: {}".format(error))
        return None, {}

    drivers = {}
    for skull in config.SKULLS:
        address = skull.get("tmc_address")
        if address is None:
            continue
        try:
            driver = TMC2209(uart, slave_address=address)
            ioin = driver.read_register(REG_IOIN)
            if (ioin >> 24) != 0x21:
                print(
                    "{}: unexpected TMC2209 version IOIN=0x{:08x}".format(
                        skull["name"], ioin
                    )
                )
                continue
            driver.configure_uart_mode()
            driver.write_register_confirmed(REG_GLOBALSCALER, 0)
            driver.configure_current(
                skull["run_current_ma"],
                skull["hold_current_ma"],
                config.TMC_SENSE_RESISTOR_OHMS,
                config.TMC_VSENSE_VOLTS,
            )
            driver.configure_microsteps(config.MICROSTEPS)
            drivers[skull["name"]] = driver
            print(
                "{}: TMC2209 configured at address {} (1/{} microsteps, {} mA run / {} mA hold)".format(
                    skull["name"],
                    address,
                    config.MICROSTEPS,
                    skull["run_current_ma"],
                    skull["hold_current_ma"],
                )
            )
        except Exception as error:
            print(
                "Could not configure TMC for {} at address {}: {}".format(
                    skull["name"], address, error
                )
            )

    return uart, drivers


def verify_signal_wiring(driver, step_pin, dir_pin):
    """Verify STEP and DIR pin toggles reach TMC2209 inputs while EN is disabled."""
    try:
        step_pin.value(0)
        dir_pin.value(0)
        time.sleep_ms(5)

        baseline = driver.read_register(REG_IOIN)
        enn_disabled = bool(baseline & IOIN_ENN_BIT)

        # STEP high test
        step_pin.value(1)
        time.sleep_ms(5)
        step_high = driver.read_register(REG_IOIN)
        step_high_ok = bool(step_high & IOIN_STEP_BIT)

        # STEP low test
        step_pin.value(0)
        time.sleep_ms(5)
        step_low = driver.read_register(REG_IOIN)
        step_low_ok = not bool(step_low & IOIN_STEP_BIT)

        # DIR high test
        dir_pin.value(1)
        time.sleep_ms(5)
        dir_high = driver.read_register(REG_IOIN)
        dir_high_ok = bool(dir_high & IOIN_DIR_BIT)

        # DIR low test
        dir_pin.value(0)
        time.sleep_ms(5)
        dir_low = driver.read_register(REG_IOIN)
        dir_low_ok = not bool(dir_low & IOIN_DIR_BIT)

        all_ok = step_high_ok and step_low_ok and dir_high_ok and dir_low_ok
        if all_ok:
            print(
                "  Signal test PASSED: STEP & DIR pins toggle driver IOIN (ENN={})".format(
                    int(enn_disabled)
                )
            )
        else:
            print(
                "  Signal test FAILED: STEP_high={} STEP_low={} DIR_high={} DIR_low={}".format(
                    step_high_ok, step_low_ok, dir_high_ok, dir_low_ok
                )
            )
        return all_ok
    except Exception as error:
        print("  Signal test error reading TMC registers: {}".format(error))
        return False


def read_driver_status(driver, label):
    """Read and report ENN, actual current scale, and standstill status."""
    try:
        ioin = driver.read_register(REG_IOIN)
        status = driver.read_register(REG_DRV_STATUS)
        enn_pin = bool(ioin & IOIN_ENN_BIT)
        enabled = not enn_pin
        current_scale = (status & DRV_STATUS_CS_ACTUAL_MASK) >> 16
        standstill = bool(status & DRV_STATUS_STST_BIT)
        print(
            "  [{}] ENN={} enabled={} cs_actual={} standstill={} (IOIN=0x{:08x} DRV_STATUS=0x{:08x})".format(
                label, enn_pin, enabled, current_scale, standstill, ioin, status
            )
        )
        return {
            "enn": enn_pin,
            "enabled": enabled,
            "cs_actual": current_scale,
            "standstill": standstill,
            "ioin": ioin,
            "status": status,
        }
    except Exception as error:
        print("  [{}] Failed to read driver status: {}".format(label, error))
        return None


def move_steps(
    step_pin, direction_pin, direction, count, step_period_us=TEST_STEP_PERIOD_US
):
    direction_pin.value(direction)
    time.sleep_ms(20)
    delay_us = step_period_us // 2
    for _ in range(count):
        step_pin.value(1)
        time.sleep_us(delay_us)
        step_pin.value(0)
        time.sleep_us(delay_us)


def move_steps_slewed(
    step_pin,
    direction_pin,
    direction,
    count,
    min_rate=config.MIN_TRACKING_STEP_RATE,
    max_rate=config.MAX_STEP_RATE,
    slew_pps_s=config.TRACKING_STEP_RATE_SLEW_PPS_S,
):
    """Move steps with acceleration and deceleration ramping obeying min/max speed and slew."""
    if count <= 0:
        return
    direction_pin.value(direction)
    time.sleep_ms(20)

    if slew_pps_s <= 0 or max_rate <= min_rate:
        delay_us = max(1, int(1000000 / max_rate))
        half_delay1 = delay_us // 2
        half_delay2 = delay_us - half_delay1
        for _ in range(count):
            step_pin.value(1)
            time.sleep_us(half_delay1)
            step_pin.value(0)
            time.sleep_us(half_delay2)
        return

    accel_steps = int((max_rate * max_rate - min_rate * min_rate) / (2.0 * slew_pps_s))
    if accel_steps * 2 > count:
        accel_steps = count // 2

    ramp_delays_us = []
    min_rate_sq = float(min_rate * min_rate)
    two_a = 2.0 * slew_pps_s
    for i in range(accel_steps):
        rate = math.sqrt(min_rate_sq + two_a * i)
        rate = max(float(min_rate), min(float(max_rate), rate))
        ramp_delays_us.append(max(1, int(1000000.0 / rate)))

    cruise_delay_us = max(1, int(1000000.0 / max_rate))
    decel_start = count - accel_steps

    for step_index in range(count):
        if step_index < accel_steps:
            period_us = ramp_delays_us[step_index]
        elif step_index >= decel_start:
            decel_index = count - 1 - step_index
            period_us = ramp_delays_us[decel_index]
        else:
            period_us = cruise_delay_us

        half_delay1 = period_us // 2
        half_delay2 = period_us - half_delay1
        step_pin.value(1)
        time.sleep_us(half_delay1)
        step_pin.value(0)
        time.sleep_us(half_delay2)


def main():
    check_only = any(
        arg in sys.argv for arg in ("--check-only", "--signals-only", "--no-motion")
    )

    enable_inactive = 1 if config.ENABLE_ACTIVE_LOW else 0
    enable_active = 0 if config.ENABLE_ACTIVE_LOW else 1

    # Pre-initialize pins for all skulls to safe/disabled state
    skull_controls = []
    for skull in config.SKULLS:
        step_pin = Pin(skull["step_pin"], Pin.OUT, value=0)
        dir_pin = Pin(skull["dir_pin"], Pin.OUT, value=0)
        enable_pin = Pin(skull["enable_pin"], Pin.OUT, value=enable_inactive)
        skull_controls.append(
            {
                "skull": skull,
                "step_pin": step_pin,
                "dir_pin": dir_pin,
                "enable_pin": enable_pin,
            }
        )

    revolution_time_s = (STEPS_PER_REVOLUTION * TEST_STEP_PERIOD_US) / 1000000.0
    speed_deg_s = 360.0 / revolution_time_s if revolution_time_s > 0 else 0.0
    min_speed_deg_s = (config.MIN_TRACKING_STEP_RATE * 360.0) / STEPS_PER_REVOLUTION
    max_speed_deg_s = (config.MAX_STEP_RATE * 360.0) / STEPS_PER_REVOLUTION
    min_speed_30_deg_time_s = STEPS_30_DEG / config.MIN_TRACKING_STEP_RATE

    accel_steps = int(
        (config.MAX_STEP_RATE**2 - config.MIN_TRACKING_STEP_RATE**2)
        / (2.0 * config.TRACKING_STEP_RATE_SLEW_PPS_S)
    )
    accel_time_s = (
        config.MAX_STEP_RATE - config.MIN_TRACKING_STEP_RATE
    ) / config.TRACKING_STEP_RATE_SLEW_PPS_S
    cruise_steps = max(0, STEPS_PER_REVOLUTION - 2 * accel_steps)
    cruise_time_s = cruise_steps / config.MAX_STEP_RATE
    slewed_360_time_s = (2.0 * accel_time_s) + cruise_time_s

    print(
        "Motor test: {} skull(s), {} steps per 360-deg revolution (1/{} microsteps)".format(
            len(config.SKULLS), STEPS_PER_REVOLUTION, config.MICROSTEPS
        )
    )
    if check_only:
        print("Mode: CHECK ONLY (motion skipped; testing signals and enable states)")
    else:
        print(
            "Motion tests per skull:\n"
            "  1) 360 deg constant: {:.1f} deg/s ({:.2f} s)\n"
            "  2) 30 deg min speed: {:.1f} deg/s ({} pps, {:.2f} s)\n"
            "  3) 360 deg slewed:   {} -> {} -> {} pps ({:.1f} to {:.1f} deg/s, slew {} pps/s, {:.2f} s)".format(
                speed_deg_s,
                revolution_time_s,
                min_speed_deg_s,
                config.MIN_TRACKING_STEP_RATE,
                min_speed_30_deg_time_s,
                config.MIN_TRACKING_STEP_RATE,
                config.MAX_STEP_RATE,
                config.MIN_TRACKING_STEP_RATE,
                min_speed_deg_s,
                max_speed_deg_s,
                config.TRACKING_STEP_RATE_SLEW_PPS_S,
                slewed_360_time_s,
            )
        )
    print("Keep motor shafts and linkages clear.\n")

    tmc_uart, drivers = configure_tmc_drivers()

    try:
        for index, item in enumerate(skull_controls, start=1):
            skull = item["skull"]
            step_pin = item["step_pin"]
            dir_pin = item["dir_pin"]
            enable_pin = item["enable_pin"]
            driver = drivers.get(skull["name"])

            print(
                "\n[{}/{}] Testing {}:".format(index, len(skull_controls), skull["name"])
            )

            # 1. STEP/DIR Signal line test while disabled (from pan_signal_test.py)
            if driver is not None:
                print("  Checking STEP/DIR electrical signals via IOIN readback (disabled)...")
                signals_ok = verify_signal_wiring(driver, step_pin, dir_pin)
                if not signals_ok:
                    print(
                        "  WARNING: Signal verification failed for {}! Check STEP/DIR wiring.".format(
                            skull["name"]
                        )
                    )
                read_driver_status(driver, "pre-enable status")

            # 2. Driver enable & current status check (from pan_enable_test.py)
            print("  Enabling driver...")
            enable_pin.value(enable_active)
            time.sleep_ms(100)

            if driver is not None:
                read_driver_status(driver, "enabled status")

            # 3. Motion test (if not in check-only mode)
            if not check_only:
                inverted = skull.get("direction_inverted", False)
                fwd_dir = 1 if inverted else 0
                rev_dir = 0 if inverted else 1

                # Test 1: Constant-rate full revolution
                print(
                    "  [1/3] Spinning 360 degrees forward at constant test speed ({} steps)...".format(
                        STEPS_PER_REVOLUTION
                    )
                )
                move_steps(step_pin, dir_pin, fwd_dir, STEPS_PER_REVOLUTION)

                print("  Pausing 500 ms...")
                time.sleep_ms(500)

                print(
                    "  [1/3] Spinning 360 degrees back at constant test speed ({} steps)...".format(
                        STEPS_PER_REVOLUTION
                    )
                )
                move_steps(step_pin, dir_pin, rev_dir, STEPS_PER_REVOLUTION)

                print("  Pausing 500 ms...")
                time.sleep_ms(500)

                # Test 2: Min-speed jog (30 degrees)
                print(
                    "  [2/3] Moving 30 degrees forward at min speed ({} pps, {} steps)...".format(
                        config.MIN_TRACKING_STEP_RATE, STEPS_30_DEG
                    )
                )
                move_steps(
                    step_pin, dir_pin, fwd_dir, STEPS_30_DEG, MIN_SPEED_STEP_PERIOD_US
                )

                print("  Pausing 500 ms...")
                time.sleep_ms(500)

                print(
                    "  [2/3] Moving 30 degrees back at min speed ({} pps, {} steps)...".format(
                        config.MIN_TRACKING_STEP_RATE, STEPS_30_DEG
                    )
                )
                move_steps(
                    step_pin, dir_pin, rev_dir, STEPS_30_DEG, MIN_SPEED_STEP_PERIOD_US
                )

                print("  Pausing 500 ms...")
                time.sleep_ms(500)

                # Test 3: Slewed full revolution (abiding by min speed, max speed, slew rate)
                print(
                    "  [3/3] Full 360-deg rotation forward with slew ({} -> {} -> {} pps, {} steps)...".format(
                        config.MIN_TRACKING_STEP_RATE,
                        config.MAX_STEP_RATE,
                        config.MIN_TRACKING_STEP_RATE,
                        STEPS_PER_REVOLUTION,
                    )
                )
                move_steps_slewed(
                    step_pin,
                    dir_pin,
                    fwd_dir,
                    STEPS_PER_REVOLUTION,
                    min_rate=config.MIN_TRACKING_STEP_RATE,
                    max_rate=config.MAX_STEP_RATE,
                    slew_pps_s=config.TRACKING_STEP_RATE_SLEW_PPS_S,
                )

                print("  Pausing 500 ms...")
                time.sleep_ms(500)

                print(
                    "  [3/3] Full 360-deg rotation back with slew ({} -> {} -> {} pps, {} steps)...".format(
                        config.MIN_TRACKING_STEP_RATE,
                        config.MAX_STEP_RATE,
                        config.MIN_TRACKING_STEP_RATE,
                        STEPS_PER_REVOLUTION,
                    )
                )
                move_steps_slewed(
                    step_pin,
                    dir_pin,
                    rev_dir,
                    STEPS_PER_REVOLUTION,
                    min_rate=config.MIN_TRACKING_STEP_RATE,
                    max_rate=config.MAX_STEP_RATE,
                    slew_pps_s=config.TRACKING_STEP_RATE_SLEW_PPS_S,
                )
            else:
                print("  [Check-only mode: motion skipped]")

            # 4. Disable driver and verify return to standstill / disabled state
            print("  Disabling driver.")
            enable_pin.value(enable_inactive)
            step_pin.value(0)
            time.sleep_ms(20)

            if driver is not None:
                read_driver_status(driver, "disabled status")

            if index < len(skull_controls):
                time.sleep_ms(500)

        print("\nAll skull tests completed; all drivers disabled.")
    finally:
        for item in skull_controls:
            item["enable_pin"].value(enable_inactive)
            item["step_pin"].value(0)
        if tmc_uart is not None:
            tmc_uart.deinit()


main()
