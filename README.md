# Halloween Skull Tracker

MicroPython project for an ESP32-WROOM-32E/uPesy board, Hi-Link LD2450 radar,
two BTT TMC2209 V1.3 stepper drivers, two NEMA17 motors, and two NeoPixel eye
LEDs. The intended prop parks looking down with eyes off, wakes when the radar
detects a person, and pans/tilts to follow a target within sensor and mechanism
limits.

## Current Status

The LD2450 UART stream has been tested on the ESP32 and produced valid target
frames at about 10 Hz. The pan TMC2209 replies at 57600 baud, address 0. UART setup
verified GCONF, accepted GLOBALSCALER and IHOLD_IRUN writes through IFCNT, and
read back CHOPCONF for 1/16 microsteps. A no-load pan jog visibly moved a tape
flag about 36 degrees out and back. Enable and STEP/DIR input diagnostics also
verified the corresponding TMC2209 IOIN bits.

The full skull tracker is **not ready to run**. In particular, `main.py` does
not yet initialize the TMC2209 over UART at startup. Current and microstep
settings are volatile and must be reapplied after driver power is cycled. The
tilt motor, assembled mechanism, and LEDs have not yet been commissioned.

## Hardware And Wiring

- MCU: uPesy ESP32 Low Power DevKit, ESP32-WROOM-32E. It was flashed with
  MicroPython v1.29.0 `ESP32_GENERIC`; this replaced the prior ESP-IDF image and
  custom partition table.
- Radar: Hi-Link LD2450, powered at 5 V, UART2 on GPIO16 RX / GPIO17 TX,
  256000 baud, 8 data bits, no parity, 1 stop bit. Radar TX connects to ESP32
  GPIO16; radar RX to GPIO17 is optional for this receive-only test. Share GND.
- Pan driver: BTT TMC2209 V1.3. STEP GPIO25, DIR GPIO26, active-low EN GPIO27,
  VIO to ESP32 3V3, GND to common ground, VM to a separate 12 V motor supply.
- Pan UART: ESP32 GPIO22 RX directly to the BTT `RX`/UART pad. GPIO23 TX
  connects to that same pad through 1 kOhm. Leave the driver's separate `TX`
  pad unconnected and leave R10 in its factory state. The driver responded at
  57600 baud; `config.py` reflects this.
- Pan motor: StepperOnline 17HE19-2004S, 1.8 degrees/full step, rated 2 A per
  phase. Black/blue are one coil (black A1, blue A2); red/green are the other
  (red B1, green B2). This wiring has been tested with the bare motor.
- Keep ESP32 GND, driver GND, and motor-supply negative common. Driver VM is
  not ESP32 5 V. The measured rails were 12.2 V VM and 3.29 V VIO.
- The LD2450 is mounted upside down relative to its manual. Empirically, Y was
  positive for a person in front, and X decreased as a target moved to the
  sensor's right. In this setup +X points toward the sensor's left.
- Do not put the BTT carrier into a solderless breadboard so that `EN`,
  `VREF`, and `DIAG` share a connected strip. Use direct wiring or an adapter
  that isolates these pins. VREF and DIAG are not connected for this project.

See [wiring.md](wiring.md) for the detailed proposed GPIO map and drawing. The
local [LD2450 manual](manuals/006000_HLK-LD2450-Instruction-Manual.pdf) describes
the UART protocol and the Windows-only `ICLM_MTT.exe` visualization tool; the
Mac workflow here uses `radar_test.py` instead. The local
[BTT TMC2209 V1.3 manual](manuals/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf)
documents the driver pad layout and R10 option.

## Setup On The Mac

Python packages are kept in the project virtual environment. It already
contains `mpremote`, `esptool`, and `black`; `.gitignore` excludes `venv/`.

```sh
source venv/bin/activate
python -m unittest discover -s tests
```

The ESP32 USB serial port used so far is `/dev/cu.SLAB_USBtoUART`. Examples
below use `./venv/bin/mpremote` so activation is not required. Stop any serial
monitor that owns the port before running an `mpremote` command.

The board filesystem needs `config.py` and the module(s) imported by the chosen
script. For example, deploy the pan UART setup files with:

```sh
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp config.py :config.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp tmc2209.py :tmc2209.py
```

`mpremote ... run file.py` executes the local script on the board; imports such
as `config` and `tmc2209` must already be present on the board.

## Script Guide

| Script/module                       | Purpose                                                                                                                                                                                      | Run/use                                                                                                                                                                                                       |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `radar_test.py`                     | Radar-only UART monitor. Prints valid frame counts and decoded targets once per second; does not touch motor pins.                                                                           | Upload `config.py` and `ld2450.py`, then run `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py`. Stop with Ctrl-C.                                                                        |
| `pan_driver_setup.py`               | Finds the pan TMC2209 by probing baud rates and addresses, configures UART mode/current/1/16 microsteps, and verifies writes with IFCNT plus readable-register checks. Sends no STEP pulses. | With motor driver powered and preferably motor disconnected during initial setup, upload `config.py` and `tmc2209.py`, then run `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_driver_setup.py`. |
| `pan_enable_test.py`                | Reads TMC IOIN and DRV_STATUS, toggles active-low EN, and reports enabled/current/standstill bits. Sends no STEP pulses; motor may hold briefly.                                             | Run only after UART setup, with the shaft clear: `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_enable_test.py`.                                                                                 |
| `pan_signal_test.py`                | Toggles GPIO25 STEP and GPIO26 DIR and reads those inputs back from TMC IOIN. Holds EN disabled and does not energize/step the motor.                                                        | `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_signal_test.py`.                                                                                                                                  |
| `pan_motor_test.py`                 | No-load pan jog: 320 pulses each direction at 100 pulses/s, approximately 36 degrees out and back at 1/16. Disables EN afterward.                                                            | Run only with current settings freshly applied and bare shaft clear: `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_motor_test.py`.                                                              |
| `main.py`                           | Intended full target-following application: radar target selection, aiming, two stepper axes, and red/off eye LEDs.                                                                          | **Do not use yet.** It does not apply TMC UART settings at startup; tilt/LED/mechanical integration and full motion safety are not tested.                                                                    |
| `ld2450.py`                         | Streaming LD2450 frame parser.                                                                                                                                                               | Imported by `radar_test.py` and `main.py`; not a standalone program.                                                                                                                                          |
| `tmc2209.py`                        | TMC2209 UART CRC, register access, current scaling, and configuration helpers.                                                                                                               | Imported by pan diagnostics/setup; not standalone.                                                                                                                                                            |
| `aiming.py`                         | Radar-to-yaw/pitch geometry and clamping.                                                                                                                                                    | Imported by `main.py`; host-tested.                                                                                                                                                                           |
| `tracking.py`                       | Selects the nearest valid target initially and then the nearest track to the previous one.                                                                                                   | Imported by `main.py`; host-tested.                                                                                                                                                                           |
| `stepper.py`                        | Non-blocking STEP/DIR axis control with configured angular limits.                                                                                                                           | Imported by `main.py`; not yet validated on the assembled mechanism.                                                                                                                                          |
| `tests/test_protocol_and_aiming.py` | Host tests for LD2450 frames/parser, geometry, target continuity, and TMC UART packets/current calculations.                                                                                 | `./venv/bin/python -m unittest discover -s tests`.                                                                                                                                                            |

## Pan Driver Settings And Test Results

The tested pan configuration is:

- UART 57600 baud, slave address 0. The setup script tries the configured rate
  first, then common fallbacks including 115200, 57600, 38400, 19200, and 9600;
  duplicate baud rates are skipped. The replacement answered at 57600.
- `GCONF=0x000001c0`: UART enabled for configuration, register-selected
  microsteps, and analog VREF current scaling disabled.
- `GLOBALSCALER=0` (full scale), accepted according to `IFCNT`.
- `IHOLD_IRUN=0x00080d06`: IRUN code 13 and IHOLD code 6, calculated as about
  0.77 A RMS run and 0.39 A RMS hold using the configured 0.11-ohm sense
  resistor assumption. This is a conservative initial setting for the 2 A/phase
  motor, not its maximum rating.
- `CHOPCONF=0x14010053`: MRES readback confirms 1/16 microsteps, or 3,200
  STEP pulses per revolution for a typical 1.8-degree motor.
- `IHOLD_IRUN` is treated as write-only; the utility verifies acceptance by
  checking `IFCNT`, not by reading the register back. `CHOPCONF` and `GCONF`
  are read back.

The BTT manual's “256 microsteps” is a capability, not the active input step
resolution. The 0.7 V VREF pot setting is not controlling current under the
tested `GCONF` because `I_SCALE_ANALOG` is cleared. UART settings are volatile
and reset when driver power is cycled. Re-run `pan_driver_setup.py` after each
VM power cycle until equivalent initialization is added to `main.py`.

Verified pan tests on the current replacement driver:

- UART returned `IOIN=0x21000041` at 57600 baud/address 0.
- `pan_enable_test.py` changed ENN from high to low and back; driver status
  reported `CS_ACTUAL=6` while enabled and standstill set.
- `pan_signal_test.py` observed STEP and DIR change in `IOIN` as GPIO25/26
  were toggled, with EN disabled.
- The original 160-pulse jog was hard to see by eye. The revised 320-pulse
  jog ran successfully; the user observed a tape flag move.
- A first TMC2209 did not reply and was replaced. Whether the first module was
  damaged is unknown. Earlier solderless-breadboard wiring may have shorted
  EN/VREF/DIAG together; that carrier arrangement is not used now.
- Current host suite: 11 tests pass. These do not replace physical validation.

## Tracker Behavior And Configuration

`config.py` is the source of GPIOs, ranges, angles, and geometry. The LD2450
reports X/Y and speed, but no target Z; `main.py` estimates Z from the desired
target height minus the radar height. Starter values assume sensor height
1000 mm and target height 1200 mm above floor; measure the actual installation.
Pan starts at +/-60 degrees, approximately the radar's horizontal field of
view. Tilt limits and all pivot offsets are provisional and must be set for
the physical assembly.

Startup mode is currently `POSITION_MODE = "assume"`: the prop must be
physically positioned at the configured startup angles before power-up. There
are no homing switches or implemented StallGuard homing. The LD2450's 6 m
range is not a mechanical travel limit. The full application also needs to
initialize both drivers, handle startup pose safely, and validate tilt/LED
hardware before it can be considered ready.
