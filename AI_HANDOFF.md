# Halloween Skull Tracker: AI Handoff

Updated: 2026-10-10. Read this together with `README.md`, `wiring.md`, and `config.py` before advising on hardware or changing motor behavior. Treat observed tests below as stronger evidence than earlier conversational guesses. Do not assume untested tilt, LED, or assembled-prop behavior.

## Project Goal

Build a Halloween prop using an ESP32-WROOM-32E/uPesy low-power dev board and MicroPython. A Hi-Link LD2450 detects and tracks up to three targets. A pan NEMA17 with a second, stacked tilt NEMA17 points a skull at a selected target. Two NeoPixels in the eyes are off while idle and red while tracking. On target loss, the skull should return to a naturally downward park pose.

The desired aim point is approximately 1.2 m above the floor for children. The LD2450 reports X/Y and velocity, not target height; height must be estimated. Both axes need mechanical limits and a position-reference strategy. The user currently intends to skip homing and start from a known physical pose. No limit switches or StallGuard homing are implemented.

## Hardware Inventory And Observed Details

- MCU board: uPesy ESP32 low-power dev board with ESP32-WROOM-32E; chip query returned ESP32-D0WD-V3 (dual-core ESP32).
- Firmware: MicroPython v1.29.0 `ESP32_GENERIC`, flashed to the generic ESP32 image address after erasing the prior ESP-IDF image and custom partition table. ESP-IDF firmware/partition layout was replaced.
- Mac USB serial port: `/dev/cu.SLAB_USBtoUART`.
- Project venv: `./venv`; `mpremote`, `esptool`, and `black` installed there. `.gitignore` ignores `venv/`.
- Radar: Hi-Link LD2450, 5 V supply, UART2 at 256000 baud, GPIO16 ESP32 RX from radar TX and GPIO17 ESP32 TX to radar RX (optional for receive-only operation). Common ground required. Manual recommends more than 200 mA supply capacity. Vendor `ICLM_MTT.exe` is Windows-only; project uses its own MicroPython monitor.
- Radar was physically mounted upside down relative to manual. With the parser corrected to the manual's sign bit, tests observed a target in front with positive Y. Moving a target to the sensor's right made X decrease; for this installation +X points toward sensor-left. LD2450 does not report Z.
- Radar test produced valid frames at about 10 Hz. Example corrected tuples included `(250, 807, -16, 360)`, tuple order `(x_mm, y_mm, speed_cm_s, distance_resolution_mm)`. Resolution/gate is range-bin size, not target size or exact range.
- Pan driver: replacement BTT TMC2209 V1.3. The first module failed to respond over UART and was replaced. Whether it was damaged is unknown. The BTT board was accidentally put in a breadboard position that may have shorted its extra EN/VREF/DIAG-row contacts; that arrangement has been abandoned.
- BTT UART wiring, verified by continuity and working response: driver `RX`/UART pad is the single bidirectional PDN_UART bus with factory R10 state. ESP32 GPIO22 RX goes directly to this bus; GPIO23 TX goes through 1 kOhm to the same bus. Leave BTT separate `TX` header unused and R10 unbridged. Do not tie RX/UART to EN, VREF, or DIAG. The user has read the BTT V1.3 manual and confirms the pad marked UART is RX.
- Driver logic: VIO to ESP32 3V3; measured 3.29 V. VM to separate 12 V motor supply; measured 12.2 V. Driver GND, ESP32 GND, and PSU negative are common.
- Pan control: GPIO25 STEP, GPIO26 DIR, GPIO27 active-low EN. TMC UART hardware UART ID 1, GPIO22 RX/GPIO23 TX. The replacement replied at baud 57600, slave address 0. `config.py` records 57600. The setup utility probes the configured rate and common fallbacks including 115200 and 57600, across addresses 0-3; duplicate baud rates are skipped, and only 57600/address 0 replied.
- Pan motor: StepperOnline 17HE19-2004S, 1.8 degree full step, rated 2 A/phase. Coil pairs: black/blue and red/green. Actual driver wiring: black A1, blue A2, red B1, green B2.
- VREF potentiometer had been adjusted to 0.7 V based on prior advice. Successful UART configuration clears `GCONF.I_SCALE_ANALOG`, so the tested digital current config bypasses VREF. Do not change the pot casually. Settings are volatile and reset on VM power cycling; rerun pan setup before motor tests after a power cycle.
- The pan motor is connected now. It is a bare/unloaded motor; user attached a small tape flag and saw it move during the slow jog.
- Tilt motor/driver are not commissioned. NeoPixels are assumed daisy-chained but not tested. Skull assembly, sensor-to-pivot offsets, mechanical limits, target-height mounting, and assembled tracking are not validated.

## Current Verified Pan Settings

Last successful `pan_driver_setup.py` run reported:

```text
Probing TMC UART at 115200 baud
  ... no reply ...
Probing TMC UART at 57600 baud
TMC2209 UART replied at 57600 baud, address 0; IOIN=0x21000041
Verified GCONF=0x000001c0 for UART-controlled current and microsteps
GLOBALSCALER write accepted; IFCNT=2
IHOLD_IRUN write accepted by IFCNT; requested value=0x00080d06 (IRUN=13, IHOLD=6, IFCNT=3)
Verified CHOPCONF=0x14010053; MRES configured for 16 microsteps
No motor motion was commanded. Power-cycle resets these settings;
```

Current calculation assumes 0.11 ohm sense resistor and VSENSE=0.325 V as set in `config.py`; result is approximately 0.77 A RMS run and 0.39 A RMS hold, conservative relative to motor rating. `IHOLD_IRUN` is not treated as readable; write acceptance is checked using `IFCNT`. `GCONF` and `CHOPCONF` are genuinely read back.

Pan enable test while motor connected reported:

```text
disabled: ENN=True driver_enabled=False cs_actual=6 standstill=True IOIN=0x21000041 DRV_STATUS=0xc0060000
enabled: ENN=False driver_enabled=True cs_actual=6 standstill=True IOIN=0x21000040 DRV_STATUS=0xc0060000
disabled_again: ENN=True driver_enabled=False cs_actual=6 standstill=True IOIN=0x21000041 DRV_STATUS=0xc0060000
No STEP pulses were sent.
```

Pan STEP/DIR signal test with EN disabled observed IOIN STEP and DIR bits toggle with ESP32 GPIO25/26. The first pan jog (160 pulses each way at 400 pps) was too subtle to observe by eye. The revised jog (320 pulses each direction, 10 ms pulse period, about 100 pulses/s) ran successfully; user observed the tape flag move. No load/prop was attached.

Host suite last passed: 11 tests. `python -m py_compile *.py` and `git diff --check` also passed in the last docs/code check.

## Code Map

- `config.py`: GPIOs, UART IDs/baud, current, steps, axis limits, geometry, target height, startup pose. `DRIVER_MICROSTEPS_CONFIRMED` is currently `False`; leave false until main firmware applies and verifies driver initialization.
- `main.py`: intended complete target tracking and two-axis motion, but does not configure TMC UART settings. It currently gates all motion when `DRIVER_MICROSTEPS_CONFIRMED` is false. Do not tell user to set the flag true as a workaround; integrate pan and tilt driver initialization in main first.
- `ld2450.py`: streaming LD2450 parser. Coordinates/speed use manual-confirmed sign-magnitude convention: top bit 1 means positive; top bit 0 means negative. Bytearray trimming uses reassignment for MicroPython compatibility.
- `radar_test.py`: radar-only monitor, one status line/second, no motor pins.
- `tmc2209.py`: UART CRC, packets, register reads/writes, IFCNT write confirmation, GCONF and microstep/current helpers.
- `pan_driver_setup.py`: scans baud/address read-only until response, then configures GCONF, GLOBALSCALER, IHOLD_IRUN, and CHOPCONF. Does not pulse STEP.
- `pan_enable_test.py`: reads status and toggles EN only; may energize/hold motor briefly; no steps.
- `pan_signal_test.py`: toggles STEP/DIR while EN is disabled and reads IOIN; no motor energization/steps.
- `pan_motor_test.py`: actual no-load 36-degree out-and-back jog at about 100 pulses/s; requires current settings already applied and motor clear.
- `aiming.py`: pure radar-to-yaw/pitch geometry and clamp.
- `tracking.py`: target range filtering and nearest-track continuity.
- `stepper.py`: non-blocking STEP/DIR axis logic; not tested with assembled mechanics.
- `tests/test_protocol_and_aiming.py`: host-side parser, math, target continuity, UART CRC/packet/current tests.
- `wiring.md`: wiring pin map, power diagram, breadboard warning, and driver details.
- `manuals/006000_HLK-LD2450-Instruction-Manual.pdf`: LD2450 manual.
- `manuals/BIGTREETECH TMC2209 V1.3 User Manual.pdf`: BTT V1.3 manual.

## Commands

Use the project venv for Python tools:

```sh
./venv/bin/python -m unittest discover -s tests
./venv/bin/python -m py_compile *.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_driver_setup.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_enable_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_signal_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_motor_test.py
```

Stop the continuous radar monitor with Ctrl-C. Ensure no other process (serial
monitor/editor) owns `/dev/cu.SLAB_USBtoUART` before invoking mpremote.

`mpremote run` executes a local script on the board, but its imported modules
must exist on the board. To copy files:

```sh
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp config.py :config.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp tmc2209.py :tmc2209.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp ld2450.py :ld2450.py
```

## Safety And Next Work

1. Keep the bare pan motor clear when using `pan_motor_test.py`; disable VM and
   USB before moving motor plugs or changing wiring. Never hot-plug a stepper.
2. After any VM power cycle, rerun `pan_driver_setup.py`. Do not run the full
   tracker with `DRIVER_MICROSTEPS_CONFIRMED=True` until `main.py` itself
   configures and verifies both drivers at startup.
3. Integrate TMC initialization into main: probe/confirm baud/address, set
   `GCONF`, `GLOBALSCALER`, current, and MRES on each driver; fail closed on
   UART errors. Determine how pan and tilt get distinct slave addresses or
   separate UART buses before connecting a second driver.
4. Commission tilt alone using the same disconnected-motor/readback/no-load
   steps. Validate driver address, current, MRES, STEP/DIR, direction, limits.
5. Set actual radar mounting height and measured sensor/pan/tilt pivot offsets.
   Check target-aim equations and axis signs with a target at known positions.
6. Verify mechanical hard travel before selecting final axis angle limits;
   radar field of view is not a mechanical stop.
7. Add the two eye NeoPixels and test current limiting, 5 V supply, level
   shifting, red/off behavior.
8. Only then run integrated tracking with skull assembly attached. Implement a
   real homing solution later (switches or validated TMC StallGuard); hard stops
   without feedback are not homing sensors.

## Important Corrections From Earlier Discussion

- The original TMC UART attempt saw only echoed request bytes. The replacement
  driver answered at 57600; setup then succeeded. Do not keep troubleshooting
  RX/TX routing unless the current hardware regresses.
- The first setup implementation incorrectly tried to read back `IHOLD_IRUN`;
  this is not treated as a readable register. Current code confirms the write
  via `IFCNT` instead.
- Earlier generic TMC2208 references were not sufficient alone. The actual
  device response (`IOIN` version `0x21` in `0x21000041`), the BTT TMC2209 V1.3
  module, and successful observed configuration/motion are the project-specific
  evidence.
- Early README statements that the main tracker was ready after setting a
  confirmation flag were unsafe/out of date. Main lacks driver startup init;
  do not bypass the false flag.
