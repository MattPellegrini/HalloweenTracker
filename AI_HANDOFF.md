# Halloween Skull Tracker: AI Handoff

Updated: 2026-10-16. Read this together with `README.md`, `wiring.md`, `config.py`, and `RMT_MIGRATION_PLAN.md` before advising on hardware or changing motor behavior. Treat observed tests below as stronger evidence than earlier conversational guesses. Do not assume untested tilt, LED, or assembled-prop behavior.

See [RMT_MIGRATION_PLAN.md](RMT_MIGRATION_PLAN.md) for detailed architecture and transition specifics for hardware RMT motor pulse offloading and 1/16 microstepping.

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
- Power architecture now planned/used: 12 V input feeds TMC VM and a buck converter; buck 5 V feeds the LD2450 and uPesy ESP32 Low Power DevKit `VIN` pin. The uPesy board documentation says USB and external VIN can be connected simultaneously; USB takes priority for the board and disables external VIN internally. Do not feed the board's `5V` pin from the buck; uPesy says that pin is powered only when USB is connected. Keep grounds common and verify buck output before hookup.
- Pan control: GPIO25 STEP, GPIO26 DIR, GPIO27 active-low EN. TMC UART hardware UART ID 1, GPIO22 RX/GPIO23 TX. The replacement replied at baud 57600, slave address 0. `config.py` records 57600. The setup utility probes the configured rate and common fallbacks including 115200 and 57600, across addresses 0-3; duplicate baud rates are skipped, and only 57600/address 0 replied.
- Pan motor: StepperOnline 17HE19-2004S, 1.8 degree full step, rated 2 A/phase. Coil pairs: black/blue and red/green. Actual driver wiring: black A1, blue A2, red B1, green B2.
- VREF potentiometer had been adjusted to 0.7 V based on prior advice. Successful UART configuration clears `GCONF.I_SCALE_ANALOG`, so the tested digital current config bypasses VREF. Do not change the pot casually. Settings are volatile and reset on VM power cycling; rerun pan setup before motor tests after a power cycle.
- The pan motor is connected now. It is a bare/unloaded motor; user attached a small tape flag and saw it move during the slow jog.
- Tilt uses the same StepperOnline 17HE19-2004S motor as pan (2 A/phase), per user confirmation. The second BTT TMC2209 is soldered and now verified on the shared UART at address 1 (MS1=VIO, MS2=GND): `tilt_signal_test.py` confirmed EN/STEP/DIR IOIN toggles, and `tilt_motor_test.py` configured 0.77 A/0.39 A and 1/16 mode, then moved the bare shaft upward about 20° and back at 100 pps. NeoPixels are assumed daisy-chained but not tested. Skull assembly, sensor-to-pivot offsets, mechanical limits, target-height mounting, and assembled tracking are not validated.

## Current Verified Pan Settings

Last successful `motor_driver_setup.py` (formerly `pan_driver_setup.py`) run reported:

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

Host suite currently passes 21 tests. `./venv/bin/python -m py_compile *.py` and `git diff --check` passed. Tilt signal and bare-motor jog at shared-UART address 1 passed on hardware (that driver now runs skull 2's pan motor). Dual-axis `main.py` bench runs found both drivers, acquired targets, and moved both bare motors plausibly. The design is now two pan-only skulls; the two-skull revision has not yet been deployed or run on hardware.

## Code Map

- `config.py`: per-skull hardware map (`SKULLS`: skull 1 STEP25/DIR26/EN27/eyes13/TMC0, skull 2 STEP18/DIR19/EN21/eyes14/TMC1), currents, steps, axis limits, geometry, startup pose, tracking settings, and `TARGET_ASSIGNMENT_MODE` (`nearest_two` or `fixed_slot`). Driver current/microsteps are configured by main at startup.
- `main.py`: two pan-only skull trackers. Requires `POSITION_MODE="assume"`, assumes the configured startup angle for each skull, finds both TMC2209s at their distinct addresses, applies/verifies UART mode/current/1/8 microsteps on both before radar startup, assigns each skull a radar target per the configured mode, and disables both drivers/eyes in cleanup. Host tests pass; not yet run on hardware in two-skull form.
- `ld2450.py`: streaming LD2450 parser. `decode_frame_slots` preserves the three radar slots (None for empty) so fixed-slot assignment works; `decode_frame` keeps the compact list for compatibility. Sign-magnitude convention: top bit 1 means positive.
- `radar_test.py`: radar-only monitor, one status line/second, no motor pins.
- `tmc2209.py`: UART CRC, packets, register reads/writes, IFCNT write confirmation, GCONF and microstep/current helpers.
- `motor_driver_setup.py`: configures TMC2209 drivers via shared UART for each skull in `SKULLS` (GCONF, GLOBALSCALER, IHOLD_IRUN, CHOPCONF). Does not pulse STEP.
- `motor_test.py`: comprehensive test for each skull in `SKULLS`: verifies STEP/DIR signal lines via `IOIN` while disabled, verifies enable/status transition via `DRV_STATUS`, and spins each motor 360° forward and back. Subsumes `pan_signal_test.py` and `pan_enable_test.py` (and supports `--check-only` for non-moving tests).
- `pan_enable_test.py`: legacy single-axis enable test (superseded by `motor_test.py`).
- `pan_signal_test.py`: legacy single-axis signal test (superseded by `motor_test.py`).
- `tilt_motor_test.py`: finds/configures the tilt driver at address 1 on the shared UART and jogs it +/-20 degrees at 100 pulses/s. Verified on bare motor: GCONF/current/CHOPCONF accepted, shaft moved upward about 20° and returned, then disabled.
- `pan_tracking_test.py`: pan-only radar tracking. It shares the main controller's 2.5/1.5-degree start/stop hysteresis, 250 ms target smoothing, error-shaped 50-1600 pps speed at 1/8, gated bearing-rate feed-forward, and 4800 pps/s rate slew limiting.
- `aiming.py`: pure radar-to-yaw/pitch geometry and clamp.
- `tracking.py`: target validity filtering, nearest-two/fixed-slot assignment, error-shaped tracking speed, gated bearing-rate feed-forward, deadband hysteresis, target smoothing, and STEP-rate slew limiting.
- `stepper.py`: non-blocking STEP/DIR axis logic; not tested with assembled mechanics.
- `tests/test_protocol_and_aiming.py`: host-side parser, slot decoding, geometry, target assignment (nearest-two and fixed-slot), UART CRC/packet/current tests, and motion-shaping tests.
- `wiring.md`: wiring pin map, power diagram, breadboard warning, and driver details.
- `manuals/006000_HLK-LD2450-Instruction-Manual.pdf`: LD2450 manual.
- `manuals/BIGTREETECH TMC2209 V1.3 User Manual.pdf`: BTT V1.3 manual.

## Commands

Use the project venv for Python tools:

```sh
./venv/bin/python -m unittest discover -s tests
./venv/bin/python -m py_compile *.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_driver_setup.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_enable_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_signal_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run pan_tracking_test.py
```

Stop the continuous radar monitor with Ctrl-C. Ensure no other process (serial
monitor/editor) owns `/dev/cu.SLAB_USBtoUART` before invoking mpremote.
On the user's serial setup, Ctrl-C did not stop `pan_tracking_test.py`; the
confirmed stop was `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART reset`.
This resets the MCU and could restart an installed boot-time `main.py`. If
motion is happening and needs to stop immediately, switch off VM first.

`mpremote run` executes a local script on the board, but its imported modules
must exist on the board. To copy files:

```sh
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp config.py :config.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp tmc2209.py :tmc2209.py
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp ld2450.py :ld2450.py
```

## Safety And Next Work

1. Before any motor test, verify with power off that pan MS1/MS2 are low/open
   (address 0), tilt MS1 is tied to VIO and MS2 to GND (address 1), and both
   RX/UART pads share the GPIO22/GPIO23 bus. Never hot-plug a stepper or change
   wiring with VM powered.
2. Run `tilt_signal_test.py` first with both drivers on the bus. It targets
   address 1 only, toggles tilt STEP/DIR while EN is disabled, and confirms the
   tilt IOIN bits. Then run `tilt_motor_test.py` with the bare tilt shaft
   centered and clear; it configures the conservative current and 1/16 mode,
   jogs +/-20 degrees at 100 pps, and disables EN afterward. Verify actual
   direction and movement; do not attach the skull.
3. Validate the pan and tilt motor tests independently with bare shafts. Driver
   settings are volatile, but each motor test configures its own driver before
   stepping. Keep the 12 V motor supply accessible for an immediate stop.
4. Set actual radar mounting height and measured sensor/pan/tilt pivot offsets.
   Check target-aim equations and axis signs with a target at known positions.
5. Verify mechanical hard travel before selecting final axis angle limits;
   radar field of view is not a mechanical stop. Add the eye NeoPixels and test
   current limiting, 5 V supply, level shifting, and red/off behavior.
6. Only after both bare-axis tests and wiring checks, run `main.py` with axes
   physically placed at the configured assumed startup angles (pan 0, tilt
   0 degrees during bare-motor commissioning; park remains tilt -25), clear
   travel, and 12 V VM ready to cut. Main verifies and configures both TMCs
   before radar startup. Keep skull/linkage detached until direction, travel
   limits, tracking, and park return are confirmed. Real homing remains future
   work; hard stops are not homing sensors.

## Pan Tracking Iteration Status

The first fixed-100-pps `pan_tracking_test.py` run initialized the replacement
driver at 115200 baud/address 0, acquired targets, and printed changing pan-yaw
commands from roughly -29 to +35 degrees. That confirms live radar parsing and
angle calculation. The output did not prove physical tracking accuracy or lag;
the user had previously reported substantial lag and stationary jitter. The
test was stopped with `mpremote reset`, because Ctrl-C stopped/detached the
serial monitor without reliably interrupting the board program.

A later dual-axis `main.py` bench run found pan at shared-UART address 0 and
tilt at address 1, configured both drivers, acquired live targets, and produced
plausible pan/tilt movement at the old 80-400 pps rates. The user observed that
pan moved fine and tilt appeared sensible, but both axes were too slow. Open-
space tracking accuracy and assembled-mechanism behavior remain unvalidated.

The current workspace has new natural-motion code for two skulls that has NOT yet
been deployed to the ESP32 or physically tested:

- 1/8 microstepping doubles shaft speed versus 1/16 (0.225 deg/pulse).
- A micro-hysteresis deadband of 2 microsteps (~0.45 deg at 1/8 microstepping)
  replaces the old 2.5 deg ratchet. When standing still, it prevents the motor
  from hunting back and forth over a single rounding digit. When moving beyond
  2 microsteps, tracking is smooth and continuous down to 0 without snapping off.
- The motor driver EN is kept active while the skull is tracking (awake).
  Instead of cutting coil power and clunking on every move-stop, the TMC2209
  automatically reduces current to IHOLD (390 mA) at rest. EN is only disabled
  when entering the SLEEP / parked state or shutdown.
- Raw radar bearings feed a 350 ms exponential target smoother before the
  stepper target is updated, softening 10 Hz sensor jumps.
- STEP rate is primarily proportional to pointing error: 50 pps when the
  deadband activates, full 1600 pps at 12 degrees or more, with a 1.4 exponent.
  At 1/8 this corresponds roughly to 11.25-360 degrees/second.
- Smoothed target-bearing rate remains as feed-forward so a moving target can
  be paced even when the current pointing error is briefly small. It does not
  use LD2450 `speed_cm_s`, which is radial speed and can be near zero for
  lateral motion.
- Rate changes are slew-limited to 4800 pps/s, and direction reversal first
  brakes toward minimum speed before commanding the opposite direction.
- `main.py` instantiates two Tracker skulls with independent pan motors and
  eyes, supporting `nearest_two` or `fixed_slot` target assignment. Do not
  attach the skulls or linkage for the next test.

Next test sequence:

1. Ensure both unloaded motors and tape flags have clear travel. Verify radar
   data is valid. Re-copy `config.py`, `tracking.py`, `stepper.py`,
   `tmc2209.py`, `ld2450.py`, `aiming.py`, and either `pan_tracking_test.py`
   or `main.py` from the workspace to the ESP32.
2. Start with the pan-only tracker if you want to isolate the new speed curve.
   Then run `main.py` with pan at 0 degrees and tilt at horizontal 0 degrees.
3. Hold a stationary target and observe whether the shaft settles/stays still
   inside the deadband. If it still jitters, increase the deadband modestly.
4. Move the target slowly left/right, then faster. Observe the tape flags, the
   printed bearing/error-shaped rates, any lag/oscillation/missed steps, and
   motor/driver temperature. The console prints desired target angles, not
   encoder-measured shaft angles.
5. Keep 1/16 for now because the user found it smoother. Raise `MAX_STEP_RATE`
   for large-motion speed if needed; change microsteps only if pulse-rate
   limits or visible step resolution become the bottleneck. Update driver MRES
   and `config.MICROSTEPS` together. Lower microsteps trade angle resolution
   for more angle per pulse; they do not inherently raise torque or the motor's
   physical speed limit.
6. Do not attach the skull until the natural-motion revision, direction,
   limits, LED behavior, and park return are verified. Homing remains future
   work.

Theoretical reference values for direct drive and a 200-step motor:

| Microsteps | Pulses/revolution | Degrees/pulse | At 100 pulses/s | At 400 pulses/s | At 800 pulses/s |
| ---------: | ----------------: | ------------: | --------------: | --------------: | --------------: |
|       1/16 |              3200 |        0.1125 |     11.25 deg/s |        45 deg/s |        90 deg/s |
|        1/8 |              1600 |         0.225 |      22.5 deg/s |        90 deg/s |       180 deg/s |
|        1/4 |               800 |          0.45 |        45 deg/s |       180 deg/s |       360 deg/s |

These are pulse-rate calculations, not guaranteed motor performance. For a
target crossing laterally at speed `v` and range `r`, angular rate near the
centerline is approximately `v/r` radians/s. The LD2450 reports around 10 Hz,
so at 2 m and 3-5 m/s lateral motion the target direction changes about
8.6-14.3 degrees between radar frames; at 6 m the corresponding change is about
2.9-4.8 degrees. Consider this sensor update interval along with motor speed.

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
  confirmation flag were unsafe/out of date. Main now initializes both TMCs at
  startup and had one successful slow-speed bare-axis bench run; the current
  natural-motion revision still needs hardware validation.
