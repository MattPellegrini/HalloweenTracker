# Multi-Target Radar Trackers

MicroPython project for an ESP32-WROOM-32E/uPesy board, Hi-Link LD2450 radar,
two BTT TMC2209 V1.3 stepper drivers, two NEMA17 yaw motors, and two pairs of
NeoPixel indicator LEDs — one motor + one LED pair per tracker prop. Each tracker parks
facing forward with LEDs off, wakes with illuminated indicators when the radar detects a person,
and rotates independently in yaw to follow its assigned target.

## Current Status

The project architecture is **two independent 1-axis (yaw-only) trackers**
sharing a single ESP32 MCU, one Hi-Link LD2450 mmWave radar, and one shared-wire
TMC2209 UART bus. The earlier single 2-axis (yaw/tilt) prototype was
pivoted to this dual 1-axis layout; the second tracker reuses the driver, wiring,
and NEMA17 motor originally allocated for tilt.

- **Radar**: The LD2450 produces valid 3-target tracking frames at ~10 Hz over
  UART2 at 256000 baud. The coordinate mapping has been verified on hardware
  (+Y is forward, +X is toward the sensor's left with the inverted physical mount).
- **Stepper Drivers**: Both BTT TMC2209 V1.3 drivers are addressed on a shared
  single-wire UART bus (Tracker 1 at address 0, Tracker 2 at address 1). Digital
  UART configuration reliably verifies `GCONF` (UART mode), accepts `GLOBALSCALER`
  and `IHOLD_IRUN` writes via `IFCNT` confirmation, and configures `CHOPCONF`
  for 1/16 microstepping.
- **Motors & Motion**: StepperOnline 17HE19-2004S motors run at 1/16 microstepping
  (3,200 steps/rev, 0.1125°/step) with a conservative 770 mA RMS run / 390 mA RMS
  hold current. Step pulses are generated autonomously by the ESP32 hardware RMT
  peripheral, reaching speeds up to 6400 pps (720°/s or 2.0 rev/s) with zero CPU
  bit-banging overhead. Both motors have been bench-tested with unloaded 360° spin tests.
- **Motion Shaping**:
  - Speed is driven exponentially by pointing error ($E^{1.4}$), moving swiftly
    when far and easing gently as the target is centered.
  - Target-bearing rate feed-forward anticipates moving targets, gated off below
    3° of error to prevent radar noise from causing ticking.
  - A 150 ms low-pass target smoother filters raw radar coordinate fluctuations.
  - Slew rate limiting (14,400 pps/s) and brake-before-reverse prevent skipped steps.
  - A 2-microstep (~0.225°) micro-hysteresis deadband prevents hunting over single
    rounding digits at rest, while tracking smoothly and continuously when moving.
  - EN remains enabled while awake; the TMC2209 drops silently to IHOLD (390 mA)
    at rest with zero coil-snap clunking. EN is only disabled in SLEEP/parked state.
- **Target Assignment**: Configurable via `TARGET_ASSIGNMENT_MODE` in `config.py`:
  - `nearest_two`: Tracker 1 follows the closest target; Tracker 2 follows the next
    closest target. Either tracker idles/parks when fewer targets are present.
  - `fixed_slot`: Tracker 1 follows radar slot 1; Tracker 2 follows radar slot 2.
    An empty slot returns that tracker to its forward park pose.
- **Main Control Loop**: See [MAIN_LOOP.md](MAIN_LOOP.md) for a detailed breakdown
  and sequence diagrams of the 50 Hz real-time tracking and RMT hardware pulse slicing loop.
- **Unit Tests**: 32 host tests in `tests/test_protocol_and_aiming.py` pass.
- **Physical Commissioning**: Unloaded bare-motor movements and UART diagnostics
  are verified. Full prop linkage assembly and the second indicator pair remain
  to be commissioned.

## Hardware And Wiring

- **MCU**: uPesy ESP32 Low Power DevKit (ESP32-WROOM-32E), flashed with
  MicroPython v1.29.0 `ESP32_GENERIC`.
- **Radar**: Hi-Link LD2450, 5 V power, UART2 at 256000 baud (8 data bits, no
  parity, 1 stop bit). Radar TX connects to ESP32 GPIO16 (UART2 RX); radar RX
  to GPIO17 is optional for receive-only operation. Common ground required.
- **Shared TMC UART**:
  - ESP32 GPIO22 (RX) connects directly to the shared `RX`/UART pad of both BTT
    drivers.
  - ESP32 GPIO23 (TX) connects to that same shared line through a 1 kOhm series
    resistor.
  - Leave each driver's separate `TX` pad and `CLK` unconnected; leave R10 in its
    factory state. Both drivers share 115200 baud communication.
- **Tracker 1 (Address 0)**:
  - TMC2209 address: 0 (MS1 and MS2 left low/open).
  - STEP: GPIO25, DIR: GPIO26, active-low EN: GPIO27.
  - Eye/status LEDs: NeoPixel data DIN on GPIO13 (2 LEDs, red when tracking, off when
    idle).
- **Tracker 2 (Address 1)**:
  - TMC2209 address: 1 (strap MS1 to 3V3 / VIO, strap MS2 to GND).
  - STEP: GPIO18, DIR: GPIO19, active-low EN: GPIO21.
  - Eye/status LEDs: NeoPixel data DIN on GPIO14 (2 LEDs, red when tracking, off when
    idle).
- **Motors**: Two StepperOnline 17HE19-2004S NEMA17 steppers (1.8°/step, rated 2 A
  per phase). Coil pair 1: black (A1), blue (A2); coil pair 2: red (B1), green
  (B2).
- **Power Architecture**:
  - 12 V motor power supply feeds TMC driver `VM` and an external step-down buck
    converter.
  - Buck converter outputs regulated 5 V to feed the LD2450 radar, NeoPixels,
    and the uPesy board's `VIN` pin (do not connect buck 5 V to the board's `5V`
    pin).
  - uPesy USB and external `VIN` can be connected simultaneously: USB provides
    programming/serial and powers the ESP32 internally, while the buck powers
    the radar and motors.
  - TMC `VIO` connects to ESP32 3V3 (measured ~3.29 V).
  - All grounds are common: ESP32 GND, driver GND, buck IN-/OUT-, and 12 V PSU
    negative.
- **Breadboard Caution**: Do not mount BTT carriers into a standard solderless
  breadboard where `EN`, `VREF`, and `DIAG` share a connected strip. Use direct
  wiring or an adapter that isolates these pins. `VREF` and `DIAG` are not
  connected in this project.

See [wiring.md](wiring.md) for detailed schematics and pin diagrams.

## Setup On The Mac

Python packages are managed in a local virtual environment (`venv/`), which
includes `mpremote`, `esptool`, and `black`.

```sh
source venv/bin/activate
python -m unittest discover -s tests
```

The ESP32 serial port is typically `/dev/cu.SLAB_USBtoUART`. Examples below use
`./venv/bin/mpremote`. Stop any active serial monitor before running an
`mpremote` command.

### Deploying Files To The ESP32

To copy the core runtime modules to the board filesystem:

```sh
for file in config.py aiming.py ld2450.py stepper.py tmc2209.py tracking.py; do
  ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp "$file" ":$file"
done
```

### Running Commissioning And Diagnostic Scripts

1. **Test Radar Stream**:

   ```sh
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp radar_test.py :radar_test.py
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py
   ```

   Prints decoded target positions and frame rates once per second. Stop with
   Ctrl-C.

2. **Configure TMC2209 Drivers via Shared UART**:

   ```sh
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp motor_driver_setup.py :motor_driver_setup.py
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_driver_setup.py
   ```

   Finds and configures both drivers at addresses 0 and 1; verifies registers
   and IFCNT write counts. Sends no motor steps.

3. **Verify Pin Signals and Driver Enable (Non-Moving Check)**:

   ```sh
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp motor_test.py :motor_test.py
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py --check-only
   ```

   Verifies STEP and DIR pin continuity via `IOIN` with drivers disabled, then
   checks driver enable status without spinning the motors.

4. **Verify Motor Motion (Sequential 360° Out-And-Back Spin)**:

   ```sh
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp motor_test.py :motor_test.py
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py
   ```

   Runs signal and enable checks, then spins Skull 1 and Skull 2 steppers 360°
   out and back sequentially on bare, unloaded motor shafts.

5. **Run Full Two-Skull Tracker**:
   ```sh
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp main.py :main.py
   ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run main.py
   ```

### Stopping The Board Program

If Ctrl-C does not interrupt an active script over USB serial, reset the MCU:

```sh
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART reset
```

For an emergency stop during motor movement, disconnect or switch off the 12 V
motor power supply immediately. Never unplug a stepper motor from its driver
while 12 V VM is energized.

## Script Guide

| Script / Module                     | Purpose                                                                                                                                                                                                                                                                   | Execution / Usage                                                                                                                                                                                                     |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `main.py`                           | Main application: configures both TMC2209 drivers, polls the LD2450 radar, assigns targets (`nearest_two` or `fixed_slot`), smoothly steers both trackers independently, and controls red/off status NeoPixels.                                                           | Upload core modules and `main.py`, then run `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run main.py`.                                                                                                         |
| `radar_test.py`                     | Standalone radar monitor: prints valid frame counts and decoded target coordinates once per second; does not toggle motor pins.                                                                                                                                           | Upload `config.py` and `ld2450.py`, then run `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py`. Stop with Ctrl-C.                                                                                |
| `motor_driver_setup.py`             | Probes shared UART and configures both TMC2209 drivers (UART mode, currents, 1/16 microstepping), verifying writes via IFCNT and register readbacks. Sends no STEP pulses.                                                                                                | With motor drivers powered and shafts clear, run `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_driver_setup.py`.                                                                                      |
| `motor_test.py`                     | Comprehensive motor & driver test: verifies STEP/DIR signal line continuity via `IOIN` (disabled), verifies enable/driver status (`DRV_STATUS`), then spins each tracker motor 360° out and back sequentially. Supports `--check-only` / `--signals-only` to skip motion. | Run with bare shafts clear: `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py`. For non-moving signal check: `./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py --check-only`. |
| `config.py`                         | Central configuration: hardware pin map (`TRACKERS`), UART baud rates, microstepping, step rates, angular limits, geometry offsets, and target assignment modes.                                                                                                          | Imported by scripts; edit to tune prop parameters.                                                                                                                                                                    |
| `ld2450.py`                         | Streaming LD2450 radar frame parser supporting compact valid lists and indexed 3-slot target frames.                                                                                                                                                                      | Imported by `radar_test.py` and `main.py`.                                                                                                                                                                            |
| `tmc2209.py`                        | TMC2209 UART driver: CRC calculation, packet framing, register read/write, IFCNT write verification, current scaling, and microstepping helpers.                                                                                                                          | Imported by motor setup and controller scripts.                                                                                                                                                                       |
| `aiming.py`                         | Geometry calculations: solves yaw angle from target (X, Y) coordinates relative to prop pivot and applies travel limits.                                                                                                                                                  | Imported by `main.py`; unit tested.                                                                                                                                                                                   |
| `tracking.py`                       | Target selection logic (`choose_two_targets`), start/stop deadband hysteresis, exponential error-shaped velocity, bearing rate feed-forward, target smoothing, and slew rate limiting.                                                                                    | Imported by `main.py`; unit tested.                                                                                                                                                                                   |
| `stepper.py`                        | Non-blocking STEP/DIR stepper axis controller with hardware ESP32 RMT pulse generation, angle tracking, and soft limits.                                                                                                                                                  | Imported by `main.py`.                                                                                                                                                                                                |
| `tests/test_protocol_and_aiming.py` | Host unit test suite (32 tests) covering LD2450 parsing, 3-slot decoding, target assignment, aiming geometry, motion shaping, RMT pulse generation, and TMC2209 UART.                                                                                                     | Run on host: `./venv/bin/python -m unittest discover -s tests`.                                                                                                                                                       |

## Driver Settings And Hardware Test Results

The tested TMC2209 configuration is:

- **Shared UART Bus**: GPIO22 RX / GPIO23 TX (with 1 kOhm series resistor) at
  115200 baud (`config.py`).
- **Addressing**: Tracker 1 at slave address 0; Tracker 2 at slave address 1.
- **`GCONF = 0x000001c0`**: UART control enabled, microsteps set via register
  (`mstep_reg_select`), analog VREF current scaling disabled (`i_scale_analog = 0`).
- **`GLOBALSCALER = 0`**: Full-scale current resolution (256/256), verified
  accepted via `IFCNT`.
- **`IHOLD_IRUN = 0x00080d06`**: IRUN code 13 (~770 mA RMS) and IHOLD code 6
  (~390 mA RMS) with the 0.11-ohm sense resistor. This is a quiet, conservative
  operating current that keeps the 2 A/phase NEMA17 motors cool while providing
  ample torque.
- **`CHOPCONF = 0x14010053`**: MRES set for 1/16 microstepping (3,200 pulses per
  revolution for 1.8° steppers, or 0.1125° per step).
- **Volatile Registers**: TMC2209 registers reset to power-on defaults whenever
  driver power (`VM`/`VIO`) is interrupted. Both `main.py` and `motor_driver_setup.py`
  automatically configure and verify driver registers on startup.

### Verified Hardware Diagnostics

- **Radar UART**: LD2450 produces consistent frames at ~10 Hz at 256000 baud.
  Target X/Y coordinates respond accurately to human motion across the sensor's
  field of view.
- **Driver Detection**: Both TMC2209 modules consistently reply to register reads
  at addresses 0 and 1 with version `0x21` in `IOIN`.
- **Signal Integrity**: Verified STEP and DIR pin toggles reach TMC2209 inputs
  via `IOIN` register readback with drivers disabled (`motor_test.py`).
- **Driver Enable**: Verified active-low enable toggling, standstill flag, and
  actual current scaling (`CS_ACTUAL`) via `DRV_STATUS` in `motor_test.py`.
- **Dual Motor Spin & RMT Offloading**: `motor_test.py` confirmed 360° forward and
  reverse rotation, 30° minimum-speed jog, and 35->6400->35 pps slewed sweeps for
  both Tracker 1 and Tracker 2 steppers with zero step loss using ESP32 hardware RMT.
- **Radar Gaze Tracking**: Live target yaw tracking, deadband hysteresis, and
  smooth gaze following verified with live LD2450 radar input at 50 Hz.
- **Host Unit Tests**: All 32 unit tests pass (`python -m unittest discover -s tests`).

## Tracker Behavior And Configuration

All prop geometry, pin allocations, motor dynamics, and operational limits are
defined in `config.py`.

- **Multi-Tracker Architecture**:
  `main.py` creates a list of generic `Tracker` instances from the `TRACKERS`
  tuple in `config.py`. Each tracker encapsulates its own `StepperAxis`,
  NeoPixel eyes, target memory, target smoothing filter, bearing rate filter,
  and deadband state.
- **Target Assignment Modes (`TARGET_ASSIGNMENT_MODE`)**:
  - `"nearest_two"`: The closest radar target is assigned to Tracker 1; the second
    closest target is assigned to Tracker 2. If only one person is detected, Tracker
    1 tracks them while Tracker 2 returns to park with LEDs off.
  - `"fixed_slot"`: Tracker 1 tracks radar tracking slot 1; Tracker 2 tracks slot 2.
    If a slot is empty, that tracker parks.
- **Gaze & Parking**:
  - When tracking a target, the tracker's status LEDs shine bright red (`(255, 0, 0)`).
  - If target contact is lost for longer than `TARGET_LOST_MS` (1200 ms), the tracker
    turns off its status LEDs (`(0, 0, 0)`) and smoothly rotates back to the forward
    park position (`PARK_YAW_DEG = 0.0°`).
- **Natural Motion Shaping**:
  - **1/16 Microsteps**: Provides 0.1125° angular resolution while allowing fast,
    responsive head turns up to 720°/s at 6400 pps.
  - **Micro-Hysteresis Deadband**: 2 microsteps (~0.225°) start threshold / 0.0° stop threshold.
    The tracker remains still at rest without hunting over single rounding digits, then
    tracks smoothly and continuously to the target without snapping off and on.
  - **Quiet Holding**: The motor driver EN is kept asserted while awake. The TMC2209
    drops silently to IHOLD (390 mA) at rest without coil-snap clunking, disabling EN
    only when entering SLEEP / parked state or shutdown.
  - **Error-Scaled Velocity**: Speed scales with $(E / E_{\text{full}})^{1.4}$,
    ensuring rapid repositioning for distant targets and smooth deceleration as
    the tracker comes on target.
  - **Bearing Rate Feed-Forward**: Anticipates walking targets based on filtered
    angular velocity, eliminating lag. Gated off when pointing error is below 3°
    to prevent sensor noise from inducing vibration.
  - **Acceleration Slew & Reversal**: Rate slew is capped at 14400 pps/s. When
    reversing direction, the motor decelerates toward minimum speed before
    changing directions.
- **Startup Mode**:
  `POSITION_MODE = "assume"` requires trackers to be physically positioned facing
  straight forward (0.0° yaw) before powering on. Limit switches and StallGuard
  sensorless homing are not implemented. Angle travel is software-clamped to
  each tracker's configured `yaw_min_deg` and `yaw_max_deg` (`MIN_YAW_DEG` and `MAX_YAW_DEG`).
