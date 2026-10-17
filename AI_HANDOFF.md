# Halloween Skull Tracker: AI Handoff

Updated: 2026-10-17. Read this together with `README.md`, `MAIN_LOOP.md`, `wiring.md`, `config.py`, `RMT_MIGRATION_PLAN.md`, and `TODO.md` before advising on hardware or changing motor behavior. Treat observed test logs below as stronger evidence than earlier conversational guesses.

See [MAIN_LOOP.md](MAIN_LOOP.md) for the real-time control loop architecture and sequence diagrams, and [RMT_MIGRATION_PLAN.md](RMT_MIGRATION_PLAN.md) for the hardware RMT pulse generator and 1/16 microstepping design details.

## Project Goal

Build a multi-target Halloween prop system using an ESP32-WROOM-32E / uPesy low-power dev board and MicroPython v1.29.0. A Hi-Link LD2450 24 GHz mmWave radar detects and tracks up to three targets (~10 Hz stream over UART2). The system independently steers **two 1-axis (yaw-only) tracker props** (skulls) to follow detected people.

Each tracker consists of:

- One NEMA17 stepper motor driving horizontal yaw.
- One BTT TMC2209 V1.3 stepper driver on a shared single-wire UART bus.
- One pair of WS2812B NeoPixel indicator LEDs in the eyes.

### Architectural Evolution Note

The project originally began as a single 2-axis (yaw + tilt) prototype. It was deliberately pivoted to **two independent 1-axis (yaw-only) trackers**. The second NEMA17 motor, BTT TMC2209 driver, and wiring originally earmarked for tilt are now allocated as the yaw axis for Tracker 2. Because the LD2450 is a 2D planar radar operating in the horizontal plane, line-of-sight yaw angles depend only on relative X/Y coordinates: $\text{atan2}(\Delta X, \Delta Y)$. Vertical (Z) elevation difference has zero geometric effect on yaw, so tilt and target-height estimation were eliminated.

### Operational Behavior

- **Target Assignment (`TARGET_ASSIGNMENT_MODE`)**:
  - `"fixed_slot"` (default): Tracker 1 follows radar tracking slot 1, Tracker 2 follows radar tracking slot 2. An empty slot parks that tracker.
  - `"nearest_two"`: Tracker 1 follows the nearest valid radar target; Tracker 2 follows the next nearest. If only one person is detected, Tracker 2 idles.
- **Eye Indicators**: Off (`(0, 0, 0)`) when idle or parked; red (`(255, 0, 0)`) when actively tracking. (Next planned feature per `TODO.md`: yellow when searching, red when target-locked for > 2 seconds).
- **Parking**: On target loss for longer than `TARGET_LOST_MS` (1,200 ms), the tracker turns its eye LEDs dark and smoothly rotates back to the forward center park pose (`PARK_YAW_DEG = 0.0°`).
- **Position Mode**: `POSITION_MODE = "assume"` skips physical homing switches. Props must be physically aligned facing center (0.0° yaw) before powering the board.

## Hardware Inventory And Observed Details

- **MCU Board**: uPesy ESP32 Low Power DevKit with ESP32-WROOM-32E (ESP32-D0WD-V3 dual-core). Flashed with MicroPython v1.29.0 `ESP32_GENERIC`.
- **Development Host**: Mac serial port `/dev/cu.SLAB_USBtoUART`. Project virtual environment at `./venv` with `mpremote`, `esptool`, and `black`.
- **Radar**: Hi-Link LD2450 mmWave radar, 5 V supply, UART2 at 256000 baud (ESP32 GPIO16 RX from radar TX; GPIO17 TX to radar RX optional). Common ground required.
  - Physically mounted upside down relative to manual. Parser sign bit is corrected: +Y is forward, +X is toward sensor-left.
  - Radar streams 30-byte frames at ~10 Hz with 3 target slots: `(x_mm, y_mm, speed_cm_s, distance_resolution_mm)`.
  - Configured target gate: 300 mm to 6000 mm (`MIN_TARGET_RANGE_MM` to `MAX_TARGET_RANGE_MM`).
- **Stepper Drivers**: Two BigTreeTech (BTT) TMC2209 V1.3 modules.
  - **Shared UART Bus**: Single-wire half-duplex UART (`UART1`) at 115200 baud (`config.TMC_UART_BAUDRATE`).
    - ESP32 GPIO22 RX connects directly to the shared `RX`/UART pad on both drivers.
    - ESP32 GPIO23 TX connects to that same line through a 1 kOhm series resistor.
    - Separate BTT `TX` pads and `CLK` pins are unconnected; R10 is unbridged.
  - **Addressing**:
    - Tracker 1: Address 0 (MS1 and MS2 left low/open).
    - Tracker 2: Address 1 (MS1 tied to 3V3 / VIO, MS2 tied to GND).
  - **Logic & Motor Voltages**: VIO to ESP32 3V3 (measured ~3.29 V); VM to separate 12 V motor supply (measured ~12.2 V).
  - **Breadboard Caution**: BTT carriers must not be inserted into breadboards where `EN`, `VREF`, and `DIAG` land in the same connected strip. Use direct wiring or isolated sockets.
  - **Current Sense**: `R_SENSE = 0.11 Ohm`, `VSENSE = 0.325 V`. UART digital scaling bypasses the analog VREF potentiometer.
- **Power Architecture**:
  - 12 V PSU feeds TMC driver `VM` and an external step-down buck converter.
  - Buck converter outputs regulated 5 V to LD2450 radar, NeoPixel LEDs, and uPesy ESP32 `VIN` pin (do NOT connect external 5 V to the board's `5V` pin).
  - USB and external `VIN` can be connected simultaneously: USB provides programming/serial and takes priority internally, while buck powers radar and motors.
  - Common ground shared across ESP32, radar, drivers, buck converter, and 12 V PSU negative.
- **Motors**: Two StepperOnline 17HE19-2004S NEMA17 steppers (1.8° full step, 200 steps/rev, 2 A/phase rating).
  - Coil pairs: black/blue (A1/A2), red/green (B1/B2).
  - Tracker 1: GPIO25 STEP, GPIO26 DIR, GPIO27 active-low EN.
  - Tracker 2: GPIO18 STEP, GPIO19 DIR, GPIO21 active-low EN.
- **Eye LEDs**:
  - Tracker 1: GPIO13 data out (2 x WS2812B NeoPixels).
  - Tracker 2: GPIO14 data out (2 x WS2812B NeoPixels).

## Current Verified Driver & Motion Settings

Both TMC2209 drivers are verified and configured on the shared UART bus:

- **Baud Rate**: 115200 baud (both drivers reply with silicon revision `0x21` in `REG_IOIN`).
- **`GCONF = 0x000001c0`**: Digital UART current scaling enabled; analog VREF potentiometer bypassed; microsteps selected from register.
- **`GLOBALSCALER = 0`**: Full 256/256 scale, verified accepted via `IFCNT`.
- **`IHOLD_IRUN = 0x00080d06`**: IRUN code 13 (~770 mA RMS) and IHOLD code 6 (~390 mA RMS). Verified accepted via incrementing `IFCNT`.
- **`CHOPCONF = 0x14010053`**: MRES configured for 1/16 microstepping (3,200 pulses/revolution, 0.1125°/pulse).
- **Quiet Holding Power**: Driver EN is kept enabled while awake (`set_enabled(True)`). TMC2209 automatically drops current to `IHOLD` (390 mA) at rest without coil-snap clunking. EN is only disabled when entering SLEEP / parked state or shutdown.
- **Hardware RMT Offloading (`stepper.py`)**: Pulse trains are generated autonomously by the ESP32 hardware RMT peripheral in non-blocking slices (`RMT_BURST_SLICE_MS = 15 ms`), achieving speeds up to 6400 pps (~720°/s) with zero CPU bit-banging overhead.
- **50 Hz Real-Time Loop (`main.py`)**: Tracking pipeline runs at 50 Hz (`TRACKING_UPDATE_MS = 20 ms`) with cooperative 1 ms sleep yielding.

### Motion Shaping Pipeline (`tracking.py`)

- **Micro-Hysteresis Deadband**:
  - Start threshold: 0.225° (2 microsteps at 1/16).
  - Stop threshold: 0.0°.
  - Motor stays still at rest without hunting over single rounding digits, then tracks smoothly and continuously down to the target.
- **Error-Proportional Speed Curve**:
  $$\text{Rate} = \text{Rate}_{\text{min}} + (\text{Rate}_{\text{max}} - \text{Rate}_{\text{min}}) \cdot \left(\frac{|e| - e_{\text{stop}}}{e_{\text{full}} - e_{\text{stop}}}\right)^{1.4}$$
  Ranges from `MIN_TRACKING_STEP_RATE` (35 pps) to `MAX_STEP_RATE` (6400 pps).
- **Bearing Rate Feed-Forward**: Converts target angular velocity to step rate ($0.85\times\text{gain}$) to pace moving targets; gated off below 3.0° error to prevent sensor noise jitter.
- **Exponential Angle Smoothing**: 150 ms filter constant (`TRACKING_TARGET_SMOOTHING_MS`) softens discrete 10 Hz radar updates.
- **Slew Rate Limiting & Reversal**: Acceleration and deceleration capped at 14,400 pps/s (`TRACKING_STEP_RATE_SLEW_PPS_S`). If reversing direction while moving, decelerates to minimum speed before switching direction.

## Verified Test Results

1. **Host Unit Tests**: All 32 tests in `tests/test_protocol_and_aiming.py` pass (`python -m unittest discover -s tests`).
2. **Syntax Compilation**: `python -m py_compile *.py` compiles cleanly with zero errors.
3. **Driver Communication**: `motor_driver_setup.py` detects both drivers at addresses 0 and 1 at 115200 baud, applies GCONF, GLOBALSCALER, IHOLD_IRUN, and CHOPCONF with verified IFCNT writes.
4. **Signal and Driver Check**: `motor_test.py --check-only` confirms STEP and DIR pin toggles reach TMC2209 inputs via `IOIN` register readback while disabled, and confirms active-low enable and standstill flags via `DRV_STATUS`.
5. **Physical Motor Motion**: `motor_test.py` verified 360° forward/reverse rotations, 30° slow jogs (35 pps), and 35->6400->35 pps slewed sweeps on both Tracker 1 and Tracker 2 bare shafts with zero step loss.
6. **Live Radar Tracking**: `main.py` verified live target acquisition, 50 Hz angle interpolation, RMT pulse slicing, and graceful park return on target timeout.

## Code Map

- `config.py`: Central configuration. Defines `TRACKERS` (pin allocations, addresses, pivot offsets `yaw_pivot_x_mm`/`yaw_pivot_y_mm`, limits `yaw_min_deg`/`yaw_max_deg`), TMC UART settings, microstepping (16), step rates (35-6400 pps), slew rate, deadband, target assignment mode (`fixed_slot`), and radar parameters.
- `main.py`: Main application loop. Initializes both TMC2209 drivers via shared UART, starts LD2450 radar streaming, assigns targets, executes 50 Hz tracking pipeline with RMT pulse slicing, controls NeoPixel eyes, and handles cooperative sleep yielding.
- `stepper.py`: Non-blocking stepper axis controller. Generates step pulses using ESP32 hardware RMT peripheral with 15 ms burst slicing, real-time monotonic position tracking, soft travel limits, and host-environment bit-bang fallback.
- `tracking.py`: Mathematical tracking pipeline: `choose_two_targets` (fixed slot or nearest two), `update_deadband` (dual-threshold hysteresis), `tracking_step_rate` (error-proportional velocity + feed-forward), `slew_step_rate` (acceleration/deceleration limiting), and `smooth_toward_target` (exponential low-pass filter).
- `aiming.py`: Geometric angle solver: calculates line-of-sight yaw angle from target (X, Y) relative to tracker pivot coordinates, and applies soft clamping.
- `ld2450.py`: Streaming LD2450 radar frame parser. Decodes 30-byte frames with sign-magnitude coordinates into compact lists or 3-slot indexed frames (`decode_frame_slots`).
- `tmc2209.py`: TMC2209 UART driver: CRC calculation, packet serialization, register read/write, IFCNT write verification, current scaling, and microstep configuration.
- `motor_driver_setup.py`: Commissioning tool: probes shared UART and configures both TMC2209 drivers without sending motor steps.
- `motor_test.py`: Comprehensive motor test: verifies STEP/DIR lines via `IOIN`, checks enable/status via `DRV_STATUS`, and executes sequential 360° spin tests, slow jogs, and slewed sweeps on both motors. Supports `--check-only`.
- `radar_test.py`: Radar-only diagnostic monitor: prints decoded target coordinates and frame rates at 1 Hz without driving motors.
- `tests/test_protocol_and_aiming.py`: Host unit test suite (32 tests) covering radar parsing, slot decoding, aiming geometry, motion shaping, RMT pulse generation, and TMC2209 protocol.
- `MAIN_LOOP.md`: Real-time execution architecture, timing budgets, and sequence diagrams.
- `RMT_MIGRATION_PLAN.md`: Technical specification for hardware RMT pulse generator and 1/16 microstepping migration.
- `REMOTE_DEPLOYMENT_PLAN.md`: Architecture plan for remote headless deployment and field update workflows.
- `wiring.md`: Complete pin tables, power distribution, and schematics.
- `TODO.md`: Pending feature enhancements (e.g. searching yellow vs target_lock red LED states).

## Commands

Use the project venv for all operations:

```sh
# 1. Run host test suite
./venv/bin/python -m unittest discover -s tests

# 2. Syntax-check all Python files
./venv/bin/python -m py_compile *.py

# 3. Test radar stream (~10 Hz coordinates)
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run radar_test.py

# 4. Configure TMC2209 drivers (non-moving)
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_driver_setup.py

# 5. Non-moving pin signal and driver status check
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py --check-only

# 6. Physical motor motion test (sequential 360° out-and-back spin)
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run motor_test.py

# 7. Run main dual-tracker tracking application
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART run main.py
```

### Deploying Files to ESP32 Flash

```sh
for file in config.py aiming.py ld2450.py stepper.py tmc2209.py tracking.py main.py; do
  ./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART fs cp "$file" ":$file"
done
```

### Stopping Active Scripts

If Ctrl-C does not interrupt an active script over USB serial, reset the MCU:

```sh
./venv/bin/mpremote connect /dev/cu.SLAB_USBtoUART reset
```

For an emergency stop during motor motion, immediately switch off or disconnect the 12 V motor power supply.

## Safety Rules & Operating Guidelines

1. **Power Safety**: Never plug or unplug a stepper motor connector while 12 V `VM` is energized. Doing so will destroy the TMC2209 driver. Keep the 12 V PSU switch within immediate reach during tests.
2. **Address Strapping**: Before powering VM, verify that Tracker 1 MS1/MS2 are open/low (address 0) and Tracker 2 MS1 is tied to 3V3/VIO and MS2 to GND (address 1).
3. **Bare Shaft Testing**: Keep skull props and mechanical linkages detached from motor shafts until motion direction, travel limits, tracking smoothness, and park return are fully verified on bare shafts with tape flags.
4. **Startup Alignment**: Because `POSITION_MODE = "assume"`, physically position both motor shafts facing straight forward (0.0° yaw) before powering the MCU.

## Next Work & Action Items

1. **Live Field Validation**:
   - Run `main.py` on hardware with live walking targets in front of the radar.
   - Observe tracking smoothness, responsiveness, and settle behavior. If necessary, tune smoothing (`TRACKING_TARGET_SMOOTHING_MS = 150`), feed-forward gain (`TRACKING_FEEDFORWARD_GAIN = 0.85`), or bearing clamp (`MAX_TRACKING_BEARING_RATE_DEG_S = 90.0`).
2. **Enhanced Eye Indicator States (`TODO.md`)**:
   - Implement distinct LED states: `EYE_OFF` (idle/parked), `EYE_SEARCHING` (yellow when target is acquired but tracker is slewing), and `EYE_LOCKED` (red when tracker has stayed pointed at the target within angular tolerance for > 2 seconds).
3. **Mechanical Assembly & Calibration**:
   - Mount skulls and linkage to motor shafts.
   - Measure physical radar-to-pivot offsets and update `yaw_pivot_x_mm` / `yaw_pivot_y_mm` in `config.TRACKERS`.
   - Measure mechanical hard-stop limits and adjust `yaw_min_deg` / `yaw_max_deg` per tracker to prevent binding.
4. **Remote Deployment**:
   - Evaluate `REMOTE_DEPLOYMENT_PLAN.md` for wireless/network deployment if needed for installed props.
