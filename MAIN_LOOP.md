# Main Control Loop Architecture

This document describes the real-time execution lifecycle of the dual-skull Halloween tracker implemented in `main.py`.

---

## 1. High-Level Architecture

The project runs on an **ESP32-WROOM-32E** under **MicroPython**. It simultaneously tracks targets with a 24 GHz mmWave radar and independently aims two skulls using TMC2209-driven NEMA17 stepper motors.

```mermaid
flowchart TD
    subgraph SENSORS ["Sensor Input"]
        RADAR["LD2450 Radar\n(UART2 @ 256k baud, ~10 Hz)"]
    end

    subgraph CORE ["ESP32 Main Control Loop (main.py)"]
        POLL["1. Poll Radar Stream\n(LD2450Reader.poll)"]
        ASSIGN["2. Target Assignment\n(choose_two_targets)"]
        TRACK["3. Tracking Pipeline (50 Hz)\n- Timeout & Park\n- Exponential Smoothing\n- Deadband Hysteresis\n- Error-Proportional Speed Shaping\n- Slew Limiting & Reversal Braking"]
        RMT["4. Hardware Pulse Slicing\n(StepperAxis.run via ESP32 RMT)"]
        YIELD["5. Cooperative Yield\n(time.sleep_ms(1))"]
    end

    subgraph ACTUATORS ["Outputs & Actuators"]
        M1["Skull 1 Pan Stepper\n(1/16 microsteps, RMT pin 25)"]
        M2["Skull 2 Pan Stepper\n(1/16 microsteps, RMT pin 18)"]
        LED1["Skull 1 Eye LEDs\n(NeoPixel pin 13)"]
        LED2["Skull 2 Eye LEDs\n(NeoPixel pin 14)"]
    end

    RADAR --> POLL
    POLL -->|New frame| ASSIGN
    ASSIGN --> TRACK
    POLL -->|No frame (inter-frame)| TRACK
    TRACK --> RMT
    RMT --> M1
    RMT --> M2
    TRACK --> LED1
    TRACK --> LED2
    RMT --> YIELD
    YIELD --> POLL
```

---

## 2. Startup & Initialization Phase

Before entering the main loop, `main()` performs the following sequential hardware configuration:

1. **Safety & Startup Pose Assumption**:
   - Requires `POSITION_MODE = "assume"`.
   - Assumes both skulls are manually positioned facing center (`0.0°`, `PARK_YAW_DEG`) prior to powering the board.
2. **Tracker Object Instantiation**:
   - Creates a `Tracker` instance for each skull in `config.SKULLS`.
   - Configures GPIO pins: STEP, DIR, active-low EN, and WS2812B NeoPixel eye data pins.
   - Instantiates `StepperAxis` with ESP32 hardware RMT channels (1 MHz clock, 10 µs pulse width, 15 ms burst slice).
   - Initializes eye LEDs to dark (`config.EYE_OFF`).
3. **TMC2209 Hardware Discovery & Configuration (`find_tmc_drivers`)**:
   - Opens the shared single-wire half-duplex UART bus (`UART1`, 115200 baud).
   - Queries each driver at its configured address (`0` for Skull 1, `1` for Skull 2).
   - Verifies the TMC2209 silicon revision via `REG_IOIN` (`0x21`).
   - Configures each driver:
     - `GCONF`: Enables UART digital scaling, bypasses analog VREF potentiometer.
     - `GLOBALSCALER`: Configured to `0` (full 256/256 scale).
     - `IHOLD_IRUN`: Sets run current (770 mA RMS) and hold current (390 mA RMS). Verifies acceptance via the incrementing `REG_IFCNT` interface counter.
     - `CHOPCONF`: Sets `MRES` for 1/16 microstepping (3,200 pulses/revolution).
4. **LD2450 Radar UART Initialization**:
   - Opens `UART2` at 256000 baud on GPIO16 (RX) and GPIO17 (TX).
   - Wraps the UART in `LD2450Reader` to handle non-blocking streaming packet extraction.

---

## 3. The Main Execution Loop

The main loop runs continuously inside a `while True` loop structured around four cooperative stages:

```mermaid
sequenceDiagram
    participant Loop as Main Loop
    participant Radar as LD2450Reader
    participant Trackers as Trackers (1 & 2)
    participant RMT as ESP32 RMT Hardware

    Loop->>Radar: poll()
    alt Valid Frame Arrived (~10 Hz)
        Radar-->>Loop: 3 Target Slots
        Loop->>Trackers: choose_two_targets & on_radar_frame()
        Loop->>Trackers: update() (State, Deadband, Speed Shaping)
        opt Logging Enabled
            Loop->>Loop: format_frame_log() & print
        end
    end

    alt 50 Hz Timer Elapsed (dt >= 20 ms)
        Loop->>Trackers: update() (Periodic motion interpolation)
    end

    Loop->>Trackers: run()
    Trackers->>RMT: write_pulses() (Next 15 ms slice)
    Loop->>Loop: time.sleep_ms(1) (Cooperative yield)
```

### Stage A: Non-Blocking Radar Polling (~10 Hz)
- The LD2450 streams 30-byte frames approximately every 100 ms.
- `radar.poll()` reads available UART bytes, searches for the 4-byte synchronization header (`0xAA 0xFF 0x03 0x00`), confirms the 2-byte footer (`0x55 0xCC`), and unpacks three target slots.
- When a valid frame is received:
  1. **Delta Motion Telemetry**: Computes step count and angular displacement moved by each motor since the prior frame.
  2. **Target Assignment (`choose_two_targets`)**:
     - `fixed_slot`: Skull 1 tracks radar hardware slot 1, Skull 2 tracks slot 2.
     - `nearest_two`: Valid targets within `[300 mm, 6000 mm]` are sorted by range; Skull 1 gets the closest, Skull 2 gets the next closest.
  3. **Radar Frame Handler (`tracker.on_radar_frame`)**:
     - Updates `current_target` and refreshes `last_target_ms` (preventing target loss timeout).
     - Solves geometric line-of-sight yaw angle using `aiming.solve_angles` and clamps to `[PAN_MIN_DEG, PAN_MAX_DEG]`.
     - Computes target angular speed (bearing rate in deg/s) and applies a low-pass filter (`filtered_yaw_rate`) for feed-forward pacing.
  4. **Immediate Update**: Runs `tracker.update()` immediately to incorporate fresh radar coordinates without waiting for the next periodic tick.
  5. **Telemetry Logging**: If `config.ENABLE_FRAME_LOGGING` is enabled, formats and prints diagnostic lines (decimated by `config.FRAME_LOG_DECIMATION`).

### Stage B: Periodic 50 Hz Motion Tick (every 20 ms)
To guarantee smooth acceleration, deceleration, and responsive target loss detection between 100 ms radar frames, `tracker.update()` runs every `config.TRACKING_UPDATE_MS = 20 ms`:

1. **Target Timeout Check**:
   - If time since `last_target_ms` exceeds `config.TARGET_LOST_MS` (1,200 ms), the target is declared lost.
   - The skull turns eye LEDs dark (`config.EYE_OFF`), clears target state, and commands `config.PARK_YAW_DEG` (0°).
2. **Target Acquisition & Eye Illumination**:
   - When a valid target is active, the skull illuminates eye LEDs red (`config.EYE_RED`).
   - If waking from sleep, target smoothing is seeded directly with the initial angle to eliminate wake lag.
3. **Exponential Angle Smoothing (`smooth_toward_target`)**:
   - Applies an exponential low-pass filter ($\tau = 150\text{ ms}$) to raw radar angles:
     $$\alpha = \min\left(1.0, \frac{\Delta t}{\tau}\right)$$
     $$\text{yaw}_{\text{smoothed}} = \text{yaw}_{\text{prev}} + \alpha \cdot (\text{yaw}_{\text{raw}} - \text{yaw}_{\text{prev}})$$
   - Glides smoothly between discrete radar measurements and filters coordinate noise.
4. **Pointing Error & Deadband Hysteresis**:
   - Calculates pointing error: $e = \text{target\_yaw} - \text{current\_yaw}$.
   - Evaluates dual-threshold deadband (`update_deadband`):
     - **Stationary**: Motor remains idle until $|e| \ge 0.225^\circ$ (2 microsteps).
     - **Moving**: Once moving, motor continues tracking smoothly until error reaches $0.0^\circ$.
     - Prevents hunting or clicking over rounding digits when a target stands still.
5. **Speed Shaping (`tracking_step_rate`)**:
   - Calculates step rate using a non-linear power curve:
     $$\text{Rate}_{\text{error}} = \text{Rate}_{\text{min}} + (\text{Rate}_{\text{max}} - \text{Rate}_{\text{min}}) \cdot \left(\frac{|e| - e_{\text{stop}}}{e_{\text{full}} - e_{\text{stop}}}\right)^{1.4}$$
   - **Feed-Forward Velocity Pacing**: When $|e| \ge 3^\circ$, adds target angular velocity ($\text{bearing\_rate} \cdot \text{steps/deg} \cdot \text{gain}$). Paces fast-moving walking targets without lagging behind.
6. **Slew Rate Limiting & Reversal Braking (`slew_step_rate`)**:
   - Restricts acceleration and deceleration to `TRACKING_STEP_RATE_SLEW_PPS_S` (14,400 pps/s).
   - **Brake-Before-Reverse**: If commanded to change direction while moving, decelerates to `MIN_TRACKING_STEP_RATE` (35 pps) before reversing direction, preventing step loss.
7. **Coil Power Management**:
   - Enables driver coils (`set_enabled(True)`) while awake or moving.
   - When stationary while awake, TMC2209 automatically drops current to `IHOLD` (390 mA RMS) with zero clicking or snap.
   - Disables driver coils (`set_enabled(False)`) only when fully parked in SLEEP state to eliminate standby heat.

### Stage C: Stepper Hardware Pulse Slicing (`tracker.run()`)
- `StepperAxis.run()` checks if the previous RMT hardware burst has finished (`rmt.wait_done()`).
- Accumulates completed steps into `completed_steps`.
- Computes the pulse count for the next slice (bounded to `RMT_BURST_SLICE_MS = 15 ms`).
- Updates DIR pin if direction changed.
- Hands the pulse array directly to the ESP32 hardware RMT peripheral (`rmt.write_pulses(pulses, 1)`).
- RMT hardware streams the pulses autonomously to the STEP pin with microsecond hardware accuracy.

### Stage D: Cooperative Sleep Yield (`time.sleep_ms(1)`)
- Because the ESP32 RMT peripheral generates pulse trains autonomously in hardware, the CPU does not need to poll or busy-wait.
- Yielding for 1 ms per loop iteration prevents CPU starvation, allows background MicroPython runtime and REPL processing, and prevents MCU heating.

---

## 4. Teardown & Fault Safety

When interrupted (e.g. by `Ctrl+C` or unhandled error), the `finally:` block executes:
1. Calls `tracker.shutdown()` on all trackers:
   - Immediately stops RMT pulse trains.
   - De-initializes the ESP32 RMT channels.
   - De-energizes all motor coils (`enable_pin.value(inactive)`).
   - Sets eye LEDs to dark (`config.EYE_OFF`).
2. De-initializes the radar and TMC UART peripherals.
3. Prints a clean shutdown message.
