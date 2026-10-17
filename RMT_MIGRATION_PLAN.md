# RMT Motor Control & 1/16 Microstepping Migration Plan

## Executive Summary

This document specifies the migration of the two Halloween skull yaw stepper axes from CPU-bound GPIO bit-banging to the ESP32 hardware RMT (Remote Control) peripheral, paired with an upgrade of the BTT TMC2209 V1.3 stepper drivers from 1/8 to 1/16 microstepping.

Key architectural benefits:

- **Zero CPU pulse generation overhead**: Step pulse trains are synthesized entirely by the ESP32 RMT peripheral without busy-waiting or microsecond polling loops.
- **Higher top speed**: Maximum step rate increased to 6400 pps ($720^\circ/\text{s}$ at 1/16 microstepping, 2 rev/s), compared to the prior bit-banged 1600 pps ceiling.
- **Whisper-quiet motion & resonance elimination**: 1/16 microstepping yields 3,200 steps/rev ($0.1125^\circ$/pulse) with smooth sinusoidal coil transitions.
- **Higher tracking frequency**: Main loop update frequency increased from ~30 Hz (33 ms) to 50 Hz (20 ms), delivering responsive tracking with lower tracking lag.
- **Exact, zero-drift position accounting**: Discrete, bounded pulse slices (15–20 ms) guarantee integer pulse tracking with zero step loss or accumulation error.

---

## 1. Hardware & Driver Architecture

### TMC2209 Digital Configuration

- **Microstepping**: 1/16 microstepping configured via `CHOPCONF.MRES = 4`.
- **Full Steps / Rev**: 200 full steps $\times$ 16 microsteps = 3,200 microsteps per $360^\circ$ revolution.
- **Steps per Degree**: $3200 / 360 \approx 8.8889$ steps/degree ($0.1125^\circ$/step).
- **Current Settings**: 770 mA RMS run current, 390 mA RMS hold current (digital current scale via UART).

### ESP32 RMT Peripheral Allocation

- ESP32 contains 8 hardware RMT channels (channels 0–7).
- MicroPython `esp32.RMT` dynamically allocates available channels.
- **Clock Resolution**: $1\,\text{MHz}$ (`resolution_hz = 1_000_000`), yielding $1\,\mu\text{s}$ tick resolution.
- **Pulse Waveform**: Active-high pulse on STEP pin:
  - High duration: fixed $10\,\mu\text{s}$ (well above TMC2209 minimum $100\,\text{ns}$ STEP high time).
  - Low duration: $(\text{step\_period\_us} - 10)\,\mu\text{s}$.
  - Idle level: 0 (LOW).

---

## 2. Bounded Burst Step Generation & Position Accounting

### The Problem with Infinite Loops

Continuous hardware pulsing loops (`RMT.loop(True)`) cannot report intermediate step counts to Python without hardware interrupts, making runtime stopping and exact position tracking vulnerable to non-deterministic overshoots and step loss.

### Bounded Slice Architecture

`StepperAxis.run()` operates as a non-blocking discrete slice transmitter:

1. `StepperAxis.run()` checks `rmt.wait_done()`.
2. When the previous burst finishes:
   - The completed burst count is committed to `completed_steps`.
   - Remaining step delta to `target_steps` is computed.
3. If motion is required:
   - A slice size is computed: $\min(|target - current|, \text{steps in } 15\,\text{ms at current rate})$. Minimum 1 pulse.
   - DIR pin is set to matching direction (respecting `direction_inverted`).
   - If changing direction, a brief settle delay ensures TMC2209 DIR setup time ($\ge 20\,\text{ns}$).
   - `rmt.write_pulses()` is dispatched asynchronously.
   - `burst_in_flight` state is recorded with start timestamp and pulse count.
4. **Position Calculation**:
   - `position_steps`: When idle, returns `completed_steps`. When busy, returns `completed_steps + (direction * pulses_elapsed)`, where `pulses_elapsed = min(burst_count, elapsed_us // step_period_us)`.
   - Guarantees strictly monotonic, integer-exact position resolution with zero cumulative drift.

---

## 3. Motion & Tracking Parameter Tuning

| Parameter                       | 1/8 Microstepping (Old)              | 1/16 Microstepping + RMT (New)       | Notes                                             |
| :------------------------------ | :----------------------------------- | :----------------------------------- | :------------------------------------------------ |
| `MICROSTEPS`                    | 8                                    | 16                                   | 3200 steps/rev ($0.1125^\circ$/pulse)             |
| `MAX_STEP_RATE`                 | 1600 pps ($360^\circ/\text{s}$)      | 6400 pps ($720^\circ/\text{s}$)      | $2.0\,\text{rev}/\text{s}$ top speed              |
| `MIN_TRACKING_STEP_RATE`        | 15 pps ($\sim 3.375^\circ/\text{s}$) | 30 pps ($\sim 3.375^\circ/\text{s}$) | Prevents crawl and low-frequency resonance        |
| `TRACKING_BASE_STEP_RATE`       | 400 pps ($90^\circ/\text{s}$)        | 800 pps ($90^\circ/\text{s}$)        | Used when speed shaping is disabled               |
| `TRACKING_STEP_RATE_SLEW_PPS_S` | 4800 pps/s                           | 14,400 pps/s                         | $\sim 1620^\circ/\text{s}^2$ angular acceleration |
| `TRACKING_DEADBAND_START_DEG`   | $0.45^\circ$ (2 microsteps)          | $0.225^\circ$ (2 microsteps)         | Tighter standstill precision at 1/16              |
| `TRACKING_DEADBAND_STOP_DEG`    | $0.0^\circ$                          | $0.0^\circ$                          | Moves continuously until reaching target          |
| `TRACKING_UPDATE_MS`            | 33 ms (~30 Hz)                       | 20 ms (50 Hz)                        | Faster tracking cadence without CPU choke         |

---

## 4. Execution Phases

1. **`stepper.py` Core**:
   - Add RMT pulse generation with fallback mock support on non-ESP32 platforms (for macOS unit tests).
   - Implement slice transmission in `run()`.
   - Implement real-time `position_steps` property.
   - Add `is_busy()`, `stop()`, and `deinit()`.
2. **`config.py` Parameters**:
   - Configure 1/16 microsteps, scaled step rates, slew rates, 20 ms tracking interval, and RMT constants.
3. **`main.py` Tracking Loop**:
   - Wire RMT lifecycle into `Tracker`.
   - Update idle detection from `step_high` to `is_busy()`.
   - Add cooperative sleep (`time.sleep_ms(1)`) in main loop to reduce CPU power while steppers move.
4. **`motor_test.py` Verification**:
   - Run raw GPIO pin check first while disabled.
   - Use RMT pulse generator for constant-speed, minimum-speed, and slewed 6400 pps sweeps.
5. **Host Unit Tests & Hardware Validation**:
   - Run unit test suite on macOS (`python -m unittest discover tests`).
   - Run `py_compile` checks.
   - Test on ESP32 hardware via `mpremote`.
