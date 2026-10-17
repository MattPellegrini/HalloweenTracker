"""RMT hardware pulse generation and non-blocking STEP/DIR motion for a TMC2209-driven stepper axis."""

import time

try:
    import esp32
    _HAS_ESP32_RMT = hasattr(esp32, "RMT")
except (ImportError, AttributeError):
    _HAS_ESP32_RMT = False


class StepperAxis:
    def __init__(self, step_pin, direction_pin, enable_pin, steps_per_revolution,
                 minimum_degrees, maximum_degrees, max_step_rate,
                 enable_active_low=True, initial_degrees=0.0,
                 direction_inverted=False, rmt_resolution_hz=1000000,
                 pulse_width_us=10, burst_slice_ms=15, rmt=None):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        self.steps_per_degree = steps_per_revolution / 360.0
        self.minimum_steps = round(minimum_degrees * self.steps_per_degree)
        self.maximum_steps = round(maximum_degrees * self.steps_per_degree)
        self.max_step_rate = max_step_rate
        self.step_rate = max_step_rate
        self.enable_active_low = enable_active_low
        self.direction_inverted = direction_inverted
        self.completed_steps = round(initial_degrees * self.steps_per_degree)
        self.target_steps = self.completed_steps
        self.rmt_resolution_hz = rmt_resolution_hz
        self.pulse_width_us = pulse_width_us
        self.burst_slice_ms = burst_slice_ms
        self.burst_in_flight = 0
        self.burst_dir = 0
        self.burst_start_us = 0
        self.burst_step_period_us = 0
        self.last_dir_val = None
        self.step_high = False
        self.last_step_us = time.ticks_us()
        self.step_interval_us = max(1, int(1000000 / self.step_rate))
        self.step_pin.value(0)
        self.enabled = None

        self.rmt = None
        if rmt is not None:
            self.rmt = rmt
        elif _HAS_ESP32_RMT and hasattr(step_pin, "value"):
            try:
                self.rmt = esp32.RMT(
                    pin=step_pin,
                    resolution_hz=rmt_resolution_hz,
                    idle_level=False,
                )
            except Exception:
                self.rmt = None

        self.set_enabled(False)

    @property
    def position_steps(self):
        if self.burst_in_flight == 0:
            return self.completed_steps
        elapsed = time.ticks_diff(time.ticks_us(), self.burst_start_us)
        if elapsed <= 0 or self.burst_step_period_us <= 0:
            return self.completed_steps
        steps_done = min(self.burst_in_flight, max(0, elapsed // self.burst_step_period_us))
        return self.completed_steps + self.burst_dir * steps_done

    @position_steps.setter
    def position_steps(self, value):
        self.completed_steps = round(value)
        self.burst_in_flight = 0

    def set_enabled(self, enabled: bool):
        if getattr(self, "enabled", None) == enabled:
            return
        if not enabled:
            self.stop()
        self.enabled = enabled
        active = 0 if self.enable_active_low else 1
        inactive = 1 - active
        self.enable_pin.value(active if enabled else inactive)

    def set_target_degrees(self, degrees):
        target = round(degrees * self.steps_per_degree)
        self.target_steps = max(self.minimum_steps,
                                min(self.maximum_steps, target))

    def set_step_rate(self, steps_per_second):
        self.step_rate = max(1, min(self.max_step_rate, round(steps_per_second)))
        self.step_interval_us = max(1, int(1000000 / self.step_rate))

    def is_busy(self):
        """Return True if pulses are actively in flight."""
        if self.rmt is not None:
            if self.burst_in_flight > 0:
                if self.rmt.wait_done():
                    self.completed_steps += self.burst_dir * self.burst_in_flight
                    self.burst_in_flight = 0
                    return False
                return True
            return False
        return self.step_high

    def stop(self):
        """Immediately stop in-flight pulses and align target with current position."""
        if self.rmt is not None and self.burst_in_flight > 0:
            if not _HAS_ESP32_RMT or not isinstance(self.rmt, getattr(esp32, "RMT", type(None))):
                try:
                    self.rmt.active(False)
                except Exception:
                    pass
            elapsed = time.ticks_diff(time.ticks_us(), self.burst_start_us)
            steps_done = 0
            if elapsed > 0 and self.burst_step_period_us > 0:
                steps_done = min(self.burst_in_flight, max(0, elapsed // self.burst_step_period_us))
            self.completed_steps += self.burst_dir * int(steps_done)
            self.burst_in_flight = 0
        self.target_steps = self.completed_steps

    def deinit(self):
        """Release RMT peripheral and disable motor."""
        self.stop()
        if self.rmt is not None:
            try:
                self.rmt.deinit()
            except Exception:
                pass
            self.rmt = None
        self.set_enabled(False)

    def run(self):
        """Advance motion slice via RMT, or single step fallback."""
        if self.rmt is not None:
            if self.burst_in_flight > 0:
                if not self.rmt.wait_done():
                    return
                self.completed_steps += self.burst_dir * self.burst_in_flight
                self.burst_in_flight = 0

            difference = self.target_steps - self.completed_steps
            if difference == 0:
                return

            direction = 1 if difference > 0 else -1
            dir_pin_val = 1 if ((direction > 0) != self.direction_inverted) else 0
            if self.last_dir_val != dir_pin_val:
                self.direction_pin.value(dir_pin_val)
                self.last_dir_val = dir_pin_val

            period_us = max(1, int(1000000 / self.step_rate))
            max_slice_steps = max(1, int((self.step_rate * self.burst_slice_ms) // 1000))
            count = int(min(abs(difference), max_slice_steps))

            high_us = self.pulse_width_us
            low_us = max(high_us, min(32767, period_us - high_us))
            pulses = (high_us, low_us) * count

            self.burst_in_flight = count
            self.burst_dir = direction
            self.burst_step_period_us = period_us
            self.burst_start_us = time.ticks_us()
            self.rmt.write_pulses(pulses, 1)
            return

        # Fallback bit-banging for host test environments without esp32.RMT
        now = time.ticks_us()
        if self.step_high:
            self.step_pin.value(0)
            self.step_high = False

        difference = self.target_steps - self.completed_steps
        if difference == 0:
            return

        if time.ticks_diff(now, self.last_step_us) < self.step_interval_us:
            return

        direction = 1 if difference > 0 else -1
        self.direction_pin.value(1 if ((direction > 0) !=
                           self.direction_inverted) else 0)
        self.step_pin.value(1)
        self.step_high = True
        self.completed_steps += direction
        self.last_step_us = now
