"""Non-blocking STEP/DIR motion for a TMC2209-driven stepper axis."""

import time


class StepperAxis:
    def __init__(self, step_pin, direction_pin, enable_pin, steps_per_revolution,
                 minimum_degrees, maximum_degrees, max_step_rate,
                 enable_active_low=True, initial_degrees=0.0,
                 direction_inverted=False):
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
        self.position_steps = round(initial_degrees * self.steps_per_degree)
        self.target_steps = self.position_steps
        self.step_high = False
        self.last_step_us = time.ticks_us()
        self.step_interval_us = max(1, int(1000000 / self.step_rate))
        self.step_pin.value(0)
        self.set_enabled(False)

    def set_enabled(self, enabled):
        active = 0 if self.enable_active_low else 1
        inactive = 1 - active
        self.enable_pin.value(active if enabled else inactive)

    def set_target_degrees(self, degrees):
        target = round(degrees * self.steps_per_degree)
        self.target_steps = max(self.minimum_steps,
                                min(self.maximum_steps, target))
        self.set_enabled(self.target_steps != self.position_steps)

    def set_step_rate(self, steps_per_second):
        self.step_rate = max(1, min(self.max_step_rate, steps_per_second))
        self.step_interval_us = max(1, int(1000000 / self.step_rate))

    def run(self):
        """Advance at most one step; call frequently from the main loop."""
        now = time.ticks_us()
        if self.step_high:
            self.step_pin.value(0)
            self.step_high = False

        difference = self.target_steps - self.position_steps
        if difference == 0:
            self.set_enabled(False)
            return

        if time.ticks_diff(now, self.last_step_us) < self.step_interval_us:
            return

        direction = 1 if difference > 0 else -1
        self.direction_pin.value(1 if ((direction > 0) !=
                           self.direction_inverted) else 0)
        self.step_pin.value(1)
        self.step_high = True
        self.position_steps += direction
        self.last_step_us = now
