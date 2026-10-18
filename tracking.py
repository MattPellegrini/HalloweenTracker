"""Target filtering, target assignment, and natural motion shaping algorithms.

This module provides:
  1. Target filtering: Range checking and forward-hemisphere gating.
  2. Target assignment: Allocating radar targets to trackers ('fixed_slot' or 'nearest_two').
  3. Exponential angle smoothing: Low-pass filtering to glide smoothly over sensor jitter.
  4. Micro-hysteresis deadband: Start/stop thresholds to prevent hunting at rest.
  5. Speed shaping: Error-proportional power curve with velocity feed-forward.
  6. Rate slewing: Bounded acceleration/deceleration to protect against lost steps.
"""


def valid_target(target, minimum_range_mm, maximum_range_mm):
    """Return True when a radar slot holds an in-front target within range limits.

    Parameters:
      target: (x_mm, y_mm, speed_cm_s, resolution_mm) or None.
      minimum_range_mm: Closest valid detection distance (rejects prop chassis echoes).
      maximum_range_mm: Furthest detection distance for prop interaction.
    """
    if target is None:
        return False
    x_mm, y_mm, speed_cm_s, resolution_mm = target
    distance_squared = x_mm * x_mm + y_mm * y_mm
    # Target must be in front (y > 0) and between minimum and maximum range
    return (y_mm > 0 and
            minimum_range_mm * minimum_range_mm <= distance_squared <=
            maximum_range_mm * maximum_range_mm)


def choose_two_targets(slots, minimum_range_mm, maximum_range_mm,
                       assignment_mode):
    """Assign targets to (tracker1, tracker2); either returned target may be None.

    Assignment Modes:
      'fixed_slot':
        Tracker 1 always tracks radar hardware slot 1, Tracker 2 tracks slot 2.
        If a slot is empty or out-of-range, that specific tracker parks/idles.
        This provides deterministic 1:1 mapping to radar detection tracks.

      'nearest_two':
        Sorts all valid targets by Euclidean distance from the sensor.
        Tracker 1 tracks the closest target; Tracker 2 tracks the second closest.
        When fewer than 2 targets exist, the remaining tracker(s) park/idle.
    """
    if assignment_mode == "fixed_slot":
        first = (slots[0] if len(slots) > 0 and
                 valid_target(slots[0], minimum_range_mm, maximum_range_mm)
                 else None)
        second = (slots[1] if len(slots) > 1 and
                  valid_target(slots[1], minimum_range_mm, maximum_range_mm)
                  else None)
        return first, second

    # 'nearest_two' mode: filter all valid targets and sort by distance squared
    valid = [target for target in slots
             if valid_target(target, minimum_range_mm, maximum_range_mm)]
    valid.sort(key=lambda item: item[0] * item[0] + item[1] * item[1])
    first = valid[0] if len(valid) > 0 else None
    second = valid[1] if len(valid) > 1 else None
    return first, second


def tracking_step_rate(error_degrees, bearing_rate_deg_s, minimum_rate,
                       maximum_rate, full_speed_error_degrees,
                       minimum_error_degrees, rate_exponent,
                       steps_per_degree, feedforward_gain,
                       feedforward_min_error_deg=3.0):
    """Return an error-shaped tracking rate with target-rate feed-forward.

    Motion profile:
      - Near the target (error approaching deadband stop threshold), velocity
        eases down toward minimum_rate according to normalized_error^rate_exponent.
      - At or beyond full_speed_error_degrees, the error-proportional rate saturates
        at maximum_rate.
      - Bearing-rate feed-forward: When chasing a moving target (error >= feedforward_min_error_deg),
        target angular speed is converted to step rate to help the tracker match
        the person's walking pace without lagging behind. Below feedforward_min_error_deg of error,
        feed-forward is gated off to prevent radar coordinate noise from creating
        tick vibrations at rest.
    """
    error_span = max(0.001,
                     full_speed_error_degrees - minimum_error_degrees)
    normalized_error = max(0.0, abs(error_degrees) - minimum_error_degrees)
    normalized_error = min(1.0, normalized_error / error_span)
    error_rate = (minimum_rate + (maximum_rate - minimum_rate) *
                  (normalized_error ** rate_exponent))

    # Feed-forward is only activated when error >= threshold to prevent noise jitter
    feedforward_active = abs(error_degrees) >= feedforward_min_error_deg
    feedforward_rate = (abs(bearing_rate_deg_s) * steps_per_degree *
                        feedforward_gain) if feedforward_active else 0.0

    requested_rate = max(minimum_rate, error_rate, feedforward_rate)
    return min(maximum_rate, max(minimum_rate, round(requested_rate)))


def slew_step_rate(current_rate, desired_rate, elapsed_ms,
                   maximum_change_pps_s):
    """Ramp step rate toward desired_rate without exceeding maximum acceleration.

    Parameters:
      current_rate: Current step rate in pulses per second.
      desired_rate: Target step rate in pulses per second.
      elapsed_ms: Time in milliseconds since last rate update.
      maximum_change_pps_s: Maximum acceleration/deceleration in pulses/s^2.
    """
    if elapsed_ms <= 0:
        return current_rate
    maximum_delta = maximum_change_pps_s * elapsed_ms / 1000.0
    difference = desired_rate - current_rate
    if abs(difference) <= maximum_delta:
        return desired_rate
    if difference > 0:
        return round(current_rate + maximum_delta)
    return round(current_rate - maximum_delta)


def smooth_toward_target(current, target, elapsed_ms, smoothing_ms):
    """Apply exponential moving average to filter target angle jitter.

    Parameters:
      current: Currently smoothed angle (or None if unseeded).
      target: New raw target angle.
      elapsed_ms: Milliseconds since previous smoothing tick.
      smoothing_ms: Time constant in milliseconds (higher = smoother, lower = faster).
    """
    if current is None:
        return target
    if elapsed_ms <= 0:
        return current
    if smoothing_ms <= 0:
        return target
    alpha = min(1.0, elapsed_ms / smoothing_ms)
    return current + (target - current) * alpha


def update_deadband(error_degrees, active, start_degrees, stop_degrees):
    """Apply dual-threshold start/stop hysteresis.

    Hysteresis behavior:
      - If currently stationary (active=False): only start moving if error
        exceeds start_degrees (e.g. 0.225 deg / 2 microsteps).
      - If already moving (active=True): continue moving until error drops
        below stop_degrees (e.g. 0.0 deg / exact target).
    """
    if active:
        return error_degrees > stop_degrees
    return error_degrees >= start_degrees


class TargetSelector:
    """Legacy single-tracker target selector with nearest-neighbor track continuity.

    Retained for standalone diagnostics and backwards-compatible unit testing.
    Picks the nearest target on initial detection, then maintains continuity
    with that target across frames using nearest-neighbor Euclidean distance.
    """

    def __init__(self, minimum_range_mm, maximum_range_mm):
        self.minimum_range_mm = minimum_range_mm
        self.maximum_range_mm = maximum_range_mm
        self.previous = None

    def choose(self, targets):
        """Select the most continuous valid target from a list of radar targets."""
        candidates = [t for t in targets
                      if valid_target(t, self.minimum_range_mm, self.maximum_range_mm)]
        if not candidates:
            return None

        if self.previous is None:
            # First acquisition: choose closest to the sensor
            selected = min(candidates,
                           key=lambda item: item[0] * item[0] + item[1] * item[1])
        else:
            # Maintain track continuity: choose target closest to previous position
            old_x, old_y = self.previous
            selected = min(candidates,
                           key=lambda item: (item[0] - old_x) * (item[0] - old_x) +
                           (item[1] - old_y) * (item[1] - old_y))
        self.previous = (selected[0], selected[1])
        return selected