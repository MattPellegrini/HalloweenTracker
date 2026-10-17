"""Select stable LD2450 targets and shape natural stepper tracking motion."""


def valid_target(target, minimum_range_mm, maximum_range_mm):
    """Return True when a radar slot holds an in-front target within range."""
    if target is None:
        return False
    x_mm, y_mm, speed_cm_s, resolution_mm = target
    distance_squared = x_mm * x_mm + y_mm * y_mm
    return (y_mm > 0 and minimum_range_mm ** 2 <= distance_squared <=
            maximum_range_mm ** 2)


def choose_two_targets(slots, minimum_range_mm, maximum_range_mm,
                       assignment_mode):
    """Return the (skull1, skull2) targets; either may be None.

    Modes:
      nearest_two - skull 1 gets the nearest valid target, skull 2 the
                    next-nearest. Either may be None when few targets exist.
      fixed_slot  - skull 1 always gets radar slot 1, skull 2 slot 2. An
                    empty or invalid slot means that skull idles/parks.
    """
    if assignment_mode == "fixed_slot":
        first = slots[0] if valid_target(slots[0], minimum_range_mm,
                                         maximum_range_mm) else None
        second = slots[1] if valid_target(slots[1], minimum_range_mm,
                                          maximum_range_mm) else None
        return first, second
    valid = [target for target in slots
             if valid_target(target, minimum_range_mm, maximum_range_mm)]
    valid.sort(key=lambda item: item[0] ** 2 + item[1] ** 2)
    first = valid[0] if len(valid) > 0 else None
    second = valid[1] if len(valid) > 1 else None
    return first, second


def tracking_step_rate(error_degrees, bearing_rate_deg_s, minimum_rate,
                       maximum_rate, full_speed_error_degrees,
                       minimum_error_degrees, rate_exponent,
                       steps_per_degree, feedforward_gain):
    """Return an error-shaped tracking rate with target-rate feed-forward.

    The position-error term reaches full speed at full_speed_error_degrees and
    eases toward minimum_rate near the stop deadband. Bearing-rate feed-forward
    keeps pace with a moving target without letting sensor noise dominate near
    the target.
    """
    error_span = max(0.001,
                     full_speed_error_degrees - minimum_error_degrees)
    normalized_error = max(0.0, abs(error_degrees) - minimum_error_degrees)
    normalized_error = min(1.0, normalized_error / error_span)
    error_rate = (minimum_rate + (maximum_rate - minimum_rate) *
                  (normalized_error ** rate_exponent))
    # Feed-forward only helps when genuinely chasing a moving target. Near the
    # floor it just amplifies sensor noise, so gate it off for small errors.
    feedforward_active = abs(error_degrees) >= 3.0
    feedforward_rate = (abs(bearing_rate_deg_s) * steps_per_degree *
                        feedforward_gain) if feedforward_active else 0.0
    requested_rate = max(minimum_rate, error_rate, feedforward_rate)
    return min(maximum_rate, max(minimum_rate, round(requested_rate)))


def slew_step_rate(current_rate, desired_rate, elapsed_ms,
                   maximum_change_pps_s):
    """Move a STEP rate toward its target without an instantaneous jump."""
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
    """Return an exponentially smoothed step from current toward target."""
    if current is None:
        return target
    if elapsed_ms <= 0:
        return current
    if smoothing_ms <= 0:
        return target
    alpha = min(1.0, elapsed_ms / smoothing_ms)
    return current + (target - current) * alpha


def update_deadband(error_degrees, active, start_degrees, stop_degrees):
    """Apply start/stop hysteresis and return whether axis tracking is active."""
    if active:
        return error_degrees > stop_degrees
    return error_degrees >= start_degrees


class TargetSelector:
    def __init__(self, minimum_range_mm, maximum_range_mm):
        self.minimum_range_squared = minimum_range_mm ** 2
        self.maximum_range_squared = maximum_range_mm ** 2
        self.previous = None

    def choose(self, targets):
        candidates = []
        for target in targets:
            x_mm, y_mm, speed_cm_s, resolution_mm = target
            distance_squared = x_mm * x_mm + y_mm * y_mm
            if y_mm <= 0:
                continue
            if distance_squared < self.minimum_range_squared:
                continue
            if distance_squared > self.maximum_range_squared:
                continue
            candidates.append(target)

        if not candidates:
            return None

        if self.previous is None:
            selected = min(candidates,
                           key=lambda item: item[0] ** 2 + item[1] ** 2)
        else:
            old_x, old_y = self.previous
            selected = min(candidates,
                           key=lambda item: (item[0] - old_x) ** 2 +
                           (item[1] - old_y) ** 2)
        self.previous = (selected[0], selected[1])
        return selected