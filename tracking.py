"""Select a stable LD2450 target from up to three reported tracks."""


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