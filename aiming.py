"""Kinematic and trigonometric geometry for mapping radar targets to tracker angles.

Coordinate System Conventions:
  - Radar Origin (0, 0): Physical center of the LD2450 antenna face.
  - +Y axis: Forward into the room / detection zone.
  - +X axis: Lateral offset toward sensor-left (with current upside-down radar mounting).
  - Yaw (Pan): Rotation in the horizontal plane in degrees. 0 deg faces directly
    forward along +Y; positive yaw points toward +X (sensor-left).
"""

import math


def solve_yaw(target_x_mm, target_y_mm, yaw_pivot_x_mm=0.0, yaw_pivot_y_mm=0.0, **kwargs):
    """Compute line-of-sight yaw angle (degrees) from yaw pivot to target.

    Parameters:
      target_x_mm, target_y_mm:
        Target location in radar-centric millimetres.
      yaw_pivot_x_mm, yaw_pivot_y_mm:
        (x, y) location of the tracker's yaw axis pivot relative to the radar.

    Returns:
      yaw_deg in degrees. 0 deg points along +Y; positive yaw points toward +X.
    """
    if "pan_pivot_x_mm" in kwargs:
        yaw_pivot_x_mm = kwargs["pan_pivot_x_mm"]
    if "pan_pivot_y_mm" in kwargs:
        yaw_pivot_y_mm = kwargs["pan_pivot_y_mm"]
    dx = target_x_mm - yaw_pivot_x_mm
    dy = target_y_mm - yaw_pivot_y_mm
    return math.degrees(math.atan2(dx, dy))


def clamp(value, minimum, maximum):
    """Constrain value within the [minimum, maximum] range."""
    return max(minimum, min(maximum, value))
