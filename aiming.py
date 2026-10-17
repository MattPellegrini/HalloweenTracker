"""Kinematic and trigonometric geometry for mapping radar targets to skull angles.

Coordinate System Conventions:
  - Radar Origin (0, 0, 0): Physical center of the LD2450 antenna face.
  - +Y axis: Forward into the room / detection zone.
  - +X axis: Lateral offset toward sensor-left (with current upside-down radar mounting).
  - +Z axis: Vertical elevation above the radar plane.
  - Yaw (Pan): Rotation in the horizontal plane in degrees. 0 deg faces directly
    forward along +Y; positive yaw points toward +X (sensor-left).
  - Pitch (Tilt): Vertical elevation angle in degrees. 0 deg is horizontal;
    positive pitch angles point upward.
"""

import math


def solve_angles(target_x_mm, target_y_mm, target_z_mm,
                 pan_pivot_xyz, tilt_pivot_offset_xyz):
    """Compute line-of-sight (yaw_deg, pitch_deg) from skull pivot to target.

    Parameters:
      target_x_mm, target_y_mm, target_z_mm:
        Target location in radar-centric millimetres.
      pan_pivot_xyz:
        (x, y, z) location of the skull's pan axis pivot relative to the radar.
      tilt_pivot_offset_xyz:
        (x, y, z) mechanical offset of the tilt axis from the pan axis, expressed
        in the pan head's local coordinate frame.

    Kinematics:
      When tilt_pivot_offset is non-zero, rotating the pan axis swings the tilt
      pivot through space. A two-iteration relaxation converges on the exact
      sightline yaw. When tilt_pivot_offset is (0, 0, 0) (as in the current 1-axis
      dual-skull pan system), this simplifies directly to atan2(dx, dy).

    Returns:
      (yaw_deg, pitch_deg) in degrees.
    """
    pan_x, pan_y, pan_z = pan_pivot_xyz
    tilt_x, tilt_y, tilt_z = tilt_pivot_offset_xyz

    # Initial yaw estimate from pan pivot to target
    yaw = math.atan2(target_x_mm - pan_x, target_y_mm - pan_y)

    # Two iterations to converge if tilt pivot is offset from pan axis
    for _ in range(2):
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        tilt_pivot_world_x = pan_x + cos_yaw * tilt_x + sin_yaw * tilt_y
        tilt_pivot_world_y = pan_y - sin_yaw * tilt_x + cos_yaw * tilt_y
        yaw = math.atan2(target_x_mm - tilt_pivot_world_x,
                         target_y_mm - tilt_pivot_world_y)

    # Compute final 3D world position of the sightline origin
    sin_yaw = math.sin(yaw)
    cos_yaw = math.cos(yaw)
    tilt_pivot_world_x = pan_x + cos_yaw * tilt_x + sin_yaw * tilt_y
    tilt_pivot_world_y = pan_y - sin_yaw * tilt_x + cos_yaw * tilt_y
    tilt_pivot_world_z = pan_z + tilt_z

    dx = target_x_mm - tilt_pivot_world_x
    dy = target_y_mm - tilt_pivot_world_y
    dz = target_z_mm - tilt_pivot_world_z
    pitch = math.atan2(dz, math.sqrt(dx * dx + dy * dy))
    return math.degrees(yaw), math.degrees(pitch)


def clamp(value, minimum, maximum):
    """Constrain value within the [minimum, maximum] range."""
    return max(minimum, min(maximum, value))