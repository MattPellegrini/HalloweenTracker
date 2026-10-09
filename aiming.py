"""Pure geometry helpers for mapping radar coordinates to skull angles."""

import math


def solve_angles(target_x_mm, target_y_mm, target_z_mm,
                 pan_pivot_xyz, tilt_pivot_offset_xyz):
    """Return (yaw_deg, pitch_deg) for a target expressed in radar coordinates.

    The tilt-pivot offset is expressed in the pan assembly's local frame.
    Two iterations account for the way that offset moves as yaw changes.
    """
    pan_x, pan_y, pan_z = pan_pivot_xyz
    tilt_x, tilt_y, tilt_z = tilt_pivot_offset_xyz
    yaw = math.atan2(target_x_mm - pan_x, target_y_mm - pan_y)

    for _ in range(2):
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        tilt_pivot_world_x = pan_x + cos_yaw * tilt_x + sin_yaw * tilt_y
        tilt_pivot_world_y = pan_y - sin_yaw * tilt_x + cos_yaw * tilt_y
        yaw = math.atan2(target_x_mm - tilt_pivot_world_x,
                         target_y_mm - tilt_pivot_world_y)

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
    return max(minimum, min(maximum, value))