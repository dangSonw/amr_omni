import math
from typing import Sequence, Tuple


def evaluate_safety_zone(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    vx: float = 0.0,
    vy: float = 0.0,
    slow_zone_m: float = 1.0,
    stop_zone_m: float = 0.5,
    min_valid_range_m: float = 0.08,
) -> Tuple[bool, float, float]:
    """
    Evaluates proximity to obstacles from 2D range measurements for omni chassis.

    Returns:
        (safety_stop: bool, speed_factor: float, min_distance: float)
    """
    if stop_zone_m <= 0.0 or slow_zone_m <= stop_zone_m:
        raise ValueError("Invalid zone thresholds: must have 0 < stop_zone < slow_zone")

    speed = math.hypot(vx, vy)
    moving = speed > 0.02
    heading_rad = math.atan2(vy, vx) if moving else 0.0
    cone_half_angle_rad = math.radians(75.0)  # Directional safety cone ±75 deg
    immediate_bubble_m = max(min_valid_range_m + 0.05, stop_zone_m * 0.4)

    min_dist = float("inf")
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < min_valid_range_m:
            continue

        if moving:
            beam_angle = angle_min + i * angle_increment
            angle_diff = math.atan2(
                math.sin(beam_angle - heading_rad),
                math.cos(beam_angle - heading_rad)
            )
            in_motion_cone = abs(angle_diff) <= cone_half_angle_rad
            in_immediate_bubble = r < immediate_bubble_m

            if not (in_motion_cone or in_immediate_bubble):
                continue

        if r < min_dist:
            min_dist = r

    if min_dist < stop_zone_m:
        return True, 0.0, min_dist
    elif min_dist < slow_zone_m:
        return False, 0.5, min_dist
    else:
        return False, 1.0, min_dist

