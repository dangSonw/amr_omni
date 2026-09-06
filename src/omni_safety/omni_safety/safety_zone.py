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

    min_dist = float("inf")
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < min_valid_range_m:
            continue
        if r < min_dist:
            min_dist = r

    if min_dist < stop_zone_m:
        return True, 0.0, min_dist
    elif min_dist < slow_zone_m:
        return False, 0.5, min_dist
    else:
        return False, 1.0, min_dist

