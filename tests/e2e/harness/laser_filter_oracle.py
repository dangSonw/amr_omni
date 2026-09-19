"""
Authoritative Reference Oracle for LaserScanBoxFilter Footprint Masking.
Implements geometric footprint masking according to PROJECT.md (F3.5) and amr_omni.docx.
Box limits: [-0.135, 0.135] x [-0.135, 0.135] m.
"""
import math
import numpy as np


class LaserFootprintFilterOracle:
    """
    Evaluates 2D laser scan points against chassis footprint mask.
    Removes chassis self-reflections without masking external obstacles.
    """

    def __init__(self, min_x: float = -0.135, max_x: float = 0.135,
                 min_y: float = -0.135, max_y: float = 0.135):
        self.min_x = float(min_x)
        self.max_x = float(max_x)
        self.min_y = float(min_y)
        self.max_y = float(max_y)

    def is_point_inside(self, x: float, y: float) -> bool:
        """Returns True if (x, y) is inside the chassis footprint box."""
        return (self.min_x <= x <= self.max_x) and (self.min_y <= y <= self.max_y)

    def filter_cartesian_point(self, x: float, y: float) -> tuple[float, float, bool]:
        """
        Filters point:
        If inside footprint, returns (nan, nan, False).
        If outside, returns (x, y, True).
        """
        if self.is_point_inside(x, y):
            return float('nan'), float('nan'), False
        return float(x), float(y), True

    def filter_scan_ranges(self, ranges: list[float], angle_min: float,
                           angle_increment: float, laser_offset_x: float = 0.0,
                           laser_offset_y: float = 0.0) -> list[float]:
        """
        Filter polar laser scan ranges. Any ray ending inside the chassis footprint
        is masked to NaN.
        """
        filtered_ranges = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                filtered_ranges.append(r)
                continue

            angle = angle_min + i * angle_increment
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # Transform from laser_link to base_link
            x_base = x_laser + laser_offset_x
            y_base = y_laser + laser_offset_y

            if self.is_point_inside(x_base, y_base):
                filtered_ranges.append(float('nan'))
            else:
                filtered_ranges.append(r)

        return filtered_ranges
