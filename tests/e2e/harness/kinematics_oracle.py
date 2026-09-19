"""
Authoritative Reference Oracle for Mecanum Kinematics and Kr Error Compensation.
Implements equations from PROJECT.md (F1.3) and amr_omni.docx.
"""
import math
import numpy as np


class MecanumKinematicsOracle:
    """
    Reference Mecanum Kinematics model supporting individual wheel radius compensation (Kr).
    """

    def __init__(self, wheelbase_m: float = 0.1312, track_width_m: float = 0.1312,
                 wheel_radii: tuple[float, float, float, float] = (0.03, 0.03, 0.03, 0.03),
                 max_wheel_speed_rad_s: float = 100.0):
        if wheelbase_m <= 0 or track_width_m <= 0:
            raise ValueError("Wheelbase and track width must be positive")
        if any(r <= 0 for r in wheel_radii):
            raise ValueError("Wheel radii must be positive")
        if max_wheel_speed_rad_s <= 0:
            raise ValueError("max_wheel_speed_rad_s must be positive")

        self.wheelbase_m = float(wheelbase_m)
        self.track_width_m = float(track_width_m)
        self.wheel_radii = tuple(float(r) for r in wheel_radii)
        self.max_wheel_speed_rad_s = float(max_wheel_speed_rad_s)

        # Geometry
        self.d = float(np.sqrt(0.5))
        self.radius = 0.5 * float(np.hypot(self.wheelbase_m, self.track_width_m))

        # Kinematics coupling matrix M
        # Wheel 1 (FR: -45 deg), Wheel 2 (FL: +45 deg), Wheel 3 (RL: +135 deg), Wheel 4 (RR: -135 deg)
        self.M = np.array([
            [ self.d,  self.d, self.radius],
            [-self.d,  self.d, self.radius],
            [-self.d, -self.d, self.radius],
            [ self.d, -self.d, self.radius],
        ], dtype=np.float64)

        # Moore-Penrose pseudo-inverse of M
        self.M_pinv = 0.25 * np.array([
            [ 1.0 / self.d, -1.0 / self.d, -1.0 / self.d,  1.0 / self.d],
            [ 1.0 / self.d,  1.0 / self.d, -1.0 / self.d, -1.0 / self.d],
            [ 1.0 / self.radius, 1.0 / self.radius, 1.0 / self.radius, 1.0 / self.radius],
        ], dtype=np.float64)

    def inverse_kinematics(self, vx: float, vy: float, wz: float, apply_scaling: bool = True) -> tuple[float, float, float, float]:
        """Convert body twist (vx, vy, wz) to 4 wheel angular speeds (rad/s) with Kr compensation."""
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            raise ValueError("Twist velocities must be finite")

        body_twist = np.array([vx, vy, wz], dtype=np.float64)
        linear_wheel_speeds = self.M @ body_twist

        # Individual radius compensation Kr = diag(r1, r2, r3, r4)
        angular_wheel_speeds = np.zeros(4, dtype=np.float64)
        for i in range(4):
            angular_wheel_speeds[i] = linear_wheel_speeds[i] / self.wheel_radii[i]

        if apply_scaling:
            max_val = float(np.max(np.abs(angular_wheel_speeds)))
            scale = max(1.0, max_val / self.max_wheel_speed_rad_s)
            angular_wheel_speeds = angular_wheel_speeds / scale

        return tuple(float(s) for s in angular_wheel_speeds)

    def forward_kinematics(self, wheel_speeds_rad_s: tuple[float, float, float, float]) -> tuple[float, float, float]:
        """Convert 4 wheel angular speeds (rad/s) with Kr compensation back to body twist (vx, vy, wz)."""
        if len(wheel_speeds_rad_s) != 4:
            raise ValueError("Four wheel speeds required")
        if not all(math.isfinite(s) for s in wheel_speeds_rad_s):
            raise ValueError("Wheel speeds must be finite")

        # Convert angular speeds back to linear wheel speeds using individual radii Kr
        linear_wheel_speeds = np.zeros(4, dtype=np.float64)
        for i in range(4):
            linear_wheel_speeds[i] = float(wheel_speeds_rad_s[i]) * self.wheel_radii[i]

        body_twist = self.M_pinv @ linear_wheel_speeds
        return float(body_twist[0]), float(body_twist[1]), float(body_twist[2])

    def round_trip_error(self, vx: float, vy: float, wz: float) -> float:
        """Compute Euclidean norm between original twist and forward(inverse(twist))."""
        wheel_speeds = self.inverse_kinematics(vx, vy, wz, apply_scaling=False)
        reconstructed_twist = self.forward_kinematics(wheel_speeds)
        diff = np.array([vx - reconstructed_twist[0],
                         vy - reconstructed_twist[1],
                         wz - reconstructed_twist[2]])
        return float(np.linalg.norm(diff))
