"""
Authoritative Reference Oracle for Spatial Lever-Arm and Temporal Latency Compensation.
Implements equations from PROJECT.md (F3.1, F3.2) and Calib_2.docx.
"""
import numpy as np


class LeverArmCompensator:
    """
    Rigid-body kinematic lever-arm compensation oracle.
    Relates body-center acceleration a_B to sensor-frame acceleration a_S:
      a_S = R_B^S * (a_B + alpha x p + omega x (omega x p))
      a_B = (R_B^S)^T * a_S - alpha x p - omega x (omega x p)
    """

    def __init__(self, p_lever_arm: tuple[float, float, float] = (0.05, 0.0, 0.0),
                 r_b_s: np.ndarray | None = None):
        self.p = np.array(p_lever_arm, dtype=np.float64)  # Lever-arm vector [px, py, pz]
        if r_b_s is None:
            self.R_b_s = np.eye(3, dtype=np.float64)
        else:
            self.R_b_s = np.asarray(r_b_s, dtype=np.float64)

    def compute_centrifugal_acceleration(self, omega: np.ndarray) -> np.ndarray:
        """Compute omega x (omega x p)."""
        w = np.asarray(omega, dtype=np.float64)
        return np.cross(w, np.cross(w, self.p))

    def compute_tangential_acceleration(self, alpha: np.ndarray) -> np.ndarray:
        """Compute alpha x p."""
        a = np.asarray(alpha, dtype=np.float64)
        return np.cross(a, self.p)

    def sensor_to_body_acceleration(self, a_sensor: np.ndarray,
                                    omega: np.ndarray,
                                    alpha: np.ndarray) -> np.ndarray:
        """
        Strip lever-arm tangential and centrifugal acceleration from raw sensor measurement
        to recover true base_link linear acceleration.
        """
        a_s = np.asarray(a_sensor, dtype=np.float64)
        a_centrifugal = self.compute_centrifugal_acceleration(omega)
        a_tangential = self.compute_tangential_acceleration(alpha)
        # Transform sensor frame to body frame
        a_in_body_frame = self.R_b_s.T @ a_s
        a_base = a_in_body_frame - a_tangential - a_centrifugal
        return a_base

    def body_to_sensor_acceleration(self, a_base: np.ndarray,
                                    omega: np.ndarray,
                                    alpha: np.ndarray) -> np.ndarray:
        """Forward kinematic model: what sensor measures given true body motion."""
        a_b = np.asarray(a_base, dtype=np.float64)
        a_centrifugal = self.compute_centrifugal_acceleration(omega)
        a_tangential = self.compute_tangential_acceleration(alpha)
        return self.R_b_s @ (a_b + a_tangential + a_centrifugal)


class TemporalLatencyCompensator:
    """
    Temporal latency compensation oracle.
    Calculates dynamic displacement delta_s = v * delta_t and aligns asynchronous time-series.
    """

    def __init__(self, latency_s: float = 0.020):
        self.latency_s = float(latency_s)

    def compensate_position(self, velocity_mps: float, measured_pos: float) -> float:
        """Compute time-aligned position given forward velocity and latency."""
        return measured_pos + velocity_mps * self.latency_s

    def align_timestamp(self, measurement_timestamp: float) -> float:
        """Subtract communication/computation delay from timestamp."""
        return measurement_timestamp - self.latency_s

    @staticmethod
    def interpolate_signal(query_time: float, timestamps: np.ndarray, values: np.ndarray) -> float:
        """Piecewise linear interpolation for asynchronous sensor time synchronization."""
        t_arr = np.asarray(timestamps, dtype=np.float64)
        v_arr = np.asarray(values, dtype=np.float64)
        return float(np.interp(query_time, t_arr, v_arr))
