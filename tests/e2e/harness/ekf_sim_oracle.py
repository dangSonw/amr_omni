"""
Authoritative Reference Oracle for 2D EKF State Estimation and Sensor Fusion.
Implements planar state estimation model fusing wheel odometry (vx, vy, wz) and IMU (yaw, wz, ax, ay)
following PROJECT.md (F3.3) and amr_omni.docx.
"""
import math
import numpy as np


class EKFSimOracle2D:
    """
    Planar EKF State Estimator Oracle.
    State vector: [x, y, yaw, vx, vy, wz]^T (6 states)
    Fuses:
      - Wheel Odometry: [vx, vy, wz]
      - IMU: [yaw, wz, ax, ay]
    """

    def __init__(self, process_noise_diag: list[float] | None = None,
                 initial_cov_diag: list[float] | None = None):
        # State: [x, y, yaw, vx, vy, wz]
        self.state = np.zeros(6, dtype=np.float64)

        if initial_cov_diag is None:
            initial_cov_diag = [1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2]
        self.P = np.diag(np.array(initial_cov_diag, dtype=np.float64))

        if process_noise_diag is None:
            process_noise_diag = [0.01, 0.01, 0.005, 0.05, 0.05, 0.02]
        self.Q = np.diag(np.array(process_noise_diag, dtype=np.float64))

        # Default measurement noises
        self.R_odom = np.diag([0.02, 0.02, 0.01])       # [vx, vy, wz]
        self.R_imu = np.diag([0.005, 0.01, 0.1, 0.1])    # [yaw, wz, ax, ay]

    def predict(self, dt: float):
        """Kinematic state propagation step."""
        if dt <= 0.0:
            return

        x, y, yaw, vx, vy, wz = self.state

        # Propagate pose using body-frame velocities rotated to world frame
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        dx_world = (vx * cos_y - vy * sin_y) * dt
        dy_world = (vx * sin_y + vy * cos_y) * dt
        dyaw = wz * dt

        self.state[0] += dx_world
        self.state[1] += dy_world
        self.state[2] = (self.state[2] + dyaw + math.pi) % (2.0 * math.pi) - math.pi

        # State transition Jacobian F
        F = np.eye(6, dtype=np.float64)
        F[0, 2] = (-vx * sin_y - vy * cos_y) * dt
        F[0, 3] = cos_y * dt
        F[0, 4] = -sin_y * dt
        F[1, 2] = (vx * cos_y - vy * sin_y) * dt
        F[1, 3] = sin_y * dt
        F[1, 4] = cos_y * dt
        F[2, 5] = dt

        self.P = F @ self.P @ F.T + self.Q * dt
        # Maintain symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_odom(self, vx: float, vy: float, wz: float, r_cov: np.ndarray | None = None):
        """Update filter with wheel odometry twist."""
        z = np.array([vx, vy, wz], dtype=np.float64)
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 3] = 1.0  # vx
        H[1, 4] = 1.0  # vy
        H[2, 5] = 1.0  # wz

        R = r_cov if r_cov is not None else self.R_odom
        self._apply_measurement_update(z, H, R)

    def update_imu(self, yaw: float, wz: float, ax: float = 0.0, ay: float = 0.0,
                   r_cov: np.ndarray | None = None):
        """Update filter with IMU measurements."""
        z = np.array([yaw, wz], dtype=np.float64)
        H = np.zeros((2, 6), dtype=np.float64)
        H[0, 2] = 1.0  # yaw
        H[1, 5] = 1.0  # wz

        R = r_cov[:2, :2] if r_cov is not None else self.R_imu[:2, :2]

        # Angle wrap for yaw innovation
        y = z - H @ self.state
        y[0] = (y[0] + math.pi) % (2.0 * math.pi) - math.pi

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state += K @ y
        self.state[2] = (self.state[2] + math.pi) % (2.0 * math.pi) - math.pi

        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def _apply_measurement_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        y = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state += K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def is_healthy(self) -> bool:
        """Check for NaN/Inf and positive diagonal covariances."""
        if not np.all(np.isfinite(self.state)):
            return False
        if not np.all(np.isfinite(self.P)):
            return False
        diag = np.diag(self.P)
        if np.any(diag <= 0.0):
            return False
        return True
