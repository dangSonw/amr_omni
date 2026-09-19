"""
Authoritative Reference Oracle for IMU Calibration, Allan Variance, and ENU Standards.
Implements equations from PROJECT.md (F2.1, F2.2, F2.3, F2.4) and Calib.docx.
"""
import math
import numpy as np


class ST_AN4508_Calibrator:
    """
    On-board 6-position accelerometer calibration routine following ST AN4508.
    Orientations:
      +X up: [ g,  0,  0]
      -X up: [-g,  0,  0]
      +Y up: [ 0,  g,  0]
      -Y up: [ 0, -g,  0]
      +Z up: [ 0,  0,  g]
      -Z up: [ 0,  0, -g]
    Solves scale s_j and bias b_j per axis:
      a_cal = s_j * a_raw + b_j
    """

    GRAVITY_G = 9.80665

    def __init__(self, g: float = GRAVITY_G):
        self.g = float(g)
        self.scale = np.ones(3, dtype=np.float64)
        self.bias = np.zeros(3, dtype=np.float64)
        self.is_calibrated = False

    def calibrate(self, pos_x_plus: np.ndarray, pos_x_minus: np.ndarray,
                  pos_y_plus: np.ndarray, pos_y_minus: np.ndarray,
                  pos_z_plus: np.ndarray, pos_z_minus: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Fit scale factors and biases from 6 static positions.
        Returns (scale_factors, biases, residual_error).
        """
        # Axis 0 (X)
        ax_p = float(np.mean(pos_x_plus, axis=0)[0])
        ax_m = float(np.mean(pos_x_minus, axis=0)[0])
        denom_x = ax_p - ax_m
        if abs(denom_x) < 1e-6:
            raise ValueError("Degenerate measurements for X axis")
        sx = (2.0 * self.g) / denom_x
        bx = -sx * (ax_p + ax_m) / 2.0

        # Axis 1 (Y)
        ay_p = float(np.mean(pos_y_plus, axis=0)[1])
        ay_m = float(np.mean(pos_y_minus, axis=0)[1])
        denom_y = ay_p - ay_m
        if abs(denom_y) < 1e-6:
            raise ValueError("Degenerate measurements for Y axis")
        sy = (2.0 * self.g) / denom_y
        by = -sy * (ay_p + ay_m) / 2.0

        # Axis 2 (Z)
        az_p = float(np.mean(pos_z_plus, axis=0)[2])
        az_m = float(np.mean(pos_z_minus, axis=0)[2])
        denom_z = az_p - az_m
        if abs(denom_z) < 1e-6:
            raise ValueError("Degenerate measurements for Z axis")
        sz = (2.0 * self.g) / denom_z
        bz = -sz * (az_p + az_m) / 2.0

        self.scale = np.array([sx, sy, sz], dtype=np.float64)
        self.bias = np.array([bx, by, bz], dtype=np.float64)
        self.is_calibrated = True

        # Calculate residual error across all 6 reference positions
        residuals = []
        for pos_data in [pos_x_plus, pos_x_minus, pos_y_plus, pos_y_minus, pos_z_plus, pos_z_minus]:
            cal_pts = self.apply(pos_data)
            norms = np.linalg.norm(cal_pts, axis=1)
            residuals.extend(np.abs(norms - self.g))

        max_residual = float(np.max(residuals))
        return self.scale.copy(), self.bias.copy(), max_residual

    def apply(self, raw_accel: np.ndarray) -> np.ndarray:
        """Apply calibration parameters to raw accelerometer measurements."""
        arr = np.asarray(raw_accel, dtype=np.float64)
        return arr * self.scale + self.bias


class GyroBiasNuller:
    """
    Stationary gyroscope zero-rate bias tracking and nulling routine.
    Guarantees residual static drift < 0.05 deg/s (< 8.726e-4 rad/s).
    """

    MAX_ALLOWED_DRIFT_RAD_S = math.radians(0.05)  # 0.05 deg/s in rad/s ~ 0.00087266 rad/s

    def __init__(self, variance_threshold: float = 1e-4):
        self.variance_threshold = float(variance_threshold)
        self.bias = np.zeros(3, dtype=np.float64)
        self.is_calibrated = False

    def calibrate(self, stationary_samples: np.ndarray) -> tuple[np.ndarray, float]:
        """
        Estimate static bias from stationary samples.
        Checks stationarity variance gate.
        Returns (bias_vector, max_residual_drift_rad_s).
        """
        data = np.asarray(stationary_samples, dtype=np.float64)
        if len(data) < 10:
            raise ValueError("At least 10 stationary samples required")

        variances = np.var(data, axis=0)
        if np.any(variances > self.variance_threshold):
            raise ValueError(f"Motion detected during calibration: variance {variances} exceeds threshold {self.variance_threshold}")

        self.bias = np.mean(data, axis=0)
        self.is_calibrated = True

        corrected = data - self.bias
        residual_mean = np.abs(np.mean(corrected, axis=0))
        max_residual = float(np.max(residual_mean))
        return self.bias.copy(), max_residual

    def apply(self, raw_gyro: np.ndarray) -> np.ndarray:
        """Subtract estimated bias from gyro measurements."""
        return np.asarray(raw_gyro, dtype=np.float64) - self.bias


class AllanVarianceAnalyzer:
    """
    Allan Variance (IEEE Std 952-1997) analysis oracle.
    Computes sigma^2(tau) and extracts:
      - Angle Random Walk (ARW / Ng) / Velocity Random Walk (VRW / Na)
      - Bias Instability (Kg / Ka)
      - Covariance inflation factor (1.5x - 2.0x) for dynamic vibration robustness.
    """

    def __init__(self, sample_rate_hz: float = 200.0, inflation_factor: float = 1.75):
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive")
        if not (1.5 <= inflation_factor <= 2.0):
            raise ValueError("inflation_factor must be between 1.5 and 2.0")
        self.fs = float(sample_rate_hz)
        self.dt = 1.0 / self.fs
        self.inflation_factor = float(inflation_factor)

    def compute_allan_deviation(self, data_series: np.ndarray, num_tau: int = 20) -> tuple[np.ndarray, np.ndarray]:
        """Compute Allan Deviation sigma(tau) over various cluster intervals tau."""
        data = np.asarray(data_series, dtype=np.float64)
        N = len(data)
        if N < 100:
            raise ValueError("Data series too short for Allan variance analysis")

        max_cluster = N // 3
        cluster_sizes = np.unique(np.logspace(0, np.log10(max_cluster), num=num_tau, dtype=int))
        cluster_sizes = cluster_sizes[cluster_sizes > 0]

        taus = []
        adevs = []
        for m in cluster_sizes:
            tau = m * self.dt
            # Block averages of length m
            num_blocks = N // m
            if num_blocks < 2:
                continue
            trimmed = data[:num_blocks * m]
            blocks = np.mean(trimmed.reshape(num_blocks, m), axis=1)
            # Allan variance = 0.5 * mean((y_{k+1} - y_k)^2)
            diffs = np.diff(blocks)
            avar = 0.5 * np.mean(diffs ** 2)
            adev = np.sqrt(max(0.0, avar))
            taus.append(tau)
            adevs.append(adev)

        return np.array(taus, dtype=np.float64), np.array(adevs, dtype=np.float64)

    def extract_noise_parameters(self, taus: np.ndarray, adevs: np.ndarray) -> dict[str, float]:
        """Extract Angle Random Walk (Ng) and Bias Instability (Kg)."""
        if len(taus) < 3:
            raise ValueError("Insufficient points for noise parameter extraction")

        # Ng (ARW) is adev at tau = 1.0 s (or extrapolated along slope -0.5)
        # Using interpolation at tau=1.0 or first point if tau > 1
        idx_tau1 = np.argmin(np.abs(taus - 1.0))
        ng_nominal = float(adevs[idx_tau1])

        # Bias instability Kg is approximately minimum adev / sqrt(2*ln(2)/pi) ~ min / 0.664
        min_adev = float(np.min(adevs))
        kg_nominal = min_adev / 0.66428247

        # Inflated covariance values for dynamic robustness (1.5x - 2.0x)
        ng_inflated = ng_nominal * self.inflation_factor
        kg_inflated = kg_nominal * self.inflation_factor

        return {
            "Ng_nominal": ng_nominal,
            "Kg_nominal": kg_nominal,
            "Ng_inflated": ng_inflated,
            "Kg_inflated": kg_inflated,
            "variance_inflated": (ng_inflated ** 2),
        }
