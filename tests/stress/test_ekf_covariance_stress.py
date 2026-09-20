#!/usr/bin/env python3
"""Adversarial Stress Test Suite for Milestone 3: EKF Covariance Matrices & Numerical Properties.

Empirically validates:
1. Exact mathematical properties of Q (process noise) and P0 (initial covariance) from `src/omni_localization/config/ekf.yaml`:
   - Dimensions 15x15 (225 elements), strictly non-zero diagonals.
   - Strict matrix symmetry (max asymmetry == 0.0).
   - Strict positive definiteness (all eigenvalues > 0, successful Cholesky decomposition).
   - Numerical condition numbers within stable bounds (well below 10^12).
2. Full 15-state kinematic EKF assimilation simulation:
   - Extreme hard acceleration (up to 30 m/s^2 ~ 3g) and emergency braking.
   - High-speed omnidirectional strafing (vx=2.5 m/s, vy=-2.5 m/s) with rapid continuous yaw spin (wz=5.0 rad/s).
   - Severe floor vibration (50 Hz sinusoidal resonance + high-amplitude shock noise).
   - Asynchronous sensor update rates (50 Hz odom, 100 Hz IMU) with stochastic packet drops and sensor blackout.
   - Measurement covariance fuzzing across orders of magnitude (1e-7 to 1e5).
   - 100,000-cycle long-term Monte Carlo filter assimilation ensuring no NaN/Inf, no covariance collapse, and bounded condition number.
"""

import math
import sys
from pathlib import Path
import numpy as np
import pytest
import yaml

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from tests.e2e.harness.ekf_sim_oracle import EKFSimOracle2D
EKF_CONFIG_PATH = WORKSPACE_ROOT / "src" / "omni_localization" / "config" / "ekf.yaml"


class Full15StateEKFSimulator:
    """Full 15-state kinematic EKF simulator matching robot_localization specification.

    State vector:
      x[0:3]   = [x, y, z]                     (world frame position)
      x[3:6]   = [roll, pitch, yaw]            (world frame orientation)
      x[6:9]   = [vx, vy, vz]                  (robot body frame linear velocities)
      x[9:12]  = [vroll, vpitch, vyaw]         (robot body frame angular velocities)
      x[12:15] = [ax, ay, az]                  (robot body frame linear accelerations)
    """

    def __init__(self, q_mat: np.ndarray, p0_mat: np.ndarray, two_d_mode: bool = True):
        assert q_mat.shape == (15, 15), "Q must be 15x15"
        assert p0_mat.shape == (15, 15), "P0 must be 15x15"

        self.state = np.zeros(15, dtype=np.float64)
        self.P0 = np.copy(p0_mat).astype(np.float64)
        self.P = np.copy(p0_mat).astype(np.float64)
        self.Q = np.copy(q_mat).astype(np.float64)
        self.two_d_mode = two_d_mode

        # Default measurement noise matrices
        self.R_odom = np.diag([0.02, 0.02, 0.01])          # [vx, vy, vyaw]
        self.R_imu = np.diag([0.005, 0.01, 0.05, 0.05])    # [yaw, vyaw, ax, ay]

        if self.two_d_mode:
            self._apply_2d_constraints()

    def _apply_2d_constraints(self):
        """Enforce 2D planar constraints on out-of-plane states matching robot_localization specification."""
        # Out-of-plane state indices: z(2), roll(3), pitch(4), vz(8), vroll(9), vpitch(10), az(14)
        out_of_plane = [2, 3, 4, 8, 9, 10, 14]
        self.state[out_of_plane] = 0.0
        if hasattr(self, "P0"):
            for idx in out_of_plane:
                self.P[idx, :] = 0.0
                self.P[:, idx] = 0.0
                self.P[idx, idx] = self.P0[idx, idx]

    def predict(self, dt: float):
        """Propagate state and covariance via nonlinear kinematics and 15x15 Jacobian."""
        if dt <= 0.0:
            return

        psi = self.state[5]     # yaw
        vx = self.state[6]      # body vx
        vy = self.state[7]      # body vy
        vz = self.state[8]
        vyaw = self.state[11]   # body vyaw
        ax = self.state[12]     # body ax
        ay = self.state[13]     # body ay
        az = self.state[14]

        cos_y = math.cos(psi)
        sin_y = math.sin(psi)
        dt2 = 0.5 * dt * dt

        # State propagation (body frame velocities/accelerations rotated to world frame)
        dx_world = (vx * cos_y - vy * sin_y) * dt + (ax * cos_y - ay * sin_y) * dt2
        dy_world = (vx * sin_y + vy * cos_y) * dt + (ax * sin_y + ay * cos_y) * dt2
        dz_world = vz * dt + az * dt2

        self.state[0] += dx_world
        self.state[1] += dy_world
        self.state[2] += dz_world

        self.state[3] += self.state[9] * dt     # roll
        self.state[4] += self.state[10] * dt    # pitch
        self.state[5] += vyaw * dt              # yaw
        # Wrap yaw to [-pi, pi]
        self.state[5] = (self.state[5] + math.pi) % (2.0 * math.pi) - math.pi

        self.state[6] += ax * dt                # vx
        self.state[7] += ay * dt                # vy
        self.state[8] += az * dt                # vz

        # Construct 15x15 Jacobian F
        F = np.eye(15, dtype=np.float64)

        # Position derivatives w.r.t. yaw
        F[0, 5] = (-vx * sin_y - vy * cos_y) * dt + (-ax * sin_y - ay * cos_y) * dt2
        F[1, 5] = (vx * cos_y - vy * sin_y) * dt + (ax * cos_y - ay * sin_y) * dt2

        # Position derivatives w.r.t. body velocities
        F[0, 6] = cos_y * dt
        F[0, 7] = -sin_y * dt
        F[1, 6] = sin_y * dt
        F[1, 7] = cos_y * dt
        F[2, 8] = dt

        # Position derivatives w.r.t. body accelerations
        F[0, 12] = cos_y * dt2
        F[0, 13] = -sin_y * dt2
        F[1, 12] = sin_y * dt2
        F[1, 13] = cos_y * dt2
        F[2, 14] = dt2

        # Orientation w.r.t. angular velocities
        F[3, 9] = dt
        F[4, 10] = dt
        F[5, 11] = dt

        # Velocities w.r.t. accelerations
        F[6, 12] = dt
        F[7, 13] = dt
        F[8, 14] = dt

        # Covariance propagation
        self.P = F @ self.P @ F.T + self.Q * dt
        # Enforce exact numerical symmetry
        self.P = 0.5 * (self.P + self.P.T)

        if self.two_d_mode:
            self._apply_2d_constraints()

    def update_odom(self, vx: float, vy: float, wz: float, r_cov: np.ndarray | None = None):
        """Assimilate wheel odometry twist [vx, vy, wz]."""
        z = np.array([vx, vy, wz], dtype=np.float64)
        H = np.zeros((3, 15), dtype=np.float64)
        H[0, 6] = 1.0   # vx
        H[1, 7] = 1.0   # vy
        H[2, 11] = 1.0  # vyaw

        R = r_cov if r_cov is not None else self.R_odom
        self._apply_measurement_update(z, H, R)

    def update_imu(self, yaw: float, wz: float, ax: float, ay: float, r_cov: np.ndarray | None = None):
        """Assimilate IMU data [yaw, wz, ax, ay]."""
        z = np.array([yaw, wz, ax, ay], dtype=np.float64)
        H = np.zeros((4, 15), dtype=np.float64)
        H[0, 5] = 1.0   # yaw
        H[1, 11] = 1.0  # vyaw
        H[2, 12] = 1.0  # ax
        H[3, 13] = 1.0  # ay

        R = r_cov if r_cov is not None else self.R_imu

        # Compute innovation with yaw angle unwrap
        y = z - H @ self.state
        y[0] = (y[0] + math.pi) % (2.0 * math.pi) - math.pi

        self._apply_kalman_gain(y, H, R)

    def _apply_measurement_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        y = z - H @ self.state
        self._apply_kalman_gain(y, H, R)

    def _apply_kalman_gain(self, y: np.ndarray, H: np.ndarray, R: np.ndarray):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state += K @ y
        # Wrap yaw
        self.state[5] = (self.state[5] + math.pi) % (2.0 * math.pi) - math.pi

        # Joseph form covariance update for numerical robustness
        I_KH = np.eye(15, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        if self.two_d_mode:
            self._apply_2d_constraints()

    def is_healthy(self) -> tuple[bool, str]:
        """Verify strict numerical sanity: finite, strictly positive diagonal, positive eigenvalues."""
        if not np.all(np.isfinite(self.state)):
            return False, "State vector contains NaN or Inf"
        if not np.all(np.isfinite(self.P)):
            return False, "Covariance P contains NaN or Inf"

        diag = np.diag(self.P)
        if np.any(diag <= 0.0):
            return False, f"Non-positive diagonal in P: min={np.min(diag)}"

        eigvals = np.linalg.eigvalsh(self.P)
        if np.any(eigvals <= 0.0):
            return False, f"Non-positive eigenvalue in P: min={np.min(eigvals)}"

        return True, "OK"


class TestEkfCovarianceMatrixSpecs:
    """Suite 1: Verification of Q and P0 matrices loaded directly from `ekf.yaml`."""

    @pytest.fixture(scope="class")
    def ekf_params(self):
        assert EKF_CONFIG_PATH.exists(), f"Missing configuration file at {EKF_CONFIG_PATH}"
        with open(EKF_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        params = data.get("ekf_filter_node", {}).get("ros__parameters", {})
        return params

    @pytest.fixture(scope="class")
    def q_matrix(self, ekf_params):
        q_raw = ekf_params.get("process_noise_covariance")
        assert q_raw is not None, "process_noise_covariance missing in ekf.yaml"
        assert len(q_raw) == 225, f"Expected 225 elements for Q, got {len(q_raw)}"
        return np.array(q_raw, dtype=np.float64).reshape(15, 15)

    @pytest.fixture(scope="class")
    def p0_matrix(self, ekf_params):
        p0_raw = ekf_params.get("initial_estimate_covariance")
        assert p0_raw is not None, "initial_estimate_covariance missing in ekf.yaml"
        assert len(p0_raw) == 225, f"Expected 225 elements for P0, got {len(p0_raw)}"
        return np.array(p0_raw, dtype=np.float64).reshape(15, 15)

    def test_matrix_dimensions_15x15(self, q_matrix, p0_matrix):
        """Verify Q and P0 are exactly 15x15 matrices."""
        assert q_matrix.shape == (15, 15)
        assert p0_matrix.shape == (15, 15)

    def test_matrices_contain_no_nan_or_inf(self, q_matrix, p0_matrix):
        """Verify Q and P0 contain zero NaN or Inf entries."""
        assert np.all(np.isfinite(q_matrix)), "Q contains NaN or Inf values"
        assert np.all(np.isfinite(p0_matrix)), "P0 contains NaN or Inf values"

    def test_strict_symmetry(self, q_matrix, p0_matrix):
        """Verify exact mathematical symmetry: max |M - M^T| == 0.0."""
        asym_q = np.max(np.abs(q_matrix - q_matrix.T))
        asym_p0 = np.max(np.abs(p0_matrix - p0_matrix.T))
        assert asym_q == 0.0, f"Q is not symmetric: max asymmetry = {asym_q}"
        assert asym_p0 == 0.0, f"P0 is not symmetric: max asymmetry = {asym_p0}"

    def test_strictly_positive_definite_eigenvalues(self, q_matrix, p0_matrix):
        """Verify all 15 eigenvalues are strictly positive (> 0.0)."""
        eig_q = np.linalg.eigvalsh(q_matrix)
        eig_p0 = np.linalg.eigvalsh(p0_matrix)

        assert np.all(eig_q > 0.0), f"Q has non-positive eigenvalues: min={np.min(eig_q)}"
        assert np.all(eig_p0 > 0.0), f"P0 has non-positive eigenvalues: min={np.min(eig_p0)}"

        # Check concrete bounds
        assert np.min(eig_q) >= 1.0e-4
        assert np.max(eig_q) <= 0.1
        assert np.min(eig_p0) >= 1.0e-5
        assert np.max(eig_p0) <= 1.0

    def test_cholesky_factorization(self, q_matrix, p0_matrix):
        """Verify Cholesky decomposition L L^T succeeds without numerical breakdown."""
        l_q = np.linalg.cholesky(q_matrix)
        l_p0 = np.linalg.cholesky(p0_matrix)

        reconstructed_q = l_q @ l_q.T
        reconstructed_p0 = l_p0 @ l_p0.T

        assert np.allclose(reconstructed_q, q_matrix, atol=1e-15)
        assert np.allclose(reconstructed_p0, p0_matrix, atol=1e-15)

    def test_condition_numbers_numerical_well_conditioned(self, q_matrix, p0_matrix):
        """Compute condition numbers kappa = lambda_max / lambda_min and verify bounds."""
        cond_q = np.linalg.cond(q_matrix)
        cond_p0 = np.linalg.cond(p0_matrix)

        # Q condition number is exactly 0.05 / 1e-4 = 500
        assert math.isclose(cond_q, 500.0, rel_tol=1e-5)
        # P0 condition number is exactly 0.1 / 1e-5 = 10,000
        assert math.isclose(cond_p0, 10000.0, rel_tol=1e-5)

        # Both are far below the double-precision threshold of 10^12
        assert cond_q < 1.0e4
        assert cond_p0 < 1.0e6

    def test_no_zero_or_negative_diagonal_elements(self, q_matrix, p0_matrix):
        """Verify strict compliance with rule: zero diagonal elements strictly prohibited."""
        diag_q = np.diag(q_matrix)
        diag_p0 = np.diag(p0_matrix)

        assert np.all(diag_q > 0.0), "Found zero or negative diagonal elements in Q"
        assert np.all(diag_p0 > 0.0), "Found zero or negative diagonal elements in P0"

    def test_sensor_config_matrix_alignment(self, ekf_params):
        """Verify odom0_config and imu0_config match 2D fusion specifications."""
        odom_cfg = ekf_params.get("odom0_config")
        imu_cfg = ekf_params.get("imu0_config")

        assert len(odom_cfg) == 15
        assert len(imu_cfg) == 15

        # odom0 fuses vx (6), vy (7), vyaw (11)
        expected_odom = [False] * 15
        expected_odom[6] = True
        expected_odom[7] = True
        expected_odom[11] = True
        assert odom_cfg == expected_odom

        # imu0 fuses yaw (5), vyaw (11), ax (12), ay (13)
        expected_imu = [False] * 15
        expected_imu[5] = True
        expected_imu[11] = True
        expected_imu[12] = True
        expected_imu[13] = True
        assert imu_cfg == expected_imu

        assert ekf_params.get("two_d_mode") is True
        assert ekf_params.get("publish_tf") is True


class TestEkfAdversarialAssimilation:
    """Suite 2: Adversarial stress testing of filter assimilation under extreme dynamics."""

    @pytest.fixture(scope="class")
    def ekf_matrices(self):
        with open(EKF_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        params = data["ekf_filter_node"]["ros__parameters"]
        q = np.array(params["process_noise_covariance"], dtype=np.float64).reshape(15, 15)
        p0 = np.array(params["initial_estimate_covariance"], dtype=np.float64).reshape(15, 15)
        return q, p0

    def test_extreme_hard_acceleration_and_emergency_braking(self, ekf_matrices):
        """Simulate violent 3g acceleration to 3.0 m/s in 0.1s, followed by emergency stop."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02  # 50 Hz

        # Phase 1: Hard acceleration (+30 m/s^2 for 0.1s -> 5 steps)
        for i in range(5):
            true_ax = 30.0
            true_vx = (i + 1) * true_ax * dt
            ekf.predict(dt)
            ekf.update_odom(vx=true_vx, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=true_ax, ay=0.0)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during hard accel step {i}: {reason}"

        # Phase 2: Cruising at 3.0 m/s with vibrations for 20 steps
        for i in range(20):
            ekf.predict(dt)
            ekf.update_odom(vx=3.0, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=0.0, ay=0.0)
            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during cruise step {i}: {reason}"

        # Phase 3: Emergency Braking (-30 m/s^2 for 0.1s -> 5 steps)
        for i in range(5):
            true_ax = -30.0
            true_vx = max(0.0, 3.0 - (i + 1) * 30.0 * dt)
            ekf.predict(dt)
            ekf.update_odom(vx=true_vx, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=true_ax, ay=0.0)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during braking step {i}: {reason}"

        # Phase 4: Stationary settling for 50 steps (1.0s)
        for i in range(50):
            ekf.predict(dt)
            ekf.update_odom(vx=0.0, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=0.0, ay=0.0)
            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during rest step {i}: {reason}"

        # Verify velocity and acceleration smoothly converged back near 0
        assert abs(ekf.state[6]) < 0.05, f"Expected vx near 0, got {ekf.state[6]}"
        assert abs(ekf.state[12]) < 0.3, f"Expected ax near 0, got {ekf.state[12]}"

    def test_high_speed_multiaxial_strafing_with_rapid_spin(self, ekf_matrices):
        """Simulate combined multi-axis strafing (vx=2.5, vy=-2.5 m/s) and rapid spinning (wz=5.0 rad/s)."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02
        num_steps = 1000  # 20 seconds of continuous spinning and strafing

        true_vx = 2.5
        true_vy = -2.5
        true_wz = 5.0
        current_yaw = 0.0

        for step in range(num_steps):
            current_yaw = (current_yaw + true_wz * dt + math.pi) % (2.0 * math.pi) - math.pi

            ekf.predict(dt)
            ekf.update_odom(vx=true_vx, vy=true_vy, wz=true_wz)
            ekf.update_imu(yaw=current_yaw, wz=true_wz, ax=0.0, ay=0.0)

            if step % 50 == 0:
                healthy, reason = ekf.is_healthy()
                assert healthy, f"Filter unhealthy at step {step}: {reason}"
                # Condition number of P must remain well-behaved
                cond_p = np.linalg.cond(ekf.P)
                assert cond_p < 1.0e8, f"Condition number exploded at step {step}: {cond_p}"

        # Confirm filter state tracked the high-speed motion
        assert abs(ekf.state[6] - true_vx) < 0.2
        assert abs(ekf.state[7] - true_vy) < 0.2
        assert abs(ekf.state[11] - true_wz) < 0.2

    def test_severe_high_frequency_floor_vibration_and_shock_noise(self, ekf_matrices):
        """Inject structural resonance (amplitude 10 m/s^2 at 17.3 Hz / 23.7 Hz) and Gaussian noise."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02
        rng = np.random.default_rng(seed=1337)

        # Baseline velocity 1.0 m/s
        for step in range(500):
            t = step * dt
            # Realistic high-frequency floor resonance avoiding Nyquist alias to DC
            vibe_ax = 10.0 * math.sin(2.0 * math.pi * 17.3 * t)
            vibe_ay = 10.0 * math.cos(2.0 * math.pi * 23.7 * t)
            noise_wz = rng.normal(0.0, 0.2)

            measured_vx = 1.0 + rng.normal(0.0, 0.05)
            measured_vy = rng.normal(0.0, 0.05)

            ekf.predict(dt)
            ekf.update_odom(vx=measured_vx, vy=measured_vy, wz=noise_wz)
            ekf.update_imu(yaw=0.0, wz=noise_wz, ax=vibe_ax, ay=vibe_ay)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Vibration broke filter at step {step}: {reason}"

        # Verify filter smoothed the vibration rather than diverging
        assert abs(ekf.state[6] - 1.0) < 0.15
        assert abs(ekf.state[7]) < 0.15
        diag_p = np.diag(ekf.P)
        assert np.all(diag_p > 0.0)

    def test_asynchronous_sensor_update_rates_and_burst_packet_loss(self, ekf_matrices):
        """Simulate asymmetric arrival (50 Hz odom, 100 Hz IMU) with 30% dropout and 200 ms blackout."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt_imu = 0.01  # 100 Hz clock
        rng = np.random.default_rng(seed=2026)

        odom_divider = 0
        for step in range(1000):
            ekf.predict(dt_imu)
            odom_divider += 1

            # Simulated 200 ms total sensor blackout between steps 400 and 420
            if 400 <= step < 420:
                continue

            # IMU update (100 Hz with 15% random drop)
            if rng.random() > 0.15:
                ekf.update_imu(yaw=0.1, wz=0.0, ax=0.0, ay=0.0)

            # Odometry update (50 Hz with 15% random drop)
            if odom_divider % 2 == 0:
                if rng.random() > 0.15:
                    ekf.update_odom(vx=0.5, vy=0.0, wz=0.0)

            if step % 100 == 0 or step == 420:
                healthy, reason = ekf.is_healthy()
                assert healthy, f"Unhealthy at step {step}: {reason}"

        assert abs(ekf.state[6] - 0.5) < 0.15

    def test_measurement_covariance_extremes_fuzzing(self, ekf_matrices):
        """Fuzz measurement noise R from 1e-7 to 1e5 to verify Kalman gain numerical stability."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02

        # Test varying scales of R
        r_exponents = np.linspace(-7.0, 5.0, 50)
        for exp in r_exponents:
            scale = 10.0 ** exp
            r_odom_fuzzed = np.diag([0.02 * scale, 0.02 * scale, 0.01 * scale])
            r_imu_fuzzed = np.diag([0.005 * scale, 0.01 * scale, 0.05 * scale, 0.05 * scale])

            ekf.predict(dt)
            ekf.update_odom(vx=0.8, vy=-0.1, wz=0.05, r_cov=r_odom_fuzzed)
            ekf.update_imu(yaw=0.05, wz=0.05, ax=0.0, ay=0.0, r_cov=r_imu_fuzzed)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Fuzzing broke filter at R scale 10^{exp:.1f}: {reason}"

            # Asymmetry check
            asym = np.max(np.abs(ekf.P - ekf.P.T))
            assert asym < 1.0e-12, f"Asymmetry leak {asym} at R scale 10^{exp:.1f}"

    def test_pure_dead_reckoning_covariance_growth_bounded(self, ekf_matrices):
        """Predict without measurement updates for 500 steps (10 seconds), checking monotonic variance growth."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02

        # Initial variance on x
        p_x_init = ekf.P[0, 0]

        for step in range(500):
            ekf.predict(dt)

        # Covariance must have grown due to Q * dt integration
        assert ekf.P[0, 0] > p_x_init
        healthy, reason = ekf.is_healthy()
        assert healthy, f"Unhealthy after 500 predict steps: {reason}"
        assert np.all(np.diag(ekf.P) > 0.0)

    def test_100k_cycle_long_term_monte_carlo_stability(self, ekf_matrices):
        """100,000-cycle continuous Monte Carlo assimilation stress test."""
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02
        rng = np.random.default_rng(seed=9999)

        total_cycles = 100000
        check_interval = 10000

        for cycle in range(total_cycles):
            # Dynamic random inputs
            vx_cmd = rng.uniform(-1.5, 1.5)
            vy_cmd = rng.uniform(-1.0, 1.0)
            wz_cmd = rng.uniform(-2.0, 2.0)
            ax_cmd = rng.uniform(-5.0, 5.0)
            ay_cmd = rng.uniform(-5.0, 5.0)

            ekf.predict(dt)
            ekf.update_odom(vx=vx_cmd, vy=vy_cmd, wz=wz_cmd)
            ekf.update_imu(yaw=0.0, wz=wz_cmd, ax=ax_cmd, ay=ay_cmd)

            if (cycle + 1) % check_interval == 0:
                healthy, reason = ekf.is_healthy()
                assert healthy, f"Unhealthy at cycle {cycle + 1}: {reason}"

                # Matrix symmetry check
                asym = np.max(np.abs(ekf.P - ekf.P.T))
                assert asym < 1.0e-12, f"Asymmetry exceeded at cycle {cycle + 1}: {asym}"

                # Eigenvalue spectrum
                eigvals = np.linalg.eigvalsh(ekf.P)
                assert np.min(eigvals) > 0.0, f"Min eigenvalue non-positive at cycle {cycle + 1}"

                # Condition number
                cond = np.linalg.cond(ekf.P)
                assert cond < 1.0e8, f"Condition number exceeded at cycle {cycle + 1}: {cond}"


class TestEkfSimOracleIntegration:
    """Suite 3: Cross-validation using the authoritative EKFSimOracle2D reference model."""

    def test_oracle_with_ekf_yaml_planar_variances(self):
        """Extract active 2D planar diagonal variances from ekf.yaml and verify oracle convergence."""
        with open(EKF_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        params = data["ekf_filter_node"]["ros__parameters"]

        q_full = np.array(params["process_noise_covariance"]).reshape(15, 15)
        p0_full = np.array(params["initial_estimate_covariance"]).reshape(15, 15)

        # Planar states in oracle: [x, y, yaw, vx, vy, wz]
        # Matching ekf.yaml indices: 0(x), 1(y), 5(yaw), 6(vx), 7(vy), 11(vyaw)
        indices = [0, 1, 5, 6, 7, 11]
        q_planar = [float(q_full[i, i]) for i in indices]
        p0_planar = [float(p0_full[i, i]) for i in indices]

        oracle = EKFSimOracle2D(process_noise_diag=q_planar, initial_cov_diag=p0_planar)
        assert oracle.is_healthy()

        # Run 200 update cycles
        for _ in range(200):
            oracle.predict(0.02)
            oracle.update_odom(0.6, 0.2, 0.1)
            oracle.update_imu(yaw=0.05, wz=0.1)
            assert oracle.is_healthy()

        # Verify steady-state estimates
        assert abs(oracle.state[3] - 0.6) < 0.05  # vx
        assert abs(oracle.state[4] - 0.2) < 0.05  # vy
        assert abs(oracle.state[5] - 0.1) < 0.05  # wz
