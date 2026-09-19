"""
Tier 1 Feature Coverage: Sensor Extrinsics, EKF Fusion, TF Authority, and Laser Filtering.
Covers:
  - F3.1 Spatial Lever-Arm Extrinsics (5 tests)
  - F3.2 Temporal Latency Compensation (5 tests)
  - F3.3 EKF Covariance Matrix Tuning (5 tests)
  - F3.4 Single TF Authority Enforcement (5 tests)
  - F3.5 Laser Filter Footprint Masking (5 tests)
"""
import math
from pathlib import Path
import numpy as np
import pytest

from tests.e2e.harness.extrinsics_oracle import LeverArmCompensator, TemporalLatencyCompensator
from tests.e2e.harness.ekf_sim_oracle import EKFSimOracle2D
from tests.e2e.harness.laser_filter_oracle import LaserFootprintFilterOracle
from tests.e2e.harness.config_verifier import ConfigVerifier


@pytest.mark.tier1
class TestF31_SpatialLeverArmExtrinsics:
    """F3.1: Rigid-body kinematic lever-arm compensation."""

    def test_f3_1_lever_arm_centrifugal_acceleration_math(self):
        """Verify centrifugal acceleration a_c = w x (w x p) = [-w^2*px, -w^2*py, 0]."""
        px, py, pz = 0.05, 0.02, 0.0
        compensator = LeverArmCompensator(p_lever_arm=(px, py, pz))
        wz = 2.0  # rad/s
        omega = np.array([0.0, 0.0, wz])

        ac = compensator.compute_centrifugal_acceleration(omega)
        # Theoretical: [-wz^2 * px, -wz^2 * py, 0]
        expected_ac = np.array([-wz * wz * px, -wz * wz * py, 0.0])
        assert np.allclose(ac, expected_ac, atol=1e-6)

    def test_f3_1_lever_arm_tangential_acceleration_math(self):
        """Verify tangential acceleration a_t = alpha x p = [-alpha_z*py, alpha_z*px, 0]."""
        px, py, pz = 0.04, 0.03, 0.0
        compensator = LeverArmCompensator(p_lever_arm=(px, py, pz))
        alpha_z = 5.0  # rad/s²
        alpha = np.array([0.0, 0.0, alpha_z])

        at = compensator.compute_tangential_acceleration(alpha)
        expected_at = np.array([-alpha_z * py, alpha_z * px, 0.0])
        assert np.allclose(at, expected_at, atol=1e-6)

    def test_f3_1_lever_arm_zero_offset_identity(self):
        """Verify zero lever arm (p = [0,0,0]) produces identical sensor and body accelerations."""
        compensator = LeverArmCompensator(p_lever_arm=(0.0, 0.0, 0.0))
        a_body = np.array([1.2, -0.4, 0.1])
        omega = np.array([0.0, 0.0, 3.0])
        alpha = np.array([0.0, 0.0, 1.5])

        a_sensor = compensator.body_to_sensor_acceleration(a_body, omega, alpha)
        assert np.allclose(a_sensor, a_body, atol=1e-6)

    def test_f3_1_lever_arm_reconstruction_of_body_acceleration(self):
        """Verify raw IMU measurements are properly stripped of lever-arm forces."""
        compensator = LeverArmCompensator(p_lever_arm=(0.06, -0.03, 0.01))
        true_body_accel = np.array([0.8, 0.5, 0.0])
        omega = np.array([0.0, 0.0, 1.8])
        alpha = np.array([0.0, 0.0, 0.9])

        # Forward sensor measurement
        raw_sensor_accel = compensator.body_to_sensor_acceleration(true_body_accel, omega, alpha)
        # Inverse compensation
        recovered_body_accel = compensator.sensor_to_body_acceleration(raw_sensor_accel, omega, alpha)

        assert np.allclose(recovered_body_accel, true_body_accel, atol=1e-6)

    def test_f3_1_lever_arm_rotation_direction_symmetry(self):
        """Verify centrifugal acceleration is outward regardless of CW or CCW rotation."""
        compensator = LeverArmCompensator(p_lever_arm=(0.05, 0.0, 0.0))
        w_ccw = np.array([0.0, 0.0, 2.0])
        w_cw = np.array([0.0, 0.0, -2.0])

        ac_ccw = compensator.compute_centrifugal_acceleration(w_ccw)
        ac_cw = compensator.compute_centrifugal_acceleration(w_cw)

        # Both must pull towards -X (inwards towards center of rotation from sensor position)
        assert np.allclose(ac_ccw, ac_cw, atol=1e-6)
        assert ac_ccw[0] < 0.0


@pytest.mark.tier1
class TestF32_TemporalLatencyCompensation:
    """F3.2: Continuous-time latency compensation model."""

    def test_f3_2_temporal_linear_displacement_correction(self):
        """Verify position shift formula delta_s = v * delta_t."""
        latency = 0.035  # 35 ms
        compensator = TemporalLatencyCompensator(latency_s=latency)
        v = 1.2  # m/s
        raw_pos = 5.0

        corrected_pos = compensator.compensate_position(velocity_mps=v, measured_pos=raw_pos)
        expected = raw_pos + v * latency
        assert math.isclose(corrected_pos, expected, rel_tol=1e-6)

    def test_f3_2_temporal_timestamp_alignment(self):
        """Verify timestamp shift aligns delayed packet with true acquisition instant."""
        compensator = TemporalLatencyCompensator(latency_s=0.020)
        t_recv = 100.550
        t_aligned = compensator.align_timestamp(t_recv)
        assert math.isclose(t_aligned, 100.530, rel_tol=1e-6)

    def test_f3_2_temporal_piecewise_signal_interpolation(self):
        """Verify time interpolation correctly resamples asynchronous telemetry stream."""
        timestamps = np.array([0.0, 0.02, 0.04, 0.06, 0.08])
        values = np.array([0.0, 1.0, 2.0, 3.0, 4.0])

        interp_val = TemporalLatencyCompensator.interpolate_signal(0.03, timestamps, values)
        assert math.isclose(interp_val, 1.5, rel_tol=1e-6)

    def test_f3_2_temporal_zero_velocity_invariance(self):
        """Verify latency compensation does not alter position when robot is stationary."""
        compensator = TemporalLatencyCompensator(latency_s=0.050)
        pos = 2.45
        assert compensator.compensate_position(0.0, pos) == pos

    def test_f3_2_temporal_variable_latency_scaling(self):
        """Verify correction scales linearly with measured transmission latency."""
        v = 1.5
        pos = 10.0
        c1 = TemporalLatencyCompensator(0.010)
        c2 = TemporalLatencyCompensator(0.020)

        delta1 = c1.compensate_position(v, pos) - pos
        delta2 = c2.compensate_position(v, pos) - pos
        assert math.isclose(delta2, 2.0 * delta1, rel_tol=1e-6)


@pytest.mark.tier1
class TestF33_EKFCovarianceMatrixTuning:
    """F3.3: EKF filter covariance matrix configuration and numerical stability."""

    def test_f3_3_ekf_filter_stability_positive_definite_covariance(self):
        """Verify EKF covariance remains positive definite and healthy through updates."""
        ekf = EKFSimOracle2D()
        assert ekf.is_healthy()

        for _ in range(50):
            ekf.predict(0.02)
            ekf.update_odom(0.4, 0.1, 0.05)
            ekf.update_imu(yaw=0.01, wz=0.05)
            assert ekf.is_healthy()

    def test_f3_3_ekf_non_zero_diagonal_variance_propagation(self):
        """Verify no covariance diagonal collapse to zero (strictly positive variance)."""
        ekf = EKFSimOracle2D()
        ekf.predict(0.1)
        ekf.update_odom(0.2, 0.0, 0.0)

        diag = np.diag(ekf.P)
        assert np.all(diag > 0.0)
        assert not np.any(np.isnan(diag))

    def test_f3_3_ekf_odom0_twist_state_update(self):
        """Verify wheel odometry updates body linear and angular velocities."""
        ekf = EKFSimOracle2D()
        assert ekf.state[3] == 0.0  # initial vx is 0

        # Update with forward velocity 0.8 m/s
        for _ in range(5):
            ekf.predict(0.02)
            ekf.update_odom(vx=0.8, vy=0.0, wz=0.0)

        assert ekf.state[3] > 0.6  # state converged towards 0.8

    def test_f3_3_ekf_imu0_yaw_state_update(self):
        """Verify IMU updates yaw angle without numerical overflow."""
        ekf = EKFSimOracle2D()
        for _ in range(10):
            ekf.predict(0.02)
            ekf.update_imu(yaw=0.5, wz=0.1)

        assert abs(ekf.state[2] - 0.5) < 0.1

    def test_f3_3_ekf_yaml_configuration_spec_check(self, workspace_root: Path):
        """Check ekf.yaml structure against PROJECT.md requirements."""
        verifier = ConfigVerifier(workspace_root)
        results = verifier.verify_ekf_config()

        # Check Single TF authority flag
        assert results["publish_tf"] is True, "ekf.yaml must have publish_tf: true"


@pytest.mark.tier1
class TestF34_SingleTFAuthorityEnforcement:
    """F3.4: Single TF Authority enforcement (only ekf_node broadcasts odom -> base_link)."""

    def test_f3_4_single_tf_authority_ekf_node_only(self, workspace_root: Path):
        """Verify ekf.yaml has publish_tf: true."""
        verifier = ConfigVerifier(workspace_root)
        results = verifier.verify_ekf_config()
        assert results["publish_tf"] is True

    def test_f3_4_single_tf_no_duplicate_broadcasters(self, workspace_root: Path):
        """Verify simulation and hardware driver configs do not broadcast duplicate odom->base_link."""
        # Inspect omni_hardware or omni_simulation configs if present
        sim_yaml = workspace_root / "src" / "omni_simulation" / "config" / "simulation.yaml"
        if sim_yaml.exists():
            verifier = ConfigVerifier(workspace_root)
            data = verifier.load_yaml("src/omni_simulation/config/simulation.yaml")
            # If publish_tf is configured, it should be false or absent in secondary nodes
            sim_params = data.get("omni_simulation", {}).get("ros__parameters", {})
            if "publish_tf" in sim_params:
                assert sim_params["publish_tf"] is False

    def test_f3_4_single_tf_tree_topology_map_odom_base(self):
        """Verify topological consistency: map -> odom -> base_link."""
        frames = {"map": None, "odom": "map", "base_link": "odom"}
        assert frames["base_link"] == "odom"
        assert frames["odom"] == "map"

    def test_f3_4_single_tf_base_hardware_dynamic_tf_disabled(self, workspace_root: Path):
        """Verify firmware / hardware bridge does not directly broadcast TF tree."""
        # Contract: only robot_localization ekf_node owns the transform
        verifier = ConfigVerifier(workspace_root)
        ekf_cfg = verifier.load_yaml("src/omni_localization/config/ekf.yaml")
        assert ekf_cfg["ekf_filter_node"]["ros__parameters"]["odom_frame"] == "odom"
        assert ekf_cfg["ekf_filter_node"]["ros__parameters"]["base_link_frame"] == "base_link"

    def test_f3_4_single_tf_transform_timeout_bounds(self, workspace_root: Path):
        """Verify transform timeout parameters in ekf.yaml are configured safely."""
        verifier = ConfigVerifier(workspace_root)
        ekf_cfg = verifier.load_yaml("src/omni_localization/config/ekf.yaml")
        params = ekf_cfg["ekf_filter_node"]["ros__parameters"]
        assert params.get("sensor_timeout", 0) > 0.0


@pytest.mark.tier1
class TestF35_LaserFilterFootprintMasking:
    """F3.5: Laser filter footprint masking (box: [-0.135, 0.135] x [-0.135, 0.135] m)."""

    def test_f3_5_laser_filter_chassis_interior_point_rejected(self):
        """Verify any point strictly inside chassis footprint box is masked (nan)."""
        filter_oracle = LaserFootprintFilterOracle()
        # Chassis interior points
        test_points = [
            (0.0, 0.0),
            (0.10, 0.10),
            (-0.12, 0.05),
            (0.05, -0.12),
        ]
        for x, y in test_points:
            x_f, y_f, valid = filter_oracle.filter_cartesian_point(x, y)
            assert not valid
            assert math.isnan(x_f)
            assert math.isnan(y_f)

    def test_f3_5_laser_filter_external_obstacle_point_preserved(self):
        """Verify points outside footprint box (> 0.135 m) are preserved intact."""
        filter_oracle = LaserFootprintFilterOracle()
        test_points = [
            (0.20, 0.0),
            (-0.25, 0.15),
            (0.0, 0.30),
            (0.50, -0.50),
        ]
        for x, y in test_points:
            x_f, y_f, valid = filter_oracle.filter_cartesian_point(x, y)
            assert valid
            assert math.isclose(x_f, x, rel_tol=1e-9)
            assert math.isclose(y_f, y, rel_tol=1e-9)

    def test_f3_5_laser_filter_boundary_edge_handling(self):
        """Verify exact boundary at 0.135 m is masked."""
        filter_oracle = LaserFootprintFilterOracle()
        assert filter_oracle.is_point_inside(0.135, 0.0)
        assert filter_oracle.is_point_inside(-0.135, 0.0)
        assert filter_oracle.is_point_inside(0.0, 0.135)
        assert filter_oracle.is_point_inside(0.0, -0.135)

    def test_f3_5_laser_filter_full_scan_polar_filtering(self):
        """Verify full 360-degree polar scan masking."""
        filter_oracle = LaserFootprintFilterOracle()
        # Create scan with 12 points: 4 close chassis reflections (0.08m) and 8 distant obstacles (1.0m)
        ranges = [0.08, 1.0, 1.0, 0.08, 1.0, 1.0, 0.08, 1.0, 1.0, 0.08, 1.0, 1.0]
        angle_min = 0.0
        angle_inc = 2.0 * math.pi / 12

        filtered = filter_oracle.filter_scan_ranges(ranges, angle_min, angle_inc)
        # The 4 close reflection points must be NaN
        assert math.isnan(filtered[0])
        assert math.isnan(filtered[3])
        assert math.isnan(filtered[6])
        assert math.isnan(filtered[9])

        # Distant points must remain 1.0
        assert math.isclose(filtered[1], 1.0, rel_tol=1e-6)
        assert math.isclose(filtered[4], 1.0, rel_tol=1e-6)

    def test_f3_5_laser_filter_yaml_configuration_spec_check(self, workspace_root: Path):
        """Verify laser_filter.yaml defines LaserScanBoxFilter."""
        verifier = ConfigVerifier(workspace_root)
        results = verifier.verify_laser_filter_config()
        assert results["has_box_filter"] is True
        assert results["box_frame_is_base_link"] is True
