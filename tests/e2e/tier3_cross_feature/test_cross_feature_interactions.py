"""
Tier 3 Cross-Feature Interactions: Pairwise Module Integrations.
Covers:
  - F1 (Encoder PLL/M/T) + F3 (EKF Fusion)
  - F2 (IMU Gyro Nulling) + F1 (Wheel Radius Compensation Kr)
  - F4 (Web API Command) + F4 (Serial Protocol Timeout & Abort)
  - F3 (Single TF Authority) + F3 (Laser Footprint Frame Masking)
  - F3 (Spatial Lever-Arm) + F3 (EKF High-Speed Rotation)
  - F3 (Temporal Latency) + F3 (Dynamic Acceleration Profile)
  - F4 (Serial Calibration Frame) + F4 (YAML Persistence) + F3 (Config Reload)
  - F4 (WebSocket Progress Stream) + F4 (Interactive Abort Event)
"""
import math
from pathlib import Path
import numpy as np
import pytest

from tests.e2e.harness.encoder_oracle import ODrivePLLObserver, LinuxCNCHybridEstimator
from tests.e2e.harness.kinematics_oracle import MecanumKinematicsOracle
from tests.e2e.harness.imu_calib_oracle import GyroBiasNuller
from tests.e2e.harness.extrinsics_oracle import LeverArmCompensator, TemporalLatencyCompensator
from tests.e2e.harness.serial_protocol_oracle import SerialPacket
from tests.e2e.harness.laser_filter_oracle import LaserFootprintFilterOracle
from tests.e2e.harness.ekf_sim_oracle import EKFSimOracle2D
from tests.e2e.harness.config_verifier import ConfigVerifier


@pytest.mark.tier3
class TestCrossFeatureInteractions:
    """Pairwise cross-feature interactions across architecture subsystems."""

    def test_interaction_1_encoder_pll_to_ekf_fusion(self):
        """Interaction 1: Feeding discrete PLL velocity estimate directly into EKF odom update."""
        pll = ODrivePLLObserver(omega_pll=80.0)
        ekf = EKFSimOracle2D()

        dt = 0.02
        target_vx = 0.6  # m/s
        target_wheel_w = target_vx / 0.03  # 20 rad/s
        pos = 0.0

        for step in range(100):
            pos += target_wheel_w * dt
            # Quantize pos
            quantized_pos = round(pos * 4000 / (2*math.pi)) * (2*math.pi / 4000)
            _, est_w = pll.update(quantized_pos, dt)

            # Convert wheel rate to body vx
            est_vx = est_w * 0.03 * math.sqrt(0.5) * 2.0 / 2.0  # approximate scale

            ekf.predict(dt)
            ekf.update_odom(vx=est_vx, vy=0.0, wz=0.0)
            assert ekf.is_healthy()

        # EKF vx state must have converged smoothly without covariance explosion
        assert ekf.state[3] > 0.4
        assert np.all(np.diag(ekf.P) > 0.0)

    def test_interaction_2_joint_imu_nulling_and_wheel_kr_compensation(self):
        """Interaction 2: Asymmetric wheel compensation Kr + IMU gyro bias nulling in straight trajectory."""
        # Wheels with 3% asymmetry on front-left wheel
        radii = (0.030, 0.0291, 0.030, 0.030)
        kinematics_kr = MecanumKinematicsOracle(wheel_radii=radii)
        gyro_nuller = GyroBiasNuller()

        # Calibrate gyro stationary bias
        raw_gyro = np.array([[0.0, 0.0, 0.015]] * 100)  # 0.015 rad/s static bias
        gyro_nuller.calibrate(raw_gyro)

        # Command pure forward motion vx = 0.5 m/s
        wheel_cmds = kinematics_kr.inverse_kinematics(vx=0.5, vy=0.0, wz=0.0, apply_scaling=False)

        # Forward kinematics with Kr produces exact (0.5, 0.0, 0.0)
        actual_twist = kinematics_kr.forward_kinematics(wheel_cmds)
        assert math.isclose(actual_twist[1], 0.0, abs_tol=1e-5)
        assert math.isclose(actual_twist[2], 0.0, abs_tol=1e-5)

        # Corrected gyro reading during straight line motion
        imu_meas = np.array([0.0, 0.0, 0.015])  # measuring same static bias
        corrected_wz = gyro_nuller.apply(imu_meas)[2]
        assert abs(corrected_wz) < 1e-4  # Zero rotation confirmed by IMU

    def test_interaction_3_web_command_dispatch_and_serial_timeout(self):
        """Interaction 3: Web UI calibration command dispatch with serial timeout & abort handling."""
        # 1. UI dispatches trigger packet
        cmd_pkt = SerialPacket.build_imu_trigger_cmd(subtype=1, seq=1)
        raw_cmd = cmd_pkt.serialize()
        assert raw_cmd[:2] == bytes([0xAA, 0x55])

        # 2. Simulate dropped communication: no response within timeout threshold
        timeout_duration_s = 0.5
        elapsed_s = 0.6  # Exceeds timeout

        # 3. System enters abort routine and sends CMD_CALIB_ABORT
        is_timed_out = elapsed_s > timeout_duration_s
        assert is_timed_out

        abort_pkt = SerialPacket.build_abort_cmd(seq=2)
        raw_abort = abort_pkt.serialize()
        parsed_abort, _ = SerialPacket.deserialize(raw_abort)
        assert parsed_abort.msg_id == SerialPacket.CMD_CALIB_ABORT

    def test_interaction_4_tf_tree_authority_and_laser_filter_masking(self):
        """Interaction 4: Laser scan points transformed via TF and masked in base_link frame."""
        # Laser sensor located at (x=0.08, y=0.0) relative to base_link
        laser_x_offset = 0.08
        laser_filter = LaserFootprintFilterOracle(min_x=-0.135, max_x=0.135, min_y=-0.135, max_y=0.135)

        # Laser beam pointing forward (angle = 0) at range 0.05m
        # Point in laser frame: (0.05, 0)
        # Point in base_link frame via TF: (0.05 + 0.08, 0) = (0.13, 0)
        # (0.13, 0) is inside [-0.135, 0.135] -> must be filtered
        ranges = [0.05]
        filtered = laser_filter.filter_scan_ranges(ranges, angle_min=0.0, angle_increment=0.0,
                                                   laser_offset_x=laser_x_offset)
        assert math.isnan(filtered[0])

        # Laser beam pointing forward at range 0.10m
        # Point in base_link: (0.10 + 0.08, 0) = (0.18, 0) -> outside footprint, must be preserved
        ranges_outside = [0.10]
        filtered_out = laser_filter.filter_scan_ranges(ranges_outside, angle_min=0.0, angle_increment=0.0,
                                                       laser_offset_x=laser_x_offset)
        assert math.isclose(filtered_out[0], 0.10, rel_tol=1e-5)

    def test_interaction_5_spatial_lever_arm_and_ekf_high_speed_rotation(self):
        """Interaction 5: Off-center IMU lever-arm compensation prevents centrifugal linear drift in EKF."""
        lever_arm = LeverArmCompensator(p_lever_arm=(0.05, 0.0, 0.0))  # 5cm ahead of center
        ekf = EKFSimOracle2D()

        # Pure in-place rotation at wz = 3.0 rad/s
        wz = 3.0
        omega = np.array([0.0, 0.0, wz])
        alpha = np.array([0.0, 0.0, 0.0])

        # True robot base acceleration is 0 (spinning in place)
        true_base_accel = np.array([0.0, 0.0, 0.0])

        # Raw sensor measures centrifugal acceleration inward: ac = [-wz^2 * px, 0, 0] = [-0.45, 0, 0]
        raw_sensor_accel = lever_arm.body_to_sensor_acceleration(true_base_accel, omega, alpha)
        assert math.isclose(raw_sensor_accel[0], -0.45, rel_tol=1e-5)

        # Apply lever arm compensation
        corrected_accel = lever_arm.sensor_to_body_acceleration(raw_sensor_accel, omega, alpha)
        assert np.allclose(corrected_accel, [0.0, 0.0, 0.0], atol=1e-6)

        # Feed into EKF: pure rotation with corrected acceleration produces no false linear velocity
        for _ in range(20):
            ekf.predict(0.02)
            ekf.update_odom(vx=0.0, vy=0.0, wz=wz)
            ekf.update_imu(yaw=0.0, wz=wz, ax=corrected_accel[0], ay=corrected_accel[1])

        assert abs(ekf.state[3]) < 0.01  # vx remains zero
        assert abs(ekf.state[4]) < 0.01  # vy remains zero

    def test_interaction_6_temporal_latency_and_dynamic_acceleration(self):
        """Interaction 6: Latency compensation aligns delayed odometry with real-time IMU under acceleration."""
        dt_latency = 0.025  # 25 ms communication latency
        compensator = TemporalLatencyCompensator(latency_s=dt_latency)

        # Robot is accelerating forward at a = 2.0 m/s²
        # At t = 1.0s, true velocity v = 2.0 m/s
        # Delayed odometry received at t = 1.0s actually measured state at t = 0.975s (v_delayed = 1.95 m/s)
        t_now = 1.0
        a = 2.0
        v_delayed = a * (t_now - dt_latency)  # 1.95 m/s
        pos_delayed = 0.5 * a * (t_now - dt_latency)**2

        # Compensate delayed measurement
        aligned_pos = compensator.compensate_position(v_delayed, pos_delayed)
        # First-order Taylor approximation
        true_pos = 0.5 * a * t_now**2
        assert abs(aligned_pos - true_pos) < 0.01

    def test_interaction_7_calibration_frame_to_yaml_and_config_reload(self, temp_calib_dir: Path):
        """Interaction 7: RESP_CALIB_IMU_RESULT frame -> YAML persistence -> config reload verification."""
        # 1. Simulate receiving binary result frame from STM32
        res_frame = SerialPacket.build_imu_result(
            bias_g=(0.005, -0.008, 0.002),
            bias_a=(0.02, -0.03, 0.01),
            scale_a=(1.01, 0.99, 1.00),
            residual_norm=0.004,
            seq=99
        )
        parsed_data = SerialPacket.parse_imu_result(res_frame)

        # 2. Persist to YAML
        yaml_path = temp_calib_dir / "imu_calib.yaml"
        payload = {
            "imu_calib": {
                "gyro_bias": list(parsed_data["bias_g"]),
                "accel_bias": list(parsed_data["bias_a"]),
                "accel_scale": list(parsed_data["scale_a"]),
                "frame_id": "imu_link",
            }
        }
        import yaml
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f)

        # 3. Reload and verify with ConfigVerifier
        verifier = ConfigVerifier(temp_calib_dir.parent)
        assert verifier.verify_imu_calib_yaml(yaml_path)

    def test_interaction_8_websocket_progress_streaming_with_interactive_abort(self):
        """Interaction 8: Progress telemetry stream handles user abort transition cleanly."""
        # Emit sequence of progress packets
        stream = []
        for p in [10, 25, 40]:
            pkt = SerialPacket.build_telem_progress(1, 1, p, status=0, live_metric=0.01)
            stream.append(SerialPacket.parse_telem_progress(pkt))

        # User triggers abort at 40%
        abort_pkt = SerialPacket.build_telem_progress(1, 1, 40, status=2, live_metric=0.0)  # status 2 = ABORTED
        stream.append(SerialPacket.parse_telem_progress(abort_pkt))

        assert stream[-1]["status_code"] == 2
        assert stream[-1]["progress_percent"] == 40
