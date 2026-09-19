"""
Tier 4 Real-World Workload Scenarios: Full System Life-Cycle Scenarios.
Covers:
  - Scenario 1: Complex Multi-Segment Trajectory (Figure-8 & Slalom Driving)
  - Scenario 2: Emergency High-Speed Braking (1.5 m/s to 0 in 200 ms without EKF divergence)
  - Scenario 3: High-Frequency Floor Vibration & Dynamic Shock (Covariance Inflation Robustness)
  - Scenario 4: Full Calibration Lifecycle (Trigger -> Stream -> Flash -> Disk YAML Persistence)
  - Scenario 5: Long-Duration Stationarity Drift Test (60-Second Hold, Drift < 0.05 deg/s)
  - Scenario 6: Laser Obstacle Detection in Cluttered Dynamic Environment
"""
import math
from pathlib import Path
import numpy as np
import pytest

from tests.e2e.harness.encoder_oracle import ODrivePLLObserver
from tests.e2e.harness.kinematics_oracle import MecanumKinematicsOracle
from tests.e2e.harness.imu_calib_oracle import GyroBiasNuller
from tests.e2e.harness.serial_protocol_oracle import SerialPacket
from tests.e2e.harness.laser_filter_oracle import LaserFootprintFilterOracle
from tests.e2e.harness.ekf_sim_oracle import EKFSimOracle2D
from tests.e2e.harness.config_verifier import ConfigVerifier


@pytest.mark.tier4
class TestRealWorldWorkloadScenarios:
    """Full system life-cycle and realistic operational workload scenarios."""

    def test_scenario_1_complex_multi_segment_trajectory(self):
        """Scenario 1: Slalom & Figure-8 trajectory combining forward, strafe, and yaw rotation."""
        kinematics = MecanumKinematicsOracle()
        ekf = EKFSimOracle2D()

        dt = 0.02  # 50 Hz control loop
        total_time = 6.0  # 6 seconds simulation
        steps = int(total_time / dt)

        # Generate Figure-8 velocity trajectory
        for step in range(steps):
            t = step * dt
            # Sinusoidal forward, strafing, and turning commands
            cmd_vx = 0.4 * math.cos(2.0 * math.pi * 0.25 * t)
            cmd_vy = 0.2 * math.sin(4.0 * math.pi * 0.25 * t)
            cmd_wz = 0.3 * math.sin(2.0 * math.pi * 0.25 * t)

            # Inverse kinematics to wheel speeds
            wheel_speeds = kinematics.inverse_kinematics(cmd_vx, cmd_vy, cmd_wz, apply_scaling=True)
            # Reconstruct forward kinematics (simulating wheel odometry measurement)
            odom_twist = kinematics.forward_kinematics(wheel_speeds)

            # Propagate and update EKF
            ekf.predict(dt)
            ekf.update_odom(vx=odom_twist[0], vy=odom_twist[1], wz=odom_twist[2])
            ekf.update_imu(yaw=ekf.state[2], wz=odom_twist[2])

            assert ekf.is_healthy(), f"EKF diverged at step {step}, t={t}s"

        # At the end of the trajectory, state covariance must remain bounded and healthy
        diag_p = np.diag(ekf.P)
        assert np.all(diag_p > 0.0)
        assert np.all(diag_p < 10.0)  # No unbounded variance growth

    def test_scenario_2_emergency_high_speed_braking(self):
        """Scenario 2: Emergency braking from 1.5 m/s to 0 m/s in 200 ms with sharp deceleration jerk."""
        ekf = EKFSimOracle2D()
        dt = 0.01  # 100 Hz
        current_vx = 1.5  # Cruising at 1.5 m/s

        # Cruising phase (1 second)
        for _ in range(100):
            ekf.predict(dt)
            ekf.update_odom(vx=current_vx, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=0.0, ay=0.0)

        assert math.isclose(ekf.state[3], 1.5, rel_tol=0.05)

        # Braking phase: 1.5 m/s to 0 in 200 ms (20 steps) -> a = -7.5 m/s² deceleration
        decel_a = -7.5
        for _ in range(20):
            current_vx = max(0.0, current_vx + decel_a * dt)
            ekf.predict(dt)
            ekf.update_odom(vx=current_vx, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=decel_a, ay=0.0)
            assert ekf.is_healthy(), "EKF diverged during emergency braking deceleration!"

        # Standstill hold (0.5 seconds)
        for _ in range(50):
            ekf.predict(dt)
            ekf.update_odom(vx=0.0, vy=0.0, wz=0.0)
            ekf.update_imu(yaw=0.0, wz=0.0, ax=0.0, ay=0.0)
            assert ekf.is_healthy()

        # Final state must be fully at rest with vx < 0.05 m/s and no numerical NaN
        assert abs(ekf.state[3]) < 0.05
        assert np.all(np.isfinite(ekf.state))

    def test_scenario_3_floor_vibration_and_covariance_inflation(self):
        """Scenario 3: 50 Hz floor vibration and dynamic bumps with inflated covariance robustness."""
        # Uninflated vs Inflated EKF
        # Nominal Q vs Inflated Q (1.8x std = 3.24x variance)
        ekf_inflated = EKFSimOracle2D(process_noise_diag=[0.01, 0.01, 0.005, 0.05 * 3.24, 0.05 * 3.24, 0.02 * 3.24])

        dt = 0.01
        np.random.seed(42)

        for step in range(200):
            t = step * dt
            # 50 Hz sinusoidal vibration superimposed on accelerometer (e.g. motor harmonics / tile joints)
            vibration_noise = 0.8 * math.sin(2.0 * math.pi * 50.0 * t) + np.random.normal(0, 0.2)
            raw_ax = vibration_noise

            ekf_inflated.predict(dt)
            ekf_inflated.update_odom(vx=0.5, vy=0.0, wz=0.0)
            ekf_inflated.update_imu(yaw=0.0, wz=0.0, ax=raw_ax, ay=0.0)
            assert ekf_inflated.is_healthy()

        # The inflated filter does not blow up under heavy 50Hz vibration
        assert abs(ekf_inflated.state[4]) < 0.1  # vy lateral drift remains small
        assert np.all(np.diag(ekf_inflated.P) > 0.0)

    def test_scenario_4_full_calibration_lifecycle(self, temp_calib_dir: Path):
        """Scenario 4: Complete calibration lifecycle: Web trigger -> Progress telemetry -> Flash -> YAML."""
        # Step 1: Web UI sends trigger command
        trigger_pkt = SerialPacket.build_imu_trigger_cmd(subtype=1, seq=1)
        raw_cmd = trigger_pkt.serialize()
        parsed_cmd, _ = SerialPacket.deserialize(raw_cmd)
        assert parsed_cmd.msg_id == SerialPacket.CMD_CALIB_TRIGGER_IMU

        # Step 2: MCU responds with ACK
        ack_pkt = SerialPacket(msg_id=SerialPacket.RESP_ACK_NACK, payload=bytes([parsed_cmd.msg_id, 0]), seq=2)
        raw_ack = ack_pkt.serialize()
        parsed_ack, _ = SerialPacket.deserialize(raw_ack)
        assert parsed_ack.payload[1] == 0  # ACK

        # Step 3: Stream progress packets from 0% to 100%
        progress_log = []
        for pct in [10, 30, 60, 90, 100]:
            p_pkt = SerialPacket.build_telem_progress(calib_type=1, stage=1, percent=pct, status=0, live_metric=0.01)
            p_parsed = SerialPacket.parse_telem_progress(SerialPacket.deserialize(p_pkt.serialize())[0])
            progress_log.append(p_parsed["progress_percent"])
        assert progress_log == [10, 30, 60, 90, 100]

        # Step 4: MCU emits calculated calibration results
        calib_res_pkt = SerialPacket.build_imu_result(
            bias_g=(0.002, -0.004, 0.001),
            bias_a=(0.05, -0.02, 0.03),
            scale_a=(1.005, 0.995, 1.002),
            residual_norm=0.0025,
            seq=10
        )
        res_parsed = SerialPacket.parse_imu_result(SerialPacket.deserialize(calib_res_pkt.serialize())[0])
        assert res_parsed["residual_norm"] < 0.005

        # Step 5: Jetson UI sends FLASH_COMMIT command
        commit_pkt = SerialPacket.build_flash_commit_cmd(seq=11)
        parsed_commit, _ = SerialPacket.deserialize(commit_pkt.serialize())
        assert parsed_commit.msg_id == SerialPacket.CMD_CALIB_FLASH_COMMIT

        # Step 6: Jetson persists to disk YAML
        yaml_path = temp_calib_dir / "imu_calib.yaml"
        payload = {
            "imu_calib": {
                "gyro_bias": list(res_parsed["bias_g"]),
                "accel_bias": list(res_parsed["bias_a"]),
                "accel_scale": list(res_parsed["scale_a"]),
                "frame_id": "imu_link",
            }
        }
        import yaml
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f)

        # Step 7: Localization stack reloads and validates config
        verifier = ConfigVerifier(temp_calib_dir.parent)
        assert verifier.verify_imu_calib_yaml(yaml_path)

    def test_scenario_5_long_duration_stationarity_drift(self):
        """Scenario 5: 60-second stationary hold; cumulative yaw drift remains strictly < 3.0 degrees."""
        nuller = GyroBiasNuller()
        true_bias = np.array([0.0, 0.0, 0.010])  # 0.010 rad/s raw static bias

        dt = 0.02  # 50 Hz
        total_time_s = 60.0  # 60 seconds
        steps = int(total_time_s / dt)

        np.random.seed(101)
        # Synthetic 60-second stationary gyro readings (bias + low noise)
        noise = np.random.normal(0, 1e-4, size=(steps, 3))
        raw_stream = true_bias + noise

        # Without calibration: cumulative drift over 60s is 0.01 * 60 = 0.6 rad (~34.4 degrees!)
        uncalibrated_drift_deg = math.degrees(np.sum(raw_stream[:, 2]) * dt)
        assert abs(uncalibrated_drift_deg) > 30.0

        # Calibrate using initial 5-second stationary window (250 samples)
        nuller.calibrate(raw_stream[:250])

        # Apply nulling across entire 60s stream
        calibrated_stream = nuller.apply(raw_stream)
        calibrated_drift_deg = abs(math.degrees(float(np.sum(calibrated_stream[:, 2]) * dt)))

        # Acceptance threshold: cumulative drift < 3.0 degrees (equivalent to < 0.05 deg/s * 60s)
        assert calibrated_drift_deg < 3.0, f"Drift {calibrated_drift_deg} deg exceeds 3.0 deg threshold!"

    def test_scenario_6_cluttered_corridor_laser_filtering(self):
        """Scenario 6: Cluttered warehouse aisle with chassis self-reflections and obstacle scans."""
        laser_filter = LaserFootprintFilterOracle(min_x=-0.135, max_x=0.135, min_y=-0.135, max_y=0.135)

        # Simulated 360-degree LiDAR scan (36 beams spaced 10 degrees)
        # Beams at 0, 90, 180, 270 deg hit chassis edge (r = 0.10 m)
        # Other beams hit corridor walls (r = 0.80 m) or pallet boxes (r = 0.40 m)
        ranges = []
        for i in range(36):
            deg = i * 10
            if deg in [0, 90, 180, 270]:
                ranges.append(0.10)  # Chassis internal reflection
            elif 40 <= deg <= 50:
                ranges.append(0.35)  # Pallet obstacle
            else:
                ranges.append(0.80)  # Wall

        filtered = laser_filter.filter_scan_ranges(
            ranges=ranges,
            angle_min=0.0,
            angle_increment=math.radians(10),
            laser_offset_x=0.0,
            laser_offset_y=0.0
        )

        # 1. All chassis reflections must be masked to NaN
        assert math.isnan(filtered[0])
        assert math.isnan(filtered[9])
        assert math.isnan(filtered[18])
        assert math.isnan(filtered[27])

        # 2. Obstacle and wall beams must be 100% preserved
        assert math.isclose(filtered[4], 0.35, rel_tol=1e-5)   # 40 deg obstacle
        assert math.isclose(filtered[12], 0.80, rel_tol=1e-5)  # Wall
        assert math.isclose(filtered[30], 0.80, rel_tol=1e-5)  # Wall
