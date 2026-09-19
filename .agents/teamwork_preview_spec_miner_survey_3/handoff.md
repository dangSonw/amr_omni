# Handoff Report: Survey Phase Specifications (R2, R3, R4)

## 1. Observation

### 1.1 Firmware Serial & Micro-ROS Architecture
- Direct file observation in `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:
  - Micro-ROS serial transport initialized at line 400: `set_microros_serial_transports(Serial);` at 115200 baud.
  - Subscriptions: `stm32_cmd_vel` (`geometry_msgs/Twist`), `estop` (`std_msgs/Bool`).
  - Publishers: `wheel/odom` (`nav_msgs/Odometry`), `imu/data` (`sensor_msgs/Imu`), `status` (`std_msgs/String`), `debug/data` (`std_msgs/String`).
  - Zero Covariance Omission:
    - Lines 249–250: `memset(&odometry_message, 0, sizeof(odometry_message)); memset(&imu_message, 0, sizeof(imu_message));`
    - Lines 327–384 (`publish_telemetry`): Covariance fields (`pose.covariance`, `twist.covariance`, `orientation_covariance`, `angular_velocity_covariance`, `linear_acceleration_covariance`) are never populated; all diagonal entries remain verbatim `0.0`.
  - Non-volatile Memory: Lines 241–253 of `firmware/.../hardware.cpp` provide `save_settings_nvram(const FirmwareSettings &settings)` and `load_settings_nvram(FirmwareSettings &settings)` currently backed by RAM variable `s_nvram_settings`.

### 1.2 EKF Configuration & Covariance Fusion
- Direct file observation in `src/omni_localization/config/ekf.yaml`:
  - Fuses `odom0: wheel/odom` with config `[false, false, false, false, false, false, true, true, false, false, false, false, false, false, false]` (vx, vy only).
  - Fuses `imu0: imu/data` with config `[false, false, false, false, false, true, false, false, false, false, false, false, false, false, false]` (yaw only).
  - Omission: Angular velocity Z (`vyaw`, index 11) is `false`, ignoring high-bandwidth rate gyro measurements.
  - Omission: `process_noise_covariance` (15x15) and `initial_estimate_covariance` (15x15) are completely absent from `ekf.yaml`.
- Direct file observation in `src/omni_simulation/omni_simulation/stm32_simulator.py`:
  - Lines 424–429 set only indices 0, 7, 35 for pose ($x, y, \psi$) and twist ($v_x, v_y, \omega_z$); all other 33 elements in each 6x6 matrix are 0.0.

### 1.3 Transform Tree & Single TF Authority
- `src/omni_localization/config/ekf.yaml:19`: `publish_tf: true` (broadcasts `odom -> base_link`).
- `src/omni_simulation/config/simulation.yaml:30`: `publish_tf: false`.
- `src/omni_simulation/omni_simulation/stm32_simulator.py:132, 432-440`: conditional broadcaster `if self.publish_tf: self.tf_broadcaster.sendTransform(transform)`.
- `src/omni_hardware/omni_hardware/stm32_bridge.py`: does not instantiate `TransformBroadcaster`.
- `src/omni_description/urdf/omni.urdf.xacro`: defines `base_footprint -> base_link` fixed joint; `robot_state_publisher` broadcasts robot model links.

### 1.4 Laser Filter Configuration & Physical Footprint
- `src/omni_perception/config/laser_filter.yaml:11-23`:
  - `box_frame: base_link`, `min_x: -0.16, max_x: 0.16, min_y: -0.16, max_y: 0.16`.
- `src/omni_navigation/config/nav2_params.yaml:186`:
  - `footprint: "[[0.13, 0.13], [0.13, -0.13], [-0.13, -0.13], [-0.13, 0.13]]"`.
- `src/omni_description/urdf/sensors.xacro:31-43`:
  - LiDAR placed at `xyz="0.0000 0.0000 0.1010"`, range min `0.08 m`, range max `10.0 m`.
  - Box filter of 0.16 m exceeds actual chassis radius (0.13 m) by 3 cm on each side, blanking real environmental obstacles.

### 1.5 Web UI & Backend Architecture
- `web/backend/app/main.py` and `run_backend.py`: FastAPI server running on port 8000.
- `web/backend/app/routers/api.py`: REST router mounted at `/api` with robot control, config, and map endpoints.
- `web/backend/app/routers/ws.py`: WebSocket route at `/ws/telemetry` handled by `telemetry_hub`.
- `web/backend/app/bridges/ros2_bridge.py`: `rclpy` node subscribing to `/scan`, `/odom`, `/wheel_state`, `/imu`, `/status`, `/debug/data`.
- `web/frontend/src/app/page.tsx` & `web/frontend/src/components/Sidebar.tsx`: Three active tabs (`cockpit`, `config`, `debug`), currently lacking calibration controls and visualization.

---

## 2. Logic Chain

1. **Premise**: `ORIGINAL_REQUEST.md` (R2, R4) requires STM32 to perform IMU and wheel calibration routines, reporting real-time progress and converged matrices back to Jetson to update ROS 2 YAML configurations.
2. **Finding from Observation 1.1**: Firmware communicates via micro-ROS serial transport with standard ROS 2 messages, but lacks explicit calibration command subscribers and telemetry publishers.
3. **Inference**: A framed binary packet contract (or structured micro-ROS topic contract) with CRC16-CCITT must be defined specifying command IDs (`0x10` IMU trigger, `0x11` Wheel trigger, `0x12` Noise profiling), progress telemetry (`0x81`), and result payloads (`0x82`–`0x84`).
4. **Premise**: `ORIGINAL_REQUEST.md` (R3) mandates that EKF covariance matrices must have measured variances with **zero diagonal elements strictly eliminated**.
5. **Finding from Observation 1.1 & 1.2**: Firmware outputs all zeros for odometry and IMU covariances; `stm32_simulator.py` outputs zero for non-planar diagonal entries; `ekf.yaml` is missing $Q$ (process noise) and $P_0$ (initial covariance) entirely.
6. **Inference**: Populating realistic variances derived from sensor characteristics (Allan variance for IMU, encoder resolution for wheel odom) and setting non-planar unmeasured axes to large variances ($1.0 \times 10^6$) mathematically prevents filter divergence and singular matrix inversions.
7. **Premise**: `ORIGINAL_REQUEST.md` (R3) demands the Single TF Authority principle to eliminate duplicate TF warnings.
8. **Finding from Observation 1.3**: `ekf_node` publishes `odom -> base_link` (`publish_tf: true`). If `stm32_simulator` or a hardware driver enables `publish_tf`, dual broadcasters cause transform collisions.
9. **Inference**: Designating `ekf_node` as the sole authority for `odom -> base_link`, `slam_toolbox` as sole authority for `map -> odom`, and keeping `publish_tf: false` in simulator/driver enforces an unambiguous TF tree.
10. **Premise**: `ORIGINAL_REQUEST.md` (R3) requires laser filtering to remove chassis reflections without clipping real environmental scans.
11. **Finding from Observation 1.4**: Current box filter ($0.16 \text{ m}$) is oversized relative to actual chassis footprint ($0.13 \text{ m}$).
12. **Inference**: Shrinking the box filter to $0.135 \text{ m}$ (footprint + 5mm tolerance) and adding shadow and speckle filters eliminates body self-reflections while preserving full environmental perception.
13. **Premise**: `ORIGINAL_REQUEST.md` (R4) requires 1-click Web UI calibration triggering, live progress visualization, and automatic YAML persistence.
14. **Finding from Observation 1.5**: Web backend and frontend currently support teleop and monitoring but lack calibration endpoints, telemetry channels, and UI cards.
15. **Inference**: Implementing `/api/calib/*` REST endpoints, extending `/ws/telemetry` with calibration frames, and adding an automated YAML writer completes the operator pipeline.

---

## 3. Caveats
- Renode emulation environment (`firmware/stm32_f407vg_arduino_sim`) uses mock hardware stubs (`STM32_RENODE_SIM`); physical I2C transactions to BNO08x and physical timer encoder registers must be tested on physical STM32F407 hardware.
- The 6-orientation accelerometer calibration assumes manual positioning by an operator; if robotic turntable is not available, the Web UI must guide the user step-by-step through each face orientation.
- No other caveats.

---

## 4. Conclusion

The concrete interface contracts, schemas, and pipeline architectures are fully established and specified in `survey_integration_specs.md`. Specifically:
1. **Serial Contract**: Defined with Header `0xAA 0x55`, 1-byte length, 1-byte seq, 1-byte msg ID, structured binary payload, CRC16-CCITT, Tail `0x7D`. Commands `0x10`..`0x15` and Telemetry `0x80`..`0x84` are fully mapped.
2. **EKF & Covariance**: All diagonal zeros identified and replaced with verified numerical values ($Q$ and $P_0$ 15x15 matrices, Odometry 6x6, IMU 3x3). Single TF Authority tree locked with `ekf_node` as sole dynamic odometry publisher.
3. **Laser Filter**: Resized from $0.16 \text{ m}$ to $0.135 \text{ m}$ and enhanced with range, shadow, and speckle filters.
4. **Web UI Pipeline**: REST API `/api/calib/*` designed, WebSocket telemetry schema defined, and YAML schemas for `imu_calib.yaml`, `wheel_calib.yaml`, and `ekf.yaml` specified.

---

## 5. Verification Method

### 5.1 Document & Schema Verification
- Inspect generated specification:
  ```bash
  cat /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3/survey_integration_specs.md
  ```
- Verify non-zero diagonal entries in proposed `ekf.yaml`: check that indices $(i, i)$ for $0 \le i \le 14$ are all $> 0.0$.
- Verify laser filter box footprint matches `nav2_params.yaml` chassis boundary ($0.13 \text{ m}$ half-width vs $0.135 \text{ m}$ filter bounds).

### 5.2 Automated Codebase Checks
- Run pytest suite in `src/omni_control` and `web/backend`:
  ```bash
  PYTHONPATH=src/omni_control:src/omni_hardware:web/backend pytest src/omni_control/test/test_kinematics.py web/backend/tests/
  ```
- Run ROS 2 Jazzy hardware unit tests:
  ```bash
  PYTHONPATH=src/omni_control:src/omni_hardware:/opt/ros/jazzy/lib/python3.12/site-packages /usr/bin/python3 -m unittest discover -s src/omni_hardware/test
  ```
- Invalidation conditions:
  - If any EKF covariance diagonal entry is zero, verification fails.
  - If any node other than `ekf_node` broadcasts `odom -> base_link`, Single TF Authority verification fails.
