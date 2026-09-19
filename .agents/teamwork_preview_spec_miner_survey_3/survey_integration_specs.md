# Survey & Integration Specifications: Mecanum AGV Calibration & Estimation Upgrade

## Executive Summary
This document provides the authoritative technical specification mined from the `amr_omni` codebase (`/home/sonev/amr_omni`), foundational reference materials (`temp/docs/amr_omni.docx`, `temp/docs/Calib.docx`, `temp/docs/Calib_2.docx`, `temp/docs/Encoder.docx`), reference repositories (`temp/wheeltec_ros2`, `temp/linorobot2`, `temp/III-Robot-ROS2`), and user requirements (`ORIGINAL_REQUEST.md`).

It specifies the concrete interfaces, binary packet layouts, YAML schemas, TF tree authority, and Web UI calibration pipeline for Requirements **R2** (IMU Calibration), **R3** (EKF Sensor Fusion & Laser Filtering), and **R4** (Web UI Calibration Pipeline).

---

## Features Discovered

| # | Category | Feature | Description | Inputs | Outputs | Error Behavior | Discovered Via |
|---|----------|---------|-------------|--------|---------|----------------|----------------|
| 1 | Serial Protocol | micro-ROS Serial Bridge | Current transport between STM32 and Jetson running over USB/Serial | Serial stream at 115200 baud | Topics: `wheel/odom`, `imu/data`, `status`, `debug/data` | Ping timeout resets ROS entities after 3 failures | `firmware/.../main.cpp:399-440` |
| 2 | Serial Protocol | ROS Topic Contract | Contract defining valid topics and cmd_vel speed clamp bounds | Twist message (`linear.x`, `linear.y`, `angular.z`) | Boolean validation pass/fail | Drops out-of-bound / NaN commands | `src/omni_hardware/omni_hardware/stm32_contract.py:21-38` |
| 3 | Serial Protocol | Fixed-rate Command Relay | Relays validated twist commands at 50 Hz with watchdog timeout | `safe_cmd_vel` | `stm32_cmd_vel` | Emits zero Twist if age > 0.25 s | `src/omni_hardware/omni_hardware/stm32_bridge.py:121-130` |
| 4 | Serial Protocol | Reference Binary Packet Frame | Fixed 24-byte packet framing with BCC XOR checksum used in Wheeltec | Serial bytes with 0x7B header | Deserialized 16-bit velocities and IMU raw integers | Checksum mismatch rejects frame | `temp/wheeltec_ros2/.../v650_wheeltec_robot.cpp:211-238` |
| 5 | Calibration Frame | Trigger IMU Calibration Command | Serial binary frame requesting STM32 to initiate gyro bias / accelerometer calibration | Command ID `0x10`, mode (0=Static, 1=6-Pos, 2=Tedaldi), sample count | `RESP_CALIB_ACK` (0x80) | Returns BUSY if robot moving or already calibrating | Specification Mining (R2, R4) |
| 6 | Calibration Frame | Trigger Wheel Calibration Command | Serial binary frame commanding STM32 to run wheel radius / kinematics calibration | Command ID `0x11`, motion pattern, test speed, target distance | `RESP_CALIB_ACK` (0x80) | Aborts immediately if E-Stop active or motor fault | Specification Mining (R1, R4) |
| 7 | Calibration Frame | Start Noise Profiling Command | Initiates static IMU data logging for Allan variance curve fitting | Command ID `0x12`, duration_sec, sample_rate_hz | `RESP_CALIB_ACK` (0x80) | Aborts if static detector detects vibration / motion | `temp/docs/Calib.docx`, R4 |
| 8 | Calibration Frame | Calibration Telemetry Frame | Real-time progress stream sent from STM32 to Jetson at 2-5 Hz | Elapsed time, current variance, stage counter | Telemetry packet `0x81` with progress % (0-100) | Emits status code error if motion detected during static phase | Specification Mining (R4) |
| 9 | Calibration Frame | Calibration Results Frame | Transmits converged calibration matrices and bias vectors back to Jetson | Computed matrices in STM32 RAM | Result packets `0x82` (IMU), `0x83` (Wheel), `0x84` (Noise) | Checksum CRC16-CCITT verifies data integrity | Specification Mining (R2, R3, R4) |
| 10 | EKF Fusion | Planar 2D Odometry Fusion | `robot_localization` EKF filter node fusing wheel odometry and IMU | `wheel/odom` (vx, vy), `imu/data` (yaw) | `/odometry/filtered`, dynamic TF `odom -> base_link` | Sensor timeout drops filter updates | `src/omni_localization/config/ekf.yaml:1-60` |
| 11 | EKF Fusion | Missing Covariance Diagnostics | Existing firmware & simulator leave diagonal covariance elements zero or unconfigured | Sensor message structs | All zeros in `main.cpp`; partial in `stm32_simulator.py` | EKF numerical instability or invalid weighting | `firmware/.../main.cpp:249-250`, `stm32_simulator.py:424-430` |
| 12 | TF Authority | Single TF Authority Principle | Sole authority rule designating `ekf_node` as the only broadcaster of `odom -> base_link` | Filtered state estimate | TF transform `odom -> base_link` | Simulator `publish_tf: false` eliminates duplicate TF conflict | `amr_omni.docx:Line 299-350`, `simulation.yaml:30` |
| 13 | Laser Filter | Scan Range Filter | Excludes invalid LiDAR range measurements beyond physical sensor bounds | `/scan` (`sensor_msgs/LaserScan`) | Screened range readings | Replaces values < 0.08m or > 10.0m with NaN | `src/omni_perception/config/laser_filter.yaml:3-10` |
| 14 | Laser Filter | Footprint Box Filter | 3D bounding box filter masking robot body reflections | Target frame `base_link` | Filtered LaserScan | Masks points in box `[-0.16, 0.16] x [-0.16, 0.16]` | `src/omni_perception/config/laser_filter.yaml:11-23` |
| 15 | Laser Filter | Shadows & Speckle Mitigation | Filters grazing returns and airborne dust at chassis edges | Raw scan rays | Cleaned scan rays | Removes steep angle disparities and isolated single-beam hits | `.claude/skills/lidar-filtering/SKILL.md:80-112` |
| 16 | Web Backend | FastAPI Robot REST Router | Provides RESTful controls for teleoperation, configuration, and navigation | HTTP requests | JSON response | 400/500 HTTP errors with structured detail | `web/backend/app/routers/api.py:1-207` |
| 17 | Web Backend | WebSocket Telemetry Hub | High-rate real-time bidirectional telemetry streaming to frontend | ROS topics via `Ros2Bridge` | JSON frames over `/ws/telemetry` | Graceful client unregister on disconnect | `web/backend/app/routers/ws.py:1-28` |
| 18 | Web UI Pipeline | Calibration REST API | Endpoints to trigger, step, query, and apply sensor calibrations | REST `POST /api/calib/*` | Progress, calibration matrices, diff previews | Returns error if robot is moving or safety active | Specification Mining (R4) |
| 19 | Web UI Pipeline | YAML Configuration Updater | Writes verified calibration results into persistent ROS 2 YAML configs | Calibrated parameters from STM32 | Writes `imu_calib.yaml`, `wheel_calib.yaml`, `ekf.yaml` | Atomic file write with backup prevents corruption | Specification Mining (R4) |

---

## Edge Cases

| # | Feature | Input | Observed / Specified Behavior |
|---|---------|-------|-------------------------------|
| 1 | Serial Protocol | Buffer overrun or noise injection | Parser state machine drops bytes until valid sync header (`0xAA 0x55` or `0x7B`) and validates packet length <= 128 bytes. Frame rejected if CRC16 fails. |
| 2 | IMU Calibration | Robot moved during static bias sampling | Sliding window acceleration variance exceeds threshold ($> 0.05 \text{ m/s}^2$); STM32 sets status `EXCESSIVE_MOTION`, pauses timer, and alerts Web UI. |
| 3 | Wheel Calibration | Wheel slip or obstruction during test run | Wheel speed PID error exceeds threshold ($|e| > 2.0 \text{ rad/s}$) or current limit exceeded; STM32 triggers E-Stop, aborts calibration, and reports `FAILED_MOTOR_STALL`. |
| 4 | EKF Fusion | Covariance diagonal element set to 0.0 | Extended Kalman Filter gain matrix $K = P H^T (H P H^T + R)^{-1}$ encounters singular matrix inversion or infinite gain; filter coordinates diverge to NaN. |
| 5 | EKF Fusion | Dual TF broadcaster conflict (`stm32_simulator` & `ekf_node`) | Both nodes publish transform `odom -> base_link`; TF buffer throws `TF_MULTIPLE_AUTHORITY` error, causing high-frequency coordinate flickering in RViz/Nav2. |
| 6 | Laser Filter | Box filter oversized ($0.16 \text{ m}$ vs chassis $0.13 \text{ m}$) | Genuine environmental obstacles located between 13 cm and 16 cm from robot body are erroneously blanked out with NaN, creating navigational collision hazard. |
| 7 | Web UI Pipeline | Operator closes browser during calibration | STM32 continues autonomous state machine; progress packets remain buffered or dropped; results stored in STM32 NVRAM; backend updates YAML regardless of UI connection. |
| 8 | Web UI Pipeline | Invalid YAML written due to disk full or syntax error | Backend performs atomic write via `.tmp` file and parses YAML before replacing active file; on failure, rolls back to previous valid YAML. |

---

## Detailed Technical Specifications

### 1. Serial Binary Protocol (STM32 <-> Jetson)

#### 1.1 Architectural Overview & Layering
Communication between the Jetson host and STM32 microcontroller operates either as a dedicated binary packet stream over serial (`/dev/ttyACM0` or `/dev/ttyTHS1`) or mapped through micro-ROS topics (`std_msgs/ByteMultiArray` on `calib/cmd` and `calib/status`).

To guarantee real-time determinism, memory safety, and cross-platform reliability, the protocol adopts the standard **COBS (Consistent Overhead Byte Stuffing)** or **Framed Binary Packet** architecture with a 16-bit CRC (CRC16-CCITT).

#### 1.2 Binary Frame Format
Every frame transmitted across the serial bus conforms to the following layout:

```
+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
| Header (2B)   | Length (1B)   | Seq ID (1B)   | Msg ID (1B)   | Payload (NB)  | CRC16 (2B)    | Tail (1B)     |
| 0xAA  0x55    | N (0..120)    | 0x00..0xFF    | 0x01..0xFF    | N bytes       | LSB    MSB    | 0x7D          |
+---------------+---------------+---------------+---------------+---------------+---------------+---------------+
```

- **Header**: 2 bytes fixed synchronization pattern `0xAA 0x55`.
- **Length**: 1 byte payload size $N$ ($0 \le N \le 120$).
- **Seq ID**: 1 byte rolling transaction counter for duplicate detection and packet loss tracking.
- **Msg ID**: 1 byte message identifier categorizing command, telemetry, or response.
- **Payload**: $N$ bytes of structured binary data (little-endian byte order).
- **CRC16**: 2 bytes CRC16-CCITT (Polynomial `0x1021`, Initial value `0xFFFF`, Little-Endian).
- **Tail**: 1 byte frame terminator `0x7D`.

#### 1.3 Command Frames (Jetson -> STM32)

##### A. Trigger IMU Calibration (`Msg ID = 0x10`)
Initiates the IMU intrinsic calibration routine on STM32.

```
Payload (6 bytes):
Offset | Field        | Type     | Description
-------+--------------+----------+---------------------------------------------
0      | calib_mode   | uint8_t  | 0x00: Static Gyro Bias (stationary 10-30s)
       |              |          | 0x01: 6-Orientation Accelerometer (ST AN4508)
       |              |          | 0x02: Tedaldi Multi-position (ICRA 2014)
1      | face_index   | uint8_t  | For 6-orientation: 0=+Z, 1=-Z, 2=+X, 3=-X, 4=+Y, 5=-Y
2..3   | sample_count | uint16_t | Number of samples to collect (e.g. 500 @ 50Hz = 10s)
4..5   | timeout_sec  | uint16_t | Maximum allowed duration in seconds before abort
```

##### B. Trigger Wheel & Kinematics Calibration (`Msg ID = 0x11`)
Commands STM32 to execute a controlled calibration motion or observe encoder feedback during manual translation.

```
Payload (10 bytes):
Offset | Field        | Type     | Description
-------+--------------+----------+---------------------------------------------
0      | test_type    | uint8_t  | 0x00: Linear X test (+1.0m)
       |              |          | 0x01: Lateral Y strafe test (+1.0m)
       |              |          | 0x02: In-place Rotation test (360 deg)
       |              |          | 0x03: Manual external push / Laser reference test
1      | reserved     | uint8_t  | Alignment byte (0x00)
2..5   | target_dist  | float32  | Commanded distance (meters) or angle (radians)
6..9   | test_speed   | float32  | Commanded velocity (m/s or rad/s)
```

##### C. Start Noise Profiling (`Msg ID = 0x12`)
Instructs STM32 to sample raw high-rate IMU data for Allan variance and noise covariance extraction.

```
Payload (6 bytes):
Offset | Field        | Type     | Description
-------+--------------+----------+---------------------------------------------
0..3   | duration_sec | uint32_t | Profiling duration in seconds (e.g. 60s, 3600s, 10800s)
4..5   | sample_rate  | uint16_t | Target rate in Hz (e.g. 100 Hz, 200 Hz)
```

##### D. Calibration Control Commands
- `Msg ID = 0x13`: **Abort Calibration** (`Payload: 0 bytes`). Immediately stops motors and resets calibration FSM to `IDLE`.
- `Msg ID = 0x14`: **Flash Commit** (`Payload: 0 bytes`). Writes active in-memory calibration parameters to STM32 Flash / EEPROM emulation.
- `Msg ID = 0x15`: **Flash Restore Defaults** (`Payload: 0 bytes`). Erases calibration NVRAM and restores factory defaults.

#### 1.4 Telemetry & Result Frames (STM32 -> Jetson)

##### A. Command Acknowledgment (`Msg ID = 0x80`)
```
Payload (3 bytes):
Offset | Field        | Type     | Description
-------+--------------+----------+---------------------------------------------
0      | acked_msg_id | uint8_t  | Message ID being acknowledged
1      | ack_status   | uint8_t  | 0x00: OK / Accepted
       |              |          | 0x01: REJECTED_BUSY
       |              |          | 0x02: REJECTED_INVALID_PARAM
       |              |          | 0x03: REJECTED_SAFETY_ACTIVE (E-Stop/Motor fault)
2      | error_code   | uint8_t  | Detailed subsystem error code
```

##### B. Real-time Calibration Progress (`Msg ID = 0x81`)
Streamed periodically (every 200 ms) while calibration is active.

```
Payload (12 bytes):
Offset | Field        | Type     | Description
-------+--------------+----------+---------------------------------------------
0      | calib_type   | uint8_t  | 0x10: IMU, 0x11: Wheel, 0x12: Noise
1      | current_stage| uint8_t  | Current step/face index
2      | progress_pct | uint8_t  | Progress percentage: 0..100 %
3      | state_code   | uint8_t  | 0x00: IN_PROGRESS
       |              |          | 0x01: AWAITING_NEXT_ORIENTATION
       |              |          | 0x02: CONVERGED_SUCCESS
       |              |          | 0x03: FAILED_MOTION_DETECTED
       |              |          | 0x04: FAILED_TIMEOUT
       |              |          | 0x05: FAILED_MATH_SINGULAR
4..7   | live_metric  | float32  | Current variance, residual error, or distance
8..11  | elapsed_ms   | uint32_t | Milliseconds elapsed since start
```

##### C. IMU Calibration Result Frame (`Msg ID = 0x82`)
Transmitted once upon successful completion of IMU calibration.

```
Payload (76 bytes):
Offset | Field              | Type       | Description
-------+--------------------+------------+---------------------------------------------
0..11  | gyro_bias[3]       | float32[3] | Gyro bias [bx, by, bz] in rad/s
12..23 | accel_bias[3]      | float32[3] | Accelerometer bias [bx, by, bz] in m/s^2
24..59 | accel_matrix[9]    | float32[9] | 3x3 combined matrix (Ta * Sa)^(-1) (row-major)
60..71 | gyro_scale[3]      | float32[3] | Gyro scale factor diagonal [sx, sy, sz]
72..75 | residual_rmse      | float32    | Root-mean-square residual error
```

##### D. Wheel & Kinematics Result Frame (`Msg ID = 0x83`)
Transmitted upon completion of wheel radius and kinematics calibration.

```
Payload (28 bytes):
Offset | Field              | Type       | Description
-------+--------------------+------------+---------------------------------------------
0..15  | wheel_radius_m[4]  | float32[4] | Calibrated radii for wheels 1, 2, 3, 4 (m)
16..19 | effective_l_m      | float32    | Calibrated effective wheelbase L (m)
20..23 | effective_w_m      | float32    | Calibrated effective track width W (m)
24..27 | consistency_error  | float32    | ||FK(IK(v)) - v|| error norm (< 1e-5)
```

##### E. Noise Profiling Result Frame (`Msg ID = 0x84`)
Transmitted upon completion of Allan variance noise profiling.

```
Payload (20 bytes):
Offset | Field              | Type       | Description
-------+--------------------+------------+---------------------------------------------
0..3   | gyro_noise_density | float32    | Angle Random Walk N_g (rad/s/sqrt(Hz))
4..7   | gyro_random_walk   | float32    | Rate Random Walk K_g (rad/s^2/sqrt(Hz))
8..11  | accel_noise_density| float32    | Velocity Random Walk N_a (m/s^2/sqrt(Hz))
12..15 | accel_random_walk  | float32    | Accel Random Walk K_a (m/s^3/sqrt(Hz))
16..19 | static_drift_dps   | float32    | Observed static gyro drift rate (deg/s)
```

---

### 2. EKF & Covariance Fusion (`robot_localization`)

#### 2.1 Deficiency Analysis in Existing Codebase
Inspection of the current repository reveals severe covariance vulnerabilities:
1. **Firmware Covariance Omission**: In `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:
   - Line 249: `memset(&odometry_message, 0, sizeof(odometry_message));`
   - Line 250: `memset(&imu_message, 0, sizeof(imu_message));`
   - Lines 327–384: Covariance arrays (`pose.covariance`, `twist.covariance`, `orientation_covariance`, `angular_velocity_covariance`, `linear_acceleration_covariance`) are NEVER populated. All 36 values in Odometry and 9 values in IMU remain **0.0**.
   - Consequence: In ROS 2, `robot_localization` treats zero covariance as either an uninitialized measurement or infinite confidence, leading to numerical divergence or matrix singularity.
2. **Simulator Covariance Incompleteness**: In `src/omni_simulation/omni_simulation/stm32_simulator.py`:
   - Only diagonal entries for $(x, y, \psi)$ in pose and $(v_x, v_y, \dot{\psi})$ in twist are assigned values (lines 424–429).
   - Non-planar entries ($z, \text{roll}, \text{pitch}, v_z, \dot{\phi}, \dot{\theta}$) are left as **0.0**.
3. **EKF Configuration Gaps**: In `src/omni_localization/config/ekf.yaml`:
   - `process_noise_covariance` (15x15 matrix) is completely missing, defaulting to generic fallback values.
   - `initial_estimate_covariance` (15x15 matrix) is completely missing.
   - `imu0_config` only fuses index 5 (Yaw). It fails to fuse index 11 (`vyaw` / angular velocity Z), ignoring the primary high-bandwidth sensor measurement of the gyroscope!

#### 2.2 Concrete Covariance Values
To satisfy Acceptance Criteria ("tuyệt đối không còn phần tử đường chéo nào bằng 0"), all diagonal elements must have physically justified positive numbers:

##### A. Odometry Message Covariances (`wheel/odom`)
- `pose.covariance` ($6 \times 6$ matrix, 36 elements):
  $$\text{diag} = [1.0 \times 10^{-3}, 1.0 \times 10^{-3}, 1.0 \times 10^{6}, 1.0 \times 10^{6}, 1.0 \times 10^{6}, 1.0 \times 10^{-2}]$$
  - Planar translation $(x, y)$: $\sigma_x = \sigma_y \approx 0.031 \text{ m} \implies \sigma^2 = 1.0 \times 10^{-3}$.
  - Non-planar degrees of freedom $(z, \text{roll}, \text{pitch})$: Set to $1.0 \times 10^6$ (unmeasured in 2D ground plane).
  - Planar rotation $(\text{yaw})$: $\sigma_\psi \approx 0.1 \text{ rad} \implies \sigma^2 = 1.0 \times 10^{-2}$.
- `twist.covariance` ($6 \times 6$ matrix, 36 elements):
  $$\text{diag} = [1.0 \times 10^{-2}, 1.0 \times 10^{-2}, 1.0 \times 10^{6}, 1.0 \times 10^{6}, 1.0 \times 10^{6}, 2.0 \times 10^{-2}]$$
  - Planar velocity $(v_x, v_y)$: $\sigma_v = 0.1 \text{ m/s} \implies \sigma^2 = 1.0 \times 10^{-2}$.
  - Non-planar velocity $(v_z, \omega_x, \omega_y)$: Set to $1.0 \times 10^6$.
  - Angular velocity $(\omega_z)$: $\sigma_\omega = 0.141 \text{ rad/s} \implies \sigma^2 = 2.0 \times 10^{-2}$.

##### B. IMU Message Covariances (`imu/data`)
- `orientation_covariance` ($3 \times 3$ matrix, 9 elements):
  $$\text{diag} = [1.0 \times 10^{6}, 1.0 \times 10^{6}, 2.5 \times 10^{-3}]$$
  - Yaw accuracy from BNO08x / calibrated filter: $\sigma \approx 0.05 \text{ rad} \implies \sigma^2 = 2.5 \times 10^{-3}$.
  - Roll and pitch: $1.0 \times 10^6$.
- `angular_velocity_covariance` ($3 \times 3$ matrix, 9 elements):
  $$\text{diag} = [1.0 \times 10^{6}, 1.0 \times 10^{6}, 1.0 \times 10^{-4}]$$
  - Gyroscope noise from Allan variance: $N_g \approx 1.4 \times 10^{-4} \text{ rad/s}/\sqrt{\text{Hz}} \implies \sigma^2 \approx 1.0 \times 10^{-4} \text{ rad}^2/\text{s}^2$.
- `linear_acceleration_covariance` ($3 \times 3$ matrix, 9 elements):
  $$\text{diag} = [1.0 \times 10^{-2}, 1.0 \times 10^{-2}, 1.0 \times 10^{-2}]$$
  - Accelerometer noise: $\sigma_a = 0.1 \text{ m/s}^2 \implies \sigma^2 = 1.0 \times 10^{-2} \text{ m}^2/\text{s}^4$.

##### C. Complete `ekf.yaml` Process & Initial Covariance Matrices
The updated `src/omni_localization/config/ekf.yaml` specification:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    sensor_timeout: 0.2
    transform_time_offset: 0.0
    transform_timeout: 0.05
    print_diagnostics: false
    debug: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    publish_tf: true
    publish_acceleration: false

    # Wheel Odometry input: fuses vx, vy, and vyaw
    # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    odom0: wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false

    # IMU input: fuses yaw and vyaw (angular velocity Z)
    imu0: imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    use_control: false

    # 15x15 Process Noise Covariance Q (Zero diagonal elements strictly prohibited)
    # Order: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    process_noise_covariance: [
      0.05, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.05, 0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  1e-4, 0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  1e-4,  0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   1e-4,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.03, 0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.025, 0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.025, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   1e-4, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  1e-4,  0.0,   0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   1e-4,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.02, 0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.01, 0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.01, 0.0,
      0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.01
    ]

    # 15x15 Initial Estimate Covariance P0
    initial_estimate_covariance: [
      1e-5, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  1e-5, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-1, 0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2
    ]
```

#### 2.3 Single TF Authority Principle
To eradicate duplicate TF warnings and pose jumps, the system enforces strict single-source transform assignment:

```
[ map ]
   │
   ▼  (Broadcast solely by SLAM Toolbox / AMCL)
[ odom ]
   │
   ▼  (Broadcast solely by ekf_node from robot_localization)
[ base_link ]
   │
   ├─► [ base_footprint ]  (robot_state_publisher)
   ├─► [ lidar_link_1 ]    (robot_state_publisher)
   ├─► [ imu_link_1 ]      (robot_state_publisher)
   ├─► [ camera_link_1 ]   (robot_state_publisher)
   └─► [ omni_wheel_* ]    (robot_state_publisher)
```

**Enforcement Rules**:
1. `publish_tf` in `src/omni_localization/config/ekf.yaml` **MUST be `true`**.
2. `publish_tf` in `src/omni_simulation/config/simulation.yaml` and `stm32_simulator.py` **MUST be `false`**.
3. Hardware bridge (`stm32_bridge.py`) must never broadcast `odom -> base_link`.
4. SLAM Toolbox / AMCL is the exclusive broadcaster of `map -> odom`.

---

### 3. Laser Filter Configuration

#### 3.1 Robot Footprint vs Filter Geometry
From `src/omni_description/urdf/wheels.xacro` and `src/omni_navigation/config/nav2_params.yaml`:
- Wheelbase $L = 0.1312 \text{ m}$ (center to wheel $= 0.0656 \text{ m}$).
- Trackwidth $W = 0.1312 \text{ m}$ (center to wheel $= 0.0656 \text{ m}$).
- Wheel radius $R = 0.03 \text{ m}$. Outer wheel contact edge: $0.0656 + 0.030 = 0.0956 \text{ m}$.
- Body chassis footprint in `nav2_params.yaml`: `[[0.13, 0.13], [0.13, -0.13], [-0.13, -0.13], [-0.13, 0.13]]` (a $26 \text{ cm} \times 26 \text{ cm}$ square).
- Camera protrudes at $x = 0.1035 \text{ m}$.
- LiDAR position: `xyz="0.0000 0.0000 0.1010"` (centered at $(0, 0)$, height $10.1 \text{ cm}$).

**Flaw in Current `laser_filter.yaml`**:
The current configuration sets:
```yaml
min_x: -0.16
max_x: 0.16
min_y: -0.16
max_y: 0.16
```
This is a $32 \text{ cm} \times 32 \text{ cm}$ box, clipping a $3 \text{ cm}$ perimeter band of real environment points outside the robot chassis.

#### 3.2 Complete Multi-Stage Laser Filter Chain
The recommended filter configuration in `src/omni_perception/config/laser_filter.yaml`:

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    # 1. Physical sensor range filter (matching sensors.xacro min 0.08m, max 10.0m)
    filter1:
      name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.08
        upper_threshold: 10.0
        lower_replacement_value: .nan
        upper_replacement_value: .inf

    # 2. Tight footprint box filter (matching exact 0.13m chassis + 5mm clearance)
    filter2:
      name: footprint_box_filter
      type: laser_filters/LaserScanBoxFilter
      params:
        box_frame: base_link
        min_x: -0.135
        max_x: 0.135
        min_y: -0.135
        max_y: 0.135
        min_z: 0.02
        max_z: 0.35
        invert: false

    # 3. Shadow filter to eliminate grazing edge points around robot corners
    filter3:
      name: shadow_filter
      type: laser_filters/LaserScanShadowsFilter
      params:
        min_angle: 10.0
        max_angle: 170.0
        neighbors: 2
        window: 1
        remove_shadow_start_point: true

    # 4. Speckle filter to eliminate dust particles and noise returns
    filter4:
      name: speckle_filter
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range: 2.0
        max_range_difference: 0.05
        filter_window: 2
```

---

### 4. Web UI Calibration Pipeline

#### 4.1 End-to-End Workflow Architecture
The calibration pipeline coordinates the Web UI, FastAPI backend, ROS 2 graph, and STM32 microcontroller:

```
[ Operator Web UI ]
       │
       ▼ (1. Click 'Start IMU Calibration' / REST POST /api/calib/start)
[ FastAPI Backend ]
       │
       ▼ (2. Formats binary CMD_CALIB_TRIGGER frame -> serial / micro-ROS)
[ STM32 Microcontroller ]
       │
       ├─► (3a. Computes calibration routine in FreeRTOS task)
       │
       ▼ (3b. Streams TELEM_CALIB_PROGRESS @ 5 Hz)
[ FastAPI TelemetryHub ]
       │
       ▼ (4. Pushes WebSocket update: stage, % progress, current metric)
[ Web UI Real-Time Progress Bar & Variance Chart ]
       │
       ▼ (5. STM32 completes -> sends TELEM_CALIB_RESULT frame)
[ FastAPI Calibration Service ]
       │
       ├─► (6a. Atomically updates YAML configs: imu_calib.yaml, ekf.yaml)
       ├─► (6b. Sends CMD_CALIB_FLASH_COMMIT to persist on STM32)
       │
       ▼ (7. Returns complete calibration parameters & matrix visualization)
[ Web UI Calibration Dashboard ]
```

#### 4.2 REST API Specification
The following endpoints are specified for `web/backend/app/routers/calibration.py` (mounted under `/api/calib`):

| Method | Endpoint | Request Body | Response Body | Description |
|---|---|---|---|---|
| `POST` | `/api/calib/start` | `CalibStartRequest` | `CalibActionResponse` | Triggers a calibration routine (IMU, Wheel, Noise) |
| `POST` | `/api/calib/abort` | None | `CalibActionResponse` | Aborts ongoing calibration, commands stop to STM32 |
| `POST` | `/api/calib/step` | `CalibStepRequest` | `CalibActionResponse` | Advances to next face/orientation for multi-position calib |
| `GET` | `/api/calib/status` | None | `CalibStatusResponse` | Polls current calibration state, stage, progress % |
| `GET` | `/api/calib/results`| None | `CalibResultsResponse` | Retrieves latest calibration results and YAML diffs |
| `POST` | `/api/calib/apply` | `CalibApplyRequest` | `CalibActionResponse` | Writes parameters to YAML files and Flash NVRAM |

##### Request / Response Models (`web/backend/app/models.py`)
```python
class CalibStartRequest(BaseModel):
    target: Literal["imu", "wheel", "noise"]
    mode: Optional[str] = "static_bias"  # "static_bias", "six_orientation", "tedaldi", "linear_1m"
    sample_count: int = 500
    test_speed_mps: float = 0.2
    target_dist_m: float = 1.0

class CalibStatusResponse(BaseModel):
    active: bool
    target: Optional[str] = None
    stage: str
    progress_percent: int
    status_code: str  # "IDLE", "IN_PROGRESS", "AWAITING_STEP", "SUCCESS", "FAILED"
    elapsed_sec: float
    current_metrics: dict
    error_message: Optional[str] = None

class ImuCalibResult(BaseModel):
    gyro_bias: List[float]       # [bx, by, bz] in rad/s
    accel_bias: List[float]      # [bx, by, bz] in m/s^2
    accel_scale_matrix: List[List[float]] # 3x3 matrix
    gyro_noise_density: float    # rad/s/sqrt(Hz)
    accel_noise_density: float   # m/s^2/sqrt(Hz)
    residual_rmse: float

class WheelCalibResult(BaseModel):
    wheel_radii_m: List[float]   # [r1, r2, r3, r4]
    effective_wheelbase_m: float
    effective_track_width_m: float
    consistency_error: float
```

#### 4.3 WebSocket Telemetry Integration
The existing `/ws/telemetry` payload is extended with an optional `calib` field:

```json
{
  "calib": {
    "active": true,
    "target": "imu",
    "stage": "static_gyro_bias",
    "progress_percent": 74,
    "status_code": "IN_PROGRESS",
    "elapsed_sec": 7.4,
    "live_variance": 0.000038,
    "gyro_bias_estimate": [0.0011, -0.0007, 0.0002]
  }
}
```

#### 4.4 Automated YAML Configuration Schemas

##### A. `imu_calib.yaml` (Saved in `src/omni_localization/config/imu_calib.yaml`)
```yaml
imu_calibration:
  sensor_model: "BNO08x"
  calibration_date: "2026-09-19T10:00:00Z"
  gyroscope:
    bias_rad_s: [0.00112, -0.00078, 0.00021]
    noise_density_rad_s_sqrthz: 0.00014
    random_walk_rad_s2_sqrthz: 0.000015
    static_drift_deg_s: 0.024
  accelerometer:
    bias_mps2: [0.0412, -0.0185, 0.0921]
    noise_density_mps2_sqrthz: 0.0019
    random_walk_mps3_sqrthz: 0.00012
    correction_matrix:
      [1.0021,  0.0012, -0.0031,
       0.0009,  0.9984,  0.0018,
      -0.0028,  0.0022,  1.0015]
  quality_metric:
    residual_rmse: 0.0014
    status: "PASS"
```

##### B. `wheel_calib.yaml` (Saved in `src/omni_control/config/wheel_calib.yaml`)
```yaml
wheel_calibration:
  calibration_date: "2026-09-19T10:00:00Z"
  nominal_wheel_radius_m: 0.03
  wheel_radii_m:
    wheel_1_fr: 0.03014
    wheel_2_fl: 0.02996
    wheel_3_rl: 0.03008
    wheel_4_rr: 0.02992
  kinematics_geometry:
    nominal_wheelbase_m: 0.1312
    nominal_track_width_m: 0.1312
    effective_wheelbase_m: 0.13145
    effective_track_width_m: 0.13112
  consistency:
    forward_inverse_rms_error: 3.2e-6
    status: "PASS"
```

#### 4.5 Web Frontend UI Components
The frontend will introduce a dedicated **Calibration** tab (`activeTab === "calib"`) in `web/frontend/src/components/Sidebar.tsx`:
1. **IMU Calibration Card**:
   - 1-click button: "Start Static Gyro Bias Calibration".
   - 6-orientation wizard with 3D cube illustration showing which face to turn upward.
   - Live Allan variance / gyro drift plot.
2. **Wheel Calibration Card**:
   - Automated 1m linear & 360° in-place rotation test buttons.
   - Visual display of 4 wheel radii with error deviations.
3. **EKF Covariance Tuning Card**:
   - Visual heatmaps of Process Noise Matrix $Q$ and Measurement Covariance Matrices $R$.
   - Real-time indicator confirming zero diagonal values are completely eradicated.
4. **Action Bar**:
   - "Save & Commit to Flash": Sends `/api/calib/apply`, writing YAMLs and persisting to STM32 NVRAM.
