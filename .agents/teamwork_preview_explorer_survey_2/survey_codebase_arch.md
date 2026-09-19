# Comprehensive Codebase & Architecture Survey Report: AMR Omni
**Project**: Mecanum / Omni AGV Calibration and Estimation Upgrade  
**Workspace**: `/home/sonev/amr_omni`  
**Working Directory**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2`  
**Date**: 2026-09-19  
**Investigator**: teamwork_preview_explorer_survey_2  

---

## 1. Executive Summary & Architectural Overview

The `amr_omni` repository implements a complete autonomous mobile robot (AMR) software stack for a 4-wheel omnidirectional / Mecanum robotic platform. The system operates across a dual-host paradigm:
1. **Simulation & Development Environment**: WSL2 Ubuntu 24.04 (amd64) running ROS 2 Jazzy Jalisco, Gazebo Harmonic (gz-sim 8.11.0), and PlatformIO for STM32 firmware compilation and Renode peripheral simulation.
2. **Physical Robot Target**: Jetson Nano running JetPack 4.x / L4T 32.x (Ubuntu 18.04 aarch64) with native ROS 1 Melodic and serial communication to an STM32F407VG microcontroller.
3. **Firmware Microcontroller**: STM32F407VGT6 (ARM Cortex-M4F, 168 MHz, 128 KB RAM, 1 MB Flash) executing FreeRTOS and micro-ROS (XRCE-DDS serial transport) for real-time motor PID control, encoder accumulation, IMU sampling, and safety watchdog execution.

### High-Level Data Flow Topology
```text
[ Physical Motors / Encoders ] ◄── GPIO / Timer ──► [ STM32F407VG Firmware (FreeRTOS) ]
[ IMU Sensor (BNO08x) ]        ─── I2C Bus ───────►   │ (100Hz PID / 100Hz Kalman / 50Hz IMU)
                                                      ▼
                                           micro-ROS Serial Transport (115200 baud)
                                                      │ (XRCE-DDS / UART CDC)
                                                      ▼
                                            [ micro_ros_agent ]
                                                      │
             ┌────────────────────────────────────────┼────────────────────────────────────────┐
             │ ROS 2 Jazzy Topics                     │                                        │
             ▼                                        ▼                                        ▼
    /wheel/odom (nav_msgs/Odometry)           /imu/data (sensor_msgs/Imu)            /debug/data (JSON)
             │                                        │                                        │
             └───────────────────┬────────────────────┘                                        │
                                 ▼                                                             ▼
                     [ robot_localization (EKF) ]                                  [ Jetson Web Backend ]
                     Fuses Odometry + IMU yaw                                      FastAPI + WebSockets
                     Broadcasts: odom ──► base_link                                            │
                                 │                                                             ▼
                                 ▼                                                    [ Next.js Frontend ]
                        /odometry/filtered                                            Control Cockpit
                                 │                                                    Debug Monitor
                     ┌───────────┴───────────┐                                        System Config
                     ▼                       ▼
            [ SLAM Toolbox / AMCL ]    [ Nav2 Controller ]
            map ──► odom               MPPI / DWB
```

---

## 2. Subsystem 1: STM32 Firmware Architecture

### 2.1. Codebase Location & Build Configuration
- **Active Source Project**: `firmware/stm32_f407vg_arduino_sim/`
  - Build Framework: PlatformIO (`platformio.ini`) with `framework = arduino` using `platform = ststm32`.
  - MCU Target: `board = disco_f407vg` (STM32F407VGT6).
  - Floating Point Hardware: Hardware FPU enabled (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`).
  - Middleware: FreeRTOS Kernel integrated via pre-build script (`extra_script.py`) extracting `framework-stm32cubef4` FreeRTOS ARM_CM4F port with `USE_FreeRTOS_HEAP_4`.
  - Communication Stack: `micro_ros_platformio` with `board_microros_transport = serial` and `board_microros_distro = jazzy`.
  - IMU Library: `adafruit/Adafruit BNO08x@^1.2.7`.
- **Placeholder Directories**:
  - `firmware/stm32_f407vg_stm3cube_sim/`: Configuration scaffold with empty `src/` and `include/`.
  - `firmware/stm32f1/README.md` and `firmware/stm32f4/README.md`: Non-buildable documentation placeholders.

### 2.2. Detailed File Structure
```text
firmware/stm32_f407vg_arduino_sim/
├── extra_script.py             # Pre-build Python script configuring FreeRTOS sources & paths
├── include/
│   ├── FreeRTOSConfig.h       # FreeRTOS kernel settings (1000Hz tick, 128KB heap, mutexes)
│   ├── firmware_config.h      # Physical constants (wheelbase, radius, CPR, loop periods)
│   ├── hardware.h             # Hardware abstraction layer interface (GPIO, I2C, Watchdog)
│   ├── kalman.h               # Scalar Kalman filter class for wheel speed and twist
│   ├── kinematics.h           # 4-wheel forward and inverse kinematics header
│   ├── pid.h                  # Velocity PID controller class with anti-windup & derivative filter
│   └── robot_state.h          # Global state data structures and initialization
├── platformio.ini             # PlatformIO build definition
├── src/
│   ├── hardware.cpp           # Pin assignments, encoder ISRs, BNO08x I2C driver, watchdog
│   ├── kalman.cpp             # Scalar Kalman filter implementation
│   ├── kinematics.cpp         # Forward/Inverse kinematics implementation & angle wrapping
│   ├── main.cpp               # Setup, FreeRTOS tasks (6 tasks), micro-ROS executor & topics
│   ├── pid.cpp                # Simple velocity PID implementation
│   └── robot_state.cpp        # State initialization and default firmware settings
└── test/
    ├── test_kalman/test_main.cpp      # Unity tests for ScalarKalman filter
    ├── test_kinematics/test_main.cpp  # Unity tests for kinematics consistency and saturation
    └── test_pid/test_main.cpp         # Unity tests for PID controller response
```

### 2.3. FreeRTOS Task Architecture & Concurrency
The firmware initializes and starts 6 concurrent FreeRTOS tasks inside `setup()` (`main.cpp:651-658`):

| Task Name | Stack Size | Priority | Period / Rate | Primary Responsibilities |
|---|---|---|---|---|
| `safety` | 320 words | 6 (Highest) | 20 ms (50 Hz) | Feeds hardware `IWDG` watchdog; samples physical E-stop pin (`PD0`) and motor fault pin (`PE0`); enforces command timeout (250 ms); controls status LEDs and buzzer. |
| `micro_ros` | 1024 words | 5 | 2 ms spin / 50 ms pub | Manages micro-ROS agent connection over `Serial` (115200); spins `rclc_executor`; pings agent every 1s (reconnects on 3 consecutive drops); publishes `/wheel/odom`, `/imu/data`, `/status`, `/debug/data`; receives `/stm32_cmd_vel` and `/estop`. |
| `control` | 512 words | 4 | 10 ms (100 Hz) | Evaluates safety & stale command flags; executes inverse kinematics (`compute_wheel_speeds`); runs 4 independent `WheelSpeedPid` controllers; applies feedforward + feedback PWM to motors. |
| `encoder` | 384 words | 4 | 10 ms (100 Hz) | Reads cumulative ticks from 4 hardware encoder ISR counters; computes raw differential wheel speeds; applies `ScalarKalman` filters; updates simulation state in Renode mode. |
| `imu` | 512 words | 3 | 20 ms (50 Hz) | Polls BNO08x IMU over I2C; extracts linear acceleration, gyroscope angular velocity, and rotation vector quaternion; normalizes quaternions; tracks IMU communication faults (>500 ms timeout). |
| `odometry` | 384 words | 3 | 10 ms (100 Hz) | Computes body twist ($v_x, v_y, \omega_z$) from measured wheel speeds via forward kinematics; filters velocities; computes 2D planar position ($x, y$) and yaw integration. |

- **Synchronization & Race-Condition Protection**:
  - Global `robot_state` is guarded by `state_mutex` (`SemaphoreHandle_t`, binary mutex created at `main.cpp:644`).
  - Access functions (`update_command_fields`, `update_encoder_fields`, `update_control_fields`, `update_imu_fields`, `update_pose_fields`, `copy_state`) acquire `state_mutex` with a 2 ms tick timeout before reading or writing.
  - Encoder hardware counters are read within `taskENTER_CRITICAL()` and `taskEXIT_CRITICAL()` blocks (`hardware.cpp:159-161`) to avoid corrupting 32-bit reads during ISR increments.

### 2.4. Hardware Drivers, Pin Mappings & Communication
- **Motor Actuation**:
  - PWM Output Pins: `PA8` (Wheel 0/FR), `PB4` (Wheel 1/FL), `PB5` (Wheel 2/RL), `PB9` (Wheel 3/RR).
  - Direction GPIO Pins: `PC0` (FR), `PC1` (FL), `PC2` (RL), `PC3` (RR).
  - Output scaling: PWM 0–255 mapped from normalized control command $[-1.0, 1.0]$.
- **Encoder Feedback**:
  - Phase A Pins: `PB0`, `PC6`, `PC8`, `PC10`.
  - Phase B Pins: `PB1`, `PC7`, `PC9`, `PC11`.
  - Method: External interrupt (`attachInterrupt`) on Phase A triggering on `CHANGE`, polling Phase B to decide count direction (`hardware.cpp:38-52`).
  - Resolution: `kEncoderCountsPerRevolution = 2048` counts/rev (`firmware_config.h:19`).
- **IMU Interface**:
  - Sensor: Adafruit BNO08x (Hillcrest Laboratories SH-2 sensor hub).
  - Bus: I2C (`Wire`, standard pins `PB6/SCL`, `PB7/SDA`).
  - Reports: `SH2_LINEAR_ACCELERATION` (20ms), `SH2_GYROSCOPE_CALIBRATED` (20ms), `SH2_ROTATION_VECTOR` (20ms).
- **Communication Protocol**:
  - Active: micro-ROS (XRCE-DDS) serial transport over USART (`Serial.begin(115200)`).
  - Serial Contract Specification (from `amr_omni.docx` Section 7.2):
    - Frame structure: `0x7B` (Header) | `reserved` | `reserved` | `vx` [int16_t, 2 bytes] | `vy` [int16_t, 2 bytes] | `wz` [int16_t, 2 bytes] | `Checksum XOR` | `0x7D` (Tail).
    - Scale factor: 1000 ($1.000\text{ m/s} \to 1000$). Total packet length: 24 bytes.

---

## 3. Subsystem 2: ROS 2 Workspace Packages Architecture

Under `/home/sonev/amr_omni/src/`, there are 10 packages (9 active ROS 2 Jazzy packages, and 1 ROS 1 bringup package flagged with `COLCON_IGNORE`):

```text
src/
├── omni_bringup/              # Launch orchestration & system bringup
├── omni_bringup_ros1/         # [COLCON_IGNORE] ROS 1 Melodic bringup for physical Jetson Nano
├── omni_control/              # Pure Python kinematics library & unit tests
├── omni_description/          # URDF/Xacro models, meshes, sensor links, and Gazebo plugins
├── omni_hardware/             # STM32 ROS topic contract, command watchdog & bridge
├── omni_localization/         # EKF odometry fusion (robot_localization), AMCL, SLAM Toolbox
├── omni_navigation/           # Nav2 navigation stack parameters and behavior trees
├── omni_perception/           # LaserScan filters and depthimage-to-laserscan conversion
├── omni_safety/               # Safety zone dynamic speed scaling & command watchdog policy
└── omni_simulation/           # Gazebo Harmonic world (amr_lab.sdf) & stm32_simulator node
```

### 3.1. Detailed Package Breakdown

#### 1. `omni_control`
- **Path**: `src/omni_control/omni_control/kinematics.py`
- **Responsibilities**: Standalone mathematical library implementing forward and inverse kinematics for 4-wheel omnidirectional drive. ROS-agnostic, pure NumPy/math implementation.
- **Physical Geometry Parameters**:
  - Wheel radius ($r$): $0.03\text{ m}$ (30 mm)
  - Wheelbase ($L$): $0.1312\text{ m}$ ($131.2\text{ mm}$)
  - Track width ($W$): $0.1312\text{ m}$ ($131.2\text{ mm}$)
  - Half-dimensions: $x_i = \pm 0.0656\text{ m}$, $y_i = \pm 0.0656\text{ m}$
  - Diagonal factor ($d$): $\sqrt{0.5} \approx 0.70710678118$ (wheels mounted at $45^\circ$ to chassis axes)
  - Geometric radius ($R$): $0.5 \cdot \sqrt{L^2 + W^2} = 0.5 \cdot \sqrt{0.1312^2 + 0.1312^2} \approx 0.0927725\text{ m}$
- **Kinematic Wheel Ordering**:
  - Index 0: `omni_wheel_joint_1` (FR: $+x, -y$, angle $-45^\circ$)
  - Index 1: `omni_wheel_joint_2` (FL: $+x, +y$, angle $+45^\circ$)
  - Index 2: `omni_wheel_joint_3` (RL: $-x, +y$, angle $+135^\circ$)
  - Index 3: `omni_wheel_joint_4` (RR: $-x, -y$, angle $-135^\circ$)
- **Inverse Kinematics Formula**:
  $$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix} = \frac{1}{r} \begin{bmatrix} d & d & R \\ -d & d & R \\ -d & -d & R \\ d & -d & R \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$
  Velocity saturation scaling factor: $s = \max\left(1.0, \frac{\max(|\omega_i|)}{\omega_{\max}}\right)$, with $\omega_i \leftarrow \omega_i / s$.
- **Forward Kinematics Formula** (Moore-Penrose Pseudoinverse):
  $$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1/d & -1/d & -1/d & 1/d \\ 1/d & 1/d & -1/d & -1/d \\ 1/R & 1/R & 1/R & 1/R \end{bmatrix} \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix}$$
- **Consistency**: Analytical identity $\text{forward}(\text{inverse}(\mathbf{v})) = \mathbf{v}$ holds to machine epsilon ($< 10^{-15}$).

#### 2. `omni_localization`
- **Config**: `src/omni_localization/config/ekf.yaml`
- **Launch**: `src/omni_localization/launch/ekf.launch.py`
- **Node**: `robot_localization/ekf_node`
- **Configuration Analysis**:
  - Target frames: `map_frame: map`, `odom_frame: odom`, `base_link_frame: base_link`, `world_frame: odom`.
  - Transform broadcasting: `publish_tf: true`.
  - Inputs:
    - `odom0: wheel/odom`: Configured as `[false, false, false, false, false, false, true, true, false, false, false, false, false, false, false]`. Fuses linear velocities $v_x, v_y$. Ignores angular velocity $\omega_z$.
    - `imu0: imu/data`: Configured as `[false, false, false, false, false, true, false, false, false, false, false, false, false, false, false]`. Fuses planar Yaw orientation (index 5). Ignores gyro $\omega_z$ and linear acceleration.
    - `imu0_remove_gravitational_acceleration: true`.
  - **Deficiencies Identified**:
    1. Process noise covariance (`process_noise_covariance`) and initial estimate covariance (`initial_estimate_covariance`) are not explicitly defined, falling back to default identity matrices.
    2. Does not fuse $\omega_z$ from either wheel odometry or IMU gyroscope, leaving rotational velocity unconstrained during dynamic maneuvers.
    3. Potential TF authority collision with `stm32_simulator` if simulation TF is enabled.

#### 3. `omni_perception`
- **Config**: `src/omni_perception/config/laser_filter.yaml`
- **Launch**: `src/omni_perception/launch/perception.launch.py`
- **Node**: `laser_filters/scan_to_scan_filter_chain`
- **Filters**:
  - Filter 1 (`range_filter`): `laser_filters/LaserScanRangeFilter` with `lower_threshold: 0.08`, `upper_threshold: 10.0`.
  - Filter 2 (`footprint_filter`): `laser_filters/LaserScanBoxFilter` with bounding box:
    $$\text{box\_frame} = \text{base\_link}, \quad x \in [-0.16, 0.16]\text{ m}, \quad y \in [-0.16, 0.16]\text{ m}, \quad z \in [-0.10, 0.50]\text{ m}$$
- **Topic Remapping**: Subscribes `/scan`, publishes `/scan_filtered`.
- **Architectural Observation**:
  - Currently in `omni_bringup/launch/simulation_bringup.launch.py`, `perception` is disabled by default (`default_value='false'`). Downstream SLAM and Nav2 subscribe directly to `/scan` rather than `/scan_filtered`. Enabling laser filtering requires passing `perception:=true` and aligning the topic inputs of SLAM/Nav2.

#### 4. `omni_description` (URDF & Extrinsics)
- **Files**: `urdf/omni.urdf.xacro`, `chassis.xacro`, `wheels.xacro`, `sensors.xacro`.
- **Coordinate Frames & Spatial Extrinsics (Lever-Arm)**:
  - `base_footprint` $\to$ `base_link`: Identity at ground contact.
  - `base_link` $\to$ `lidar_link_1`: Fixed joint `lidar_joint_1`:
    $$\mathbf{p}_{\text{lidar}} = [0.0000, 0.0000, 0.1010]\text{ m}, \quad \mathbf{R} = \mathbf{I} \ (0, 0, 0)$$
  - `base_link` $\to$ `imu_link_1`: Fixed joint `imu_joint_1`:
    $$\mathbf{p}_{\text{imu}} = [0.0000, 0.0000, 0.0000]\text{ m}, \quad \mathbf{R} = \mathbf{I} \ (0, 0, 0)$$
  - `base_link` $\to$ `camera_link_1`: Fixed joint `camera_joint_1`:
    $$\mathbf{p}_{\text{camera}} = [0.1035, 0.0000, 0.0630]\text{ m}, \quad \mathbf{R} = \mathbf{I} \ (0, 0, 0)$$
- **Frame Naming Inconsistency**:
  - In URDF (`sensors.xacro`), the child links are named `imu_link_1` and `lidar_link_1`.
  - In firmware (`main.cpp`) and simulation configs (`simulation.yaml`, `ekf.yaml`), the frame IDs are hardcoded to `imu_link` and `lidar_link`. This frame mismatch must be unified to ensure TF lookups in `robot_localization` resolve without warnings.

#### 5. `omni_hardware`
- **Files**: `omni_hardware/stm32_bridge.py`, `omni_hardware/stm32_contract.py`.
- **Node**: `omni_hardware/stm32_bridge`
- **Role**: Validates Twist limits ($v_{\max} = 0.54\text{ m/s}, \omega_{\max} = 3.0\text{ rad/s}$), checks command age against `command_timeout_sec = 0.25\text{ s}`, and relays commands from `safe_cmd_vel` to `stm32_cmd_vel` at 50 Hz.

#### 6. `omni_simulation`
- **Files**: `omni_simulation/stm32_simulator.py`, `worlds/amr_lab.sdf`, `config/simulation.yaml`.
- **Node**: `omni_simulation/stm32_simulator`
- **Role**: Software twin of the STM32 firmware for Gazebo Harmonic. Simulates 4-wheel closed-loop velocity PID, motor output saturation, scalar Kalman filtering, and outputs `/wheel/odom`, `/imu/data`, `/status`, and `/debug/data`.
- **TF Authority Control**: Parameter `publish_tf: false` in `simulation.yaml:30` allows disabling duplicate TF generation when EKF is active.

---

## 4. Subsystem 3: Jetson Web UI Architecture

The Web UI provides operators with a full-featured real-time monitoring and control cockpit:
- **Backend Stack**: Python 3.12, FastAPI, Starlette, Uvicorn, Pydantic v2.
- **Frontend Stack**: Next.js 14, React 18, TypeScript, Tailwind CSS, Recharts, Lucide-React.
- **Deployment Mode**: Next.js static HTML/JS bundle exported to `web/frontend/out/` and synced into `web/backend/static/`. FastAPI mounts the static directory and serves both REST/WebSockets and frontend assets on port 8000.

### 4.1. Directory Structure
```text
web/
├── backend/
│   ├── app/
│   │   ├── bridges/
│   │   │   ├── base.py         # Abstract BaseRobotBridge interface
│   │   │   ├── ros2_bridge.py  # rclpy-based bridge to ROS 2 topics
│   │   │   ├── ros1_bridge.py  # rospy-based bridge for Jetson Nano
│   │   │   └── mock_bridge.py  # Simulated telemetry generator for offline dev
│   │   ├── config.py           # Application settings, CORS, path definitions
│   │   ├── main.py             # FastAPI entrypoint, lifespan, CORS, static mounts
│   │   ├── models.py           # Pydantic data transfer schemas
│   │   ├── routers/
│   │   │   ├── api.py          # REST API endpoints
│   │   │   └── ws.py           # WebSocket endpoint /ws/telemetry
│   │   └── services/
│   │       ├── grid_planner.py # 2D costmap & A* path planning service
│   │       ├── stream_monitor.py # Packet frequency, latency & stream health tracking
│   │       └── telemetry_hub.py  # Broadcasts telemetry frames to connected WebSockets
│   ├── run_backend.py          # CLI runner script with argparse
│   └── tests/
│       ├── test_api.py         # TestClient API unit tests
│       └── test_full_system.py # End-to-end telemetry and command flow tests
└── frontend/
    ├── package.json            # NPM dependencies (React, Next, Tailwind, Recharts)
    ├── src/
    │   ├── app/page.tsx        # Main dashboard view with 3 primary tabs
    │   └── components/
    │       ├── Header.tsx           # Connection indicator, E-stop button, mode badges
    │       ├── MotionStatusCard.tsx # Vx, Vy, Wz gauge meters and compass yaw heading
    │       ├── SensorCards.tsx      # CPU, RAM, battery, active streams cards
    │       ├── LidarViewer.tsx      # Canvas 2D ray beam rendering
    │       ├── CameraFeed.tsx       # Live MJPEG streaming component
    │       ├── ConfigPanel.tsx      # Wheel geometry and PID parameter sliders
    │       └── DebugMonitor.tsx     # Recharts multi-line plots (raw vs filtered wheel speeds)
```

### 4.2. REST Endpoints & WebSocket Protocols
- **API Endpoints (`web/backend/app/routers/api.py`)**:
  - `GET /health`: Healthcheck.
  - `GET /api/status`: Robot connectivity, E-stop, mode, CPU/RAM usage.
  - `GET /api/config`: Current robot parameters (`wheel_radius_m`, `wheelbase_m`, `track_width_m`, `motor_kp`, etc.).
  - `POST /api/config`: Updates robot configuration (currently updates in-memory `self.config` only; does not write to disk).
  - `GET /api/streams`: Health, frequency, and latency metrics for 8 distinct communication streams.
  - `POST /api/cmd-vel`: Manual velocity joystick control (`linear_x`, `linear_y`, `angular_z`).
  - `POST /api/estop`: Toggle emergency stop latch.
  - `POST /api/reset-odom`: Reset odometry position to $(0, 0, 0)$.
  - `GET /api/camera/stream?type=rgb|depth`: Multipart MJPEG camera feed.
  - `GET /api/map`, `POST /api/map/save`, `POST /api/map/clear`: SLAM map retrieval and persistence.
  - `POST /api/nav/goal`, `POST /api/nav/waypoints`, `POST /api/nav/cancel`: Autonomous navigation dispatch.
- **WebSocket (`web/backend/app/routers/ws.py`)**:
  - Endpoint: `/ws/telemetry`
  - Broadcast Rate: 20 Hz (configured via `telemetry_hub.py`).
  - Payload Contents: Bundled JSON containing `wheels` (target, measured, ticks, pwm), `imu` (roll, pitch, yaw, accel, gyro, quaternion), `odom` (x, y, theta, vx, vy, wz), `lidar` (ranges, cartesian points), `status`, `debug` (`raw_ws`, `filt_ws`, `tgt_ws`, `mot_out`, `imu_q`, `body_v`, `cmd_v`).
- **Configuration Persistence Mechanism**:
  - Currently, `POST /api/config` calls `bridge.update_config(config)`, which updates the Python memory structure `self.config`. It does **not** persist values back to `src/omni_localization/config/ekf.yaml`, `src/omni_simulation/config/simulation.yaml`, or firmware headers. Implementing YAML persistence is a core deliverable for R4.

---

## 5. GitNexus Status & Impact Analysis

Per project instruction in `AGENTS.md`, GitNexus code intelligence is available via CLI:
- **Status Verification**:
  - Executable: `node .gitnexus/run.cjs`
  - Registered Repository: `amr_omni` (Note: CLI commands must include `--repo amr_omni` because multiple repositories are indexed in the global registry).
  - Index Status: 4881 symbols, 11670 relationships, 300 execution flows. Commit `de3e937` indexed.
- **Impact Analysis on Critical Symbols**:
  - Symbol `compute_wheel_speeds` (`firmware/stm32_f407vg_arduino_sim/include/kinematics.h`):
    - Direct callers: 70
    - Transitive blast radius: 130 symbols
    - Impact Risk Level: **CRITICAL**
  - Consequence: Any modification to the signature or internal scaling of kinematics functions directly impacts the firmware control loop, simulator, bridge, and verification suites. Edits must be strictly isolated to backward-compatible contracts.

---

## 6. Comprehensive Traceability Matrix: Requirements R1 to R5

The following table establishes the exact technical mapping between user requirements, theoretical references, affected source files, and concrete implementation targets:

| Req | Title & Scope | Technical Solution & Mathematical Basis | Primary Affected Files | Interface & Protocol Contracts |
|---|---|---|---|---|
| **R1** | **High-Precision Encoder Velocity Estimation** | • Implement Second-Order Phase-Locked Loop (PLL) Tracking Observer from ODrive (`Encoder.docx`):<br>$$\Delta\theta = \theta_{\text{meas}} - \hat{\theta}$$<br>$$\hat{\theta} \leftarrow \hat{\theta} + \Delta t \cdot \hat{\omega} + k_p \Delta\theta$$<br>$$\hat{\omega} \leftarrow \hat{\omega} + \Delta t \cdot k_i \Delta\theta$$<br>where $k_p = 2\xi\omega_n$, $k_i = \omega_n^2$.<br>• Eliminate low-speed quantization noise and acceleration phase lag.<br>• Automatic wheel radius scaling factor compensation ($K = \Delta x_{\text{meas}} / \Delta x_{\text{actual}}$).<br>• Mathematical consistency guarantee: $\text{FK}(\text{IK}(\mathbf{v})) == \mathbf{v}$ with error $< 10^{-5}$ and complete NaN/Inf rejection. | • `firmware/stm32_f407vg_arduino_sim/include/kalman.h` & `src/kalman.cpp` (or new `pll_observer.h/cpp`)<br>• `firmware/.../src/main.cpp` (`encoder_task`)<br>• `firmware/.../include/firmware_config.h`<br>• `firmware/.../src/kinematics.cpp`<br>• `src/omni_control/omni_control/kinematics.py`<br>• `src/omni_simulation/omni_simulation/stm32_simulator.py` | • Updates `raw_ws` and `filt_ws` in `/debug/data` JSON telemetry.<br>• Consistency check on `compute_wheel_speeds` and `compute_body_twist`. |
| **R2** | **IMU Intrinsic Calibration & ENU Filtering** | • Implement on-MCU static Gyroscope bias tracking routine:<br>$$\vec{b}_g = \frac{1}{N}\sum_{k=1}^N \vec{\omega}_{\text{raw},k}$$<br>ensuring stationary drift $< 0.05^\circ/\text{s}$.<br>• Accelerometer scale factor and cross-axis misalignment model (Tedaldi et al. / ST AN4508):<br>$$\vec{a}_{\text{calib}} = (\mathbf{T}_a \mathbf{S}_a)^{-1}(\vec{a}_{\text{raw}} - \vec{b}_a)$$<br>• Strict ROS ENU (East-North-Up) alignment: $+x$ forward, $+y$ left, $+z$ up; stationary gravity points $+z$ ($9.81\text{ m/s}^2$).<br>• Flash/NVRAM persistence routines on STM32. | • `firmware/.../include/hardware.h` & `src/hardware.cpp`<br>• `firmware/.../include/firmware_config.h`<br>• `firmware/.../src/main.cpp` (`imu_task`)<br>• New module: `firmware/.../include/imu_calibration.h` & `src/imu_calibration.cpp`<br>• `src/omni_simulation/omni_simulation/stm32_simulator.py` | • Output message: `sensor_msgs/msg/Imu` on `/imu/data` with frame `imu_link`.<br>• Calibrated bias parameters sent via `/debug/data` or dedicated service response. |
| **R3** | **Spatial/Temporal Extrinsics & EKF Covariance Tuning** | • Spatial Lever-Arm Extrinsics: Model tangential acceleration due to angular acceleration and centrifugal effects:<br>$$\vec{a}_{\text{body}} = \vec{a}_{\text{imu}} - \vec{\omega} \times (\vec{\omega} \times \vec{l}) - \dot{\vec{\omega}} \times \vec{l}$$<br>• Temporal Latency: Compensate communication delay $\Delta t$ between wheel ticks and IMU.<br>• EKF Covariance Tuning: Populate non-zero empirical measurement variances in `ekf.yaml`:<br>- Wheel odometry covariance ($v_x, v_y, \omega_z$)<br>- IMU orientation ($q_x, q_y, q_z, q_w$) and gyro covariance ($\omega_z$)<br>- Ensure no diagonal elements are zero.<br>• Enforce Single TF Authority: Only EKF broadcasts `odom` $\to$ `base_link` (disable duplicate TF in simulator).<br>• Laser filter tuning: Calibrate `footprint_filter` in `laser_filter.yaml` to exclude chassis without blinding external range. | • `src/omni_localization/config/ekf.yaml`<br>• `src/omni_localization/launch/ekf.launch.py`<br>• `src/omni_description/urdf/sensors.xacro`<br>• `src/omni_perception/config/laser_filter.yaml`<br>• `src/omni_perception/launch/perception.launch.py`<br>• `src/omni_simulation/config/simulation.yaml`<br>• `src/omni_simulation/omni_simulation/stm32_simulator.py` | • Transform hierarchy: `map` $\to$ `odom` (SLAM), `odom` $\to$ `base_link` (EKF only), `base_link` $\to$ sensors (robot_state_publisher).<br>• Topics: `/wheel/odom`, `/imu/data`, `/scan_filtered`. |
| **R4** | **Calibration Pipeline & Web UI Visualization** | • Bidirectional calibration workflow:<br>1. Web UI operator triggers calibration (IMU bias, wheel radius, noise measurement).<br>2. FastAPI backend dispatches command via Serial Contract or ROS command topic.<br>3. STM32 computes calibration parameters in real-time.<br>4. Telemetry stream streams progress (0–100%) and error metrics.<br>5. Backend receives final calibration matrix/bias and automatically updates ROS 2 YAML configuration files on disk (`ekf.yaml`, `simulation.yaml`).<br>6. Web UI displays visual validation charts. | • `web/backend/app/routers/api.py`<br>• `web/backend/app/models.py`<br>• `web/backend/app/bridges/base.py` & `ros2_bridge.py`<br>• `web/frontend/src/components/ConfigPanel.tsx` (or new `CalibrationPanel.tsx`)<br>• `src/omni_hardware/omni_hardware/stm32_contract.py` & `stm32_bridge.py`<br>• `firmware/.../src/main.cpp` | • REST APIs: `POST /api/calib/imu/start`, `POST /api/calib/wheels/start`, `GET /api/calib/status`.<br>• WebSocket JSON payload: `calib_status` (step, progress, residuals). |
| **R5** | **Architecture Optimization & Standard Libraries** | • Maximum reuse of standard ROS 2 packages (`robot_localization`, `laser_filters`, `sensor_msgs`, `nav_msgs`).<br>• Adherence to Python-first and ROS-agnostic shared core design per `RULE.md`.<br>• Zero external ad-hoc scripts; clean, maintainable modular structure.<br>• Complete automated regression test coverage across all subsystems. | • Entire codebase across `src/`, `firmware/`, `web/`, and `scripts/`. | • Standard ROS 2 interfaces.<br>• Automated test exit codes ($0 = \text{pass}$). |

---

## 7. Build, Test, and Execution Commands Reference

All commands must be executed from the repository root `/home/sonev/amr_omni`:

### 7.1. ROS 2 Workspace Build & Test
```bash
# Environment setup (WSL2 Ubuntu 24.04 amd64)
source /opt/ros/jazzy/setup.bash

# Build all 9 ROS 2 packages with symlinks
./scripts/build.sh --component ros2

# Build and run automated test suite across all ROS 2 packages
./scripts/build.sh --component ros2 --test

# Run tests on specific package (e.g. omni_control)
./scripts/build.sh --component ros2 --package omni_control --test-only

# Source the workspace overlay
source install/ros2_jazzy/local_setup.bash
```

### 7.2. STM32 Firmware Build & Test
```bash
# Check PlatformIO host dependencies
./scripts/setup.sh --check --firmware stm32_f407vg_arduino_sim

# Build firmware binary for STM32F407VG Discovery
./scripts/build.sh --component firmware --firmware stm32_f407vg_arduino_sim --environment disco_f407vg

# Direct PlatformIO CLI build
pio run --project-dir firmware/stm32_f407vg_arduino_sim --environment disco_f407vg

# Direct PlatformIO CLI test
pio test --project-dir firmware/stm32_f407vg_arduino_sim --environment disco_f407vg
```

### 7.3. Jetson Web UI Build & Test
```bash
# Build Next.js frontend static export bundle and sync to web/backend/static
./scripts/build.sh --component web

# Run backend API test suite via venv pytest (17 tests)
web/backend/.venv/bin/pytest web/backend/tests

# Launch Web Dashboard (FastAPI serving static frontend on port 8000)
./scripts/run_web.sh --port 8000

# Launch in Next.js development hot-reload mode
./scripts/run_web.sh --dev
```

### 7.4. Full System Verification
```bash
# Build and run test suites across all 3 subsystems (ROS 2 + Firmware + Web)
./scripts/build.sh --component all --test --skip-empty
```

---

## 8. Survey Conclusions & Transition to Phase 2

This architectural survey confirms that `amr_omni` possesses a robust, professional multi-tier architecture. The groundwork for the Mecanum AGV calibration and estimation upgrade is fully mapped:
1. **Firmware layer**: The FreeRTOS task structure, hardware abstraction, and micro-ROS message loop are already functional and provide clean hooks for inserting the ODrive Second-Order PLL tracking observer (R1) and the ST AN4508 / Tedaldi IMU intrinsic calibration routine (R2).
2. **Estimation & Fusion layer**: `robot_localization` EKF configuration is in place, requiring covariance matrix populating, rotational velocity fusing, and duplicate TF elimination (R3).
3. **Control & UI layer**: FastAPI and Next.js are structured with real-time WebSocket communication, ready to receive calibration trigger endpoints and automated YAML configuration saving (R4).

All findings, exact formulas, and file paths are recorded in this document and summarized in `handoff.md`.
