# Project: amr_omni Mecanum AGV Calibration & State Estimation Upgrade

## Architecture
The system integrates an STM32F407 microcontroller (firmware), ROS 2 Jazzy robotics stack, and a Jetson Web UI (FastAPI backend + Next.js frontend):
1. **STM32 Firmware (`firmware/stm32_f407vg_arduino_sim`)**:
   - Executes real-time FreeRTOS control loops.
   - Hosts the Second-Order PLL tracking observer & LinuxCNC M/T hybrid velocity estimation for 4 Mecanum wheels.
   - Executes on-board IMU intrinsic calibration routines (ST AN4508 6-position accelerometer calibration and stationary zero-rate gyroscope bias nulling).
   - Implements binary framed serial packet contract for bidirectional command/telemetry streaming with Jetson.
2. **ROS 2 Workspace Packages (`src/`)**:
   - `omni_control`: Forward and inverse Mecanum kinematics with individual wheel radius error compensation ($\mathbf{K}_r$) and verified round-trip consistency ($FK(IK(\mathbf{v})) \equiv \mathbf{v}$).
   - `omni_localization`: `robot_localization` EKF with fully populated non-zero covariance matrices (twist, IMU, process noise $\mathbf{Q}$, initial error $\mathbf{P}_0$). Strictly enforces Single TF Authority (`ekf_node` alone broadcasts `odom -> base_link`).
   - `omni_perception`: Calibrated `laser_filters` footprint mask ($[-0.135, 0.135] \times [-0.135, 0.135]$ m) eliminating chassis body blind-spot reflection without clipping external obstacles.
   - `omni_description`: Rigorous sensor lever-arm extrinsics ($\mathbf{p}_B^S$) and frame convention alignment (`imu_link`, `laser_link`, `base_link`).
3. **Jetson Web UI (`web/`)**:
   - FastAPI backend (`web/backend`): REST endpoints (`/api/calib/*`) to dispatch calibration routines, stream real-time progress via WebSockets, and persist computed calibration matrices into ROS 2 YAML configuration files.
   - Frontend (`web/frontend`): Calibration dashboard with 1-click trigger, real-time progress bar, stage indicator, and parameter display.

---

## Feature Inventory
Every feature from requirements R1-R5 mapped to its assigned milestone:

| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | F1.1 ODrive 2nd-Order PLL Observer | Discrete-time 2nd-order PLL tracking observer for wheel velocity ($k_p = 2\omega_{pll}, k_i = \omega_{pll}^2$), eliminating low-speed quantization chatter and acceleration phase lag | M1 | R1, `Encoder.docx` |
| 2 | F1.2 LinuxCNC M/T Hybrid Estimation | Dual edge-synchronized estimation at low/high speeds with 16-bit timer rollover safe difference and zero-speed watchdog | M1 | R1, `Encoder.docx` |
| 3 | F1.3 Kinematics Consistency & $\mathbf{K}_r$ | Individual wheel radius error compensation matrix $\mathbf{K}_r$, closed-form Moore-Penrose pseudo-inverse, and round-trip consistency test ($\|FK(IK(\mathbf{v})) - \mathbf{v}\| < 10^{-5}$) | M1 | R1, `amr_omni.docx` |
| 4 | F2.1 STM32 ST AN4508 Accel Calibration | On-board 6-position accelerometer calibration routine solving scale factors $s_j$ and biases $b_j$ via closed-form linear equations | M2 | R2, `Calib.docx` |
| 5 | F2.2 STM32 Gyro Bias Nulling | Stationary gyroscope zero-rate bias tracking and nulling ensuring residual drift $< 0.05^\circ/\text{s}$ | M2 | R2, `Calib.docx` |
| 6 | F2.3 REP-103 ENU Coordinate Standard | Standardized ENU frame orientation and static gravity vector handling (+9.80665 m/s² on Z at rest) | M2 | R2, `Calib.docx` |
| 7 | F2.4 Allan Variance & Covariance Inflation | Allan variance noise parameters ($N_g, N_a, K_g$) and field covariance inflation ($1.5\times - 2.0\times$) for dynamic vibration robustness | M2 | R2, `Calib.docx` |
| 8 | F3.1 Spatial Lever-Arm Extrinsics | Rigid-body kinematic lever-arm compensation ($\mathbf{a}_S = \mathbf{R}_B^S (\mathbf{a}_B + \dot{\boldsymbol{\omega}} \times \mathbf{p} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{p}))$) to cancel false centrifugal acceleration | M3 | R3, `Calib_2.docx` |
| 9 | F3.2 Temporal Latency Compensation | Continuous B-spline latency estimation principles ($\Delta s = v \cdot \Delta t$) for sensor time alignment | M3 | R3, `Calib_2.docx` |
| 10 | F3.3 EKF Covariance Matrix Tuning | Populate non-zero diagonal variances in `ekf.yaml` for twist, IMU, process noise $\mathbf{Q}$, and initial $\mathbf{P}_0$ | M3 | R3, `amr_omni.docx` |
| 11 | F3.4 Single TF Authority Enforcement | Ensure only `ekf_node` broadcasts `odom -> base_link`; disable dynamic TF in simulator/hardware driver | M3 | R3, `amr_omni.docx` |
| 12 | F3.5 Laser Filter Footprint Masking | Precise box filter ($[-0.135, 0.135] \times [-0.135, 0.135]$ m) removing chassis reflections without truncating obstacle scans | M3 | R3, `amr_omni.docx` |
| 13 | F4.1 Binary Serial Protocol Contract | Structured binary packet format (Header, ID, Payload, CRC16/XOR checksum, Tail) with command, progress, and result frames | M4 | R4, `amr_omni.docx` |
| 14 | F4.2 FastAPI Calibration Endpoints | REST endpoints (`/api/calib/start`, `/api/calib/abort`, `/api/calib/status`, `/api/calib/results`, `/api/calib/apply`) | M4 | R4, Web specs |
| 15 | F4.3 Real-Time Progress Streaming | WebSocket telemetry streaming calibration stage, progress percentage, and live metrics | M4 | R4, Web specs |
| 16 | F4.4 Automated YAML Configuration Persistence | Automatic disk persistence of calibration outputs to `imu_calib.yaml`, `wheel_calib.yaml`, and `ekf.yaml` | M4 | R4, Web specs |
| 17 | F4.5 Web UI Calibration Dashboard | Web interface with 1-click trigger, stage indicator, progress bar, parameter table, and residual error visualization | M4 | R4, Web specs |
| 18 | F5.1 Architecture Optimization & Library Reuse | Maximize reuse of ROS 2 `robot_localization`, `laser_filters`, standard math libraries, ensuring clean maintainable code | M1-M4 | R5 |
| 19 | F5.2 E2E Opaque-Box Test Suite | Comprehensive 4-tier test suite covering feature coverage, boundary conditions, cross-feature pairs, and real-world workloads | E2E, M5 | Acceptance Criteria |

---

## Milestones

| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| E2E | E2E Testing Track | Design test infra (`TEST_INFRA.md`) and implement automated 4-tier opaque-box test suite (Tiers 1-4). Published `TEST_READY.md` (151 test cases). | none | DONE |
| M1 | Encoder Velocity & Kinematics Consistency | Implemented ODrive 2nd-order PLL observer, LinuxCNC M/T hybrid velocity estimation, wheel radius error compensation $\mathbf{K}_r$, and kinematics consistency tests in firmware and `omni_control`. Verified by 2 Reviewers, 2 Challengers (50k Monte Carlo, 6 stress suites), and Forensic Auditor (CLEAN). | none | DONE |
| M2 | IMU Intrinsic Calibration & Filtering on STM32 | Implemented ST AN4508 6-position accelerometer calibration, stationary gyro bias nulling ($< 0.05^\circ/\text{s}$), Welford stationarity/variance gating, non-tautological $2\sigma$ SEM confidence bound, REP-103 ENU alignment, and Allan variance noise model. Verified across 34 PlatformIO native tests, 4/4 C++ stress suites, 20/20 Python stress tests, 2 Reviewers, 2 Challengers, and Forensic Auditor (CLEAN). | none | DONE |
| M3 | Extrinsics, Covariances, TF Authority & Laser Filter | Implement spatial lever-arm compensation, populate complete non-zero EKF covariances, enforce Single TF Authority, and configure footprint laser filter. | M1, M2 | PLANNED |
| M4 | Serial Calibration Protocol & Jetson Web UI | Implement binary serial calibration frames (command/progress/results), FastAPI `/api/calib/*` endpoints, WebSocket progress streaming, YAML disk persistence, and UI dashboard. | M1, M2 | PLANNED |
| M5 | Final Verification & Coverage Hardening | Run 100% of E2E test suite (Tiers 1-4) against integrated system, followed by Tier 5 white-box adversarial coverage hardening and forensic integrity audit. | E2E, M1-M4 | PLANNED |

---

## Interface Contracts

### 1. STM32 <-> Jetson Serial Binary Contract
- **Frame Structure**:
  `[0xAA 0x55] [Length: 1B] [Seq: 1B] [MsgID: 1B] [Payload: 0-64B] [CRC16: 2B] [0x7D]`
- **Command Frames**:
  - `0x10` (`CMD_CALIB_TRIGGER_IMU`): `[Subtype: 1B (0=Static Bias, 1=6-Pos Accel, 2=Tedaldi)]`
  - `0x11` (`CMD_CALIB_TRIGGER_WHEEL`): `[Subtype: 1B (0=Linear, 1=Strafe, 2=Rotation)]`
  - `0x12` (`CMD_CALIB_START_NOISE_PROFILE`): `[Duration_s: 2B]`
  - `0x13` (`CMD_CALIB_ABORT`): Cancel running calibration routine
  - `0x14` (`CMD_CALIB_FLASH_COMMIT`): Save parameters to STM32 internal Flash
- **Telemetry & Result Frames**:
  - `0x80` (`RESP_ACK_NACK`): `[TargetMsgID: 1B] [Status: 1B]`
  - `0x81` (`TELEM_CALIB_PROGRESS` @ 5Hz): `[CalibType: 1B] [Stage: 1B] [ProgressPercent: 1B] [StatusCode: 1B] [LiveMetric: float32]`
  - `0x82` (`RESP_CALIB_IMU_RESULT`): `[bias_g: 3x float32] [bias_a: 3x float32] [scale_a: 3x float32] [residual_norm: float32]`
  - `0x83` (`RESP_CALIB_WHEEL_RESULT`): `[radii: 4x float32] [Leff: float32] [Weff: float32] [residual_err: float32]`
  - `0x84` (`RESP_CALIB_NOISE_RESULT`): `[Ng: float32] [Kg: float32] [Na: float32] [Ka: float32]`

### 2. Jetson Web Backend <-> ROS 2 YAML Config Contract
- **IMU Configuration (`config/imu_calib.yaml`)**:
  ```yaml
  imu_calib:
    gyro_bias: [bx, by, bz]
    accel_scale: [sx, sy, sz]
    accel_bias: [ax, ay, az]
    frame_id: "imu_link"
  ```
- **Wheel Configuration (`config/wheel_calib.yaml`)**:
  ```yaml
  wheel_calib:
    wheel_radius: [r1, r2, r3, r4]
    wheelbase: 0.1312
    track_width: 0.1312
  ```
- **EKF Configuration (`src/omni_localization/config/ekf.yaml`)**:
  - `publish_tf: true`
  - Full non-zero diagonal entries for `odom0_config` ($v_x, v_y, \omega_z$), `imu0_config` ($\text{yaw}, \omega_z, a_x, a_y$).
  - Full non-zero $15 \times 15$ diagonal entries for `process_noise_covariance` and `initial_estimate_covariance`.

---

## Code Layout
- `firmware/stm32_f407vg_arduino_sim/src/`:
  - `encoder_pll.h` / `encoder_pll.cpp`: ODrive 2nd-order PLL and LinuxCNC M/T estimation.
  - `imu_calibration.h` / `imu_calibration.cpp`: ST AN4508 6-position accelerometer & gyro bias routine.
  - `serial_protocol.h` / `serial_protocol.cpp`: Binary serial frame parser & serializer.
  - `main.cpp`: FreeRTOS task integration and state machine.
- `src/omni_control/omni_control/`:
  - `kinematics.py`: Kinematics module with $\mathbf{K}_r$ radius compensation.
  - `test/test_kinematics.py`: Pytest suite for consistency verification.
- `src/omni_localization/config/`:
  - `ekf.yaml`: Hardened EKF filter configuration.
- `src/omni_perception/config/`:
  - `laser_filter.yaml`: Masking box filter.
- `web/backend/app/`:
  - `api/calib.py`: Calibration endpoints (`/api/calib/*`).
  - `services/calib_service.py`: Calibration lifecycle, serial dispatch, and YAML writer.
- `web/frontend/`:
  - Calibration dashboard interface.
- `tests/e2e/`:
  - Automated 4-tier requirement-driven opaque-box test suite and runner.
