# Milestone 4 Technical Analysis: Web UI Frontend & E2E Test Suite Validation

## Executive Summary
Milestone 4 (F4.1 - F4.5) implements the bidirectional Binary Serial Protocol Contract between the STM32F407 microcontroller and Jetson Web system, the FastAPI calibration endpoints, WebSocket real-time progress streaming, automated YAML configuration persistence, and the Next.js React-based Web UI Calibration Dashboard.

All 25/25 Tier 1 F4 tests, 53/53 filtered calibration tests, 7/7 production repo readiness tests, 157/157 full E2E tests, 207/207 repository pytest suite tests, and 34/34 PlatformIO C++ firmware native tests pass with 100% success rate. The Next.js frontend builds cleanly via `npm run build` and automatically exports static production assets to `web/backend/static/`.

---

## 1. Web UI Frontend Architecture (`web/frontend/`)

### 1.1 Stack & Framework Configuration
- **Core Framework**: Next.js 14.2.35, React 18.3.1, TypeScript 5.4.5.
- **Styling & UI**: TailwindCSS 3.4.3, Lucide-React icons, PostCSS, Autoprefixer.
- **Data Visualization**: Recharts 3.10.1 for telemetry charts (motor speeds, IMU orientations, angular velocities).
- **Export Strategy**: Configured with `output: "export"` in `next.config.mjs`, outputting static bundles to `out/`.
- **Postbuild Integration**: `"postbuild": "mkdir -p ../backend/static && rm -rf ../backend/static/_next && cp -r out/* ../backend/static/"` seamlessly serves frontend assets through FastAPI static mount.

### 1.2 Layout & Navigation (`src/app/page.tsx`, `Sidebar.tsx`, `Header.tsx`)
- Multi-tab responsive interface:
  1. **Cockpit (`cockpit`)**: 2D LiDAR navigation map (`MapNavigation.tsx`), real-time camera feed (`CameraFeed.tsx`), motion status telemetry card (`MotionStatusCard.tsx`), and continuous keyboard teleoperation (`w/a/s/d/q/e/space`).
  2. **Calibration Dashboard (`calib`)**: Mounts `CalibrationPanel.tsx` for IMU 6-orientation routine, extrinsics, and temporal latency tuning.
  3. **Robot Configuration (`config`)**: Mounts `ConfigPanel.tsx` for kinematics geometry, PID gains, speed limits, and noise profiles.
  4. **Diagnostics & Telemetry (`debug`)**: Mounts `MotorSpeedChart`, `ImuQuaternionDisplay`, `AngularVelocityChart`, `LinearVelocityChart`, `SensorCards`, `WheelDiagnostics`, and `StreamMatrix`.

### 1.3 Calibration Dashboard (`src/components/CalibrationPanel.tsx`)
The `CalibrationPanel` component provides end-to-end user interaction matching requirement R4:
- **6-Orientation ST AN4508 Stepper**: Visual grid of 6 calibration poses (`+Z`, `-Z`, `+X`, `-X`, `+Y`, `-Y`) displaying target pitch/roll angles, live orientation alignment deviation, and step completion indicators.
- **1-Click Measurement Trigger**:
  - `handleMeasureCurrentStep()`: Sends POST request to `/api/calib/face/sample` with `{ step, sample_count: 100 }`.
  - `handleCalibrateExtrinsics()`: Dispatches lever-arm and wheel radius calibration to `/api/calib/extrinsics/start` and `/api/calib/encoder/start`.
  - `handleSetSimPose()`: Directly tilts the 3D Gazebo model and STM32 simulation via `/api/calib/sim/set_pose`.
  - `handleToggle()`: Controls on-the-fly calibration compensation bypass (`/api/calib/toggle`).
  - `handleReset()`: Clears active calibration back to identity (`/api/calib/reset`).
- **Real-Time Progress Feedback**:
  - 100-sample live progress bar with percentage readout and visual sampling stage feedback.
  - Heading drift card comparing Wheel Odometry Yaw vs IMU Integrated Yaw vs Drift error, verifying stationary drift remains $< 0.05^\circ/\text{s}$.
  - Real-time gravitational acceleration monitor displaying live $a_z$ with automatic warning alert banner if $a_z \notin [6.0, 13.0]\ \text{m/s}^2$.
- **Calibrated Parameter Table**:
  - Scale Factors: $[s_x, s_y, s_z]$
  - Accelerometer Biases: $[b_x, b_y, b_z]\ \text{m/s}^2$
  - Gyroscope Biases: $[g_x, g_y, g_z]\ \text{rad/s}$
  - Standard Gravity Verification: $9.81\ \text{m/s}^2$ (PASS)
  - Inter-sensor Lever-Arm: $[p_x, p_y, p_z]\ \text{m}$
  - Temporal Latency: $\Delta t\ \text{ms}$ slider ($1.0 - 50.0\ \text{ms}$)
  - Mecanum Wheel Radii: $[r_1, r_2, r_3, r_4]\ \text{mm}$

### 1.4 Real-Time WebSocket Telemetry (`src/hooks/useRobotWs.ts`)
- Establishes connection to `/ws/telemetry`.
- Computes bidirectional latency via periodic 1000ms ping/pong heartbeats.
- Auto-reconnects with exponential backoff ($1.5\text{s} \to 30\text{s}$).
- Dispatches high-frequency velocity commands (`cmd_vel`) and E-Stop events directly over WebSocket.

---

## 2. Frontend Build and Lint Validation

### 2.1 Build Execution (`npm run build`)
- **Execution Command**: `npm run build` in `/home/sonev/amr_omni/web/frontend/`
- **Result**:
  ```
  ▲ Next.js 14.2.35
     Creating an optimized production build ...
   ✓ Compiled successfully
     Linting and checking validity of types
     Collecting page data
   ✓ Generating static pages (4/4)
     Finalizing page optimization

  Route (app)                              Size     First Load JS
  ┌ ○ /                                    131 kB          219 kB
  └ ○ /_not-found                          873 B          88.3 kB
  + First Load JS shared by all            87.5 kB

  ○  (Static)  prerendered as static content

  > amr-omni-frontend@1.0.0 postbuild
  > mkdir -p ../backend/static && rm -rf ../backend/static/_next && cp -r out/* ../backend/static/
  ```
- **Exit Code**: 0 (Clean build, zero errors).
- **Asset Distribution**: Static HTML, JavaScript chunks, and CSS successfully copied to `web/backend/static/`.

### 2.2 Linting Behavior
- Running `next lint` directly prompted interactive configuration for ESLint because `.eslintrc.json` is not stored in the repository.
- However, `next build` executes type checking and syntax validation (`Linting and checking validity of types`) as part of its core pipeline and passed with zero errors.

---

## 3. Automated Milestone 4 E2E Test Suite Validation

### 3.1 Tier 1 Feature Coverage: Binary Serial Protocol, FastAPI & YAML (`test_f4_serial_web_calib.py`)
- **Execution**: `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v`
- **Results**: 25 passed in 1.14s (100% pass rate).

| Test Class | Test Name | Focus Area | Status |
|------------|-----------|------------|--------|
| `TestF41_BinarySerialProtocolContract` | `test_f4_1_serial_header_tail_framing` | Sync bytes `0xAA 0x55` and tail `0x7D` | PASSED |
| | `test_f4_1_serial_crc16_integrity` | CRC16-CCITT calculation and parsing | PASSED |
| | `test_f4_1_serial_command_packet_serialization` | Command frames `0x10`, `0x11`, `0x12`, `0x13`, `0x14` | PASSED |
| | `test_f4_1_serial_telemetry_packet_deserialization` | Telemetry `0x81` and result frames `0x82`, `0x83`, `0x84` | PASSED |
| | `test_f4_1_serial_malformed_packet_rejection` | CRC mismatch, sync errors, truncated tail | PASSED |
| `TestF42_FastAPICalibrationEndpoints` | `test_f4_2_fastapi_app_loads_health` | FastAPI health endpoint (`/health`) | PASSED |
| | `test_f4_2_fastapi_calib_endpoints_schema_contract` | Schema definition for `/api/calib/*` | PASSED |
| | `test_f4_2_fastapi_calib_start_payload_validation` | Routine payload validation (`imu`, `wheel`, `noise`) | PASSED |
| | `test_f4_2_fastapi_calib_abort_contract` | Abort state transitions and cancellation | PASSED |
| | `test_f4_2_fastapi_calib_status_contract` | Status model schema, progress, and live metrics | PASSED |
| `TestF43_RealTimeProgressStreaming` | `test_f4_3_progress_packet_structure` | 8-byte telemetry progress payload | PASSED |
| | `test_f4_3_progress_percentage_monotonicity` | Monotonic progress progression $0 \to 100\%$ | PASSED |
| | `test_f4_3_progress_stage_transitions` | Sequential stage progression | PASSED |
| | `test_f4_3_progress_live_metric_precision` | Float32 live metric encoding fidelity | PASSED |
| | `test_f4_3_telemetry_streaming_frequency_5hz` | 5Hz telemetry interval verification (200ms) | PASSED |
| `TestF44_AutomatedYAMLConfigurationPersistence` | `test_f4_4_yaml_persistence_imu_schema` | `imu_calib.yaml` schema conformance | PASSED |
| | `test_f4_4_yaml_persistence_wheel_schema` | `wheel_calib.yaml` schema conformance | PASSED |
| | `test_f4_4_yaml_persistence_atomic_write` | Atomic temporary file replacement pattern | PASSED |
| | `test_f4_4_yaml_persistence_reload_consistency` | Exact round-trip parameter reload | PASSED |
| | `test_f4_4_yaml_persistence_directory_auto_create` | Target directory recursive auto-creation | PASSED |
| `TestF45_WebUICalibrationDashboard` | `test_f4_5_web_ui_1click_trigger_command_contract` | 1-click trigger routine dispatch | PASSED |
| | `test_f4_5_web_ui_progress_bar_range_0_to_100` | Progress bounds $[0, 100]$ | PASSED |
| | `test_f4_5_web_ui_parameter_table_schema` | IMU and wheel parameter table columns | PASSED |
| | `test_f4_5_web_ui_error_banner_on_abort` | UI error banner state representation | PASSED |
| | `test_f4_5_web_ui_residual_error_visualization_format`| Residual error chart data formatting | PASSED |

### 3.2 Filtered Calibration Suite (`-k "f4 or serial or calib or web"`)
- **Execution**: `python3 -m pytest -k "f4 or serial or calib or web" tests/e2e -v`
- **Results**: 53 passed, 104 deselected in 1.02s (100% pass rate).
- **Scope**: Combines Tier 1 F2 IMU intrinsic calibration tests, Tier 1 F4 serial/web calibration tests, Tier 3 cross-feature interactions (`test_interaction_3_web_command_dispatch_and_serial_timeout`, `test_interaction_7_calibration_frame_to_yaml_and_config_reload`, `test_interaction_8_websocket_progress_streaming_with_interactive_abort`), and Tier 4 real-world full calibration lifecycle (`test_scenario_4_full_calibration_lifecycle`).

### 3.3 Production Repository Readiness Audit (`test_production_repo_readiness.py`)
- **Execution**: `python3 -m pytest tests/e2e/test_production_repo_readiness.py -v`
- **Results**: 7 passed in 0.69s (100% pass rate).
- **M4 Specific Verifications**:
  - `test_repo_firmware_serial_protocol_files_exist`: Verified `firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp` and `include/serial_protocol.h`.
  - `test_repo_fastapi_calibration_routes_mounted`: Verified `/api/calib/start` mounted in FastAPI application.

### 3.4 Comprehensive Repository Test Coverage
1. **Full E2E Suite (`tests/e2e`)**: 157 passed in 1.97s.
2. **Complete Pytest Suite (`tests/`)**: 207 passed in 28.15s (includes 50 covariance/EKF stress tests).
3. **PlatformIO Native Firmware Suite (`pio test -e native`)**: 34 passed in 8.27s (includes kinematics, PID, IMU gyro stress, encoder PLL stress, IMU calibration, Kalman filter, encoder PLL).

---

## 4. Verification of User Acceptance Criteria for Requirement R4

| Requirement R4 Criterion | Implementation Evidence | Verification Tests | Compliance Status |
|--------------------------|-------------------------|--------------------|-------------------|
| **1-Click Calibration Trigger** | `CalibrationPanel.tsx` provides 1-click trigger buttons for ST AN4508 face sampling, extrinsics/temporal latency, compensation toggle, and reset. Dispatches to FastAPI `/api/calib/start`, `/api/calib/face/sample`, `/api/calib/extrinsics/start`. Serial frame `0x10`, `0x11`, `0x12`. | `test_f4_5_web_ui_1click_trigger_command_contract`, `test_f4_2_fastapi_calib_start_payload_validation`, `test_scenario_4_full_calibration_lifecycle` | **VERIFIED (100%)** |
| **Real-Time Progress Tracking** | `CalibrationPanel.tsx` renders progress bar with percentage readout ($0-100\%$), 6-face visual completion stepper, live RTT ping, live gyro/accel deviation monitors, and abnormal gravity warning. WebSocket telemetry streams progress (`0x81` frame @ 5Hz). | `test_f4_3_progress_packet_structure`, `test_f4_3_progress_percentage_monotonicity`, `test_f4_3_telemetry_streaming_frequency_5hz`, `test_interaction_8_websocket_progress_streaming_with_interactive_abort` | **VERIFIED (100%)** |
| **Automated YAML Configuration Persistence** | Calibration matrices computed by STM32 or Web backend persist into `config/imu_calib.yaml` and `config/wheel_calib.yaml` following strict schema via atomic temporary file renaming and automatic folder creation. | `test_f4_4_yaml_persistence_imu_schema`, `test_f4_4_yaml_persistence_wheel_schema`, `test_f4_4_yaml_persistence_atomic_write`, `test_f4_4_yaml_persistence_reload_consistency`, `test_interaction_7_calibration_frame_to_yaml_and_config_reload` | **VERIFIED (100%)** |
| **Visual Parameter Display** | `CalibrationPanel.tsx` displays live parameter table (Scale Factors $s$, Accel Biases $b_a$, Gyro Biases $b_g$, gravity norm $9.81\ \text{m/s}^2$, effective wheel radii $r_i$, lever-arm vector, temporal latency $\Delta t$). Drift monitor checks static heading drift $< 0.05^\circ/\text{s}$. | `test_f4_5_web_ui_parameter_table_schema`, `test_f4_5_web_ui_residual_error_visualization_format`, `test_f4_5_web_ui_error_banner_on_abort` | **VERIFIED (100%)** |

---

## 5. Architectural Alignment & Interface Parity
- **Parity between C++ Firmware and Python E2E Oracle**:
  - `firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp` and `tests/e2e/harness/serial_protocol_oracle.py` share identical frame sync bytes (`0xAA 0x55`), CRC16-CCITT algorithm (poly `0x1021`, init `0xFFFF`), framing overhead (7 bytes), payload cap (64 bytes), message IDs (`0x10` - `0x14`, `0x80` - `0x84`), and termination byte (`0x7D`).
- **Single Authority TF & Sensor Extrinsics**:
  - Web UI displays lever-arm inputs matching URDF joints and EKF broadcast authority (`ekf_node` alone publishes `odom -> base_link`).
