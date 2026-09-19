# End-to-End Test Infrastructure: Mecanum AGV Calibration & State Estimation

## 1. Test Philosophy & Architecture

### 1.1 Opaque-Box & Requirement-Driven Testing
The AMR Omni calibration and state estimation upgrade test suite operates under a strict **opaque-box, requirement-driven testing methodology**:
- **Interface & Behavior Verification**: Tests interact exclusively through documented external interfaces (binary serial protocol frames, REST API endpoints, WebSocket event streams, ROS 2 configuration YAML files, standard ROS message schemas, and physical kinematics transformations).
- **Zero Coupling to Internal Transients**: Tests do not assert on internal private variables or undocumented implementation transients. If an algorithm is upgraded or refactored, the test suite verifies that external contracts, mathematical invariants, convergence rates, and boundary tolerances remain intact.
- **Traceability to Authoritative Specifications**: Every test case traces directly to an explicit requirement in `ORIGINAL_REQUEST.md` (R1–R5), `PROJECT.md` (Features F1.1–F5.2), or the technical reference documents in `amr_omni/temp/docs/` (`Encoder.docx`, `Calib.docx`, `Calib_2.docx`, `amr_omni.docx`).
- **Deterministic Oracles**: For algorithmic verification (ODrive 2nd-order PLL, LinuxCNC M/T estimation, ST AN4508 6-position accelerometer calibration, Allan variance extraction, lever-arm kinematics), tests utilize closed-form mathematical oracles derived from first-principles physics and industrial standards (IEEE Std 952-1997, ST AN4508, REP-103).

### 1.2 4-Tier Test Architecture

```
+-------------------------------------------------------------------------------+
|                         4-TIER E2E TEST ARCHITECTURE                          |
+-------------------------------------------------------------------------------+
| Tier 1: Feature Coverage (>= 5 tests per feature F1.1 - F5.2)                |
|   - F1: Encoder Velocity Estimation & Kinematics Consistency                  |
|   - F2: IMU Intrinsic Calibration & Filtering on STM32                        |
|   - F3: Extrinsics, Covariances, TF Authority & Laser Filter                  |
|   - F4: Serial Calibration Protocol & Jetson Web UI                           |
|   - F5: Architecture Optimization & Test Suite Governance                     |
+-------------------------------------------------------------------------------+
| Tier 2: Boundary & Corner Cases (>= 5 tests per boundary domain)              |
|   - Creeping speeds, extreme linear/angular saturation                        |
|   - Zero-speed watchdog timeout & pulse cessation                             |
|   - 16-bit timer overflow/rollover & counter jumps                            |
|   - Extreme noise, saturated biases & low SNR conditions                      |
|   - Laser footprint grazing angles & exact bounding box edges                 |
|   - Protocol packet corruption, bit flips, truncation & packet bursts         |
|   - Kinematics singular geometry & extreme wheel radius variations            |
+-------------------------------------------------------------------------------+
| Tier 3: Cross-Feature Interactions (Pairwise Interaction Verification)        |
|   - F1 (Encoder PLL/M/T) + F3 (EKF Odometry Fusion)                           |
|   - F2 (IMU Bias Nulling) + F1 (Wheel Radius Compensation Kr)                 |
|   - F4 (Web API Command) + F4 (Serial Protocol Timeout & Abort)               |
|   - F3 (Single TF Authority) + F3 (Laser Filter Frame Masking)                |
|   - F3 (Spatial Lever-Arm Extrinsics) + F3 (EKF High-Speed Rotation)          |
|   - F3 (Temporal Latency Compensation) + F3 (Dynamic Acceleration)            |
|   - F4 (Serial Calibration Frame) + F4 (Disk YAML Persistence)                |
|   - F4 (WebSocket Progress Stream) + F4 (Interactive Abort)                   |
+-------------------------------------------------------------------------------+
| Tier 4: Real-World Workload Scenarios (Full System Life-Cycle Scenarios)      |
|   - Complex multi-segment trajectory (slalom & Figure-8 motion)               |
|   - Emergency high-speed braking (1.5 m/s to 0 in 200 ms without divergence)  |
|   - High-frequency floor vibration & dynamic shock (covariance inflation)     |
|   - Full calibration lifecycle (trigger -> stream -> flash -> YAML persistence)|
|   - Long-duration stationarity drift test (60 s hold, drift < 0.05 deg/s)     |
|   - Cluttered corridor navigation with chassis laser footprint filter         |
+-------------------------------------------------------------------------------+
```

---

## 2. Feature Inventory Coverage Table (F1.1 - F5.2)

Every feature defined in `PROJECT.md` is mapped to its test file, test tier, and minimum test threshold:

| # | Feature ID | Feature Name | Milestone | Requirement Source | Test Suite File | Tier Breakdown | Min Tests |
|---|------------|--------------|-----------|--------------------|-----------------|----------------|-----------|
| 1 | F1.1 | ODrive 2nd-Order PLL Tracking Observer | M1 | R1, `Encoder.docx` | `tests/e2e/tier1_feature_coverage/test_f1_encoder_kinematics.py` | Tier 1, 2, 3, 4 | 5 |
| 2 | F1.2 | LinuxCNC M/T Hybrid Velocity Estimation | M1 | R1, `Encoder.docx` | `tests/e2e/tier1_feature_coverage/test_f1_encoder_kinematics.py` | Tier 1, 2, 3 | 5 |
| 3 | F1.3 | Kinematics Consistency & $\mathbf{K}_r$ Compensation | M1 | R1, `amr_omni.docx` | `tests/e2e/tier1_feature_coverage/test_f1_encoder_kinematics.py` | Tier 1, 2, 3, 4 | 5 |
| 4 | F2.1 | STM32 ST AN4508 Accel Calibration | M2 | R2, `Calib.docx` | `tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py` | Tier 1, 2, 3, 4 | 5 |
| 5 | F2.2 | STM32 Stationary Gyro Bias Nulling | M2 | R2, `Calib.docx` | `tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py` | Tier 1, 2, 3, 4 | 5 |
| 6 | F2.3 | REP-103 ENU Coordinate Standard | M2 | R2, `Calib.docx` | `tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py` | Tier 1, 2 | 5 |
| 7 | F2.4 | Allan Variance & Covariance Inflation | M2 | R2, `Calib.docx` | `tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py` | Tier 1, 2, 4 | 5 |
| 8 | F3.1 | Spatial Lever-Arm Extrinsics Compensation | M3 | R3, `Calib_2.docx` | `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py` | Tier 1, 2, 3 | 5 |
| 9 | F3.2 | Temporal Latency Compensation | M3 | R3, `Calib_2.docx` | `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py` | Tier 1, 2, 3 | 5 |
| 10 | F3.3 | EKF Covariance Matrix Tuning | M3 | R3, `amr_omni.docx` | `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py` | Tier 1, 2, 3, 4 | 5 |
| 11 | F3.4 | Single TF Authority Enforcement | M3 | R3, `amr_omni.docx` | `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py` | Tier 1, 3 | 5 |
| 12 | F3.5 | Laser Filter Footprint Masking | M3 | R3, `amr_omni.docx` | `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py` | Tier 1, 2, 3, 4 | 5 |
| 13 | F4.1 | Binary Serial Protocol Contract | M4 | R4, `amr_omni.docx` | `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` | Tier 1, 2, 3, 4 | 5 |
| 14 | F4.2 | FastAPI Calibration Endpoints | M4 | R4, Web specs | `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` | Tier 1, 3, 4 | 5 |
| 15 | F4.3 | Real-Time Progress Streaming | M4 | R4, Web specs | `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` | Tier 1, 3, 4 | 5 |
| 16 | F4.4 | Automated YAML Configuration Persistence | M4 | R4, Web specs | `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` | Tier 1, 3, 4 | 5 |
| 17 | F4.5 | Web UI Calibration Dashboard | M4 | R4, Web specs | `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` | Tier 1, 4 | 5 |
| 18 | F5.1 | Architecture Optimization & Library Reuse | M1-M4 | R5 | `tests/e2e/tier1_feature_coverage/test_f5_architecture_suite.py` | Tier 1 | 5 |
| 19 | F5.2 | E2E Opaque-Box Test Suite Infrastructure | E2E, M5 | Acceptance Criteria | `tests/e2e/tier1_feature_coverage/test_f5_architecture_suite.py` | Tier 1, 2, 3, 4 | 5 |

---

## 3. Directory Structure

```
tests/e2e/
├── conftest.py                             # Pytest fixtures, environment setup, and markers
├── pytest.ini                              # Pytest configuration and log settings
├── run_tests.sh                            # Master executable test runner
├── harness/                                # Opaque-box reference models & contract validators
│   ├── __init__.py
│   ├── encoder_oracle.py                   # ODrive 2nd-order PLL and LinuxCNC M/T estimators
│   ├── kinematics_oracle.py                # Mecanum kinematics with Kr wheel radius compensation
│   ├── imu_calib_oracle.py                 # ST AN4508 6-position accel calib, gyro nulling & Allan variance
│   ├── extrinsics_oracle.py                # Rigid-body lever arm & temporal latency compensators
│   ├── serial_protocol_oracle.py           # Binary serial packet framing, CRC16 & parser
│   ├── laser_filter_oracle.py              # Laser footprint 2D box mask geometric verifier
│   ├── ekf_sim_oracle.py                   # 2D state estimation & covariance propagation oracle
│   └── config_verifier.py                  # YAML configuration and Single TF authority validators
├── tier1_feature_coverage/                 # Feature Coverage (>=5 tests per feature)
│   ├── __init__.py
│   ├── test_f1_encoder_kinematics.py       # F1.1, F1.2, F1.3 (15 tests)
│   ├── test_f2_imu_calibration.py          # F2.1, F2.2, F2.3, F2.4 (20 tests)
│   ├── test_f3_extrinsics_fusion.py        # F3.1, F3.2, F3.3, F3.4, F3.5 (25 tests)
│   ├── test_f4_serial_web_calib.py         # F4.1, F4.2, F4.3, F4.4, F4.5 (25 tests)
│   └── test_f5_architecture_suite.py       # F5.1, F5.2 (10 tests)
├── tier2_boundary_corner/                  # Boundary & Corner Cases (>=5 tests per boundary)
│   ├── __init__.py
│   ├── test_boundary_speeds_dynamics.py    # Extreme speeds & physical saturation
│   ├── test_boundary_watchdog_cessation.py # Zero-speed watchdog & pulse cessation
│   ├── test_boundary_timer_rollover.py     # 16-bit timer overflow/underflow
│   ├── test_boundary_extreme_noise_bias.py # Extreme sensor noise, saturation, low SNR
│   ├── test_boundary_laser_grazing.py      # Footprint box edge cases & grazing rays
│   ├── test_boundary_protocol_corruption.py# CRC bit flips, truncation, buffer overrun
│   └── test_boundary_math_singularities.py # Geometric singularities & zero radius
├── tier3_cross_feature/                    # Cross-Feature Interactions (Pairwise combinations)
│   ├── __init__.py
│   └── test_cross_feature_interactions.py  # 8 pairwise interaction scenarios
└── tier4_real_world/                       # Real-World Workload Scenarios
    ├── __init__.py
    └── test_real_world_scenarios.py        # 6 realistic operational driving & calib scenarios
```

---

## 4. Test Execution & Runner Usage

The master test runner is located at `tests/e2e/run_tests.sh`:

```bash
# Execute entire test suite (all 4 tiers)
./tests/e2e/run_tests.sh --all

# Execute specific tier
./tests/e2e/run_tests.sh --tier 1
./tests/e2e/run_tests.sh --tier 2
./tests/e2e/run_tests.sh --tier 3
./tests/e2e/run_tests.sh --tier 4

# Run with verbose output and summary reporting
./tests/e2e/run_tests.sh -v --report

# Direct pytest invocation
python3 -m pytest tests/e2e/ -v
```

### Exit Code Semantics
- **0**: All executed tests in the requested scope passed successfully.
- **1**: One or more tests failed or encountered errors.
- **2**: Interrupted by user or configuration syntax error.
