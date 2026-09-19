# Project Execution Plan: amr_omni Calibration & Estimation Upgrade

## Objective
Upgrade and harden the Mecanum AGV `amr_omni` calibration and state estimation architecture to meet all requirements R1-R5 and acceptance criteria in `ORIGINAL_REQUEST.md`.

## Phased Approach

### Phase 0: Comprehensive Survey & Specification Mining
- Dispatch 3 parallel explorers/spec miners:
  - **Explorer 1 (Docs & Theory Miner)**: Analyze `/home/sonev/amr_omni/temp/docs/` (`amr_omni.docx`, `Encoder.docx`, `Calib.docx`, `Calib_2.docx`), extracting mathematical formulas, algorithms (ODrive 2nd order PLL, Tedaldi/ST AN4508 IMU calib, Allan variance, Kalibr B-spline spatial/temporal, covariance values).
  - **Explorer 2 (Codebase Architecture Explorer)**: Map `/home/sonev/amr_omni/` (ROS 2 workspace packages, STM32 firmware structure, kinematics node, hardware interfaces, web UI frontend/backend). Check GitNexus context and rules in `AGENTS.md`.
  - **Explorer 3 (Protocol & Integration Miner)**: Investigate current serial communication contract between STM32 and Jetson/ROS 2, current robot_localization EKF config, laser_filter config, and Web UI calibration trigger endpoints.
- Synthesize findings into `PROJECT.md` at project root, defining:
  - Full Feature Inventory mapped to requirements R1-R5.
  - Architecture and package boundaries.
  - Milestone decomposition (Target 3-5 implementation milestones + E2E testing).
  - Interface contracts between STM32, ROS 2 nodes, and Web UI.

### Phase 1: Dual Track Execution
- **Track 1: E2E Testing Track (Requirement-Driven, Opaque-Box)**
  - Spawn E2E Testing Orchestrator.
  - Create `TEST_INFRA.md`.
  - Build automated test runner covering Tiers 1-4 (Feature Coverage >=5/feat, Boundary & Corner >=5/feat, Cross-Feature Combinations, Real-World Workloads).
  - Publish `TEST_READY.md`.
- **Track 2: Implementation Track (Milestone-based Sub-Orchestration)**
  - Milestone 1: High-precision Encoder velocity estimation (ODrive 2nd order PLL / M/T hybrid) on STM32 + Kinematics consistency verification.
  - Milestone 2: IMU intrinsic calibration & noise filtering on STM32 (Gyro bias tracking, Accel static 6-pos/detection, ENU frame).
  - Milestone 3: Intersensor (Spatial & Temporal) calibration & EKF robustness (lever-arm extrinsics, temporal latency compensation, real covariance matrix, Single TF authority, laser filter chassis blind spot).
  - Milestone 4: Calibration control protocol & Jetson Web UI integration (FastAPI endpoints, Web UI trigger & real-time feedback, YAML auto-update, binary serial protocol).

### Phase 2: Final Milestone & Adversarial Hardening
- **Phase 2A**: Run 100% of E2E test suite (Tiers 1-4) against integrated implementation; iterate until 100% pass.
- **Phase 2B**: Tier 5 Adversarial Coverage Hardening via Challenger loop (white-box gap analysis, edge case injection, vibration/jump stress testing).

### Phase 3: Final Verification & Handover
- Run Forensic Audit.
- Prepare completion documentation, user guide, and architectural report.
- Transmit completion notification and report to Sentinel.
