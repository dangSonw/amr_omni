# BRIEFING — 2026-09-19T10:45:00Z

## Mission
Implement Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32 (ImuCalibrator, platformio.ini, Unity test suite, and main.cpp telemetry covariance integration).

## 🔒 My Identity
- Archetype: teamwork_preview_worker_m2_1
- Roles: implementer, qa, specialist
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_1
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32

## 🔒 Key Constraints
- Genuine implementation only: No hardcoding test results or creating dummy/facade implementations.
- Exclusive file ownership:
  - `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
  - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
  - `firmware/stm32_f407vg_arduino_sim/src/main.cpp`
  - `firmware/stm32_f407vg_arduino_sim/platformio.ini`
  - `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration/test_main.cpp`
- ST AN4508 6-position accelerometer linear calibration: norm error < 0.05 m/s² on all 6 faces.
- Gyro bias nulling: >= 1000 samples (>= 10s), residual drift < 0.05 deg/s (8.7266e-4 rad/s).
- Welford motion disturbance rejection: reject if variance > 1e-4 rad²/s².
- REP-103 ENU coordinate alignment: X-forward, Y-left, Z-up, static gravity +9.80665 m/s² on Z.
- Allan variance noise model parameters (N_g, N_a, K_g) and field covariance inflation (alpha = 1.8 => alpha^2 = 3.24).
- Verification passes: `pio test -e native`, `pio run -e disco_f407vg`, `pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v`.

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:45:00Z

## Task Summary
- **What to build**: `ImuCalibrator` C++ class for STM32 firmware, unit tests in Unity for native platform, integration in `main.cpp`, update `platformio.ini`.
- **Success criteria**: All native unit tests pass, disco_f407vg builds cleanly, e2e python test passes.
- **Interface contracts**: Specified in ORIGINAL_REQUEST.md, PROJECT.md, and Explorer M2 Plan.

## Key Decisions Made
- [TBD]

## Artifact Index
- [TBD]

## Change Tracker
- **Files modified**: none yet
- **Build status**: untried
- **Pending issues**: none

## Quality Status
- **Build/test result**: pending
- **Lint status**: pending
- **Tests added/modified**: pending

## Loaded Skills
- None specified
