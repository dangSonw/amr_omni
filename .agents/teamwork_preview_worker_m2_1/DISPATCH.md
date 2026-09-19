## 2026-09-19T10:44:44Z

You are teamwork_preview_worker_m2_1, the Implementation Worker for Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Explorer M2 Plan: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1/m2_implementation_plan.md
Explorer M2 Handoff: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1/handoff.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md, /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md, and the Explorer M2 Plan completely before writing code.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE FILE OWNERSHIP:
You have exclusive write access to:
- `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
- `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
- `firmware/stm32_f407vg_arduino_sim/src/main.cpp`
- `firmware/stm32_f407vg_arduino_sim/platformio.ini`
- `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration/test_main.cpp`

IMPLEMENTATION TASKS:
1. Implement `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h` and `src/imu_calibration.cpp`:
   - Class `ImuCalibrator`:
     - ST AN4508 6-position accelerometer linear calibration: estimates scale factors $s_j = (\bar{a}_{+j} - \bar{a}_{-j})/(2g)$ and biases $b_j = (\bar{a}_{+j} + \bar{a}_{-j})/2$ across 6 poses (+X, -X, +Y, -Y, +Z, -Z), guaranteeing norm error $< 0.05$ m/s² on all 6 faces.
     - Stationary gyroscope zero-rate bias nulling: sample averaging over $\ge 1000$ samples ($\ge 10$ seconds), guaranteeing residual static drift $< 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s).
     - Welford motion disturbance rejection: reject calibration frames if sample variance $\sigma_\omega^2 > 10^{-4}$ rad²/s².
     - REP-103 ENU coordinate alignment: standardize body-frame readings to X-forward, Y-left, Z-up, with static gravity $+9.80665$ m/s² on Z.
     - Allan variance noise model parameters ($N_g, N_a, K_g$) and field covariance inflation ($\alpha = 1.8 \implies \alpha^2 = 3.24$).
2. Update `firmware/stm32_f407vg_arduino_sim/platformio.ini`:
   - Add `+<imu_calibration.cpp>` to `build_src_filter` in `[env:native]`.
3. Author 7-case Unity test suite in `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration/test_main.cpp` covering:
   - `test_default_calibration_is_identity`
   - `test_gyro_bias_nulling_and_drift_threshold` (residual drift $< 0.05^\circ/\text{s}$)
   - `test_gyro_motion_rejection`
   - `test_accel_6_position_calibration_and_norm_error` (norm error $< 0.05$ m/s²)
   - `test_rep103_enu_coordinate_alignment` ($+9.80665$ m/s² on Z at rest)
   - `test_allan_variance_covariance_inflation` (all diagonal covariances populated, $\alpha = 1.8$)
   - `test_invalid_inputs_and_edge_cases`
4. Integrate `ImuCalibrator` into `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:
   - In `imu_task`: apply `calibrator.calibrate_sample(sample)` to correct accelerometer and gyroscope data.
   - In `publish_telemetry`: populate non-zero diagonal entries in `imu_message.orientation_covariance`, `angular_velocity_covariance`, and `linear_acceleration_covariance` (eliminating the zero-covariance bug).

VERIFICATION REQUIREMENTS:
Run and report verbatim output for:
- `pio test -e native` in `firmware/stm32_f407vg_arduino_sim`
- `pio run -e disco_f407vg` in `firmware/stm32_f407vg_arduino_sim`
- `pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v`

When complete, write your handoff report to `handoff.md` in your working directory and notify the orchestrator via send_message.
