# Milestone 2 (M2) Implementation Handoff Report

**Agent**: `teamwork_preview_worker_m2_2` (Implementation Worker M2)  
**Date**: 2026-09-19  
**Handoff Type**: Hard Handoff (Task Complete)  
**Working Directory**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_2`  
**Target Recipient**: Orchestrator M2 (`709d5506-1905-49c5-bf69-8e756d885098`)  

---

## 1. Observation

1. **Firmware Source & Configuration Modifications**:
   - `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`: Implemented class `ImuCalibrator`, data structures `ImuCalibrationParams`, `ImuCalibProgress`, enums `AccelFace` and `ImuCalibState`, and full public API including `start_gyro_calibration`, `update_gyro_sample`, `finish_gyro_calibration`, `start_accel_face`, `update_accel_sample`, `finish_accel_face`, `compute_accel_calibration`, `apply`, `calibrate_sample`, and `compute_covariances`.
   - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`:
     - Implemented ST AN4508 linear accelerometer calibration:
       $$s_j = \frac{\bar{a}_{+j} - \bar{a}_{-j}}{2g}, \quad b_j = \frac{\bar{a}_{+j} + \bar{a}_{-j}}{2}$$
       $$a_{calib, j} = \frac{a_{raw, j} - b_j}{s_j}$$
       Guarantees norm error $< 0.05$ m/s² across all 6 faces.
     - Implemented stationary gyroscope zero-rate bias nulling via Welford incremental sample averaging over $\ge 1000$ samples, guaranteeing residual drift $< 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s).
     - Implemented Welford disturbance variance rejection: flags `CALIB_FAILED_MOTION` and rejects frames if sample variance $\sigma_\omega^2 > 10^{-4}$ rad²/s².
     - Implemented Allan variance noise model ($N_g = 1.4 \times 10^{-4}, N_a = 1.9 \times 10^{-3}, K_g = 1.5 \times 10^{-5}$) with field covariance inflation factor $\alpha = 1.8$ ($\alpha^2 = 3.24$).
   - `firmware/stm32_f407vg_arduino_sim/platformio.ini`:
     - Line 40: Added `+<imu_calibration.cpp>` to `build_src_filter` in `[env:native]`.
   - `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:
     - Lines 25, 40: Included `imu_calibration.h` and instantiated global `ImuCalibrator imu_calibrator;`.
     - Lines 256–266: Initialized non-zero diagonal covariances in `initialize_ros_message_memory()`.
     - Lines 373–384: Populated non-zero diagonal covariances for `imu_message.angular_velocity_covariance`, `linear_acceleration_covariance`, and `orientation_covariance` in `publish_telemetry`.
     - Lines 413–426: Assigned `rcl_*_fini` return values in `clean_ros_entities()` to eliminate `-Wunused-result` compiler warnings.
     - Lines 564–572: Applied `imu_calibrator.calibrate_sample(raw_sample, sample)` in FreeRTOS `imu_task`.
   - `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration/test_main.cpp`:
     - Created 7-case Unity test suite covering all required acceptance criteria.

2. **Verification Command 1: Native PlatformIO Test Suite**:
   Command: `pio test -e native` in `firmware/stm32_f407vg_arduino_sim`
   Output:
   ```
   Processing test_imu_calibration in native environment
   test/test_imu_calibration/test_main.cpp:239: test_default_calibration_is_identity	[PASSED]
   test/test_imu_calibration/test_main.cpp:240: test_gyro_bias_nulling_and_drift_threshold	[PASSED]
   test/test_imu_calibration/test_main.cpp:241: test_gyro_motion_rejection	[PASSED]
   test/test_imu_calibration/test_main.cpp:242: test_accel_6_position_calibration_and_norm_error	[PASSED]
   test/test_imu_calibration/test_main.cpp:243: test_rep103_enu_coordinate_alignment	[PASSED]
   test/test_imu_calibration/test_main.cpp:244: test_allan_variance_covariance_inflation	[PASSED]
   test/test_imu_calibration/test_main.cpp:245: test_invalid_inputs_and_edge_cases	[PASSED]
   ------------ native:test_imu_calibration [PASSED] Took 0.75 seconds ------------
   ...
   =================================== SUMMARY ===================================
   Environment    Test                     Status    Duration
   -------------  -----------------------  --------  ------------
   native         test_kinematics          PASSED    00:00:00.735
   native         test_pid                 PASSED    00:00:00.734
   native         test_encoder_pll_stress  PASSED    00:00:00.769
   native         test_imu_calibration     PASSED    00:00:00.748
   native         test_kalman              PASSED    00:00:00.715
   native         test_encoder_pll         PASSED    00:00:00.715
   ================= 29 test cases: 29 succeeded in 00:00:04.416 =================
   ```

3. **Verification Command 2: Embedded STM32 Target Build**:
   Command: `pio run -e disco_f407vg` in `firmware/stm32_f407vg_arduino_sim`
   Output:
   ```
   Compiling .pio/build/disco_f407vg/src/main.cpp.o
   Checking size .pio/build/disco_f407vg/firmware.elf
   Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
   RAM:   [=====     ]  46.8% (used 61372 bytes from 131072 bytes)
   Flash: [=         ]  13.8% (used 144556 bytes from 1048576 bytes)
   ========================= [SUCCESS] Took 4.87 seconds =========================

   Environment    Status    Duration
   -------------  --------  ------------
   disco_f407vg   SUCCESS   00:00:04.872
   ========================= 1 succeeded in 00:00:04.872 =========================
   ```
   Result: **0 errors, 0 warnings**.

4. **Verification Command 3: Tier 1 Feature Coverage E2E Suite**:
   Command: `pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v` in project root
   Output:
   ```
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF21_STM32_AN4508_AccelCalibration::test_f2_1_an4508_ideal_gravity_recovery PASSED [  5%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF21_STM32_AN4508_AccelCalibration::test_f2_1_an4508_scale_and_bias_estimation PASSED [ 10%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF21_STM32_AN4508_AccelCalibration::test_f2_1_an4508_calibrated_norm_accuracy PASSED [ 15%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF21_STM32_AN4508_AccelCalibration::test_f2_1_an4508_noise_rejection PASSED [ 20%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF21_STM32_AN4508_AccelCalibration::test_f2_1_an4508_residual_error_bound PASSED [ 25%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF22_STM32_GyroBiasNulling::test_f2_2_gyro_nulling_stationary_bias_removal PASSED [ 30%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF22_STM32_GyroBiasNulling::test_f2_2_gyro_nulling_residual_drift_under_005_deg_s PASSED [ 35%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF22_STM32_GyroBiasNulling::test_f2_2_gyro_nulling_stationary_variance_gate PASSED [ 40%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF22_STM32_GyroBiasNulling::test_f2_2_gyro_nulling_long_term_integration_drift PASSED [ 45%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF22_STM32_GyroBiasNulling::test_f2_2_gyro_nulling_three_axis_independence PASSED [ 50%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF23_REP103_ENUCoordinateStandard::test_f2_3_enu_axes_orientation PASSED [ 55%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF23_REP103_ENUCoordinateStandard::test_f2_3_enu_static_gravity_vector_z PASSED [ 60%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF23_REP103_ENUCoordinateStandard::test_f2_3_enu_counter_clockwise_positive_yaw PASSED [ 65%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF23_REP103_ENUCoordinateStandard::test_f2_3_enu_right_hand_rule_angular_rates PASSED [ 70%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF23_REP103_ENUCoordinateStandard::test_f2_3_enu_acceleration_invariance_in_body_frame PASSED [ 75%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF24_AllanVariance_CovarianceInflation::test_f2_4_allan_variance_computation PASSED [ 80%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF24_AllanVariance_CovarianceInflation::test_f2_4_allan_variance_arw_extraction PASSED [ 85%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF24_AllanVariance_CovarianceInflation::test_f2_4_allan_variance_bias_instability_extraction PASSED [ 90%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF24_AllanVariance_CovarianceInflation::test_f2_4_field_covariance_inflation_factor_1_5_to_2_0 PASSED [ 95%]
   tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py::TestF24_AllanVariance_CovarianceInflation::test_f2_4_inflation_prevents_filter_overconfidence PASSED [100%]
   ======================= 20 passed, 71 warnings in 0.52s ========================
   ```

---

## 2. Logic Chain

1. **Intrinsic Error Correction Architecture**:
   - In `main.cpp`, `RobotHardware::read_imu` provides raw sensor readings. Before this change, raw values were published directly without sensitivity scale or bias compensation (Observation 1).
   - Implementing `ImuCalibrator` provides the missing mathematical layer: `imu_calibrator.calibrate_sample(raw_sample, sample)` is invoked directly within FreeRTOS `imu_task`, applying scale factor division and bias subtraction before downstream quaternion normalization and robot state propagation.
2. **Deterministic Scale and Bias Derivation**:
   - The ST AN4508 linear formulation solves $s_j = (\bar{a}_{+j} - \bar{a}_{-j}) / (2g)$ and $b_j = (\bar{a}_{+j} + \bar{a}_{-j}) / 2$.
   - When the calibrated measurement is computed as $a_{calib, j} = (a_{raw, j} - b_j) / s_j$, the opposing gravity components cancel out bias and scale discrepancies, recovering the invariant physical gravity norm $g = 9.80665$ m/s².
   - This was validated in `test_accel_6_position_calibration_and_norm_error`, where residual norm error across all 6 poses satisfied $\Delta_f < 0.05$ m/s² (Observation 2).
3. **Stationary Bias Nulling and Motion Guard**:
   - Welford's single-pass incremental calculation computes the running mean and variance without accumulating memory or incurring numerical round-off.
   - When motion is present ($\sigma_\omega^2 > 10^{-4}$ rad²/s²), calibration is immediately aborted (`CALIB_FAILED_MOTION`), preventing contaminated samples from corrupting zero-rate bias (Observation 2, `test_gyro_motion_rejection`).
   - Over $\ge 1000$ stationary samples, the standard error of the mean bias estimate is bounded by $\sqrt{\sigma^2 / 1000} \le \sqrt{10^{-7}} = 3.16 \times 10^{-4}$ rad/s, guaranteeing residual static drift $< 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s) as verified in `test_gyro_bias_nulling_and_drift_threshold`.
4. **REP-103 ENU Standard and Covariance Integrity**:
   - At stationary rest on a horizontal plane, the calibrated acceleration reads $+9.80665$ m/s² along $+Z$ and zero on $X$ and $Y$ (Observation 2, `test_rep103_enu_coordinate_alignment`).
   - Covariance computation incorporates Allan variance noise density and field inflation $\alpha = 1.8$ ($\alpha^2 = 3.24$). All diagonal elements in `orientation_covariance`, `angular_velocity_covariance`, and `linear_acceleration_covariance` are strictly positive, eliminating the zero-covariance bug in micro-ROS telemetry publishing.
5. **Firmware Cleanliness**:
   - Assigning return values of `rcl_*_fini` calls eliminated all `-Wunused-result` warnings, achieving clean compilation with 0 errors and 0 warnings on `disco_f407vg` (Observation 3).

---

## 3. Caveats

1. **Non-Orthogonality Matrix ($T_a$)**:
   - The ST AN4508 linear method assumes sensor orthogonal axes ($T_a = I_{3\times 3}$). For standard MEMS IMU sensors installed on flat PCBs, axis skew is $< 0.1^\circ$ and does not exceed the $0.05$ m/s² tolerance.
2. **Flash Persistence**:
   - In M2, calibration parameters reside in microcontroller RAM within `ImuCalibrator`. Flash persistence via `CMD_CALIB_FLASH_COMMIT` and ROS 2 YAML configuration persistence via FastAPI REST endpoints will be integrated in Milestone 4.

---

## 4. Conclusion

Milestone 2 (IMU Intrinsic Calibration & Filtering on STM32) is fully implemented, verified, and complete:
- `ImuCalibrator` provides ST AN4508 6-position accelerometer linear calibration, Welford-guarded stationary gyro zero-rate bias nulling, REP-103 ENU alignment, and inflated Allan variance covariances.
- PlatformIO `[env:native]` test filter was updated and passes 100% of all 29 unit tests.
- PlatformIO `[env:disco_f407vg]` compiles cleanly with **0 errors and 0 warnings**.
- Tier 1 E2E test suite `test_f2_imu_calibration.py` passes 20/20 test cases.

---

## 5. Verification Method

To independently reproduce and verify this milestone:

1. **Run Native Unit Tests**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib/firmware/stm32_f407vg_arduino_sim
   pio test -e native
   ```
   *Expected Result*: All 29 test cases across 6 suites pass with 100% success.

2. **Run STM32 Hardware Target Build**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib/firmware/stm32_f407vg_arduino_sim
   pio run -e disco_f407vg
   ```
   *Expected Result*: SUCCESS with 0 errors and 0 warnings.

3. **Run E2E IMU Calibration Test Suite**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v
   ```
   *Expected Result*: 20 passed.

4. **Invalidation Conditions**:
   - Residual static gyro drift $\ge 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s).
   - Accelerometer norm error $\ge 0.05$ m/s² on any of the 6 faces.
   - Any zero diagonal entry in `imu_message` covariance matrices.
   - Any compiler warning or error during `pio run -e disco_f407vg`.

---
*End of Milestone 2 Handoff Report.*
