# Scope: Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32

## Overview
Implement the on-board IMU intrinsic calibration routines directly on STM32 firmware, including ST AN4508 6-position accelerometer calibration, stationary gyroscope zero-rate bias nulling, ENU coordinate standardization, and Allan variance noise parameter extraction.

## Features Assigned
- **F2.1: STM32 ST AN4508 Accelerometer Calibration**:
  Closed-form on-board linear estimation of scale factors $s_j = (\bar{a}_{j, +j} - \bar{a}_{j, -j})/(2g)$ and biases $b_j = (\bar{a}_{j, +j} + \bar{a}_{j, -j})/2$ across 6 static poses, with calibrated output $a_{calib, j} = (a_{raw, j} - b_j)/s_j$.
- **F2.2: STM32 Stationary Gyroscope Bias Tracking & Nulling**:
  Stationary sample averaging over $\ge 10$ seconds ensuring residual drift $< 0.05^\circ/\text{s}$ ($8.72 \times 10^{-4}$ rad/s) when at rest.
- **F2.3: REP-103 ENU Coordinate Standard**:
  Standardize body-frame IMU data to East-North-Up (X-forward, Y-left, Z-up) with static gravity $+9.80665$ m/s² along +Z.
- **F2.4: Allan Variance Noise Model & Dynamic Inflation**:
  Allan variance extraction parameters ($N_g, N_a, K_g$) and dynamic operating inflation margin ($1.5\times - 2.0\times$).

## Target Files
- `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.h`
- `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
- `firmware/stm32_f407vg_arduino_sim/src/main.cpp` (integrate calibration state machine into IMU task)
- `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration.cpp` (unit tests for IMU calibration)

## Completion Criteria
- Firmware builds with 0 errors/warnings.
- Stationary gyro drift $< 0.05^\circ/\text{s}$ verified by tests.
- Accel norm error $< 0.5\%$ ($< 0.05$ m/s²) across all 6 test poses.
- ENU coordinate outputs verified.
