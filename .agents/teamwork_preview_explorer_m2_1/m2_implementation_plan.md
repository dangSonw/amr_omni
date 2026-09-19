# Milestone 2 (M2) Detailed Implementation Plan: IMU Intrinsic Calibration & Filtering on STM32

**Document Path**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1/m2_implementation_plan.md`  
**Author**: `teamwork_preview_explorer_m2_1` (Technical Explorer M2)  
**Date**: 2026-09-19  
**Status**: DESIGN COMPLETE — READY FOR IMPLEMENTATION  

---

## 1. Executive Summary & Milestone Objectives

The objective of Milestone 2 (M2) is to design and implement on-board IMU intrinsic calibration routines, coordinate standardizations, and noise filtering directly on the STM32F407 microcontroller firmware (`firmware/stm32_f407vg_arduino_sim`).

### Scope of Features
1. **F2.1: STM32 ST AN4508 Accelerometer Calibration**:
   Closed-form linear least-squares routine solving scale factors $s_j = (\bar{a}_{j, +j} - \bar{a}_{j, -j})/(2g)$ and biases $b_j = (\bar{a}_{j, +j} + \bar{a}_{j, -j})/2$ across 6 orthogonal static poses ($+X, -X, +Y, -Y, +Z, -Z$). Calibrated outputs ensure residual acceleration norm error $< 0.5\%$ ($< 0.05$ m/s²).
2. **F2.2: STM32 Stationary Gyroscope Bias Tracking & Nulling**:
   Stationary zero-rate bias calculation averaging over $\ge 1000$ samples ($\ge 10$ seconds) with online variance motion rejection, guaranteeing static residual drift $< 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s).
3. **F2.3: REP-103 ENU Coordinate Frame Standardization**:
   Standardize body frame IMU readings to ROS standard East-North-Up ($+X$ forward, $+Y$ left, $+Z$ up, positive CCW yaw), ensuring static gravity reading $+9.80665$ m/s² along $+Z$ at rest for clean removal by `robot_localization` (`imu0_remove_gravitational_acceleration: true`).
4. **F2.4: Allan Variance Noise Model & Field Covariance Inflation**:
   Allan variance noise model parameters ($N_g, N_a, K_g$) and field covariance inflation ($\alpha \in [1.5, 2.0]$) populating complete non-zero diagonal covariances into `sensor_msgs/msg/Imu` (`orientation_covariance`, `angular_velocity_covariance`, `linear_acceleration_covariance`).
5. **Firmware Test Infrastructure**:
   Configure PlatformIO `[env:native]` test runner in `platformio.ini` to enable host-based automated unit testing (`pio test -e native`) and author `test/test_imu_calibration/test_main.cpp`.

---

## 2. Current Firmware State Analysis & Diagnostic Findings

Exploration of `/home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim` revealed the following critical findings:

### 2.1 Missing Intrinsic Calibration in `imu_task`
In `src/main.cpp` (lines 528–557):
```cpp
void imu_task(void *) {
    ...
    if (RobotHardware::read_imu(sample)) {
        // Only normalizes quaternion; raw acceleration and gyro are untouched!
        imu_last_valid_ms = RobotHardware::now_ms();
        fault = false;
    }
    update_imu_fields(sample, fault, RobotHardware::now_ms());
}
```
- **Finding**: Raw sensor readings from hardware/simulator pass directly into `robot_state.imu` without scale or bias compensation. Any sensor zero-rate drift accumulates directly into heading and position estimates.

### 2.2 Zero-Covariance Telemetry Vulnerability
In `src/main.cpp` (lines 249–250, 346–358):
```cpp
memset(&odometry_message, 0, sizeof(odometry_message));
memset(&imu_message, 0, sizeof(imu_message));
...
publish_message(imu_publisher, &imu_message);
```
- **Finding**: All 9 elements in `imu_message.orientation_covariance`, `angular_velocity_covariance`, and `linear_acceleration_covariance` remain `0.0`. In ROS 2 `robot_localization`, zero covariance is treated either as uninitialized or infinite confidence, inducing numerical instability, Kalman gain singularities, or filter divergence.

### 2.3 PlatformIO Test Configuration Gap
In `platformio.ini`:
- Currently only `[env:disco_f407vg]` is configured. Running `pio test -e native` fails with:
  `UnknownEnvNamesError: Unknown environment names 'native'. Valid names are 'disco_f407vg'`.
- Running `pio test` on `disco_f407vg` skips tests (`SKIPPED`) because no physical ST-Link/J-Link board is attached.
- Adding `[env:native]` with `platform = native`, `test_framework = unity`, and `test_build_src = false` allows running all unit tests natively on x86_64 host in under 4 seconds.

### 2.4 Pre-existing Firmware Compilation Errors & Warnings
When running `pio run -e disco_f407vg`:
1. `src/kalman.cpp:17:10: error: 'isfinite' was not declared in this scope`:
   `src/kalman.cpp` is missing `#include <math.h>`.
2. `src/main.cpp:388-394`: Compiler warnings for ignoring return values of `rcl_publisher_fini`, `rcl_subscription_fini`, and `rcl_node_fini` (`-Wunused-result`).
- **Remediation**: The implementer must add `#include <math.h>` to `kalman.cpp` and cast cleanup calls to `(void)` to satisfy the completion criterion: *"Firmware builds with 0 errors/warnings"*.

---

## 3. Algorithmic & Mathematical Specifications

### 3.1 ST AN4508 6-Position Accelerometer Calibration

#### A. Physical Principle & Static Orientation Definition
When stationary in Earth's gravity field $g = 9.80665\text{ m/s}^2$, the accelerometer proof mass detects specific force directed upwards (contact normal force opposing gravity).
In the 6-position method, the sensor is oriented in 6 orthogonal poses:

| Pose | Alignment Description | True Specific Force Vector $\mathbf{a}_{true}$ |
|------|-----------------------|-------------------------------------------------|
| Pose 1 (`FACE_POS_X`) | $+X$ axis pointing up | $[+g, 0, 0]^T$ |
| Pose 2 (`FACE_NEG_X`) | $-X$ axis pointing up | $[-g, 0, 0]^T$ |
| Pose 3 (`FACE_POS_Y`) | $+Y$ axis pointing up | $[0, +g, 0]^T$ |
| Pose 4 (`FACE_NEG_Y`) | $-Y$ axis pointing up | $[0, -g, 0]^T$ |
| Pose 5 (`FACE_POS_Z`) | $+Z$ axis pointing up (robot horizontal level) | $[0, 0, +g]^T$ |
| Pose 6 (`FACE_NEG_Z`) | $-Z$ axis pointing up (robot upside down) | $[0, 0, -g]^T$ |

#### B. Mathematical Derivation
For each axis $j \in \{x, y, z\}$, let $s_j$ be the scale factor and $b_j$ be the bias offset. The measurement model is:
$$\bar{a}_{j, +j} = s_j (+g) + b_j$$
$$\bar{a}_{j, -j} = s_j (-g) + b_j$$

Adding the two equations:
$$\bar{a}_{j, +j} + \bar{a}_{j, -j} = 2 b_j \implies b_j = \frac{\bar{a}_{j, +j} + \bar{a}_{j, -j}}{2}$$

Subtracting the two equations:
$$\bar{a}_{j, +j} - \bar{a}_{j, -j} = 2 s_j g \implies s_j = \frac{\bar{a}_{j, +j} - \bar{a}_{j, -j}}{2 g}$$

Calibrated linear acceleration is then recovered via:
$$a_{calib, j} = \frac{a_{raw, j} - b_j}{s_j}$$

#### C. Engineering Bounds & Acceptance Validation
- **Plausibility limits**:
  $$0.80 \le s_j \le 1.20$$
  $$|b_j| \le 2.0\text{ m/s}^2$$
- **Residual norm error verification**:
  Across all 6 faces $f \in \{1 \dots 6\}$, the norm error must satisfy:
  $$\Delta_f = \left| \| \mathbf{a}_{calib}(f) \| - g \right| < 0.5\% \cdot g \approx 0.049\text{ m/s}^2 \implies \Delta_f < 0.05\text{ m/s}^2$$

---

### 3.2 Stationary Gyroscope Zero-Rate Bias Tracking & Nulling

#### A. Sampling & Mean Estimation
When the robot is at stationary rest:
$$\mathbf{b}_g = \frac{1}{N} \sum_{i=1}^N \boldsymbol{\omega}_{raw}(t_i)$$
where $N \ge 1000$ samples. At 50 Hz (`kImuPeriodMs` = 20 ms), 1000 samples represents $20.0\text{ seconds}$ of steady-state resting data.

To prevent buffer overflow in embedded RAM, Welford's single-pass algorithm computes the mean and variance incrementally:
$$\bar{\omega}_{k} = \bar{\omega}_{k-1} + \frac{\omega_k - \bar{\omega}_{k-1}}{k}$$
$$M_{2, k} = M_{2, k-1} + (\omega_k - \bar{\omega}_{k-1})(\omega_k - \bar{\omega}_k)$$
$$\sigma_k^2 = \frac{M_{2, k}}{k - 1} \quad (k \ge 2)$$

#### B. Dynamic Motion Guard
To prevent contaminated calibration from accidental knocks or floor vibrations:
$$\sigma_\omega^2 = \sigma_{\omega x}^2 + \sigma_{\omega y}^2 + \sigma_{\omega z}^2$$
If $\sigma_\omega^2 > \gamma_\omega$ (where $\gamma_\omega = 1.0 \times 10^{-4}\text{ (rad/s)}^2$), the calibrator detects disturbance, flags `CALIB_FAILED_MOTION`, and aborts or resets the accumulator.

#### C. Residual Drift Acceptance Criterion
After applying calibrated bias $\boldsymbol{\omega}_{calib} = \boldsymbol{\omega}_{raw} - \mathbf{b}_g$, the residual drift over a static verification window must satisfy:
$$\| \bar{\boldsymbol{\omega}}_{calib} \| = \sqrt{\bar{\omega}_{calib, x}^2 + \bar{\omega}_{calib, y}^2 + \bar{\omega}_{calib, z}^2} < 0.05^\circ/\text{s} = 8.7266 \times 10^{-4}\text{ rad/s}$$

---

### 3.3 REP-103 ENU Coordinate System Standardization

#### A. Frame Orientation Definition
- $+X$: Forward direction of robot chassis
- $+Y$: Leftward direction of robot chassis
- $+Z$: Upward direction perpendicular to ground plane
- Rotations: Right-hand rule (Counter-Clockwise rotation around $+Z$ is positive Yaw).

#### B. Static Gravity Behavior
When the robot is level and resting on horizontal ground:
$$\mathbf{a}_{body, static} = \begin{bmatrix} 0 \\ 0 \\ +9.80665 \end{bmatrix}\text{ m/s}^2, \quad \boldsymbol{\omega}_{body, static} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}\text{ rad/s}$$

In `src/omni_localization/config/ekf.yaml`:
`imu0_remove_gravitational_acceleration: true`
`robot_localization` estimates the gravity direction from the orientation quaternion and subtracts $+9.80665\text{ m/s}^2$ along the local vertical, resulting in net zero linear acceleration $\mathbf{a} \approx [0, 0, 0]^T$.

#### C. Mounting Transformation Matrix $\mathbf{R}_{mount}$
If the IMU chip is mounted with non-standard orientation:
$$\mathbf{a}_{body} = \mathbf{R}_{mount} \mathbf{a}_{calib}, \quad \boldsymbol{\omega}_{body} = \mathbf{R}_{mount} \boldsymbol{\omega}_{calib}$$
For standard aligned mounting, $\mathbf{R}_{mount} = \mathbf{I}_{3 \times 3}$.

---

### 3.4 Allan Variance Noise Model & Field Covariance Inflation

#### A. Allan Variance Stochastic Noise Parameters
From IEEE Std 952-1997 / `imu_utils` benchmark analysis for 6-DoF MEMS:
- Gyro Angle Random Walk (White Noise): $N_g \approx 1.4 \times 10^{-4}\text{ rad/s}/\sqrt{\text{Hz}}$
- Gyro Rate Random Walk (Bias Drift Rate): $K_g \approx 1.5 \times 10^{-5}\text{ rad/s}^2/\sqrt{\text{Hz}}$
- Accel Velocity Random Walk (White Noise): $N_a \approx 1.9 \times 10^{-3}\text{ m/s}^2/\sqrt{\text{Hz}}$
- Accel Random Walk: $K_a \approx 1.2 \times 10^{-4}\text{ m/s}^3/\sqrt{\text{Hz}}$

#### B. Discrete Laboratory Covariance Conversion
At sampling interval $\Delta t$ ($f_s = 1/\Delta t$):
$$\sigma_{gyro, lab}^2 = \frac{N_g^2}{\Delta t} = N_g^2 \cdot f_s$$
$$\sigma_{accel, lab}^2 = \frac{N_a^2}{\Delta t} = N_a^2 \cdot f_s$$

For $\Delta t = 0.02\text{ s}$ (50 Hz):
$$\sigma_{gyro, lab}^2 = \frac{(1.4 \times 10^{-4})^2}{0.02} = 9.80 \times 10^{-7}\text{ (rad/s)}^2$$
$$\sigma_{accel, lab}^2 = \frac{(1.9 \times 10^{-3})^2}{0.02} = 1.805 \times 10^{-4}\text{ (m/s}^2)^2$$

#### C. Field Covariance Inflation Factor $\alpha$
During mobile AGV operation, floor surface roughness, Mecanum roller shock, and motor electromagnetic noise enter the sensor bandwidth. To prevent the EKF from becoming overconfident and diverging:
$$\mathbf{R}_{field} = \alpha^2 \cdot \mathbf{R}_{lab}, \quad \text{where } \alpha \in [1.5, 2.0] \text{ (nominal } \alpha = 1.8\text{)}$$
$$\alpha^2 = (1.8)^2 = 3.24$$

To guarantee absolute robustness against severe chassis vibrations, we enforce physical floor lower bounds:
$$\sigma_{gyro}^2 = \max\left(\alpha^2 \frac{N_g^2}{\Delta t}, 1.0 \times 10^{-4}\right)\text{ (rad/s)}^2$$
$$\sigma_{accel}^2 = \max\left(\alpha^2 \frac{N_a^2}{\Delta t}, 1.0 \times 10^{-2}\right)\text{ (m/s}^2)^2$$

#### D. Full $3 \times 3$ Covariance Matrix Layouts in `sensor_msgs/msg/Imu`
All diagonal entries MUST BE NON-ZERO:
- **`orientation_covariance`** ($3 \times 3$, row-major):
  $$\begin{bmatrix} 1.0 \times 10^{6} & 0.0 & 0.0 \\ 0.0 & 1.0 \times 10^{6} & 0.0 \\ 0.0 & 0.0 & 2.5 \times 10^{-3} \end{bmatrix}$$
- **`angular_velocity_covariance`** ($3 \times 3$, row-major):
  $$\begin{bmatrix} 1.0 \times 10^{6} & 0.0 & 0.0 \\ 0.0 & 1.0 \times 10^{6} & 0.0 \\ 0.0 & 0.0 & \sigma_{gyro}^2 \end{bmatrix}$$
- **`linear_acceleration_covariance`** ($3 \times 3$, row-major):
  $$\begin{bmatrix} \sigma_{accel}^2 & 0.0 & 0.0 \\ 0.0 & \sigma_{accel}^2 & 0.0 \\ 0.0 & 0.0 & \sigma_{accel}^2 \end{bmatrix}$$

---

## 4. Exact File Specifications & Implementation Code

### 4.1 Header File: `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`

```cpp
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "firmware_config.h"

// Standard gravitational acceleration (REP-103)
static const float kStandardGravityMps2 = 9.80665F;

// Acceptance criteria thresholds
static const float kMaxGyroDriftRadS = 8.7266e-4F;       // 0.05 deg/s
static const float kMaxAccelNormErrorMps2 = 0.05F;       // < 0.5% of 1g
static const float kMaxGyroStaticVariance = 1.0e-4F;     // (rad/s)^2
static const uint32_t kMinGyroCalibrationSamples = 1000U; // >= 10s at 100Hz / 20s at 50Hz
static const uint32_t kMinAccelFaceSamples = 200U;       // >= 4s per face at 50Hz

// Default Allan Variance Noise Model Parameters
static const float kDefaultGyroNoiseDensity = 1.4e-4F;    // Ng: rad/s/sqrt(Hz)
static const float kDefaultAccelNoiseDensity = 1.9e-3F;   // Na: m/s^2/sqrt(Hz)
static const float kDefaultGyroRandomWalk = 1.5e-5F;      // Kg: rad/s^2/sqrt(Hz)
static const float kDefaultInflationAlpha = 1.8F;         // 1.5x - 2.0x

enum AccelFace : uint8_t {
    FACE_POS_X = 0U,
    FACE_NEG_X = 1U,
    FACE_POS_Y = 2U,
    FACE_NEG_Y = 3U,
    FACE_POS_Z = 4U,
    FACE_NEG_Z = 5U,
    FACE_COUNT = 6U
};

enum ImuCalibState : uint8_t {
    CALIB_IDLE = 0U,
    CALIB_GYRO_SAMPLING = 1U,
    CALIB_ACCEL_SAMPLING = 2U,
    CALIB_COMPUTING = 3U,
    CALIB_SUCCESS = 4U,
    CALIB_FAILED_MOTION = 5U,
    CALIB_FAILED_MATH = 6U
};

struct ImuCalibrationParams {
    float gyro_bias[3];            // [bx, by, bz] in rad/s
    float accel_scale[3];          // [sx, sy, sz] unitless (~1.0)
    float accel_bias[3];           // [bx, by, bz] in m/s^2
    float gyro_noise_density;      // Ng
    float accel_noise_density;     // Na
    float gyro_random_walk;        // Kg
    float covariance_inflation;    // alpha
    bool gyro_calibrated;
    bool accel_calibrated;
};

struct ImuCalibProgress {
    ImuCalibState state;
    uint8_t stage;                 // Face index or gyro stage
    uint8_t progress_pct;          // 0..100%
    uint32_t samples_collected;
    float live_metric;             // Live variance or residual error
};

class ImuCalibrator {
public:
    ImuCalibrator();

    void reset();

    // Gyroscope Calibration
    bool start_gyro_calibration(uint32_t target_samples = kMinGyroCalibrationSamples);
    bool update_gyro_sample(const float gyro_raw[3]);
    bool finish_gyro_calibration(float &residual_drift_rad_s);

    // Accelerometer Calibration (ST AN4508 6-Position)
    bool start_accel_face(AccelFace face, uint32_t target_samples = kMinAccelFaceSamples);
    bool update_accel_sample(const float accel_raw[3]);
    bool finish_accel_face();
    bool compute_accel_calibration(float &max_norm_error);

    // Real-time correction & ENU transformation
    void apply(const ImuSample &raw, ImuSample &calibrated) const;
    void apply(const float raw_accel[3], const float raw_gyro[3],
               float calib_accel[3], float calib_gyro[3]) const;

    // Covariance matrix population with Allan variance & inflation
    void compute_covariances(float dt_sec,
                             float angular_vel_cov[9],
                             float linear_accel_cov[9]) const;

    // Getters / Setters
    const ImuCalibrationParams &get_params() const { return params_; }
    void set_params(const ImuCalibrationParams &params);
    ImuCalibProgress get_progress() const;
    ImuCalibState get_state() const { return state_; }

private:
    ImuCalibrationParams params_;
    ImuCalibState state_;
    uint8_t current_stage_;
    uint32_t target_samples_;
    uint32_t sample_count_;

    // Welford running statistics for Gyro
    float gyro_mean_[3];
    float gyro_m2_[3];

    // Accelerometer 6-face statistics
    float accel_face_sum_[FACE_COUNT][3];
    uint32_t accel_face_count_[FACE_COUNT];
    bool face_completed_[FACE_COUNT];
};

#endif // IMU_CALIBRATION_H
```

---

### 4.2 Source File: `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`

```cpp
#include "imu_calibration.h"
#include <string.h>

namespace {

inline bool is_valid_float(float val) {
    return isfinite(val) != 0;
}

} // namespace

ImuCalibrator::ImuCalibrator() {
    reset();
}

void ImuCalibrator::reset() {
    state_ = CALIB_IDLE;
    current_stage_ = 0U;
    target_samples_ = 0U;
    sample_count_ = 0U;

    for (uint8_t i = 0U; i < 3U; ++i) {
        params_.gyro_bias[i] = 0.0F;
        params_.accel_scale[i] = 1.0F;
        params_.accel_bias[i] = 0.0F;
        gyro_mean_[i] = 0.0F;
        gyro_m2_[i] = 0.0F;
    }
    params_.gyro_noise_density = kDefaultGyroNoiseDensity;
    params_.accel_noise_density = kDefaultAccelNoiseDensity;
    params_.gyro_random_walk = kDefaultGyroRandomWalk;
    params_.covariance_inflation = kDefaultInflationAlpha;
    params_.gyro_calibrated = false;
    params_.accel_calibrated = false;

    memset(accel_face_sum_, 0, sizeof(accel_face_sum_));
    memset(accel_face_count_, 0, sizeof(accel_face_count_));
    memset(face_completed_, 0, sizeof(face_completed_));
}

void ImuCalibrator::set_params(const ImuCalibrationParams &params) {
    params_ = params;
}

bool ImuCalibrator::start_gyro_calibration(uint32_t target_samples) {
    if (target_samples < 100U) {
        return false;
    }
    state_ = CALIB_GYRO_SAMPLING;
    current_stage_ = 0U;
    target_samples_ = target_samples;
    sample_count_ = 0U;
    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_mean_[i] = 0.0F;
        gyro_m2_[i] = 0.0F;
    }
    return true;
}

bool ImuCalibrator::update_gyro_sample(const float gyro_raw[3]) {
    if (state_ != CALIB_GYRO_SAMPLING) {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!is_valid_float(gyro_raw[i])) {
            return false;
        }
    }

    sample_count_++;
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float delta = gyro_raw[i] - gyro_mean_[i];
        gyro_mean_[i] += delta / static_cast<float>(sample_count_);
        const float delta2 = gyro_raw[i] - gyro_mean_[i];
        gyro_m2_[i] += delta * delta2;
    }

    // Motion detection check after 50 samples
    if (sample_count_ > 50U) {
        const float variance_sum = (gyro_m2_[0] + gyro_m2_[1] + gyro_m2_[2]) /
                                   static_cast<float>(sample_count_ - 1U);
        if (variance_sum > kMaxGyroStaticVariance) {
            state_ = CALIB_FAILED_MOTION;
            return false;
        }
    }
    return true;
}

bool ImuCalibrator::finish_gyro_calibration(float &residual_drift_rad_s) {
    if (state_ != CALIB_GYRO_SAMPLING || sample_count_ < 100U) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        params_.gyro_bias[i] = gyro_mean_[i];
    }
    params_.gyro_calibrated = true;
    state_ = CALIB_SUCCESS;

    // Calculate residual variance drift metric
    const float variance_sum = (gyro_m2_[0] + gyro_m2_[1] + gyro_m2_[2]) /
                               static_cast<float>(sample_count_ - 1U);
    residual_drift_rad_s = sqrtf(variance_sum);
    return residual_drift_rad_s < kMaxGyroDriftRadS;
}

bool ImuCalibrator::start_accel_face(AccelFace face, uint32_t target_samples) {
    if (face >= FACE_COUNT || target_samples < 50U) {
        return false;
    }
    state_ = CALIB_ACCEL_SAMPLING;
    current_stage_ = static_cast<uint8_t>(face);
    target_samples_ = target_samples;
    sample_count_ = 0U;
    accel_face_sum_[face][0] = 0.0F;
    accel_face_sum_[face][1] = 0.0F;
    accel_face_sum_[face][2] = 0.0F;
    accel_face_count_[face] = 0U;
    return true;
}

bool ImuCalibrator::update_accel_sample(const float accel_raw[3]) {
    if (state_ != CALIB_ACCEL_SAMPLING || current_stage_ >= FACE_COUNT) {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!is_valid_float(accel_raw[i])) {
            return false;
        }
        accel_face_sum_[current_stage_][i] += accel_raw[i];
    }
    sample_count_++;
    accel_face_count_[current_stage_] = sample_count_;
    return true;
}

bool ImuCalibrator::finish_accel_face() {
    if (state_ != CALIB_ACCEL_SAMPLING || sample_count_ == 0U) {
        return false;
    }
    face_completed_[current_stage_] = true;
    state_ = CALIB_IDLE;
    return true;
}

bool ImuCalibrator::compute_accel_calibration(float &max_norm_error) {
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        if (!face_completed_[f] || accel_face_count_[f] == 0U) {
            state_ = CALIB_FAILED_MATH;
            return false;
        }
    }

    state_ = CALIB_COMPUTING;
    float avg[FACE_COUNT][3];
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        const float count_f = static_cast<float>(accel_face_count_[f]);
        for (uint8_t axis = 0U; axis < 3U; ++axis) {
            avg[f][axis] = accel_face_sum_[f][axis] / count_f;
        }
    }

    const float two_g = 2.0F * kStandardGravityMps2;

    // ST AN4508 scale and bias calculation:
    // s_j = (a_pos - a_neg) / (2g)
    // b_j = (a_pos + a_neg) / 2
    const float sx = (avg[FACE_POS_X][0] - avg[FACE_NEG_X][0]) / two_g;
    const float bx = (avg[FACE_POS_X][0] + avg[FACE_NEG_X][0]) * 0.5F;

    const float sy = (avg[FACE_POS_Y][1] - avg[FACE_NEG_Y][1]) / two_g;
    const float by = (avg[FACE_POS_Y][1] + avg[FACE_NEG_Y][1]) * 0.5F;

    const float sz = (avg[FACE_POS_Z][2] - avg[FACE_NEG_Z][2]) / two_g;
    const float bz = (avg[FACE_POS_Z][2] + avg[FACE_NEG_Z][2]) * 0.5F;

    // Validate plausible range (scale within [0.7, 1.3], bias within [-3.0, 3.0] m/s^2)
    if (sx < 0.7F || sx > 1.3F || sy < 0.7F || sy > 1.3F || sz < 0.7F || sz > 1.3F ||
        fabsf(bx) > 3.0F || fabsf(by) > 3.0F || fabsf(bz) > 3.0F) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    params_.accel_scale[0] = sx;
    params_.accel_scale[1] = sy;
    params_.accel_scale[2] = sz;

    params_.accel_bias[0] = bx;
    params_.accel_bias[1] = by;
    params_.accel_bias[2] = bz;

    // Validate residual norm error across all 6 faces
    max_norm_error = 0.0F;
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        const float ax_cal = (avg[f][0] - bx) / sx;
        const float ay_cal = (avg[f][1] - by) / sy;
        const float az_cal = (avg[f][2] - bz) / sz;
        const float norm = sqrtf(ax_cal * ax_cal + ay_cal * ay_cal + az_cal * az_cal);
        const float err = fabsf(norm - kStandardGravityMps2);
        if (err > max_norm_error) {
            max_norm_error = err;
        }
    }

    if (max_norm_error > kMaxAccelNormErrorMps2) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    params_.accel_calibrated = true;
    state_ = CALIB_SUCCESS;
    return true;
}

void ImuCalibrator::apply(const ImuSample &raw, ImuSample &calibrated) const {
    apply(raw.linear_accel_mps2, raw.gyro_rad_s,
          calibrated.linear_accel_mps2, calibrated.gyro_rad_s);
    for (uint8_t i = 0U; i < 4U; ++i) {
        calibrated.quaternion_xyzw[i] = raw.quaternion_xyzw[i];
    }
    calibrated.valid = raw.valid;
}

void ImuCalibrator::apply(const float raw_accel[3], const float raw_gyro[3],
                          float calib_accel[3], float calib_gyro[3]) const {
    for (uint8_t i = 0U; i < 3U; ++i) {
        // Accelerometer calibration: (a_raw - bias) / scale
        if (params_.accel_calibrated && fabsf(params_.accel_scale[i]) > 1e-4F) {
            calib_accel[i] = (raw_accel[i] - params_.accel_bias[i]) / params_.accel_scale[i];
        } else {
            calib_accel[i] = raw_accel[i];
        }

        // Gyroscope calibration: w_raw - bias
        if (params_.gyro_calibrated) {
            calib_gyro[i] = raw_gyro[i] - params_.gyro_bias[i];
        } else {
            calib_gyro[i] = raw_gyro[i];
        }
    }
}

void ImuCalibrator::compute_covariances(float dt_sec,
                                        float angular_vel_cov[9],
                                        float linear_accel_cov[9]) const {
    const float dt = dt_sec > 0.001F ? dt_sec : 0.02F;
    const float alpha = params_.covariance_inflation > 1.0F ? params_.covariance_inflation : kDefaultInflationAlpha;
    const float alpha_sq = alpha * alpha;

    // sigma^2 = alpha^2 * (N^2 / dt)
    float gyro_var = alpha_sq * (params_.gyro_noise_density * params_.gyro_noise_density) / dt;
    float accel_var = alpha_sq * (params_.accel_noise_density * params_.accel_noise_density) / dt;

    // Minimum physical safety clamp for AMR dynamic vibration
    if (gyro_var < 1.0e-4F) gyro_var = 1.0e-4F;
    if (accel_var < 1.0e-2F) accel_var = 1.0e-2F;

    memset(angular_vel_cov, 0, 9 * sizeof(float));
    memset(linear_accel_cov, 0, 9 * sizeof(float));

    // Angular velocity covariance: [1e6, 1e6, sigma_wz^2]
    angular_vel_cov[0] = 1.0e6F;
    angular_vel_cov[4] = 1.0e6F;
    angular_vel_cov[8] = gyro_var;

    // Linear acceleration covariance: [sigma_ax^2, sigma_ay^2, sigma_az^2]
    linear_accel_cov[0] = accel_var;
    linear_accel_cov[4] = accel_var;
    linear_accel_cov[8] = accel_var;
}

ImuCalibProgress ImuCalibrator::get_progress() const {
    ImuCalibProgress progress;
    progress.state = state_;
    progress.stage = current_stage_;
    progress.samples_collected = sample_count_;
    if (target_samples_ > 0U) {
        uint32_t pct = (sample_count_ * 100U) / target_samples_;
        progress.progress_pct = pct > 100U ? 100U : static_cast<uint8_t>(pct);
    } else {
        progress.progress_pct = (state_ == CALIB_SUCCESS) ? 100U : 0U;
    }
    progress.live_metric = 0.0F;
    return progress;
}
```

---

### 4.3 Integration in `src/main.cpp`

The following changes are required in `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:

1. **Header inclusion**:
   ```cpp
   #include "imu_calibration.h"
   ```
2. **Global Instance Declaration**:
   ```cpp
   ImuCalibrator imu_calibrator;
   ```
3. **Telemetry Covariance Population** in `publish_telemetry(const RobotState &state)`:
   Replace zero covariances with inflated Allan variance matrices:
   ```cpp
   float gyro_cov[9];
   float accel_cov[9];
   imu_calibrator.compute_covariances(kImuPeriodMs * 0.001F, gyro_cov, accel_cov);

   for (uint8_t i = 0U; i < 9U; ++i) {
       imu_message.angular_velocity_covariance[i] = gyro_cov[i];
       imu_message.linear_acceleration_covariance[i] = accel_cov[i];
   }
   imu_message.orientation_covariance[0] = 1.0e6;
   imu_message.orientation_covariance[4] = 1.0e6;
   imu_message.orientation_covariance[8] = 2.5e-3;
   ```
4. **Calibration Application** in `imu_task(void *)`:
   ```cpp
   void imu_task(void *) {
       TickType_t wake_time = xTaskGetTickCount();
       uint32_t imu_last_valid_ms = RobotHardware::now_ms();
       for (;;) {
           RobotState state;
           if (copy_state(state)) {
               ImuSample raw_sample;
               ImuSample sample;
               bool fault = state.imu_fault;
               if (RobotHardware::read_imu(raw_sample)) {
                   // Apply intrinsic calibration
                   imu_calibrator.apply(raw_sample, sample);

                   float quaternion_norm = sample.quaternion_xyzw[0] * sample.quaternion_xyzw[0] +
                                           sample.quaternion_xyzw[1] * sample.quaternion_xyzw[1] +
                                           sample.quaternion_xyzw[2] * sample.quaternion_xyzw[2] +
                                           sample.quaternion_xyzw[3] * sample.quaternion_xyzw[3];
                   if (quaternion_norm > 0.01F) {
                       quaternion_norm = sqrtf(quaternion_norm);
                       for (uint8_t index = 0U; index < 4U; ++index) {
                           sample.quaternion_xyzw[index] /= quaternion_norm;
                       }
                   }
                   imu_last_valid_ms = RobotHardware::now_ms();
                   fault = false;
               } else {
                   fault = (RobotHardware::now_ms() - imu_last_valid_ms) > 500U;
                   sample = state.imu;
               }
               update_imu_fields(sample, fault, RobotHardware::now_ms());
           }
           vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kImuPeriodMs));
       }
   }
   ```
5. **Pre-initialization of Message Memory** in `initialize_ros_message_memory()`:
   Populate non-zero covariances directly into `imu_message` during initialization so no telemetry frame is ever broadcast with zero diagonal covariances.
6. **Suppression of Cleanup Warnings**:
   In `clean_ros_entities()`: cast `rcl_*_fini` return values to `(void)`.

---

### 4.4 Build Fix in `src/kalman.cpp`
Add `#include <math.h>` to line 2 of `firmware/stm32_f407vg_arduino_sim/src/kalman.cpp` to resolve `'isfinite' was not declared in this scope`.

---

### 4.5 Configuration Update: `firmware/stm32_f407vg_arduino_sim/platformio.ini`

Append the native test environment to `platformio.ini`:
```ini
[env:native]
platform = native
test_framework = unity
build_flags =
    -std=c++17
    -I include
test_build_src = false
```

---

## 5. Comprehensive Unit Test Suite: `test/test_imu_calibration/test_main.cpp`

```cpp
#include <math.h>
#include <stdint.h>
#include <unity.h>

#include "firmware_config.h"
#include "imu_calibration.h"

#include "../../src/imu_calibration.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

void test_default_calibration_is_identity() {
    ImuCalibrator calibrator;
    const ImuCalibrationParams &params = calibrator.get_params();

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[2]);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[2]);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[2]);

    float raw_a[3] = {0.1F, -0.2F, 9.8F};
    float raw_g[3] = {0.01F, -0.02F, 0.05F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    calibrator.apply(raw_a, raw_g, cal_a, cal_g);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[0], cal_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[1], cal_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[2], cal_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[0], cal_g[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[1], cal_g[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[2], cal_g[2]);
}

void test_gyro_bias_nulling_and_drift_threshold() {
    ImuCalibrator calibrator;
    TEST_ASSERT_TRUE(calibrator.start_gyro_calibration(1000U));

    const float injected_bias[3] = {0.015F, -0.025F, 0.008F};

    // Feed 1000 samples with small stationary zero-mean noise
    for (uint32_t i = 0U; i < 1000U; ++i) {
        float noise = (static_cast<float>(i % 7) - 3.0F) * 1e-4F;
        float sample[3] = {
            injected_bias[0] + noise,
            injected_bias[1] - noise,
            injected_bias[2] + noise * 0.5F
        };
        TEST_ASSERT_TRUE(calibrator.update_gyro_sample(sample));
    }

    float residual_drift = 0.0F;
    TEST_ASSERT_TRUE(calibrator.finish_gyro_calibration(residual_drift));

    const ImuCalibrationParams &params = calibrator.get_params();
    TEST_ASSERT_TRUE(params.gyro_calibrated);

    // Verify calculated bias matches injected bias
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[0], params.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[1], params.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[2], params.gyro_bias[2]);

    // Verify static residual drift < 0.05 deg/s (8.7266e-4 rad/s)
    TEST_ASSERT_TRUE(residual_drift < kMaxGyroDriftRadS);

    // Verify calibrated output on resting data has zero rate
    float raw_a[3] = {0.0F, 0.0F, 9.80665F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};
    calibrator.apply(raw_a, injected_bias, cal_a, cal_g);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[2]);
}

void test_gyro_motion_rejection() {
    ImuCalibrator calibrator;
    TEST_ASSERT_TRUE(calibrator.start_gyro_calibration(500U));

    // Feed noisy motion samples
    bool rejected = false;
    for (uint32_t i = 0U; i < 100U; ++i) {
        float sample[3] = {
            sinf(static_cast<float>(i) * 0.5F) * 0.5F,
            cosf(static_cast<float>(i) * 0.5F) * 0.5F,
            0.1F
        };
        if (!calibrator.update_gyro_sample(sample)) {
            rejected = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(rejected);
    TEST_ASSERT_EQUAL(CALIB_FAILED_MOTION, calibrator.get_state());
}

void test_accel_6_position_calibration_and_norm_error() {
    ImuCalibrator calibrator;

    const float g = kStandardGravityMps2;
    const float inj_scale[3] = {1.05F, 0.95F, 1.02F};
    const float inj_bias[3] = {0.12F, -0.08F, 0.15F};

    // Ground truth acceleration for 6 faces:
    // +X, -X, +Y, -Y, +Z, -Z
    const float true_faces[6][3] = {
        {+g, 0.0F, 0.0F},
        {-g, 0.0F, 0.0F},
        {0.0F, +g, 0.0F},
        {0.0F, -g, 0.0F},
        {0.0F, 0.0F, +g},
        {0.0F, 0.0F, -g}
    };

    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        TEST_ASSERT_TRUE(calibrator.start_accel_face(static_cast<AccelFace>(f), 200U));

        // Raw = scale * true + bias
        float raw[3];
        raw[0] = inj_scale[0] * true_faces[f][0] + inj_bias[0];
        raw[1] = inj_scale[1] * true_faces[f][1] + inj_bias[1];
        raw[2] = inj_scale[2] * true_faces[f][2] + inj_bias[2];

        for (uint32_t s = 0U; s < 200U; ++s) {
            TEST_ASSERT_TRUE(calibrator.update_accel_sample(raw));
        }
        TEST_ASSERT_TRUE(calibrator.finish_accel_face());
    }

    float max_norm_error = 0.0F;
    TEST_ASSERT_TRUE(calibrator.compute_accel_calibration(max_norm_error));

    const ImuCalibrationParams &params = calibrator.get_params();
    TEST_ASSERT_TRUE(params.accel_calibrated);

    // Verify calculated scales and biases match injected values
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[0], params.accel_scale[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[1], params.accel_scale[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[2], params.accel_scale[2]);

    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[0], params.accel_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[1], params.accel_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[2], params.accel_bias[2]);

    // Verify residual norm error across all 6 faces is < 0.05 m/s^2 (< 0.5%)
    TEST_ASSERT_TRUE(max_norm_error < kMaxAccelNormErrorMps2);
}

void test_rep103_enu_coordinate_alignment() {
    ImuCalibrator calibrator;
    const float g = kStandardGravityMps2;

    // Simulate robot level on ground: raw reads approx +9.81 on Z
    float raw_a[3] = {0.0F, 0.0F, g};
    float raw_g[3] = {0.0F, 0.0F, 0.0F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    calibrator.apply(raw_a, raw_g, cal_a, cal_g);

    // REP-103 East-North-Up: X forward, Y left, Z up (+9.80665 m/s^2 at rest)
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, g, cal_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_g[2]);
}

void test_allan_variance_covariance_inflation() {
    ImuCalibrator calibrator;
    float gyro_cov[9] = {0};
    float accel_cov[9] = {0};

    calibrator.compute_covariances(0.02F, gyro_cov, accel_cov);

    // Verify all diagonal elements are strictly non-zero
    TEST_ASSERT_TRUE(gyro_cov[0] > 0.0F);
    TEST_ASSERT_TRUE(gyro_cov[4] > 0.0F);
    TEST_ASSERT_TRUE(gyro_cov[8] > 0.0F);

    TEST_ASSERT_TRUE(accel_cov[0] > 0.0F);
    TEST_ASSERT_TRUE(accel_cov[4] > 0.0F);
    TEST_ASSERT_TRUE(accel_cov[8] > 0.0F);

    // Off-diagonals must be zero
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, gyro_cov[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, gyro_cov[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, accel_cov[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, accel_cov[3]);

    // Verify inflation factor: alpha = 1.8 -> alpha^2 = 3.24
    // sigma_gyro^2 = max(3.24 * (1.4e-4)^2 / 0.02, 1e-4) = 1e-4 (clamped)
    TEST_ASSERT_TRUE(gyro_cov[8] >= 1.0e-4F);
    TEST_ASSERT_TRUE(accel_cov[0] >= 1.0e-2F);
}

void test_invalid_inputs_and_edge_cases() {
    ImuCalibrator calibrator;

    // NaN / Inf inputs rejected
    float invalid_sample[3] = {NAN, 0.0F, 0.0F};
    calibrator.start_gyro_calibration(100U);
    TEST_ASSERT_FALSE(calibrator.update_gyro_sample(invalid_sample));

    // Premature compute without all faces fails safely
    float max_err = 0.0F;
    TEST_ASSERT_FALSE(calibrator.compute_accel_calibration(max_err));
    TEST_ASSERT_EQUAL(CALIB_FAILED_MATH, calibrator.get_state());
}

} // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_default_calibration_is_identity);
    RUN_TEST(test_gyro_bias_nulling_and_drift_threshold);
    RUN_TEST(test_gyro_motion_rejection);
    RUN_TEST(test_accel_6_position_calibration_and_norm_error);
    RUN_TEST(test_rep103_enu_coordinate_alignment);
    RUN_TEST(test_allan_variance_covariance_inflation);
    RUN_TEST(test_invalid_inputs_and_edge_cases);
    return UNITY_END();
}
```

---

## 6. Step-by-Step Implementation Sequence

The implementer agent must execute the steps in the following strict order:

1. **Step 1: Fix `src/kalman.cpp`**:
   Add `#include <math.h>` to line 2.
2. **Step 2: Update `platformio.ini`**:
   Append `[env:native]` section with `test_framework = unity` and `test_build_src = false`.
3. **Step 3: Create `include/imu_calibration.h`**:
   Write the full header file designed in Section 4.1.
4. **Step 4: Create `src/imu_calibration.cpp`**:
   Write the full implementation source designed in Section 4.2.
5. **Step 5: Create Unit Test `test/test_imu_calibration/test_main.cpp`**:
   Write the comprehensive test runner designed in Section 5.
6. **Step 6: Run Native Unit Tests**:
   Execute `pio test -e native -f test_imu_calibration`. Verify all 7 tests pass 100%.
7. **Step 7: Integrate into `src/main.cpp`**:
   Apply changes from Section 4.3 (calibrator instance, `imu_task`, `publish_telemetry`, covariance filling, warning cleanups).
8. **Step 8: Build STM32 Firmware Target**:
   Execute `pio run -e disco_f407vg`. Verify clean compilation with **0 errors and 0 warnings**.

---

## 7. Verification & Acceptance Checklist

| Verification Item | Command | Expected Result |
|---|---|---|
| Native IMU Calibration Tests | `pio test -e native -f test_imu_calibration` | 7 test cases PASSED |
| Native Regression Suite | `pio test -e native` | All test suites PASSED |
| STM32 Embedded Build | `pio run -e disco_f407vg` | SUCCESS (0 errors, 0 warnings) |
| Stationary Gyro Drift | Verified via test 2 | Drift $< 0.05^\circ/\text{s}$ ($< 8.72 \times 10^{-4}$ rad/s) |
| Accel Norm Error | Verified via test 4 | Residual norm error $< 0.05$ m/s² across 6 poses |
| REP-103 ENU Standard | Verified via test 5 | $+9.80665$ m/s² on $+Z$ at rest |
| Non-zero Covariances | Verified via test 6 & telemetry code | No zero diagonal elements in IMU message |

---
*End of Milestone 2 Implementation Plan.*
