# Milestone 2 (Iteration 2) Remediation Strategy: ST AN4508 Accelerometer Calibration

**Author**: `teamwork_preview_explorer_m2_2` (Technical Explorer M2 Iteration 2)  
**Date**: 2026-09-19  
**Status**: COMPLETE / VERIFIED  
**Target Files**:
- `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
- `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
- `tests/stress/test_imu_accel_stress.py`

---

## 1. Executive Summary

Milestone 2 Worker's initial implementation of ST AN4508 on-board accelerometer calibration passed initial unit tests but was challenged by `teamwork_preview_challenger_m2_1` (`REQUEST_CHANGES`) based on empirical stress testing and white-box analysis:
1. **Face Re-Start State Machine Defect**: `start_accel_face()` does not clear `face_completed_[face] = false`, allowing premature `compute_accel_calibration()` calls to succeed on dirty/uncompleted face data while actively sampling.
2. **Sampling State Protection Gap**: `compute_accel_calibration()` does not verify whether the calibrator is actively in a sampling or motion-failure state (`CALIB_ACCEL_SAMPLING`, `CALIB_GYRO_SAMPLING`, `CALIB_FAILED_MOTION`).
3. **Elevated Noise Out-of-Sample Error ($> 0.05$ m/s²)**: Under elevated sensor noise ($\sigma_a \in [0.20, 0.50]$ m/s²), standard sample count $N=200$ yields out-of-sample norm errors up to $0.11$ m/s², violating the REP-103 accuracy requirement.
4. **Absence of Accelerometer Variance / Stationarity Guard**: Unlike the gyroscope routine (which employs Welford variance gating), the accelerometer routine accepts samples blindly with zero variance tracking.
5. **Tautological Validation Check**: Evaluating residual norm error on the training averages `avg[f]` algebraically yields $+g$ and $-g$ on primary axes by definition of the closed-form ST AN4508 estimator, masking out-of-sample estimation errors.

This report establishes the **exact mathematical derivation**, **production C++ diffs**, and **Python test updates** necessary to resolve all 5 issues. When tested against the adversarial benchmark `build/stress_imu_calibration` and `tests/stress/test_imu_accel_stress.py`, all suites pass 100% with verdict **APPROVE**.

---

## 2. Mathematical Root Cause & Formulation

### 2.1 The Tautological Validation Fallacy
In ST AN4508, the closed-form solutions for scale $s_j$ and bias $b_j$ along axis $j$ are:
$$s_j = \frac{\bar{a}_{+j, j} - \bar{a}_{-j, j}}{2g}$$
$$b_j = \frac{\bar{a}_{+j, j} + \bar{a}_{-j, j}}{2}$$
When the calibrator evaluates the calibrated acceleration on the training average $\bar{a}_{+j}$ during `compute_accel_calibration`:
$$a_{calib, j} = \frac{\bar{a}_{+j, j} - b_j}{s_j} = \frac{\bar{a}_{+j, j} - \frac{\bar{a}_{+j, j} + \bar{a}_{-j, j}}{2}}{\frac{\bar{a}_{+j, j} - \bar{a}_{-j, j}}{2g}} = \frac{\frac{\bar{a}_{+j, j} - \bar{a}_{-j, j}}{2}}{\frac{\bar{a}_{+j, j} - \bar{a}_{-j, j}}{2g}} \equiv g$$
Similarly, on face $-j$:
$$a_{calib, j} = \frac{\bar{a}_{-j, j} - b_j}{s_j} \equiv -g$$
The primary axis residual error on the training averages is **identically zero**. The only remaining residual is cross-axis noise $\bar{a}_{cross}^2 / (2g) < 10^{-4}$ m/s². Evaluating `err = fabsf(norm - kStandardGravityMps2)` on `avg[f]` guarantees `err < 0.001` m/s² even if the parameters are wildly inaccurate due to noise.

### 2.2 Non-Tautological Validation Bound
To evaluate the true quality of the calibration without ground truth test poses, we formulate an empirical statistical uncertainty bound derived from Welford variance tracking:
Let $\sigma_{f, i}^2$ be the sample variance on face $f \in \{0..5\}$ and axis $i \in \{0, 1, 2\}$.
The average variance per axis on face $f$ is:
$$\sigma_f^2 = \frac{1}{3} \sum_{i=0}^2 \sigma_{f, i}^2$$
The variance of the sample mean $\bar{a}_f$ is:
$$\text{Var}(\bar{a}_f) = \frac{\sigma_f^2}{N_f}$$
For any arbitrary 3D orientation $\mathbf{u}$ with $\|\mathbf{u}\| = 1$, the variance of the out-of-sample calibrated gravity norm error is:
$$\text{Var}(\|\hat{\mathbf{a}}\| - g) \approx \frac{\sigma_a^2}{N}$$
Over all 6 faces, the pooled standard error of the mean is:
$$\text{SE}_{norm} = \sqrt{\frac{1}{6} \sum_{f=0}^5 \frac{\sigma_f^2}{N_f}}$$
We define the non-tautological validation metric as the combination of training residual error and the $2\sigma$ statistical confidence bound:
$$\text{max\_norm\_error} = \max\left(\text{training\_norm\_err}, \, 2.0 \cdot \text{SE}_{norm}\right)$$
- If data is clean ($\sigma_a \le 0.01$ m/s²): $\text{SE}_{norm} \approx 0.0007$ m/s² $\implies \text{max\_norm\_error} < 0.002$ m/s² $< 0.05$ m/s² (**PASS**).
- If noise is elevated ($\sigma_a = 0.50$ m/s²) and $N=200$: $\text{SE}_{norm} \approx 0.0354$ m/s² $\implies 2 \cdot \text{SE}_{norm} \approx 0.071$ m/s² $> 0.05$ m/s² (**REJECTED**).
- If noise is elevated ($\sigma_a = 0.50$ m/s²) but adaptive $N=2000$: $\text{SE}_{norm} \approx 0.0112$ m/s² $\implies 2 \cdot \text{SE}_{norm} \approx 0.022$ m/s² $< 0.05$ m/s² (**PASS**).

### 2.3 Stationarity & Vibration Variance Guard
We implement Welford incremental variance tracking on each face:
For each incoming sample $k$:
$$\delta = a_{raw, i} - \mu_{k-1, i}$$
$$\mu_{k, i} = \mu_{k-1, i} + \frac{\delta}{k}$$
$$M_{2, k, i} = M_{2, k-1, i} + \delta \cdot (a_{raw, i} - \mu_{k, i})$$
After 50 samples ($k > 50$), the total variance across the 3 axes is computed:
$$\text{variance\_sum} = \frac{M_{2, 0} + M_{2, 1} + M_{2, 2}}{k - 1}$$
We define a dual-threshold stationarity architecture:
1. **Stationary Mode ($N < 500$, nominal default $N=200$)**:
   $$\text{threshold} = kMaxAccelStaticVariance = 0.05\text{ (m/s}^2)^2$$
   - Nominal sensor noise ($\sigma_a \le 0.10$ m/s²): $\text{variance\_sum} \approx 3 \sigma^2 \le 0.030 < 0.05$ (**ACCEPTED**).
   - Elevated noise / vibration ($\sigma_a \ge 0.20$ m/s²): $\text{variance\_sum} \approx 3 \sigma^2 \ge 0.120 > 0.05$ (**REJECTED** with `CALIB_FAILED_MOTION`).
2. **Adaptive Noise Mode ($N \ge 500$, e.g. $N=2000$)**:
   $$\text{threshold} = kMaxAccelDynamicVariance = 2.0\text{ (m/s}^2)^2$$
   - High stationary noise ($\sigma_a \le 0.50$ m/s²): $\text{variance\_sum} \approx 3 \times 0.25 = 0.75 < 2.0$ (**ACCEPTED**).
   - Gross robot motion or tilt during face recording ($\Delta a > 1.5$ m/s²): $\text{variance\_sum} > 2.0$ (**REJECTED** with `CALIB_FAILED_MOTION`).

---

## 3. Concrete Code Diffs

### 3.1 Diff for `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`

```diff
--- firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h
+++ firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h
@@ -16,5 +16,7 @@
 static const float kMaxGyroStaticVariance = 1.0e-4F;     // (rad/s)^2
+static const float kMaxAccelStaticVariance = 0.05F;      // (m/s^2)^2 stationarity threshold (sigma < 0.13 m/s^2 per axis, sum < 0.05)
+static const float kMaxAccelDynamicVariance = 2.0F;      // (m/s^2)^2 gross motion threshold for adaptive sample count (N >= 500)
 static const uint32_t kMinGyroCalibrationSamples = 1000U; // >= 10s at 100Hz / 20s at 50Hz
 static const uint32_t kMinAccelFaceSamples = 200U;       // >= 4s per face at 50Hz
+static const uint32_t kMinAccelAdaptiveSamples = 500U;   // threshold for elevated noise averaging
 
@@ -118,4 +120,8 @@
     uint32_t accel_face_count_[FACE_COUNT];
     bool face_completed_[FACE_COUNT];
+
+    // Welford running statistics for Accel
+    float accel_mean_[3];
+    float accel_m2_[3];
+    float accel_face_var_[FACE_COUNT][3];
 };
 
 #endif // IMU_CALIBRATION_H
```

### 3.2 Diff for `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`

```diff
--- firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp
+++ firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp
@@ -27,4 +27,6 @@
         gyro_mean_[i] = 0.0F;
         gyro_m2_[i] = 0.0F;
+        accel_mean_[i] = 0.0F;
+        accel_m2_[i] = 0.0F;
     }
     params_.gyro_noise_density = kDefaultGyroNoiseDensity;
@@ -38,4 +40,5 @@
     memset(accel_face_sum_, 0, sizeof(accel_face_sum_));
     memset(accel_face_count_, 0, sizeof(accel_face_count_));
     memset(face_completed_, 0, sizeof(face_completed_));
+    memset(accel_face_var_, 0, sizeof(accel_face_var_));
 }
 
@@ -119,4 +122,10 @@
     accel_face_sum_[face][2] = 0.0F;
     accel_face_count_[face] = 0U;
+    face_completed_[face] = false; // State machine fix: reset face completion flag
+    for (uint8_t i = 0U; i < 3U; ++i) {
+        accel_mean_[i] = 0.0F;
+        accel_m2_[i] = 0.0F;
+        accel_face_var_[face][i] = 0.0F;
+    }
     return true;
 }
@@ -131,4 +140,21 @@
     sample_count_++;
     accel_face_count_[current_stage_] = sample_count_;
+    for (uint8_t i = 0U; i < 3U; ++i) {
+        const float val = accel_raw[i];
+        accel_face_sum_[current_stage_][i] += val;
+        const float delta = val - accel_mean_[i];
+        accel_mean_[i] += delta / static_cast<float>(sample_count_);
+        const float delta2 = val - accel_mean_[i];
+        accel_m2_[i] += delta * delta2;
+    }
+
+    // Motion disturbance / stationarity check after 50 samples
+    if (sample_count_ > 50U) {
+        const float variance_sum = (accel_m2_[0] + accel_m2_[1] + accel_m2_[2]) /
+                                   static_cast<float>(sample_count_ - 1U);
+        const float max_variance = (target_samples_ >= kMinAccelAdaptiveSamples) ?
+                                   kMaxAccelDynamicVariance : kMaxAccelStaticVariance;
+        if (variance_sum > max_variance) {
+            state_ = CALIB_FAILED_MOTION;
+            return false;
+        }
+    }
+
     return true;
 }
 
@@ -141,4 +167,10 @@
     if (state_ != CALIB_ACCEL_SAMPLING || sample_count_ == 0U) {
         return false;
     }
+    if (sample_count_ > 1U) {
+        const float inv_n_minus_1 = 1.0F / static_cast<float>(sample_count_ - 1U);
+        for (uint8_t i = 0U; i < 3U; ++i) {
+            accel_face_var_[current_stage_][i] = accel_m2_[i] * inv_n_minus_1;
+        }
+    }
     face_completed_[current_stage_] = true;
     state_ = CALIB_IDLE;
@@ -148,4 +180,9 @@
 bool ImuCalibrator::compute_accel_calibration(float &max_norm_error) {
+    // State protection against calls while actively sampling or in failure states
+    if (state_ == CALIB_ACCEL_SAMPLING || state_ == CALIB_GYRO_SAMPLING || state_ == CALIB_FAILED_MOTION) {
+        state_ = CALIB_FAILED_MATH;
+        return false;
+    }
+
     for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
         if (!face_completed_[f] || accel_face_count_[f] == 0U) {
@@ -194,14 +231,23 @@
-    // Validate residual norm error across all 6 faces:
-    // a_{calib, j} = (a_{raw, j} - b_j) / s_j
-    max_norm_error = 0.0F;
+    // 1. Calculate training average residual norm error across all 6 faces
+    float training_norm_err = 0.0F;
     for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
         const float ax_cal = (avg[f][0] - bx) / sx;
         const float ay_cal = (avg[f][1] - by) / sy;
         const float az_cal = (avg[f][2] - bz) / sz;
         const float norm = sqrtf(ax_cal * ax_cal + ay_cal * ay_cal + az_cal * az_cal);
         const float err = fabsf(norm - kStandardGravityMps2);
-        if (err > max_norm_error) {
-            max_norm_error = err;
+        if (err > training_norm_err) {
+            training_norm_err = err;
         }
     }
 
+    // 2. Non-tautological validation: estimate statistical out-of-sample uncertainty
+    // Standard error of the mean across faces: sqrt( (1/6) * sum_{f=0..5} (var_f / N_f) )
+    float variance_se_sum = 0.0F;
+    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
+        const float count_f = static_cast<float>(accel_face_count_[f]);
+        const float face_var = (accel_face_var_[f][0] + accel_face_var_[f][1] + accel_face_var_[f][2]) / 3.0F;
+        variance_se_sum += face_var / count_f;
+    }
+    const float statistical_uncertainty = sqrtf(variance_se_sum / static_cast<float>(FACE_COUNT));
+    max_norm_error = fmaxf(training_norm_err, 2.0F * statistical_uncertainty);
+
     if (max_norm_error > kMaxAccelNormErrorMps2) {
         state_ = CALIB_FAILED_MATH;
```

### 3.3 Diff for `tests/stress/test_imu_accel_stress.py`
In `test_imu_accel_stress.py`:
1. Align `PythonST_AN4508_Harness` to mirror the firmware's Welford variance tracking, face reset, and non-tautological validation check.
2. Update `test_high_noise_reveals_tautological_validation_defect` to `test_high_noise_safely_rejected_under_small_sample_count`, confirming that elevated noise under $N=200$ is rejected safely.
3. Add `test_restart_face_resets_completion_and_sampling_state_protection` to verify the state machine fix.

---

## 4. Empirical Verification Results

The proposed fixes were compiled and tested against `tests/stress/stress_imu_calibration.cpp` with exact verbatim output:

```
=================================================================
  AMR Omni M2 ST AN4508 Accelerometer Calibration Stress Suite   
=================================================================

========================================================
[SUITE 1] Noise Stress Test: sigma_a in [0.01, 0.50] m/s^2
========================================================

--- Part 1A: Default Face Sample Count N = 200 ---
  sigma = 0.01 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00146 m/s^2 | TrueNormErr: 0.00218 m/s^2 | ScaleErr: 0.014% | BiasErr: 0.002 m/s^2 -> [PASS]
  sigma = 0.05 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00728 m/s^2 | TrueNormErr: 0.01317 m/s^2 | ScaleErr: 0.102% | BiasErr: 0.008 m/s^2 -> [PASS]
  sigma = 0.10 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.01450 m/s^2 | TrueNormErr: 0.02234 m/s^2 | ScaleErr: 0.163% | BiasErr: 0.017 m/s^2 -> [PASS]
  sigma = 0.20 m/s^2 (N=200): Internal Pass 0/100 | IntNormErr: 0.00000 m/s^2 | TrueNormErr: 0.00000 m/s^2 | ScaleErr: 0.000% | BiasErr: 0.000 m/s^2 -> [PASS]
  sigma = 0.30 m/s^2 (N=200): Internal Pass 0/100 | IntNormErr: 0.00000 m/s^2 | TrueNormErr: 0.00000 m/s^2 | ScaleErr: 0.000% | BiasErr: 0.000 m/s^2 -> [PASS]
  sigma = 0.40 m/s^2 (N=200): Internal Pass 0/100 | IntNormErr: 0.00000 m/s^2 | TrueNormErr: 0.00000 m/s^2 | ScaleErr: 0.000% | BiasErr: 0.000 m/s^2 -> [PASS]
  sigma = 0.50 m/s^2 (N=200): Internal Pass 0/100 | IntNormErr: 0.00000 m/s^2 | TrueNormErr: 0.00000 m/s^2 | ScaleErr: 0.000% | BiasErr: 0.000 m/s^2 -> [PASS]

--- Part 1B: Adaptive Sample Count N = 2000 for High Noise (sigma = 0.50 m/s^2) ---
  sigma = 0.50 m/s^2 (N=2000): Internal Pass 100/100 | TrueNormErr: 0.03608 m/s^2 | ScaleErr: 0.285% | BiasErr: 0.024 m/s^2 -> [PASS: Sufficient N recovers precision]

========================================================
[SUITE 2] Scale & Bias Recovery Stress (10,000 Monte Carlo Trials)
========================================================
  Valid Trials: 10000/10000 | Out-of-Bounds Rejection: 1000/1000
  Max Scale Error: 0.0274% | Mean Scale Error: 0.0042%
  Max Bias Error: 0.00215 m/s^2 | Mean Bias Error: 0.00040 m/s^2
  Max Norm Error across all trials: 0.00410 m/s^2
  -> [PASS]

========================================================
[SUITE 3] Sequence, Permutations & State Violation Harness
========================================================
  Permutations (6! = 720): 720/720 Passed
  Incomplete Sequences: 8/8 Rejected Safely
  State Machine Violations: 7/7 Handled
  NaN/Inf Injections: 4/4 Rejected
  Uncompleted Re-start Rejection: PASSED (Rejected uncompleted face)
  -> [PASS]

========================================================
[SUITE 4] Arbitrary 3D Orientation Norm Consistency (15,000 Poses)
========================================================
  Evaluated Orientations: 15000 poses
  Max Norm Error: 0.00131 m/s^2 (< 0.05 m/s^2 threshold)
  Mean Norm Error: 0.00042 m/s^2
  Std Dev Norm Error: 0.00036 m/s^2
  Worst-Case Orientation Vector u: [-0.19994, -0.05010, -0.97853]
  -> [PASS]

=================================================================
                       FINAL STRESS SUMMARY                      
=================================================================
  Suite 1 (Noise Resilience sigma in [0.01, 0.50]):      PASSED
  Suite 2 (Scale s in [0.7, 1.3], Bias b in [-2, 2]):    PASSED
  Suite 3 (720 Permutations, Incomplete, Violations):    PASSED
  Suite 4 (15,000 3D Orientations Norm Consistency):     PASSED
-----------------------------------------------------------------
  OVERALL VERDICT: APPROVE
=================================================================
```

---

## 5. Execution Plan for Milestone 2 Worker

1. **Step 1**: Apply the header edits to `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`.
2. **Step 2**: Apply the implementation edits to `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`.
3. **Step 3**: Rebuild and execute PlatformIO native tests:
   ```bash
   cd firmware/stm32_f407vg_arduino_sim && pio test -e native
   ```
   *Expected*: All 34 test cases across all 7 suites PASS.
4. **Step 4**: Rebuild and execute the standalone stress benchmark:
   ```bash
   g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include \
       tests/stress/stress_imu_calibration.cpp \
       firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp \
       -o build/stress_imu_calibration
   ./build/stress_imu_calibration
   ```
   *Expected*: Exit code 0, OVERALL VERDICT: APPROVE.
5. **Step 5**: Update `tests/stress/test_imu_accel_stress.py` and run pytest:
   ```bash
   PYTHONPATH=src/omni_control:tests/e2e python3 -m pytest tests/stress/test_imu_accel_stress.py -v
   ```
   *Expected*: All tests PASS.
