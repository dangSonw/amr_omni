# Milestone 2 (M2) Adversarial Challenger Report: IMU Accelerometer Calibration

**Agent**: `teamwork_preview_challenger_m2_1` (Challenger 1 M2 — Empirical Challenger)  
**Role**: Empirical Challenger / Critic & Specialist  
**Working Directory**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1`  
**Target Recipient**: Orchestrator M2 (`709d5506-1905-49c5-bf69-8e756d885098`)  
**Verdict**: **REQUEST_CHANGES**  

---

## 1. Observation

### 1.1 Firmware Code Inspection
1. **File**: `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
   - Line 15: `static const float kMaxAccelNormErrorMps2 = 0.05F; // < 0.5% of 1g`
   - Line 18: `static const uint32_t kMinAccelFaceSamples = 200U; // >= 4s per face at 50Hz`
2. **File**: `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
   - **Tautological Validation Check (Lines 196–211)**:
     ```cpp
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
     ```
     Because $sx = (\bar{a}_{+x} - \bar{a}_{-x}) / (2g)$ and $bx = (\bar{a}_{+x} + \bar{a}_{-x}) / 2$, the term $(\bar{a}_{+x} - bx)/sx$ is algebraically identical to $+g$, and $(\bar{a}_{-x} - bx)/sx \equiv -g$. The primary axis residual error on training averages is identically zero, meaning `max_norm_error` evaluated on `avg[f]` is virtually guaranteed to be $< 0.001$ m/s² regardless of how noisy or inaccurate the scale and bias estimates actually are.
   - **Face Re-Start State Machine Bug (Lines 109–122 & 148–154)**:
     ```cpp
     bool ImuCalibrator::start_accel_face(AccelFace face, uint32_t target_samples) {
         if (face >= FACE_COUNT || target_samples < 20U) {
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
         // DEFECT: face_completed_[face] is NOT reset to false!
         return true;
     }
     ```
     If a face was previously completed (e.g. from an earlier calibration cycle), and `start_accel_face()` is called again, `face_completed_[face]` remains `true`. If `update_accel_sample()` is called with even a single sample, and then `compute_accel_calibration()` is triggered while `state_ == CALIB_ACCEL_SAMPLING`, `compute_accel_calibration()` does not check if the calibrator is actively sampling, sees `face_completed_[f] == true` and `accel_face_count_[f] > 0`, and calculates calibration parameters from incomplete sampling data, returning `CALIB_SUCCESS`!
   - **Absence of Accelerometer Motion / Vibration Gating (Lines 124–137)**:
     Unlike `update_gyro_sample()` (which maintains Welford incremental variance and rejects frames if variance exceeds `kMaxGyroStaticVariance`), `update_accel_sample()` performs raw summation with zero variance or stationarity checks.

---

### 1.2 Standalone C++ Benchmark Execution (`build/stress_imu_calibration`)
**Command**:
```bash
g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include \
    tests/stress/stress_imu_calibration.cpp \
    firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp \
    -o build/stress_imu_calibration
./build/stress_imu_calibration
```
**Verbatim Output**:
```
=================================================================
  AMR Omni M2 ST AN4508 Accelerometer Calibration Stress Suite   
=================================================================

========================================================
[SUITE 1] Noise Stress Test: sigma_a in [0.01, 0.50] m/s^2
========================================================

--- Part 1A: Default Face Sample Count N = 200 ---
  sigma = 0.01 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00000 m/s^2 | TrueNormErr: 0.00218 m/s^2 | ScaleErr: 0.014% | BiasErr: 0.002 m/s^2 -> [PASS]
  sigma = 0.05 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00001 m/s^2 | TrueNormErr: 0.01317 m/s^2 | ScaleErr: 0.102% | BiasErr: 0.008 m/s^2 -> [PASS]
  sigma = 0.10 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00006 m/s^2 | TrueNormErr: 0.02234 m/s^2 | ScaleErr: 0.163% | BiasErr: 0.017 m/s^2 -> [PASS]
  sigma = 0.20 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00021 m/s^2 | TrueNormErr: 0.05291 m/s^2 | ScaleErr: 0.421% | BiasErr: 0.039 m/s^2 -> [FAIL: Out-of-sample norm error > 0.05 m/s^2]
  sigma = 0.30 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00043 m/s^2 | TrueNormErr: 0.06676 m/s^2 | ScaleErr: 0.476% | BiasErr: 0.050 m/s^2 -> [FAIL: Out-of-sample norm error > 0.05 m/s^2]
  sigma = 0.40 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00076 m/s^2 | TrueNormErr: 0.10008 m/s^2 | ScaleErr: 0.544% | BiasErr: 0.071 m/s^2 -> [FAIL: Out-of-sample norm error > 0.05 m/s^2]
  sigma = 0.50 m/s^2 (N=200): Internal Pass 100/100 | IntNormErr: 0.00103 m/s^2 | TrueNormErr: 0.10969 m/s^2 | ScaleErr: 0.730% | BiasErr: 0.086 m/s^2 -> [FAIL: Out-of-sample norm error > 0.05 m/s^2]

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
  Uncompleted Re-start Rejection: FAILED (Firmware accepted uncompleted face while actively SAMPLING!)
  -> [FAIL]

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
  Suite 1 (Noise Resilience sigma in [0.01, 0.50]):      FAILED
  Suite 2 (Scale s in [0.7, 1.3], Bias b in [-2, 2]):    PASSED
  Suite 3 (720 Permutations, Incomplete, Violations):    FAILED
  Suite 4 (15,000 3D Orientations Norm Consistency):     PASSED
-----------------------------------------------------------------
  OVERALL VERDICT: REQUEST_CHANGES
=================================================================
```

---

### 1.3 Python Adversarial Pytest Suite (`tests/stress/test_imu_accel_stress.py`)
**Command**:
```bash
PYTHONPATH=src/omni_control:tests/e2e python3 -m pytest tests/stress/test_imu_accel_stress.py -v
```
**Verbatim Output**:
```
tests/stress/test_imu_accel_stress.py::TestNoiseResilienceSuite::test_low_and_medium_noise_convergence[0.01] PASSED [  5%]
tests/stress/test_imu_accel_stress.py::TestNoiseResilienceSuite::test_low_and_medium_noise_convergence[0.05] PASSED [ 10%]
tests/stress/test_imu_accel_stress.py::TestNoiseResilienceSuite::test_low_and_medium_noise_convergence[0.1] PASSED [ 15%]
tests/stress/test_imu_accel_stress.py::TestNoiseResilienceSuite::test_high_noise_reveals_tautological_validation_defect PASSED [ 21%]
tests/stress/test_imu_accel_stress.py::TestNoiseResilienceSuite::test_sufficient_sample_count_recovers_accuracy_under_high_noise PASSED [ 26%]
tests/stress/test_imu_accel_stress.py::TestScaleBiasRecoverySuite::test_monte_carlo_scale_bias_recovery PASSED [ 31%]
tests/stress/test_imu_accel_stress.py::TestScaleBiasRecoverySuite::test_out_of_bounds_scale_rejection[0.69] PASSED [ 36%]
tests/stress/test_imu_accel_stress.py::TestScaleBiasRecoverySuite::test_out_of_bounds_scale_rejection[1.31] PASSED [ 42%]
tests/stress/test_imu_accel_stress.py::TestScaleBiasRecoverySuite::test_out_of_bounds_bias_rejection[-3.1] PASSED [ 47%]
tests/stress/test_imu_accel_stress.py::TestScaleBiasRecoverySuite::test_out_of_bounds_bias_rejection[3.1] PASSED [ 52%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_all_720_permutations_succeed PASSED [ 57%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[0] PASSED [ 63%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[1] PASSED [ 68%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[2] PASSED [ 73%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[3] PASSED [ 78%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[4] PASSED [ 84%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_incomplete_face_sequence_rejection[5] PASSED [ 89%]
tests/stress/test_imu_accel_stress.py::TestSequencesAndStateViolationsSuite::test_nan_and_inf_sample_rejection PASSED [ 94%]
tests/stress/test_imu_accel_stress.py::TestArbitrary3DOrientationsSuite::test_gravity_norm_invariance_under_arbitrary_rotations PASSED [100%]
============================== 19 passed in 2.39s ==============================
```

---

## 2. Logic Chain

1. **Failure of Norm Error Invariant Under Elevated Noise ($\sigma_a \in [0.20, 0.50]$ m/s²)**:
   - In Observation 1.1, `kMinAccelFaceSamples` defaults to 200 samples.
   - For noise $\sigma_a \in [0.20, 0.50]$ m/s² (realistic under chassis motor hum or mechanical vibration), the standard error of the mean for each face is $\sigma_{\bar{a}} = \sigma_a / \sqrt{N}$. At $\sigma_a = 0.50$ m/s² and $N = 200$, $\sigma_{\bar{a}} = 0.0354$ m/s².
   - Estimating the bias as $b = (\bar{a}_{+x} + \bar{a}_{-x})/2$ yields bias standard deviation $\sigma_b = \sigma_{\bar{a}} / \sqrt{2} = 0.025$ m/s². Over 100 trials, 2-sigma to 3-sigma deviations produce bias error $\Delta b \approx 0.086$ m/s² and scale error $\Delta s \approx 0.73\%$.
   - When calibrated parameters are applied to true physical gravity, the resulting calibrated norm error reaches **$0.10969$ m/s²** (Observation 1.2, Suite 1), which directly violates the acceptance criterion ($\|a_{calib}\| - 9.80665 < 0.05$ m/s²).

2. **Tautological Validation Loop in `compute_accel_calibration()`**:
   - In Observation 1.1, lines 196–206 evaluate `ax_cal = (avg[f][0] - bx) / sx`.
   - By mathematical substitution:
     $$\frac{avg[\text{FACE\_POS\_X}][0] - bx}{sx} = \frac{\bar{a}_{+x} - \frac{\bar{a}_{+x} + \bar{a}_{-x}}{2}}{\frac{\bar{a}_{+x} - \bar{a}_{-x}}{2g}} = \frac{\frac{\bar{a}_{+x} - \bar{a}_{-x}}{2}}{\frac{\bar{a}_{+x} - \bar{a}_{-x}}{2g}} \equiv g$$
   - The primary axis error is algebraically zero. The only remaining error is cross-axis noise $\Delta_{cross}^2 / (2g)$. For $\Delta_{cross} \approx 0.035$ m/s², $\Delta^2 / (2g) \approx 6 \times 10^{-5}$ m/s².
   - Consequently, `compute_accel_calibration` computed `IntNormErr = 0.00103` m/s² even when the true out-of-sample error was $0.10969$ m/s², giving a false `CALIB_SUCCESS` confirmation (Observation 1.2 & 1.3).

3. **Face Re-Start State Machine Flaw**:
   - In Observation 1.1, `start_accel_face()` clears `accel_face_sum_` and `accel_face_count_`, but does **not** clear `face_completed_[face] = false;`.
   - In Observation 1.2 (Suite 3), when face 0 was re-started and fed only 1 sample without calling `finish_accel_face()`, calling `compute_accel_calibration()` succeeded with `CALIB_SUCCESS` (state 4) while the calibrator was actively sampling (`CALIB_ACCEL_SAMPLING`).
   - This allows corrupted, premature, or aborted face recordings to bypass completion checks and silently corrupt the calibration parameters in production.

4. **Remediation Feasibility**:
   - In Observation 1.2 (Part 1B) and Observation 1.3, when sample count is scaled appropriately (e.g. $N = 2000$ at $\sigma_a = 0.50$ m/s²), the standard error is suppressed and the true out-of-sample norm error drops to **$0.03608$ m/s²** ($< 0.05$ m/s²).
   - Furthermore, Suite 2 proved that the underlying closed-form solver is highly accurate: under nominal noise, scale recovery error is bounded by $0.0274\%$ ($< 0.05\%$) and bias error is bounded by $0.00215$ m/s² ($< 0.005$ m/s²).
   - Suite 4 proved that under valid calibration, gravity norm consistency is preserved across 15,000 arbitrary 3D orientations (worst-case error $0.00131$ m/s²).

---

## 3. Caveats

1. **Physical MEMS Sensor Noise Floor**:
   - Typical stationary MEMS accelerometers (e.g. LSM6DSO / BMI088) exhibit noise density $N_a \approx 1.9 \times 10^{-3}$ m/s²/$\sqrt{\text{Hz}}$, which at 50 Hz corresponds to $\sigma_a \approx 0.01$ m/s². At this nominal noise floor, $N = 200$ samples is sufficient and yields norm error $0.00218$ m/s² (Suite 1). The failure occurs when external vibrations (robot fans, motors, or rough surfaces) drive $\sigma_a \ge 0.20$ m/s².
2. **Flash Persistence**:
   - Flash commitment (`CMD_CALIB_FLASH_COMMIT`) is scheduled for Milestone 4 and was not in scope for M2.

---

## 4. Conclusion

**Verdict: REQUEST_CHANGES**

The M2 implementation of the ST AN4508 accelerometer calibration routine is mathematically sound in ideal conditions, but exhibits two critical defects that must be resolved before production approval:

### Required Changes for Implementation Worker M2:
1. **Fix Face Re-Start State Machine Bug (`firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`)**:
   - In `start_accel_face(AccelFace face, uint32_t target_samples)`:
     Add: `face_completed_[face] = false;`
   - In `compute_accel_calibration(float &max_norm_error)`:
     Add check: `if (state_ == CALIB_ACCEL_SAMPLING || state_ == CALIB_GYRO_SAMPLING) { state_ = CALIB_FAILED_MATH; return false; }`
2. **Add Accelerometer Stationarity / Variance Guard**:
   - Implement sample variance tracking (or check $\sigma_a < 0.15$ m/s²) during face sampling, analogous to `kMaxGyroStaticVariance` in the gyro routine.
   - If variance is elevated, either reject the face with `CALIB_FAILED_MOTION` or recommend/require $N \ge 1000$ samples.
3. **Enhance Norm Error Validation in `compute_accel_calibration`**:
   - Do not rely solely on the self-fitted training averages `avg[f]`. Validate calibration by evaluating the standard error of the mean or testing residual norm variance $\sqrt{\sum \sigma_f^2 / N} < 0.05$.

---

## 5. Verification Method

To independently verify these findings:

1. **Run Standalone Empirical Stress Benchmark**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include \
       tests/stress/stress_imu_calibration.cpp \
       firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp \
       -o build/stress_imu_calibration
   ./build/stress_imu_calibration
   ```
   *Current Result*: Exits with code 1; fails Suite 1 (elevated noise) and Suite 3 (uncompleted re-start bug).

2. **Run Pytest Adversarial Stress Suite**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   PYTHONPATH=src/omni_control:tests/e2e python3 -m pytest tests/stress/test_imu_accel_stress.py -v
   ```
   *Current Result*: 19 passed (demonstrates mathematical breakdown under high noise and uncompleted state transitions).

3. **Invalidation Conditions for Approval**:
   - `start_accel_face` must clear `face_completed_[face] = false`.
   - `compute_accel_calibration` must reject when called during active sampling (`CALIB_ACCEL_SAMPLING`).
   - Out-of-sample calibrated norm error must remain $< 0.05$ m/s² across $\sigma_a \in [0.01, 0.50]$ m/s² by enforcing sufficient sample count or stationary variance filtering.
