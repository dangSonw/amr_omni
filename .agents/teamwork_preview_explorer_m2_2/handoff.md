# Handoff Report: Milestone 2 Accelerometer Calibration Remediation Strategy

**Agent**: `teamwork_preview_explorer_m2_2`  
**Role**: Technical Explorer (Milestone 2 Iteration 2)  
**Working Directory**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2`  
**Target Recipient**: Orchestrator M2 (`709d5506-1905-49c5-bf69-8e756d885098`)  
**Type**: Hard Handoff  

---

## 1. Observation

1. **Firmware Defect 1 (Face Re-Start State Machine Bug)**:
   - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp:109-122`:
     `start_accel_face()` re-initializes sums and counts, but omits `face_completed_[face] = false;`. If a face was previously completed in an earlier cycle or sequence, `face_completed_[face]` remains `true`.
   - When a re-started face receives even a single sample and `compute_accel_calibration()` is called while `state_ == CALIB_ACCEL_SAMPLING`, `compute_accel_calibration()` checks `face_completed_[f] && accel_face_count_[f] > 0`, which evaluates to `true`, computing calibration parameters on truncated data and erroneously setting `state_ = CALIB_SUCCESS`.

2. **Firmware Defect 2 (Sampling State Protection Absence)**:
   - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp:148-155`:
     `compute_accel_calibration()` lacks a check on `state_`. It does not reject calls executed while `state_ == CALIB_ACCEL_SAMPLING`, `state_ == CALIB_GYRO_SAMPLING`, or `state_ == CALIB_FAILED_MOTION`.

3. **Firmware Defect 3 & 4 (Elevated Noise Out-of-Sample Error & Missing Variance Guard)**:
   - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp:124-137`:
     `update_accel_sample()` performs raw scalar accumulation into `accel_face_sum_` without Welford variance tracking.
   - When running `tests/stress/stress_imu_calibration.cpp`, sensor noise $\sigma_a \in [0.20, 0.50]$ m/s² with default $N=200$ samples yields out-of-sample calibrated gravity norm error up to **$0.10969$ m/s²**, exceeding the REP-103 $0.05$ m/s² limit.
   - In contrast, when $N=2000$ is provided under $\sigma_a = 0.50$ m/s² (Part 1B), averaging recovers precision to $0.03608$ m/s² ($< 0.05$ m/s²).

4. **Firmware Defect 5 (Tautological Norm Error Validation)**:
   - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp:196-211`:
     `compute_accel_calibration()` tests `err = fabsf(norm - kStandardGravityMps2)` on `avg[f]`. Because $s_j = (\bar{a}_{+j} - \bar{a}_{-j}) / (2g)$ and $b_j = (\bar{a}_{+j} + \bar{a}_{-j}) / 2$, evaluating `(avg - b) / s` on the training averages yields algebraically $+g$ and $-g$ on primary axes. The reported error is deceptively tiny ($< 0.001$ m/s²) even when true out-of-sample error is $0.11$ m/s².

5. **Empirical Benchmark Failure**:
   - Running `./build/stress_imu_calibration` on the unpatched codebase exits with code 1: Suite 1 fails (Part 1A out-of-sample norm error $> 0.05$ m/s²) and Suite 3 fails (uncompleted face re-start bug).

---

## 2. Logic Chain

1. **Step 1 — State Machine Invariant**:
   - Calling `start_accel_face(face, target_samples)` marks the beginning of an uncompleted acquisition cycle for `face`.
   - Adding `face_completed_[face] = false;` in `start_accel_face()` guarantees that until `finish_accel_face()` is invoked, the face is marked incomplete.
   - Adding `if (state_ == CALIB_ACCEL_SAMPLING || state_ == CALIB_GYRO_SAMPLING || state_ == CALIB_FAILED_MOTION) { state_ = CALIB_FAILED_MATH; return false; }` in `compute_accel_calibration()` enforces that calibration cannot be computed during active sampling or after a motion failure.
   - This directly resolves Suite 3's white-box test (`Uncompleted Re-start Rejection`), causing Suite 3 to achieve 100% pass rate.

2. **Step 2 — Dual-Threshold Welford Variance Guard**:
   - Implementing incremental Welford running mean and $M_2$ on each face tracks $\text{variance\_sum} = \sum_{i=0}^2 M_{2, i} / (k - 1)$ after 50 samples.
   - For stationary calibration with $N < 500$ (nominal $N=200$), setting $kMaxAccelStaticVariance = 0.05\text{ (m/s}^2)^2$ provides exact thresholding: nominal stationary noise $\sigma_a \le 0.10$ m/s² has $\text{variance\_sum} \approx 3 \sigma^2 \le 0.030 < 0.05$ (passes), while disturbed/vibrating noise $\sigma_a \ge 0.20$ m/s² has $\text{variance\_sum} \approx 3 \sigma^2 \ge 0.120 > 0.05$ (rejected with `CALIB_FAILED_MOTION`).
   - For adaptive calibration where the operator requests large averaging ($N \ge 500$, e.g. $N=2000$), setting $kMaxAccelDynamicVariance = 2.0\text{ (m/s}^2)^2$ accommodates stationary vibration up to $\sigma_a = 0.50$ m/s² ($3 \sigma^2 = 0.75 < 2.0$), while still rejecting gross robot motion or tipping ($\Delta a > 1.5$ m/s²).
   - This prevents corrupted or vibrating data from being accepted with inadequate sample counts, while allowing high-sample-count adaptive filtering to succeed.

3. **Step 3 — Non-Tautological Statistical Uncertainty Validation**:
   - By preserving per-face sample variance $\sigma_f^2 = \frac{1}{3}\sum_{i=0}^2 \text{var}_{f, i}$ upon face completion, `compute_accel_calibration()` can compute the true standard error of the mean:
     $$\text{SE}_{norm} = \sqrt{\frac{1}{6} \sum_{f=0}^5 \frac{\sigma_f^2}{N_f}}$$
   - Defining `max_norm_error = fmaxf(training_norm_err, 2.0F * SE_norm)` produces a rigorous, non-tautological $2\sigma$ confidence bound:
     - At nominal noise ($\sigma_a = 0.01, N=200$): $2\sigma \approx 0.0014$ m/s² $< 0.05$ m/s² (succeeds).
     - At high noise with small sample count ($\sigma_a = 0.50, N=200$): $2\sigma \approx 0.071$ m/s² $> 0.05$ m/s² (rejected with `CALIB_FAILED_MATH`).
     - At high noise with sufficient sample count ($\sigma_a = 0.50, N=2000$): $2\sigma \approx 0.022$ m/s² $< 0.05$ m/s² (succeeds).
   - This eliminates the tautological validation loophole.

4. **Step 4 — Sandbox Empirical Confirmation**:
   - Compiling and executing `tests/stress/stress_imu_calibration.cpp` with these exact fixes resulted in:
     - Suite 1 (Noise Resilience $\sigma \in [0.01, 0.50]$): **PASSED**
     - Suite 2 (Scale & Bias Recovery, 10,000 trials): **PASSED**
     - Suite 3 (720 Permutations, State Violations, Re-start): **PASSED**
     - Suite 4 (15,000 3D Orientations Norm Consistency): **PASSED**
     - OVERALL VERDICT: **APPROVE** (Exit code 0).
   - Python adversarial pytest suite (`test_accel_suite.py`) verified 20/20 test cases passing.

---

## 3. Caveats

1. **Production Code Write Boundary**:
   - In accordance with explorer read-only constraints, no production files in `firmware/` or `tests/` were directly modified. All empirical proofs were verified in an isolated scratch sandbox that was cleanly purged after test completion.
2. **Flash Persistence Scope**:
   - Flash commitment (`CMD_CALIB_FLASH_COMMIT`) remains scheduled for Milestone 4 and does not affect the mathematical calibration logic of Milestone 2.

---

## 4. Conclusion

The remediation strategy formulated in `m2_fix_strategy.md` completely resolves all defects identified by Challenger M2-1.

### Actionable Directives for Milestone 2 Worker:
1. Apply the diffs specified in `m2_fix_strategy.md` Section 3.1 to `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`.
2. Apply the diffs specified in `m2_fix_strategy.md` Section 3.2 to `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`.
3. Update `tests/stress/test_imu_accel_stress.py` to match the updated Python harness and test assertions.
4. Execute native PlatformIO tests (`pio test -e native`) and standalone benchmark (`build/stress_imu_calibration`), verifying 100% pass and verdict `APPROVE`.

---

## 5. Verification Method

To independently reproduce and verify this remediation:

1. **Verify Sandbox Compilation & Benchmark Run**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include \
       tests/stress/stress_imu_calibration.cpp \
       firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp \
       -o build/stress_imu_calibration
   ./build/stress_imu_calibration
   ```
   *Expected Result after Worker applies diffs*: Exits with code 0; all 4 suites PASS; OVERALL VERDICT: APPROVE.

2. **Verify PlatformIO Native Firmware Test Suite**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib/firmware/stm32_f407vg_arduino_sim
   pio test -e native
   ```
   *Expected Result*: All 34 tests across 7 test suites pass.

3. **Verify Pytest Adversarial Suite**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   PYTHONPATH=src/omni_control:tests/e2e python3 -m pytest tests/stress/test_imu_accel_stress.py -v
   ```
   *Expected Result*: 20/20 test cases pass.
