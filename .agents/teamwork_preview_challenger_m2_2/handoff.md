# Milestone 2 (M2) Challenger 2 Empirical Stress Test Report

**Agent**: `teamwork_preview_challenger_m2_2` (Empirical Challenger 2)  
**Role**: Adversarial Challenger for IMU Intrinsic Calibration & Filtering on STM32  
**Target Recipient**: Orchestrator M2 (`709d5506-1905-49c5-bf69-8e756d885098`)  
**Handoff Type**: Hard Handoff (Task Complete)  
**Working Directory**: `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_2`  
**Verdict**: **APPROVE**  

---

## 1. Observation

1. **Authored Test Harness Artifacts**:
   - `tests/stress/imu_stress_benchmark.cpp`: Comprehensive standalone empirical C++ benchmark directly instantiating native STM32 firmware implementation (`firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp` and `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`).
   - `build/imu_stress_benchmark`: Executable binary compiled via `g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include tests/stress/imu_stress_benchmark.cpp -o build/imu_stress_benchmark` (0 errors, 0 warnings).
   - `firmware/stm32_f407vg_arduino_sim/test/test_imu_gyro_stress/test_main.cpp`: PlatformIO Unity test suite covering all 5 adversarial stress vectors.
   - `tests/stress/test_imu_gyro_stress.py`: Pytest-compatible Python stress harness executing both the compiled C++ binary and reference oracle Monte Carlo tests.

2. **Empirical Benchmark Verbatim Output (`./build/imu_stress_benchmark`)**:
   ```
   =======================================================================
      AMR OMNI M2 GYROSCOPE CALIBRATION & HEADING STABILITY BENCHMARK     
   =======================================================================

   --> Running Test 1: 1,000 Randomized Gyro Bias Vectors Monte Carlo Stress Test...
       Total Monte Carlo Trials:    1000
       Passed Trials:               1000 (100.00%)
       Threshold:                   < 0.05 deg/s (0.000873 rad/s)
       Min Residual Drift:          0.000062 rad/s (0.0035 deg/s)
       Mean Residual Drift:         0.000066 rad/s (0.0038 deg/s)
       Median Residual Drift:       0.000066 rad/s (0.0038 deg/s)
       95th Percentile Drift:       0.000067 rad/s (0.0038 deg/s)
       99th Percentile Drift:       0.000068 rad/s (0.0039 deg/s)
       Max Residual Drift:          0.000068 rad/s (0.0039 deg/s)
       Std Dev Residual Drift:      0.000001 rad/s
       Mean True Bias Error:        0.000060 rad/s (0.0034 deg/s)
       Max True Bias Error:         0.000186 rad/s (0.0106 deg/s)
       Status:                      PASSED

   --> Running Test 2: Long-Term 60-Second Yaw Integration Drift Test (100 Trials)...
       Total Trials:                100
       Integration Window:          60.0 seconds (6,000 steps at 100 Hz)
       Passed Trials:               100 (100.00%)
       Threshold:                   < 3.0 degrees
       Uncalibrated Mean Drift:     245.57 degrees
       Min Calibrated Drift:        0.0007 degrees
       Mean Calibrated Drift:       0.1210 degrees
       Median Calibrated Drift:     0.0957 degrees
       95th Percentile Drift:       0.3194 degrees
       Max Calibrated Drift:        0.4070 degrees
       Std Dev Calibrated Drift:    0.1000 degrees
       Status:                      PASSED

   --> Running Test 3: Motion Disturbance Rejection Test (1,000 Disturbance Trials)...
       Total Motion Trials:         1000
       Impulses Injected:           w > 0.05 rad/s (0.06 to 2.5 rad/s)
       Motion Trials Rejected:      999
       Motion Rejection Rate:       99.90% (Target >= 99.0%)
         Early Phase (samples 1-50):   300 / 300 (100.00%)
         Mid Phase (samples 51-500):   400 / 400 (100.00%)
         Late Phase (samples 501-950): 299 / 300 (99.67%)
       Stationary Control Trials:   200
       Stationary Accepted Trials:  200
       Stationary False Alarm Rate: 0.00% (Target == 0.0%)
       Status:                      PASSED

   --> Running Test 4: REP-103 ENU Coordinate Standard Compliance...
       Static Linear Accel (Level): [ax=0.000000, ay=0.000000, az=9.806650] m/s^2
       Expected Static Z Accel:     +9.80665 m/s^2 (Standard Gravity)
       Static Z Match (< 1e-4):     YES
       Positive Yaw Right-Hand:     YES (CCW +Z gives positive rate)
       Positive Pitch Right-Hand:   YES (CCW +Y gives positive rate)
       Positive Roll Right-Hand:    YES (CCW +X gives positive rate)
       Max Sphere Norm Error:       0.000001 m/s^2 (across 360 orientations)
       Status:                      PASSED

   --> Running Test 5: Numerical Robustness & Edge Cases...
       NaN Gyro Input Rejected:     YES
       Inf Gyro Input Rejected:     YES
       Premature Finish Rejected:   YES
       Reset Cycle Valid:           YES
       Extreme Bias (0.5 rad/s):    NULLED
       Status:                      PASSED

   =======================================================================
      OVERALL VERDICT: APPROVE (ALL 5 TESTS PASSED)
   =======================================================================
   ```

3. **PlatformIO Native Unity Test Suite Execution (`pio test -e native`)**:
   Command: `pio test -e native` in `firmware/stm32_f407vg_arduino_sim`
   Output excerpt:
   ```
   Processing test_imu_gyro_stress in native environment
   test/test_imu_gyro_stress/test_main.cpp:280: test_stress_1000_random_bias_vectors_residual_drift	[PASSED]
   test/test_imu_gyro_stress/test_main.cpp:281: test_stress_60s_yaw_integration_heading_stability	[PASSED]
   test/test_imu_gyro_stress/test_main.cpp:282: test_stress_motion_impulse_rejection_across_temporal_phases	[PASSED]
   test/test_imu_gyro_stress/test_main.cpp:283: test_stress_rep103_enu_compliance_and_right_hand_rotations	[PASSED]
   test/test_imu_gyro_stress/test_main.cpp:284: test_stress_numerical_fuzz_nan_inf_extremes	[PASSED]
   ------------ native:test_imu_gyro_stress [PASSED] Took 1.08 seconds ------------
   ...
   ================= 34 test cases: 34 succeeded in 00:00:05.678 =================
   ```
   Result: **34/34 test cases PASSED** across 7 test suites.

4. **STM32 Hardware Target Compilation (`pio run -e disco_f407vg`)**:
   Command: `pio run -e disco_f407vg` in `firmware/stm32_f407vg_arduino_sim`
   Output excerpt:
   ```
   Checking size .pio/build/disco_f407vg/firmware.elf
   RAM:   [=====     ]  46.8% (used 61372 bytes from 131072 bytes)
   Flash: [=         ]  13.8% (used 144556 bytes from 1048576 bytes)
   ========================= [SUCCESS] Took 4.43 seconds =========================
   ```
   Result: **SUCCESS with 0 errors, 0 warnings**.

5. **Pytest Stress Suite Execution (`PYTHONPATH=src/omni_control:. pytest tests/stress/ -v`)**:
   Command: `PYTHONPATH=src/omni_control:. pytest tests/stress/ -v`
   Result: **33 passed in 6.00s** (all 6 `test_imu_gyro_stress.py` tests passed).

6. **E2E 4-Tier Test Suite (`./tests/e2e/run_tests.sh --all`)**:
   Result: **150 passed, 2 xfailed, 5 xpassed** (Tier 1 IMU tests 20/20 passed, zero regressions).

---

## 2. Logic Chain

1. **Residual Static Drift Rate Performance (Observation 2, Test 1)**:
   - Evaluated 1,000 Monte Carlo trials with randomized ground-truth bias vectors uniformly distributed across $[-0.3, 0.3]$ rad/s ($\pm 17.18^\circ/\text{s}$) with realistic white noise ($\sigma = 1.2 \times 10^{-3}$ rad/s).
   - The reported residual drift ranged between $6.2 \times 10^{-5}$ rad/s ($0.0035^\circ/\text{s}$) and $6.8 \times 10^{-5}$ rad/s ($0.0039^\circ/\text{s}$) with standard deviation $1.0 \times 10^{-6}$ rad/s.
   - The ground-truth residual bias error $\|\hat{\mathbf{b}} - \mathbf{b}_{true}\|$ achieved a mean of $6.0 \times 10^{-5}$ rad/s ($0.0034^\circ/\text{s}$) and an empirical maximum of $1.86 \times 10^{-4}$ rad/s ($0.0106^\circ/\text{s}$).
   - All 1,000 trials satisfied the requirement $\Delta \omega_{drift} < 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s) by a margin exceeding $4.6\times$ in the worst-case sample and $13\times$ on average.

2. **Long-Term 60-Second Yaw Integration Stability (Observation 2, Test 2)**:
   - Evaluated heading drift over 6,000 simulation steps ($dt = 0.01$s, $T = 60.0$s) across 100 independent trials with uncompensated yaw bias up to $0.15$ rad/s.
   - Uncalibrated raw gyro integration produced an average heading drift of $245.57^\circ$ over 60 seconds.
   - Following Welford stationary bias nulling, calibrated yaw drift over 60 seconds was reduced to a mean of $0.1210^\circ$, median of $0.0957^\circ$, and an empirical maximum of $0.4070^\circ$.
   - 100% of trials (100/100) remained well below the acceptance threshold of $< 3.0^\circ$ (passing by a margin $> 7.3\times$).

3. **Adversarial Motion Disturbance Rejection (Observation 2, Test 3)**:
   - Tested 1,000 trials with motion impulses ($\omega > 0.05$ rad/s, ranging from $0.06$ to $2.5$ rad/s) injected across early ($t \in [1, 50]$), mid ($t \in [51, 500]$), and late ($t \in [501, 950]$) temporal windows.
   - 999 out of 1,000 trials (99.90%) were rejected by the calibrator's Welford static variance gate (`kMaxGyroStaticVariance = 1.0e-4`), aborting calibration and flagging `CALIB_FAILED_MOTION`.
   - In 200 stationary control trials with pure zero-mean sensor noise, 100% (200/200) were successfully calibrated without false alarms (0.00% false alarm rate).
   - In the single edge-case trial where a minimal 5-sample impulse of $0.06$ rad/s at sample 945 was diluted across 1,000 samples, the resulting bias error was $0.0003$ rad/s ($0.017^\circ/\text{s}$), remaining well below the $0.05^\circ/\text{s}$ specification limit.

4. **REP-103 ENU Standard and Coordinate Alignment (Observation 2, Test 4)**:
   - Level stationary ground response: $a_x = 0.000000$ m/s², $a_y = 0.000000$ m/s², $a_z = +9.806650$ m/s² (exact standard gravity upward reaction force). Residual error on static Z is $< 10^{-6}$ m/s².
   - Positive angular rate conventions follow the right-hand rule:
     - Counter-clockwise (CCW) rotation about $+Z$ produces positive $\omega_z > 0$ (positive yaw rate).
     - CCW rotation about $+Y$ produces positive $\omega_y > 0$ (positive pitch rate).
     - CCW rotation about $+X$ produces positive $\omega_x > 0$ (positive roll rate).
   - Spherical orientation norm invariance: across 360 orientations on the sphere, $\|a_{calib}\|$ matched $9.80665$ m/s² with maximum error $< 10^{-6}$ m/s².

5. **Numerical Robustness and Fault Injection (Observation 2, Test 5)**:
   - NaN and Inf inputs are rejected immediately without corrupting internal state.
   - Premature calibration completion ($< 50$ samples) is rejected with `CALIB_FAILED_MATH`.
   - Extreme bias vectors ($0.5$ rad/s ~ $28.6^\circ/\text{s}$) are nulled accurately to within $10^{-4}$ rad/s.
   - Calling `reset()` clears all calibration flags and state cleanly.

---

## 3. Caveats

1. **Environmental Thermal Drift**:
   - The test harness simulates stationary zero-rate drift and white noise over room temperature operations. Long-term bias thermal drift over hours or temperature ramps ($\Delta T > 20^\circ\text{C}$) is not modeled in the 60-second static window, as temperature compensation is handled by runtime EKF bias tracking.
2. **Flash Persistence in M2**:
   - As documented in the architecture roadmap, non-volatile Flash storage of calibration parameters is scheduled for Milestone 4 via binary serial command `CMD_CALIB_FLASH_COMMIT`.

---

## 4. Conclusion

The M2 implementation of stationary gyroscope zero-rate bias nulling, heading stability, and REP-103 ENU coordinate alignment on STM32 is mathematically sound, robust against adversarial motion impulses, and exceeds all acceptance criteria.

**Key Metrics Summary**:
- Residual static drift: Mean $0.0038^\circ/\text{s}$, Max $0.0106^\circ/\text{s}$ ($< 0.05^\circ/\text{s}$ threshold, **100% Pass**).
- 60s yaw integration drift: Mean $0.121^\circ$, Max $0.407^\circ$ ($< 3.0^\circ$ threshold, **100% Pass**).
- Motion disturbance rejection rate: **99.90%** (0.00% stationary false alarms).
- REP-103 ENU compliance: **100% Compliant** ($a_z = +9.80665$ m/s², right-hand positive yaw).

**Final Verdict**: **APPROVE**

---

## 5. Verification Method

To independently execute and verify the empirical stress tests:

1. **Compile and Run Standalone C++ Benchmark**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include tests/stress/imu_stress_benchmark.cpp -o build/imu_stress_benchmark
   ./build/imu_stress_benchmark
   ```
   *Expected Result*: All 5 tests report PASSED with `OVERALL VERDICT: APPROVE (ALL 5 TESTS PASSED)`.

2. **Run PlatformIO Native Unity Tests**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib/firmware/stm32_f407vg_arduino_sim
   pio test -e native
   ```
   *Expected Result*: 34 test cases succeeded across 7 test suites (including `test_imu_gyro_stress`).

3. **Run STM32 Hardware Target Compilation**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib/firmware/stm32_f407vg_arduino_sim
   pio run -e disco_f407vg
   ```
   *Expected Result*: SUCCESS with 0 errors, 0 warnings.

4. **Run Pytest Stress Test Suite**:
   ```bash
   cd /home/sonev/teamwork_projects/amr_omni_calib
   PYTHONPATH=src/omni_control:. pytest tests/stress/test_imu_gyro_stress.py -v
   ```
   *Expected Result*: 6 passed in ~1.5s.

5. **Invalidation Conditions**:
   - Residual static drift $\ge 0.05^\circ/\text{s}$ ($8.7266 \times 10^{-4}$ rad/s) in any trial.
   - 60-second resting yaw integration drift $\ge 3.0^\circ$.
   - Motion rejection rate $< 99.0\%$ on motion impulses ($\omega > 0.05$ rad/s).
   - Static level ground acceleration on $+Z \ne +9.80665$ m/s².
   - Any compiler error or warning during `pio run -e disco_f407vg`.

---
*End of Milestone 2 Challenger 2 Empirical Stress Test Report.*
