# BRIEFING — 2026-09-19T11:25:00Z

## Mission
Empirically stress-test the ST AN4508 6-position accelerometer calibration routine on STM32 firmware and verify scale/bias recovery, noise resilience, sequence handling, and 3D orientation norm consistency.

## 🔒 My Identity
- Archetype: empirical_challenger
- Roles: critic, specialist
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: M2
- Instance: 1 of 2

## 🔒 Key Constraints
- Review-only — do NOT modify implementation code (report findings/verdict, write standalone test harnesses)
- Empirical verification mandatory — run tests directly, do NOT trust unverified claims
- Do not place source, tests, or test data inside `.agents/`
- Send message to parent upon completion

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T11:20:26Z

## Review Scope
- **Files to review**:
  - `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
  - `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
  - `firmware/stm32_f407vg_arduino_sim/test/test_imu_calibration/test_main.cpp`
  - `firmware/stm32_f407vg_arduino_sim/src/main.cpp`
  - `tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py`
- **Interface contracts**: ST AN4508 6-position accelerometer calibration ($s_j = (\bar{a}_{+j} - \bar{a}_{-j})/(2g)$, $b_j = (\bar{a}_{+j} + \bar{a}_{-j})/2$), REP-103 ENU standards
- **Review criteria**: Empirical correctness under stress, noise tolerance ($\sigma_a \in [0.01, 0.5]$), recovery of extreme scale/bias ($s \in [0.7, 1.3]$, $b \in [-2.0, 2.0]$), out-of-order & incomplete face handling, arbitrary 3D orientation norm consistency.

## Attack Surface
- **Hypotheses tested**:
  1. Hypothesis: Norm error $\|a_{calib}\| - g < 0.05$ holds across $\sigma_a \in [0.01, 0.50]$ m/s² with default $N=200$. -> DISPROVED. Fails for $\sigma_a \ge 0.20$ m/s² (error up to $0.10969$ m/s²).
  2. Hypothesis: `compute_accel_calibration` detects residual norm error exceeding 0.05 m/s². -> DISPROVED. Tautological primary-axis error is algebraically zero on training means ($IntNormErr \le 0.001$ m/s²).
  3. Hypothesis: State machine rejects computing calibration when a face is re-started and actively sampling. -> DISPROVED. `start_accel_face` fails to reset `face_completed_[face] = false`, allowing premature compute.
  4. Hypothesis: Scale and bias are recovered accurately across $s \in [0.7, 1.3]$ and $b \in [-2.0, 2.0]$ m/s². -> CONFIRMED. Scale error $\le 0.0274\%$, bias error $\le 0.00215$ m/s².
  5. Hypothesis: All 720 face sequence permutations succeed. -> CONFIRMED.
  6. Hypothesis: Post-calibration 3D orientation norm invariance holds. -> CONFIRMED with sufficient calibration data (error $0.00131$ m/s²).
- **Vulnerabilities found**:
  - Defect 1: Norm error exceeds 0.05 m/s² under noise $\sigma_a \in [0.20, 0.50]$ m/s² with default $N=200$.
  - Defect 2: Tautological validation loop in `compute_accel_calibration`.
  - Defect 3: Missing `face_completed_[face] = false` in `start_accel_face` and lack of state check in `compute_accel_calibration`.
  - Defect 4: Missing motion/vibration variance gate in accelerometer sampling.
- **Untested angles**: Flash persistence (Milestone 4 scope).

## Loaded Skills
- None specified

## Key Decisions Made
- Authored standalone C++ benchmark runner `tests/stress/stress_imu_calibration.cpp` and compiled executable `build/stress_imu_calibration`.
- Authored pytest test suite `tests/stress/test_imu_accel_stress.py`.
- Formulated definitive verdict: REQUEST_CHANGES based on empirical failures in Suite 1 and Suite 3.

## Artifact Index
- `DISPATCH.md` — dispatch log
- `BRIEFING.md` — persistent working memory
- `progress.md` — liveness heartbeat
- `handoff.md` — final assessment and verdict
- `tests/stress/stress_imu_calibration.cpp` — standalone C++ stress test harness
- `build/stress_imu_calibration` — compiled benchmark executable
- `tests/stress/test_imu_accel_stress.py` — Python/pytest adversarial test suite
