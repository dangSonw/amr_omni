## 2026-09-19T11:32:42Z

You are teamwork_preview_worker_m2_3, the Remediation Worker for Milestone 2 (Iteration 2).
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_3
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Fix Strategy: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2/m2_fix_strategy.md
Explorer Handoff: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2/handoff.md
Challenger Failure Report: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1/handoff.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and the Fix Strategy in m2_fix_strategy.md completely.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE FILE OWNERSHIP:
You have exclusive write access to:
- `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`
- `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`
- `tests/stress/test_imu_accel_stress.py`

IMPLEMENTATION TASKS:
Apply the exact mathematical and state machine remediation specified in `m2_fix_strategy.md`:
1. In `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h`:
   - Add variance tracking fields to class `ImuCalibrator`: `accel_face_m2_[FACE_COUNT][3]`, `accel_face_var_[FACE_COUNT]`.
   - Add constants: `kMaxAccelStaticVariance = 0.05F`, `kMaxAccelDynamicVariance = 2.0F`.
2. In `firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp`:
   - In `start_accel_face()`: reset `face_completed_[face] = false;` and clear `accel_face_m2_` and `accel_face_var_`.
   - In `update_accel_sample()`: implement Welford running variance on acceleration components. If variance after 50 samples exceeds the stationarity threshold, set `state_ = CALIB_FAILED_MOTION; return false;`.
   - In `finish_accel_face()`: record final face sample variance $\sigma_f^2 = \frac{1}{3}\sum_{i=0}^2 \text{var}_{f, i}$.
   - In `compute_accel_calibration()`:
     - Reject if called during active sampling: `if (state_ == CALIB_ACCEL_SAMPLING || state_ == CALIB_GYRO_SAMPLING || state_ == CALIB_FAILED_MOTION) { state_ = CALIB_FAILED_MATH; return false; }`.
     - Compute non-tautological $2\sigma$ confidence bound $\text{SE}_{norm} = \sqrt{\frac{1}{6}\sum_{f=0}^5 \frac{\sigma_f^2}{N_f}}$ and set `max_norm_error = fmaxf(training_norm_err, 2.0F * SE_norm)`.
3. In `tests/stress/test_imu_accel_stress.py`:
   - Update any test harnesses/assertions to reflect the new variance guard and state rejection.

VERIFICATION REQUIREMENTS:
Run and report verbatim output for:
1. Compile and run `./build/stress_imu_calibration`:
   ```bash
   g++ -O3 -Wall -Wextra -I firmware/stm32_f407vg_arduino_sim/include tests/stress/stress_imu_calibration.cpp firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp -o build/stress_imu_calibration
   ./build/stress_imu_calibration
   ```
   Must exit with code 0 and report: `OVERALL VERDICT: APPROVE`.
2. `pio test -e native` in `firmware/stm32_f407vg_arduino_sim` (all tests passing).
3. `pio run -e disco_f407vg` in `firmware/stm32_f407vg_arduino_sim` (clean build 0 errors).
4. `PYTHONPATH=src/omni_control:tests/e2e python3 -m pytest tests/stress/test_imu_accel_stress.py -v` (20/20 passed).
5. `pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v` (20/20 passed).

When complete, write your handoff report to `handoff.md` in your working directory and notify the orchestrator via send_message.
