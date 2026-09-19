## 2026-09-19T11:20:26Z
You are teamwork_preview_reviewer_m2_1, Reviewer 1 for Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_reviewer_m2_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Worker M2 Handoff: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_2/handoff.md
Test Ready Sign-off: /home/sonev/teamwork_projects/amr_omni_calib/TEST_READY.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and the Worker M2 Handoff completely.

OBJECTIVE:
Independently review the work product of Milestone 2 for correctness, completeness, and adherence to requirements R2 and acceptance criteria:
1. Examine `firmware/stm32_f407vg_arduino_sim/include/imu_calibration.h` and `src/imu_calibration.cpp`:
   - ST AN4508 6-position accelerometer linear calibration ($s_j = (\bar{a}_{+j} - \bar{a}_{-j})/(2g)$, $b_j = (\bar{a}_{+j} + \bar{a}_{-j})/2$, norm error $< 0.05$ m/s²).
   - Stationary gyroscope zero-rate bias nulling ($\ge 1000$ samples, residual drift $< 0.05^\circ/\text{s}$).
   - REP-103 ENU alignment (+9.80665 m/s² on Z at rest).
   - Allan variance noise model parameters and dynamic inflation.
2. Examine integration in `firmware/stm32_f407vg_arduino_sim/src/main.cpp`:
   - Calibration applied to raw IMU samples in `imu_task`.
   - Telemetry covariances: all diagonal elements strictly non-zero (orientation, angular velocity, linear acceleration).
3. Run and verify all builds and test suites:
   - `cd /home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim && pio test -e native`
   - `cd /home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim && pio run -e disco_f407vg`
   - `cd /home/sonev/teamwork_projects/amr_omni_calib && pytest tests/e2e/tier1_feature_coverage/test_f2_imu_calibration.py -v`
4. Provide an explicit verdict in your handoff report: **APPROVE** or **REQUEST_CHANGES**.

Write your handoff report to `handoff.md` in your working directory and notify the orchestrator via send_message.
