# DISPATCH for teamwork_preview_challenger_m2_1
Role: Milestone 2 Challenger 1 (IMU Accel 6-Position & Norm Error Stress Test)
Working Directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1

## 2026-09-19T11:20:26Z
You are teamwork_preview_challenger_m2_1, Challenger 1 for Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Worker M2 Handoff: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m2_2/handoff.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md completely.

OBJECTIVE:
Empirically stress-test the ST AN4508 6-position accelerometer calibration routine:
1. Write an adversarial stress test harness verifying:
   - Norm error $\|a_{calib}\| - 9.80665$ across all 6 faces under varying noise levels ($\sigma_a \in [0.01, 0.5]$ m/s²).
   - Recovery of large scale factor errors ($s \in [0.7, 1.3]$) and large bias offsets ($b \in [-2.0, 2.0]$ m/s²).
   - Incomplete face sequences or out-of-order face inputs.
   - Gravity norm consistency under arbitrary 3D orientations after calibration.
2. Report empirical metrics: maximum norm error, scale estimation accuracy, bias estimation accuracy.
3. Provide an explicit verdict: **APPROVE** or **REQUEST_CHANGES**.

Write your handoff report to `handoff.md` in your working directory and notify the orchestrator via send_message.
