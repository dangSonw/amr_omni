## 2026-09-19T11:43:16Z
You are teamwork_preview_explorer_m3_1, Technical Explorer 1 for Milestone 3 (M3) — EKF Configuration & Covariance Tuning.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m3_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Codebase: /home/sonev/amr_omni

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md completely.

OBJECTIVE:
Investigate and design the exact technical updates for the robot_localization EKF configuration:
1. Inspect `/home/sonev/amr_omni/src/omni_localization/config/ekf.yaml`:
   - Inspect existing `process_noise_covariance` (15x15) and `initial_estimate_covariance` (15x15).
   - Identify all zero diagonal entries that cause filter stagnation or unobservable states.
   - Design physically grounded non-zero values for all diagonal entries:
     - Process noise: position ($x, y, z$), orientation (roll, pitch, yaw), linear velocity ($v_x, v_y, v_z$), angular velocity ($\omega_x, \omega_y, \omega_z$), linear acceleration ($a_x, a_y, a_z$).
     - Initial estimate covariance: realistic uncertainties for stationary startup.
2. Inspect sensor input configuration:
   - `odom0`: source topic (`/odom` or `/wheel/odom`), active states ($v_x, v_y, \omega_z$), differential setting, queue size.
   - `imu0`: source topic (`/imu/data`), active states ($\text{yaw}, \omega_z, a_x, a_y$ or appropriate REP-103 states), gravitational acceleration removal, differential setting.
3. Review E2E test requirements in `/home/sonev/teamwork_projects/amr_omni_calib/tests/e2e/tier1_feature_coverage/test_f3_extrinsics_tf.py` and ensure the proposed `ekf.yaml` satisfies all test assertions.

SCOPE BOUNDARIES:
- Read-only exploration. DO NOT modify production source files.
- Write your analysis to:
  `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m3_1/m3_ekf_analysis.md`
  And write your handoff to:
  `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m3_1/handoff.md`

When complete, send a message to parent notifying completion.
