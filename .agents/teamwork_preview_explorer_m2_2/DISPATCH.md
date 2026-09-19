## 2026-09-19T11:26:10Z

You are teamwork_preview_explorer_m2_2, Technical Explorer for Milestone 2 (Iteration 2).
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Gate Status: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_1/GATE_STATUS.md
Challenger M2-1 Handoff (Failure Details): /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_challenger_m2_1/handoff.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and the Challenger M2-1 Handoff report completely.

CONTEXT:
Milestone 2 Worker implemented ST AN4508 and stationary gyro bias nulling, passing 2 Reviewers and Forensic Auditor. However, Challenger 1 (`teamwork_preview_challenger_m2_1`) issued a REQUEST_CHANGES verdict with concrete empirical findings:
1. **Face Re-Start State Machine Bug**: `start_accel_face(face, target_samples)` does NOT clear `face_completed_[face] = false`. Calling `compute_accel_calibration()` while re-sampling an uncompleted face succeeds erroneously while state is `CALIB_ACCEL_SAMPLING`.
2. **Sampling State Protection**: `compute_accel_calibration()` does not reject when called while `state_ == CALIB_ACCEL_SAMPLING` or `state_ == CALIB_GYRO_SAMPLING`.
3. **Elevated Noise Out-of-Sample Norm Error**: When sensor noise is elevated ($\sigma_a \in [0.20, 0.50]$ m/s²), default $N=200$ samples results in out-of-sample calibrated gravity norm error $> 0.05$ m/s² (up to $0.11$ m/s²).
4. **Accelerometer Stationarity / Variance Guard**: Unlike gyroscope sampling (which has Welford variance gating), accelerometer sampling lacks variance tracking to detect excessive motion or mechanical vibration during face collection.

OBJECTIVE:
Investigate and formulate the exact remediation strategy for `include/imu_calibration.h`, `src/imu_calibration.cpp`, and test suites:
- Formulate exact code diffs for resetting `face_completed_` in `start_accel_face`.
- Formulate state protection in `compute_accel_calibration`.
- Formulate Welford accelerometer stationarity/variance tracking during face sampling with a clear variance rejection gate (e.g. `kMaxAccelStaticVariance = 0.04F` (m/s²)² or adaptive sample count $N \ge 500$).
- Formulate proper non-tautological validation checks.
- Verify how these fixes will make `build/stress_imu_calibration` and `tests/stress/test_imu_accel_stress.py` pass 100%.

SCOPE BOUNDARIES:
- Read-only exploration. DO NOT modify production source files.
- Write your report to:
  `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2/m2_fix_strategy.md`
  And write your handoff to:
  `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_2/handoff.md`

When complete, send a message to parent notifying completion.
