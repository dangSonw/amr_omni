## 2026-09-20T07:12:05Z
You are teamwork_preview_worker_m3_2, working on Milestone 3: Extrinsics, Covariances, Single TF Authority & Laser Filter Masking.
Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_worker_m3_2

MANDATORY FIRST STEP:
Read /home/sonev/amr_omni/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/PROJECT.md before doing any work.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

Context and Explorer Findings:
Read the comprehensive findings prepared by the 3 M3 Explorers:
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m3_1/handoff.md and m3_ekf_analysis.md
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m3_2/handoff.md and m3_tf_analysis.md
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m3_3/handoff.md and m3_extrinsics_laser_analysis.md

Scope and Deliverables for Milestone 3:
1. `src/omni_localization/config/ekf.yaml`:
   - Set transform_timeout: 0.05
   - Update `odom0_config`: indices 6, 7, 11 (vx, vy, vyaw) set to true.
   - Update `imu0_config`: indices 5, 11, 12, 13 (yaw, vyaw, ax, ay) set to true.
   - Populate full 15x15 (225 elements) strictly positive diagonal `process_noise_covariance` and `initial_estimate_covariance` matrices as specified in m3_ekf_analysis.md.
2. `src/omni_perception/config/laser_filter.yaml`:
   - Update `footprint_filter` box parameters to calibrated bounds: min_x: -0.135, max_x: 0.135, min_y: -0.135, max_y: 0.135.
3. Single TF Authority & Extrinsics Alignment:
   - Ensure ekf_node alone publishes `odom -> base_link` (publish_tf: true in ekf.yaml, publish_tf: false in simulation.yaml).
   - Check URDF/xacro (`chassis.xacro`, `sensors.xacro`) per m3_tf_analysis.md and m3_extrinsics_laser_analysis.md to prevent dual-parent TF tree issues.
4. Guidelines:
   - Follow GitNexus guidelines in AGENTS.md.
   - Apply Ponytail standard (.claude/skills/ponytail): clean, concise, stdlib/native ROS 2, no bloat.

Verification:
Run tests:
- `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py -v`
- `python3 -m pytest tests/e2e/test_production_repo_readiness.py -v`
- `python3 -m pytest -k "ekf or f3 or covariance or laser" tests/e2e -v`
Ensure all M3-related tests pass and audit readiness test for M3 passes.

Write your handoff report to `/home/sonev/amr_omni/.agents/teamwork_preview_worker_m3_2/handoff.md` and send a message back to parent when complete.
