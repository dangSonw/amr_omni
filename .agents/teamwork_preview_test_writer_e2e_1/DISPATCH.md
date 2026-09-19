# DISPATCH for teamwork_preview_test_writer_e2e_1
Role: E2E Test Writer
Scope: E2E Testing Track (Tiers 1-4)
Working Directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_test_writer_e2e_1

## 2026-09-19T10:12:21Z
You are teamwork_preview_test_writer_e2e_1, the E2E Test Writer for the Mecanum AGV calibration and estimation upgrade project.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_test_writer_e2e_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md completely.

OBJECTIVE:
Design and build the comprehensive, requirement-driven, opaque-box E2E test suite:
1. Create /home/sonev/teamwork_projects/amr_omni_calib/TEST_INFRA.md following the template in Project Pattern:
   - Test philosophy (opaque-box, requirement-driven, testing external interfaces).
   - Feature inventory coverage table (mapping tests to features F1.1-F5.2).
   - 4-Tier test architecture.
2. Implement automated test cases under /home/sonev/teamwork_projects/amr_omni_calib/tests/e2e/:
   - Tier 1: Feature Coverage (>=5 test cases per feature across R1-R5 features).
   - Tier 2: Boundary & Corner Cases (>=5 test cases per feature with boundaries: extreme speeds, zero-speed watchdog, rollover, maximum noise, saturated biases).
   - Tier 3: Cross-Feature Interactions (pairwise: encoder velocity + EKF fusion, IMU calib + wheel calib, Web UI command + serial timeout, TF tree + laser filter).
   - Tier 4: Real-World Workload Scenarios (realistic robot trajectories, sharp acceleration & braking without EKF divergence, full calibration lifecycle to YAML).
3. Create a master test runner script: `/home/sonev/teamwork_projects/amr_omni_calib/tests/e2e/run_tests.sh` (or pytest runner) that can execute all tiers or individual tiers, returning exit code 0 when all pass.
4. Verify tests can execute. Document any currently failing tests (as implementation is in progress).
5. When test infrastructure and test cases are ready, publish /home/sonev/teamwork_projects/amr_omni_calib/TEST_READY.md with summary table.
6. Write your handoff.md in your working directory.

SCOPE BOUNDARIES:
- DO NOT modify production implementation code (firmware, ROS 2 packages, web backend).
- Only write test cases in tests/e2e/ and documentation files (TEST_INFRA.md, TEST_READY.md, handoff.md).

When complete, send a message to parent notifying that TEST_READY.md is published.
