# Scope: E2E Testing Track

## Architecture & Test Philosophy
- Requirement-driven, opaque-box testing derived directly from ORIGINAL_REQUEST.md and PROJECT.md.
- Verification mechanism must NOT depend on internal implementation details.
- System test runner executes end-to-end scenarios covering firmware algorithms, kinematics consistency, EKF state estimation under vibration, and Web UI calibration workflows.

## Deliverables
1. `TEST_INFRA.md` at project root documenting test architecture, runner commands, and tier thresholds.
2. Complete 4-tier automated test suite under `/home/sonev/teamwork_projects/amr_omni_calib/tests/e2e/` (or package test suites):
   - Tier 1: Feature Coverage (>=5 tests per feature)
   - Tier 2: Boundary & Corner Cases (>=5 tests per feature with boundary conditions)
   - Tier 3: Cross-Feature Combinations (pairwise interactions)
   - Tier 4: Real-World Workload Scenarios (realistic driving, high-speed braking, calibration life-cycle)
3. `TEST_READY.md` at project root upon test suite completion.

## Completion Criteria
- Test runner executes with exit code 0 when all tests pass.
- Full documentation of test coverage in TEST_READY.md.
