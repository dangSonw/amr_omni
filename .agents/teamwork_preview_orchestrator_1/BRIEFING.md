# BRIEFING — 2026-09-19T11:32:45Z

## Mission
Orchestrate the end-to-end upgrade of Mecanum AGV `amr_omni` calibration and state estimation system per ORIGINAL_REQUEST.md requirements R1-R5 and acceptance criteria.

## 🔒 My Identity
- Archetype: teamwork_preview_orchestrator
- Roles: orchestrator, user_liaison, human_reporter, successor
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_1
- Original parent: parent (Sentinel)
- Original parent conversation ID: d998665c-170e-4bf6-93e4-163920e53677

## 🔒 My Workflow
- **Pattern**: Project Pattern (Dual Track: Implementation Track + E2E Testing Track)
- **Scope document**: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
1. **Decompose**: Survey completed -> PROJECT.md created with Feature Inventory and 5 Milestones + E2E Track -> Interface contracts defined
2. **Dispatch & Execute**:
   - E2E Testing Track: COMPLETED (151 test cases, TEST_INFRA.md, TEST_READY.md published)
   - Milestone 1: COMPLETED and PASSED GATE (16/16 native tests, disco_f407vg built, 13/13 ROS 2 tests, 50k Monte Carlo, Forensic Auditor CLEAN)
   - Milestone 2: COMPLETED and PASSED GATE (unanimous approval: 34 native tests, 4/4 stress suites, 20/20 Python stress tests, 2 Reviewers, 2 Challengers, Forensic Auditor CLEAN)
   - Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter (IN PROGRESS)
   - Milestone 4: Serial Calibration Protocol & Jetson Web UI [pending M3]
3. **On failure**: Retry -> Replace -> Skip -> Redistribute -> Redesign
4. **Succession**: Self-succeed at 16 spawns
- **Work items**:
  0. Survey & Spec Mining [DONE]
  1. PROJECT.md definition [DONE]
  2. E2E Testing Track [DONE — TEST_READY.md published]
  3. Milestone 1: Encoder PLL & Kinematics Consistency [DONE — GATE PASSED]
  4. Milestone 2: IMU Intrinsic Calibration & Filtering on STM32 [DONE — GATE PASSED]
  5. Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter [IN PROGRESS]
  6. Milestone 4: Serial Calibration Protocol & Jetson Web UI [pending]
  7. Milestone 5: E2E Verification & Adversarial Hardening [pending]
- **Current phase**: 1 (Dual Track Execution)
- **Current focus**: Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter

## 🔒 Key Constraints
- DISPATCH-ONLY orchestrator: MUST NOT write code nor solve problems directly. NEVER modify source files directly.
- NEVER run build/test commands yourself — require workers to do so.
- NEVER investigate or explore the problem at the code level — dispatch Explorers for technical investigation.
- You MAY use file-editing tools ONLY for metadata/state files (.md) in .agents/ folder and PROJECT.md / TEST_READY.md.
- Hard audit enforcement: Forensic Auditor integrity violation is a binary veto. No advancement if audit fails.
- Never reuse a subagent after handoff — always spawn fresh.
- Succession threshold: 16 spawns.

## Current Parent
- Conversation ID: d998665c-170e-4bf6-93e4-163920e53677
- Updated: 2026-09-19T11:12:29Z

## Key Decisions Made
- Milestone 2 Gate PASSED unanimously following successful remediation of face re-start bug, state protection, and Welford stationarity gating.
- Proceeding to Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter (R3).

## Team Roster
| Agent | Type | Work Item | Status | Conv ID |
|-------|------|-----------|--------|---------|
| miner_survey_1 | teamwork_preview_spec_miner | Mathematical theory & docx analysis | completed | b0a1c2e1-2c69-4e07-ab92-fd795514fe12 |
| explorer_survey_2 | teamwork_preview_explorer | amr_omni codebase mapping & GitNexus | completed | 71dbd81e-e99d-455e-8c10-61889f550bdb |
| miner_survey_3 | teamwork_preview_spec_miner | Protocols, EKF, Laser Filter, Web UI | completed | 4bb55b3f-13f2-4347-8881-30d286c147fc |
| test_writer_e2e_1 | teamwork_preview_test_writer | E2E 4-Tier Test Suite & TEST_INFRA.md | completed | f66d8d5a-7659-4561-8583-20a97c565cc6 |
| explorer_m1_1 | teamwork_preview_explorer | Milestone 1 Implementation Design | completed | 7723ddba-bf23-4ea8-8f59-c0d52def030a |
| explorer_m2_1 | teamwork_preview_explorer | Milestone 2 Implementation Design | completed | 3712b4d0-d63c-498a-92d6-d229eb3e3504 |
| worker_m1_1 | teamwork_preview_worker | Milestone 1 Implementation & Tests | completed | d04c53b8-a609-4713-bef4-c712118fd7b2 |
| reviewer_m1_1 | teamwork_preview_reviewer | M1 Review (Correctness & Tests) | completed | 3d0ba7d3-69ef-4d0d-af12-2fa2f1cb71ef |
| reviewer_m1_2 | teamwork_preview_reviewer | M1 Review (Robustness & Stability) | completed | 524b4463-d081-4a1f-a177-95bb8ff36e1e |
| challenger_m1_1 | teamwork_preview_challenger | M1 Stress (PLL & Dynamics) | completed | a1cf943f-089a-4529-9dc0-c0617c15f6c9 |
| challenger_m1_2 | teamwork_preview_challenger | M1 Stress (Kinematics Invariants) | completed | 57676407-c4e3-48cf-86c1-a5638a1a41da |
| auditor_m1_1 | teamwork_preview_auditor | M1 Forensic Integrity Audit | completed | c050867b-3d70-4568-8527-0d12690c34ce |
| worker_m2_2 | teamwork_preview_worker | Milestone 2 Implementation & Tests | completed | 219e3e2c-9fff-4c55-a783-dc7f10d7e538 |
| reviewer_m2_1 | teamwork_preview_reviewer | M2 Review (Correctness & Tests) | completed | 39d98da8-5e8b-4cbe-910e-55544e709897 |
| reviewer_m2_2 | teamwork_preview_reviewer | M2 Review (Robustness & Covariances) | completed | 9eeba6e3-531c-4bfa-a240-8ff44778d7e0 |
| challenger_m2_1 | teamwork_preview_challenger | M2 Stress (AN4508 Accel & Norm Error) | completed | 22b2e408-595b-41a8-82e3-e52b86c39470 |
| challenger_m2_2 | teamwork_preview_challenger | M2 Stress (Gyro Drift & Disturbance) | completed | dbf337aa-6b4a-40e7-8021-e7b0fc79d067 |
| auditor_m2_1 | teamwork_preview_auditor | M2 Forensic Integrity Audit | completed | 87dd524d-98ed-420a-8301-80b3d6956ac2 |
| explorer_m2_2 | teamwork_preview_explorer | M2 Remediation Plan (Iteration 2) | completed | e543b29f-a31c-4607-85d3-903437d81499 |
| worker_m2_3 | teamwork_preview_worker | M2 Remediation Implementation (Iter 2) | completed | b414c4c2-62cc-4918-9cb3-e3bbc9df414a |
| challenger_m2_3 | teamwork_preview_challenger | M2 Stress (Remediation Verification) | completed | 4e6aba75-28da-4007-9d2d-c124c4bffd84 |
| auditor_m2_2 | teamwork_preview_auditor | M2 Forensic Integrity Audit (Iter 2) | completed | 5353090a-c95a-4690-b09a-980975c26574 |
| explorer_m3_1 | teamwork_preview_explorer | M3 EKF Configuration & Covariances | completed | caa2f2cc-23af-46de-8387-951c26642ef7 |
| explorer_m3_2 | teamwork_preview_explorer | M3 Single TF Authority Audit | completed | a6d1a262-362c-4684-913b-c9205c6b0829 |
| explorer_m3_3 | teamwork_preview_explorer | M3 Extrinsics & Laser Filter Design | completed | e3eba7e2-95e1-4c3a-818a-0c272d4033af |
| worker_m3_1 | teamwork_preview_worker | Milestone 3 Implementation & Tests | in-progress | 165c0ee9-e567-4116-ad3f-b182b5709678 |

## Succession Status
- Succession required: no
- Spawn count: 26 / 128
- Pending subagents: 165c0ee9-e567-4116-ad3f-b182b5709678
- Predecessor: none
- Successor: not yet spawned

## Active Timers
- Heartbeat cron: 709d5506-1905-49c5-bf69-8e756d885098/task-220 (every 10m)
- Safety timer: covered by heartbeat cron
- On succession: kill all timers before spawning successor

## Artifact Index
- /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md — Original User Request
- /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md — Global architecture and milestones
- /home/sonev/teamwork_projects/amr_omni_calib/TEST_INFRA.md — E2E Test Suite Infrastructure
- /home/sonev/teamwork_projects/amr_omni_calib/TEST_READY.md — E2E Test Suite Ready Sign-off
- /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_1/GATE_STATUS.md — Gate Status
- /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_1/progress.md — Progress tracker
