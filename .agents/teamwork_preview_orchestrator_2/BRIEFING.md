# BRIEFING — 2026-09-20T07:11:30Z

## Mission
Orchestrate the AMR Omni Mecanum AGV project execution: verify prior milestones M1-M2, complete M3 (EKF Covariances, Single TF Authority, Laser Filter Masking), M4 (Serial Calibration Protocol & Web UI), and M5 (100% pass of 191+ tests including E2E and stress tests) with forensic integrity.

## 🔒 My Identity
- Archetype: teamwork_preview_orchestrator
- Roles: orchestrator, user_liaison, human_reporter, successor
- Working directory: /home/sonev/amr_omni/.agents/teamwork_preview_orchestrator_2
- Original parent: parent
- Original parent conversation ID: 1a8ac5a2-a18d-4c72-aa2f-92d3a9ba4de1

## 🔒 My Workflow
- **Pattern**: Project Pattern (Dual Track: Implementation Track + E2E Testing Track)
- **Scope document**: /home/sonev/amr_omni/PROJECT.md
1. **Decompose**:
   - Survey & E2E Testing Track: COMPLETED (151 test cases, TEST_READY.md published)
   - Milestone 1: Encoder PLL & Kinematics Consistency [DONE — GATE PASSED]
   - Milestone 2: IMU Intrinsic Calibration & Filtering on STM32 [DONE — GATE PASSED]
   - Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter [IN PROGRESS]
   - Milestone 4: Serial Calibration Protocol & Jetson Web UI [PLANNED]
   - Milestone 5: E2E Verification & Adversarial Coverage Hardening (191+ tests pass 100%) [PLANNED]
2. **Dispatch & Execute**:
   - Direct iteration loop: Explorer -> Worker -> Reviewers + Challengers + Forensic Auditor -> Gate
3. **On failure**: Retry -> Replace -> Skip -> Redistribute -> Redesign
4. **Succession**: Self-succeed at 16 spawns
- **Work items**:
  0. Survey & E2E Test Suite [DONE]
  1. Milestone 1: Encoder PLL & Kinematics Consistency [DONE]
  2. Milestone 2: IMU Intrinsic Calibration & Filtering on STM32 [DONE]
  3. Milestone 3: Extrinsics, Covariances, TF Authority & Laser Filter [IN PROGRESS]
  4. Milestone 4: Serial Calibration Protocol & Jetson Web UI [PLANNED]
  5. Milestone 5: Full Test Suite Validation (191+ tests pass 100%) & Hardening [PLANNED]
- **Current phase**: 1 (Dual Track Execution)
- **Current focus**: Milestone 3 Implementation & Gate Verification

## 🔒 Key Constraints
- DISPATCH-ONLY orchestrator: MUST delegate ALL work to subagents. NEVER write, modify, or create source code files directly.
- NEVER run build/test commands yourself — require workers to do so.
- NEVER investigate or explore the problem at the code level — dispatch Explorers for technical investigation.
- You MAY use file-editing tools ONLY for metadata/state files (.md) in your .agents/ folder and PROJECT.md / TEST_READY.md.
- Hard audit enforcement: Forensic Auditor integrity violation is a binary veto. No advancement if audit fails.
- Never reuse a subagent after handoff — always spawn fresh.
- Ponytail standard: concise code, stdlib + ROS 2 native nodes, eliminate boilerplate/unneeded abstractions (YAGNI), minimal diff.
- Single TF Authority: only ekf_node broadcasts odom -> base_link TF.
- Parity binary serial protocol frames between STM32 C++ and Simulator.
- 100% passing tests (pytest tests/, 191+ tests).

## Current Parent
- Conversation ID: 1a8ac5a2-a18d-4c72-aa2f-92d3a9ba4de1
- Updated: 2026-09-20T07:11:30Z

## Key Decisions Made
- Inherited verified milestones M1 and M2 from teamwork_preview_orchestrator_1.
- M3 analysis completed by explorers m3_1, m3_2, m3_3.
- Need worker to implement M3 deliverables: ekf.yaml covariances, laser_filter.yaml footprint box filter ([-0.135, 0.135]), URDF base_footprint/base_link hierarchy.
- Following M3, dispatch M4 (Serial protocol + Web UI) and M5 (full 191+ test suite verification).

## Team Roster
| Agent | Type | Work Item | Status | Conv ID |
|-------|------|-----------|--------|---------|
| worker_m3_2 | teamwork_preview_worker | Milestone 3 Implementation & Tests | completed | 8a7de43e-51d8-42ac-9b9a-1fa0e3bf8899 |
| reviewer_m3_1 | teamwork_preview_reviewer | M3 Review (Correctness & Tests) | completed | f2d3bb18-0ac7-4e5d-99ef-a5d5cd3c5881 |
| reviewer_m3_2 | teamwork_preview_reviewer | M3 Review (Robustness & Single TF) | completed | f46419d8-3321-4f9a-8cc4-48d661e2dd0a |
| challenger_m3_1 | teamwork_preview_challenger | M3 Stress (Covariances & SPD) | completed | 15eab346-07b3-4be9-a014-f5aec9b93400 |
| challenger_m3_2 | teamwork_preview_challenger | M3 Stress (TF Topology & Laser) | completed | 6c1a2d8c-0379-4dc7-bb40-978fddf8e935 |
| auditor_m3_1 | teamwork_preview_auditor | M3 Forensic Integrity Audit | completed | 438989fd-56ae-4ff3-aca5-a7cc96fe3545 |
| explorer_m4_1 | teamwork_preview_explorer | M4 Serial Protocol Parity Exploration | completed | adb1384b-e6c3-4a5c-a2d7-c0bdae58fb97 |
| explorer_m4_2 | teamwork_preview_explorer | M4 FastAPI Backend & YAML Exploration | completed | 8a696b51-e17b-448f-a543-f3ee3d108921 |
| explorer_m4_3 | teamwork_preview_explorer | M4 Web Frontend & E2E Exploration | completed | c8252645-031d-4c6c-a4f5-7ec8752596bc |
| worker_m4_1 | teamwork_preview_worker | M4 Implementation & Unit Tests | completed | 6630952d-59a0-4a66-9ab0-a34632de5cfd |
| reviewer_m4_1 | teamwork_preview_reviewer | M4 Review (C++ Firmware Serial Protocol) | completed | 47d591e7-82bc-473e-934c-5e247af7860f |
| reviewer_m4_2 | teamwork_preview_reviewer | M4 Review (FastAPI & YAML Persistence) | completed | 5c818522-77a9-4a83-9586-c0303b973f62 |
| challenger_m4_1 | teamwork_preview_challenger | M4 Stress (Serial Codec Fuzzing) | completed | c83c0a8b-7c59-4f13-bbd3-54101c6ee829 |
| challenger_m4_2 | teamwork_preview_challenger | M4 Stress (Concurrent YAML Writes) | errored | d2a1f155-e6af-4232-986a-1fe9da82ae7f |
| auditor_m4_1 | teamwork_preview_auditor | M4 Forensic Integrity Audit | completed | acc0ef21-3b48-4486-86f2-0d822e798fc9 |
| worker_m4_2 | teamwork_preview_worker | M4 Atomic YAML Concurrency Hardening | completed | c56f2ed4-28e1-454c-8d48-241d5815afeb |
| reviewer_m5_1 | teamwork_preview_reviewer | M5 Phase 1 Full Test Suite Verification | completed | 76a17d79-e34d-4fab-b55c-bc19d940f016 |
| challenger_m5_1 | teamwork_preview_challenger | M5 Phase 2 Firmware Adversarial Hardening | completed | 9f6f949d-a989-4803-ab86-11c6c9dd0aac |
| challenger_m5_2 | teamwork_preview_challenger | M5 Phase 2 ROS2 & Web Adversarial Hardening | completed | 25fa8a68-7f70-44a3-8b5e-69717af9fa6e |
| auditor_m5_1 | teamwork_preview_auditor | M5 Final Project Forensic Integrity Audit | completed | 9028fb34-de4e-448e-af96-437fe16a94d4 |

## Succession Status
- Succession required: no (project complete)
- Spawn count: 20 / 128
- Pending subagents: none
- Predecessor: teamwork_preview_orchestrator_1
- Successor: none (completed)

## Active Timers
- Heartbeat cron: e684f9d6-654f-439a-9e8c-99049f9780b5/task-255
- Safety timer: covered by heartbeat cron
- On succession: kill all timers before spawning successor

## Artifact Index
- /home/sonev/amr_omni/ORIGINAL_REQUEST.md — Original User Request
- /home/sonev/amr_omni/PROJECT.md — Global architecture and milestones
- /home/sonev/amr_omni/TEST_INFRA.md — E2E Test Suite Infrastructure
- /home/sonev/amr_omni/TEST_READY.md — E2E Test Suite Ready Sign-off
- /home/sonev/amr_omni/.agents/teamwork_preview_orchestrator_2/DISPATCH.md — Dispatch log
- /home/sonev/amr_omni/.agents/teamwork_preview_orchestrator_2/progress.md — Progress tracker
- /home/sonev/amr_omni/.agents/teamwork_preview_orchestrator_2/GATE_STATUS.md — Gate status tracker
