# BRIEFING — 2026-09-19T10:01:05Z

## Mission
Investigate and specify concrete interface, protocol, and fusion requirements for R2, R3, and R4 (Serial Binary Protocol, EKF & Covariance Fusion, Laser Filter Configuration, Web UI Calibration Pipeline) for Mecanum AGV calibration and estimation upgrade.

## 🔒 My Identity
- Archetype: teamwork_preview_spec_miner
- Roles: Specification Miner
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: Survey Phase

## 🔒 Key Constraints
- Read-only analysis of codebase /home/sonev/amr_omni
- DO NOT modify any code or configuration files in /home/sonev/amr_omni
- Write only to /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3
- Concrete packet layouts, YAML schemas, TF tree diagrams, and Web API endpoint designs required

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:01:05Z

## Task Summary
- **What to build**: Specification report and handoff for Serial Binary Protocol, EKF & Covariance Fusion, Laser Filter Configuration, and Web UI Calibration Pipeline.
- **Success criteria**: Complete `survey_integration_specs.md` and `handoff.md` with concrete schemas, packet byte layouts, TF tree analysis, and API specs.
- **Interface contracts**: ORIGINAL_REQUEST.md
- **Code layout**: Read from `/home/sonev/amr_omni`, write to `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3/`

## Key Decisions Made
- [initialization]
- Serial Contract: Specified framed binary layout (0xAA 0x55, length, seq, msg_id, payload, CRC16-CCITT, 0x7D) covering calibration commands (0x10..0x15) and telemetry frames (0x80..0x84).
- EKF Covariance: Identified zero diagonal omission in firmware (odometry & IMU) and simulator; specified complete non-zero diagonal entries for 6x6 pose/twist, 3x3 IMU, and 15x15 process noise Q and initial covariance P0.
- Single TF Authority: Confirmed ekf_node as exclusive publisher of odom -> base_link, simulator publish_tf: false, and slam_toolbox as exclusive map -> odom publisher.
- Laser Filter: Identified 0.16m box filter oversizing; specified 0.135m footprint box filter + shadow filter + speckle filter.
- Web UI Pipeline: Designed FastAPI /api/calib/* REST endpoints, WebSocket /ws/telemetry calib payload, and YAML schemas (imu_calib.yaml, wheel_calib.yaml, ekf.yaml).

## Artifact Index
- DISPATCH.md — record of dispatch instructions
- BRIEFING.md — persistent state and context
- progress.md — liveness heartbeat and step tracking
- survey_integration_specs.md — comprehensive specification mining document
- handoff.md — 5-component handoff report
