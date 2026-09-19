# BRIEFING — 2026-09-19T11:51:50Z

## Mission
Investigate and audit the amr_omni codebase for Single TF Authority compliance, identify all TF broadcasters, verify test requirements, and propose precise configurations and diffs.

## 🔒 My Identity
- Archetype: explorer
- Roles: Technical Explorer 2 (Milestone 3 — Single TF Authority Audit & Enforcement)
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m3_2
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: M3 (Single TF Authority Audit & Enforcement)

## 🔒 Key Constraints
- Read-only investigation — do NOT implement / do NOT modify production source files
- All output to /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m3_2
- Detailed findings in m3_tf_analysis.md and handoff.md
- Use send_message to report to parent

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T11:43:30Z

## Investigation State
- **Explored paths**:
  - `src/omni_localization/config/ekf.yaml`, `launch/ekf.launch.py`, `launch/localization.launch.py`, `launch/slam.launch.py`
  - `src/omni_simulation/omni_simulation/stm32_simulator.py`, `config/simulation.yaml`, `launch/simulation.launch.py`
  - `src/omni_hardware/omni_hardware/stm32_bridge.py`, `config/hardware.yaml`
  - `src/omni_description/urdf/omni.urdf.xacro`, `chassis.xacro`, `sensors.xacro`
  - `src/omni_bringup/launch/simulation_bringup.launch.py`
  - `src/omni_perception/config/laser_filter.yaml`, `launch/perception.launch.py`
  - `tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py`
- **Key findings**:
  - `ekf_node` is confirmed as the designated sole authority for dynamic `odom -> base_link` transform.
  - `stm32_simulator.py` defaults to `publish_tf: False`, and `simulation.yaml` sets `publish_tf: false`.
  - `stm32_bridge.py` does not broadcast any TF.
  - Critical topological bug found in URDF: `chassis.xacro` defines `base_footprint -> base_link` which conflicts with EKF's `odom -> base_link`, giving `base_link` two parents. URDF must be modified so `base_link` is parent of `base_footprint`.
  - Sensor frame aliasing (`imu_link` vs `imu_link_1`, `laser_link` vs `lidar_link_1`) identified.
  - 100% of Single TF Authority E2E tests (25/25 in test_f3) pass.
- **Unexplored areas**: None for M3 Single TF Authority.

## Key Decisions Made
- Fully documented the 5 components of handoff and detailed analysis report in `m3_tf_analysis.md`.
- Formulated exact diffs for `ekf.yaml`, `simulation.yaml`, `omni.urdf.xacro`, `chassis.xacro`, and `sensors.xacro`.

## Artifact Index
- DISPATCH.md — Dispatch log
- BRIEFING.md — Situational awareness
- progress.md — Liveness heartbeat
- m3_tf_analysis.md — Comprehensive technical analysis report
- handoff.md — 5-component handoff report
