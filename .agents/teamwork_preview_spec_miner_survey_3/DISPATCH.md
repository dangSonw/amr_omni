## 2026-09-19T10:00:47Z
You are teamwork_preview_spec_miner_survey_3, working on the Survey Phase for the Mecanum AGV calibration and estimation upgrade.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Codebase: /home/sonev/amr_omni

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md completely.

OBJECTIVE:
Investigate and specify the concrete interface, protocol, and fusion requirements for R2, R3, and R4:
1. Serial Binary Protocol (STM32 <-> Jetson):
   - Current serial packet structure in /home/sonev/amr_omni (header, packet ID, payload, checksum/CRC).
   - Requirements for calibration command frames: Trigger IMU calibration, Trigger wheel calibration, Start noise profiling.
   - Requirements for calibration telemetry/result frames: real-time calibration progress percentage, estimated biases/matrices, status codes.
2. EKF & Covariance Fusion (`robot_localization`):
   - Inspect existing EKF YAML config (`ekf.yaml` / `robot_localization.yaml`).
   - Identify diagonal zeros or default covariance values that need actual measured variances.
   - Determine which node publishes `odom -> base_link` transform. Enforce Single TF Authority principle (prevent duplicate TF from motor driver / odometry node vs EKF).
3. Laser Filter Configuration:
   - Identify current laser filter config or launch files (`laser_filters`).
   - Define exact angular or box filter geometry to mask chassis/frame blind spots without clipping real environmental scans.
4. Web UI Calibration Pipeline:
   - Current Web UI REST / WebSocket architecture in the repo.
   - Workflow: Operator triggers calib via Web UI -> Backend sends serial command to STM32 -> STM32 computes -> STM32 sends back results -> Backend updates YAML config (e.g. `imu_calib.yaml`, `wheel_calib.yaml`, `ekf.yaml`) -> Web UI displays plots/tables.

SCOPE BOUNDARIES:
- Read-only analysis.
- DO NOT modify any code or configuration files.
- Write only to your working directory.

OUTPUT REQUIREMENTS:
Write your detailed findings to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3/survey_integration_specs.md`
And write your final handoff to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3/handoff.md`

Your handoff must provide concrete packet layouts, YAML schemas, TF tree diagrams, and Web API endpoint designs.

When finished, send a message to the orchestrator notifying completion.
