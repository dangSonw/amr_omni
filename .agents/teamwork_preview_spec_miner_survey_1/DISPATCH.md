## 2026-09-19T10:00:47Z
You are teamwork_preview_spec_miner_survey_1, working on the Survey Phase for the Mecanum AGV calibration and estimation upgrade.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md completely.

OBJECTIVE:
Investigate and extract the full mathematical models, algorithms, equations, and technical specifications from the reference documents in /home/sonev/amr_omni/temp/docs:
1. `amr_omni.docx`: Architectural overview, Single TF authority rule, STM32 contract principles, Kinematics consistency, Covariance tuning.
2. `Encoder.docx`: Motor velocity estimation: Second-Order PLL Tracking Observer (ODrive), LinuxCNC M/T hybrid velocity estimation, wheel radius error compensation, forward/inverse kinematics consistency formulas.
3. `Calib.docx`: IMU intrinsic calibration: Tedaldi et al. (ICRA 2014) model, `imu_tk`, ST AN4508 6-position static calibration, Allan variance (`imu_utils`) parameters for gyro bias and accel scale/misalignment, ENU coordinate conversion.
4. `Calib_2.docx`: Continuous-time B-spline spatial extrinsics (lever-arm kinematics between base_link, IMU, LiDAR) and temporal latency compensation principles (Kalibr toolbox).

NOTE ON READING .DOCX FILES:
You can inspect and extract text from .docx files using python scripts (e.g. using `python3 -c "import docx..."` or by treating .docx as a zip archive and inspecting `word/document.xml`).

SCOPE BOUNDARIES:
- Read-only analysis and specification extraction.
- DO NOT modify any code or configuration files.
- Write only to your working directory.

OUTPUT REQUIREMENTS:
Write your detailed findings to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/survey_theory_specs.md`
And write your final handoff to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/handoff.md`

Your handoff must be self-contained and summarize:
- Complete mathematical equations for R1 (PLL Observer, M/T, Kinematics)
- Complete calibration procedure & equations for R2 (IMU bias, accel 6-pos, Allan variance, ENU)
- Extrinsics & latency compensation models for R3
- Recommended parameter values and tolerances.

When finished, send a message to the orchestrator notifying completion.
