## 2026-09-19T10:00:47Z
You are teamwork_preview_explorer_survey_2, working on the Survey Phase for the Mecanum AGV calibration and estimation upgrade.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Codebase: /home/sonev/amr_omni

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/AGENTS.md completely.

OBJECTIVE:
Investigate and map the existing architecture of the AMR Omni system across the entire codebase at /home/sonev/amr_omni:
1. STM32 Firmware:
   - Identify where the STM32 codebase resides (e.g. firmware directory, PlatformIO/STM32CubeIDE/Makefile project, RTOS vs bare metal).
   - Find encoder reading logic, timer interrupts, motor control, and IMU communication drivers.
   - Find existing serial communication code (UART/USB CDC packet handling, protocol formats).
2. ROS 2 Workspace Packages:
   - Enumerate all ROS 2 packages under /home/sonev/amr_omni.
   - Map kinematics node (`amr_kinematics` or similar): forward kinematics, inverse kinematics, wheel geometry parameters (wheelbase, track width, wheel radius).
   - Locate robot_localization config, EKF launch files, sensor topic publishers (/odom, /imu/data, /scan).
   - Locate laser_filter configurations and URDF/TF transform broadcasters.
3. Jetson Web UI:
   - Map the Web backend (FastAPI / Python) and frontend (Vue/React/HTML).
   - Check existing API endpoints, WebSocket/SSE connections, and configuration saving mechanisms.
4. GitNexus Status & Impact:
   - Check GitNexus context if available per AGENTS.md.

SCOPE BOUNDARIES:
- Read-only exploration.
- DO NOT modify any code or configuration files.
- Write only to your working directory.

OUTPUT REQUIREMENTS:
Write your detailed findings to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2/survey_codebase_arch.md`
And write your final handoff to:
`/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2/handoff.md`

Your handoff must be self-contained and detail:
- Complete directory & file structure for STM32 firmware, ROS 2 packages, and Web UI.
- File paths that will be affected by R1, R2, R3, R4, R5.
- Existing Kinematics formulas and interface contracts.
- Existing build & test commands for each subsystem.

When finished, send a message to the orchestrator notifying completion.
