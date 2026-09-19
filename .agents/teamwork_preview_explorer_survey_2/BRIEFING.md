# BRIEFING — 2026-09-19T10:11:00Z

## Mission
Survey and map the existing architecture of the AMR Omni system (STM32 firmware, ROS 2 packages, Jetson Web UI, GitNexus status) for Mecanum AGV calibration and estimation upgrade.

## 🔒 My Identity
- Archetype: explorer
- Roles: survey, architectural mapping, synthesis
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: Survey Phase

## 🔒 Key Constraints
- Read-only investigation — do NOT implement
- Do NOT modify any code or configuration files in /home/sonev/amr_omni or outside working directory
- Write only to /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_survey_2

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:11:00Z

## Investigation State
- **Explored paths**:
  - `firmware/stm32_f407vg_arduino_sim/` (`main.cpp`, `hardware.cpp`, `kinematics.cpp`, `kalman.cpp`, `pid.cpp`, `platformio.ini`)
  - `src/` (10 packages: `omni_bringup`, `omni_control`, `omni_description`, `omni_hardware`, `omni_localization`, `omni_navigation`, `omni_perception`, `omni_safety`, `omni_simulation`, `omni_bringup_ros1`)
  - `web/` (`backend/` FastAPI app, routers, models, bridges, services; `frontend/` Next.js, components, pages)
  - `scripts/` (`setup.sh`, `build.sh`, `run_sim.sh`, `run_web.sh`, `calibrate.sh`)
  - `temp/docs/` (`amr_omni.docx`, `Encoder.docx`, `Calib.docx`, `Calib_2.docx`)
  - GitNexus CLI (`node .gitnexus/run.cjs query/impact --repo amr_omni`)
- **Key findings**:
  - Firmware runs FreeRTOS with 6 tasks and micro-ROS serial transport; encoder task uses backward difference with ScalarKalman smoothing (candidate for ODrive PLL observer R1).
  - IMU BNO08x driver is uncalibrated on MCU; static bias tracking and scale factor matrix needed (R2).
  - EKF `ekf.yaml` has default unpopulated covariances, does not fuse rotational rate wz, and risks duplicate TF broadcast against `stm32_simulator` (R3).
  - Laser filter has bounding box in `omni_perception`, but downstream nodes subscribe directly to `/scan` (R3).
  - Web backend currently updates config only in memory; needs calibration APIs and automated YAML saving on disk (R4).
  - Test suites validated: ROS 2 packages (`omni_control` 38 tests pass), Web backend (17 tests pass), PlatformIO project configuration.
- **Unexplored areas**: None for survey phase; full codebase mapped.

## Key Decisions Made
- Fully documented directory structures, formulas, interface contracts, and build/test commands in `survey_codebase_arch.md` and `handoff.md`.

## Artifact Index
- `DISPATCH.md` — Incoming user dispatch log
- `BRIEFING.md` — Agent working memory
- `progress.md` — Liveness heartbeat and step tracker
- `survey_codebase_arch.md` — Comprehensive architectural mapping report
- `handoff.md` — Self-contained 5-component handoff report
