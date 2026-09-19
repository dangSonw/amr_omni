## 2026-09-19T10:12:21Z
You are teamwork_preview_explorer_m2_1, the Technical Explorer for Milestone 2 (M2) — IMU Intrinsic Calibration & Filtering on STM32.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Scope Document: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_m2_1/SCOPE.md
Theory Specs: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/survey_theory_specs.md
Integration Specs: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_3/survey_integration_specs.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md, /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md, and your SCOPE.md completely.

OBJECTIVE:
Investigate and design the exact technical implementation for Milestone 2:
1. Review firmware at /home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim:
   - Current IMU task in `src/main.cpp`.
   - Design `imu_calibration.h` and `imu_calibration.cpp`:
     - ST AN4508 6-position accelerometer calibration routine: linear least-squares calculation of scale $s_j = (\bar{a}_{+j} - \bar{a}_{-j})/(2g)$ and bias $b_j = (\bar{a}_{+j} + \bar{a}_{-j})/2$ for X, Y, Z axes.
     - Stationary gyroscope zero-rate bias calculation: averaging over $\ge 1000$ samples ($\ge 10$ seconds), ensuring static residual drift $< 0.05^\circ/\text{s}$ ($8.72 \times 10^{-4}$ rad/s).
     - Standardize body frame IMU readings to REP-103 ENU (X forward, Y left, Z up) with static gravity $+9.80665$ m/s² on Z.
     - Allan variance noise model parameters ($N_g, N_a, K_g$) and field covariance inflation ($1.5\times - 2.0\times$).
2. Review firmware test setup (`platformio.ini`, `pio test -e native`) and identify where to add unit tests for the IMU calibration routine.
3. Formulate the exact implementation changes, files to be created/modified, and verification test commands.
4. Output your detailed plan to:
   `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1/m2_implementation_plan.md`
   And write your self-contained handoff to:
   `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m2_1/handoff.md`

SCOPE BOUNDARIES:
- Read-only exploration. DO NOT modify production source files.
- Write only to your working directory.

When complete, send a message to parent notifying completion.
