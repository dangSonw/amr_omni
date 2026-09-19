# BRIEFING — 2026-09-19T10:08:30Z

## Mission
Survey Phase: Investigate and extract full mathematical models, algorithms, equations, and technical specifications from reference docs in /home/sonev/amr_omni/temp/docs for Mecanum AGV calibration and estimation upgrade.

## 🔒 My Identity
- Archetype: specification_miner
- Roles: Teamwork specialist, Specification Miner
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: Survey Phase (Mecanum AGV Calibration & Estimation)

## 🔒 Key Constraints
- Read-only analysis and specification extraction.
- DO NOT modify any code or configuration files.
- Write only to working directory.
- Complete mathematical rigor: extract exact equations, formulas, state vectors, parameters, error models.

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:08:30Z

## Task Summary
- **What to build**: Comprehensive theoretical specifications document (`survey_theory_specs.md`) and self-contained handoff report (`handoff.md`).
- **Success criteria**: Complete mathematical equations for R1 (PLL Observer, M/T, Kinematics), R2 (IMU bias, accel 6-pos, Allan variance, ENU), R3 (lever-arm extrinsics, latency compensation), recommended parameters and tolerances.
- **Interface contracts**: ORIGINAL_REQUEST.md
- **Code layout**: Read-only survey. Artifacts in `.agents/teamwork_preview_spec_miner_survey_1/`.

## Key Decisions Made
- Extracted and analyzed all 4 docx files (`amr_omni.docx`, `Encoder.docx`, `Calib.docx`, `Calib_2.docx`).
- Formulated exact mathematical state equations, transfer functions, and discrete updates for ODrive 2nd-order PLL observer ($k_p = 2\omega_{pll}, k_i = \omega_{pll}^2$).
- Formulated LinuxCNC M/T hybrid velocity estimation and two's complement 16-bit rollover logic.
- Proved Mecanum Moore-Penrose pseudoinverse closed-form consistency $FK(IK(v)) \equiv v$ with zero round-trip error.
- Formulated ST AN4508 6-position accelerometer least squares and Tedaldi et al. (ICRA 2014) non-orthogonal 12-parameter IMU calibration.
- Formulated Allan variance stochastic parameters ($N_g, K_g, N_a, K_a, B$) and lab-to-field covariance inflation rule ($1.5\times - 2.0\times$).
- Formulated spatial lever-arm kinematics (Coriolis & centripetal acceleration compensation) and temporal latency compensation with continuous B-splines (Kalibr).
- Codified Single TF Authority rule, EKF measurement & process noise covariance matrices, laser box filter geometry, and binary serial contract.

## Artifact Index
- `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/survey_theory_specs.md` — Detailed theoretical specifications (Features Discovered table, Edge Cases table, full mathematical derivations)
- `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/handoff.md` — Final self-contained handoff report
- `/home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_spec_miner_survey_1/progress.md` — Liveness heartbeat & progress log
