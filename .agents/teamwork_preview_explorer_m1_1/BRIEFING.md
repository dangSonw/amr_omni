# BRIEFING — 2026-09-19T10:19:00Z

## Mission
Investigate and design exact technical implementation for Milestone 1: Encoder Velocity Estimation (firmware PLL + M/T hybrid + rollover/watchdog) and Kinematics Consistency (ROS 2 kinematics Kr calibration matrix + closed-form Moore-Penrose pseudoinverse).

## 🔒 My Identity
- Archetype: explorer
- Roles: Technical Explorer for Milestone 1 (M1)
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m1_1
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: Milestone 1 (M1) — Encoder Velocity Estimation & Kinematics Consistency

## 🔒 Key Constraints
- Read-only investigation — do NOT implement / modify production files
- Write only to own folder (.agents/teamwork_preview_explorer_m1_1)
- Follow GitNexus rules per AGENTS.md

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:19:00Z

## Investigation State
- **Explored paths**:
  - `firmware/stm32_f407vg_arduino_sim/src/main.cpp:508-525` (`encoder_task`)
  - `firmware/stm32_f407vg_arduino_sim/src/hardware.cpp`, `src/kalman.cpp`, `src/kinematics.cpp`
  - `firmware/stm32_f407vg_arduino_sim/include/firmware_config.h`, `kinematics.h`, `hardware.h`
  - `firmware/stm32_f407vg_arduino_sim/platformio.ini`
  - `src/omni_control/omni_control/kinematics.py`
  - `src/omni_control/test/test_kinematics.py`
  - `src/omni_simulation/omni_simulation/stm32_simulator.py`
- **Key findings**:
  - Current firmware computes velocity via backward difference into `ScalarKalman`, causing chatter at low speed and phase lag during acceleration.
  - Designed Second-Order PLL tracking observer ($\omega_{pll} = 20.0$ rad/s, $k_p = 40.0, k_i = 400.0, \zeta = 1.0$), 16-bit rollover safe math `(int16_t)(curr - prev)`, zero-speed watchdog (50 ms timeout), and LinuxCNC M/T decay envelope.
  - Discovered build bug in `firmware/.../src/kalman.cpp:17` (missing `#include <math.h>`).
  - Discovered `platformio.ini` lacks `[env:native]` for running host Unity tests.
  - Designed $\mathbf{K}_r$ matrix addition to `kinematics.py`, proving mathematical round trip $\|FK(IK(\mathbf{v})) - \mathbf{v}\| \equiv 1.76 \times 10^{-16} \ll 10^{-5}$.
  - Ran GitNexus impact analysis on `compute_wheel_speeds`: confirmed CRITICAL risk (70 direct callers, 130 blast radius), ensured backward compatibility.
- **Unexplored areas**: None for M1 scope. Ready for implementation by M1 implementer.

## Key Decisions Made
- Selected $\omega_{pll} = 20.0$ rad/s for 100 Hz FreeRTOS task, ensuring $T_s \cdot \omega_{pll} = 0.2 < 1.0$ discrete stability.
- Formulated PLL state tracking using position error $e_{pos}$ rather than accumulated counts, preserving single-precision float accuracy indefinitely.
- Preserved `compute_wheel_speeds` and `kinematics.py` signatures via default parameters/fallbacks to maintain 100% backward compatibility.

## Artifact Index
- DISPATCH.md — Incoming task dispatch record
- BRIEFING.md — Situational awareness
- progress.md — Liveness heartbeat
- m1_implementation_plan.md — Detailed technical implementation plan for M1
- handoff.md — 5-component hard handoff report
