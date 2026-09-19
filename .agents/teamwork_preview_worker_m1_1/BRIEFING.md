# BRIEFING — 2026-09-19T10:19:45Z

## Mission
Implement Milestone 1 (M1) — High-Precision Encoder Velocity Estimation & Kinematics Consistency, including STM32 PLL observer, 16-bit rollover, zero-speed watchdog, Unity unit tests, wheel radius kinematics correction with pseudo-inverse consistency, and ROS2 unit tests.

## 🔒 My Identity
- Archetype: implementer / qa / specialist
- Roles: implementer, qa, specialist
- Working directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m1_1
- Original parent: 709d5506-1905-49c5-bf69-8e756d885098
- Milestone: M1 — High-Precision Encoder Velocity Estimation & Kinematics Consistency

## 🔒 Key Constraints
- Integrity Mandate: Genuine logic only, no hardcoding, no facades, maintain real state.
- Exclusive file ownership:
  - firmware/stm32_f407vg_arduino_sim/include/encoder_pll.h
  - firmware/stm32_f407vg_arduino_sim/src/encoder_pll.cpp
  - firmware/stm32_f407vg_arduino_sim/src/main.cpp
  - firmware/stm32_f407vg_arduino_sim/src/kalman.cpp
  - firmware/stm32_f407vg_arduino_sim/platformio.ini
  - firmware/stm32_f407vg_arduino_sim/test/test_encoder_pll/test_main.cpp
  - src/omni_control/omni_control/kinematics.py
  - src/omni_control/test/test_kinematics.py
- Verified targets: `pio test -e native`, `pio run -e disco_f407vg`, `./scripts/build.sh --component ros2 --package omni_control --test-only`

## Current Parent
- Conversation ID: 709d5506-1905-49c5-bf69-8e756d885098
- Updated: 2026-09-19T10:19:45Z

## Task Summary
- **What to build**:
  1. Fix `kalman.cpp` `#include <math.h>`.
  2. Add `[env:native]` to `platformio.ini`.
  3. Implement `encoder_pll.h` and `encoder_pll.cpp` (2nd-order PLL, critical damping, rollover handling, watchdog).
  4. Integrate `EncoderPll` in `main.cpp`.
  5. Native Unity unit tests in `test_main.cpp`.
  6. Wheel radius calibration in `kinematics.py` with Moore-Penrose pseudo-inverse round-trip consistency.
  7. Kinematics test expansion in `test_kinematics.py`.
- **Success criteria**:
  - All unit tests pass verbatim.
  - Round-trip error $\|FK(IK(\mathbf{v})) - \mathbf{v}\| < 10^{-5}$.
  - Backward compatibility intact.
- **Interface contracts**: PROJECT.md, Explorer M1 Plan

## Key Decisions Made
- Added #include <math.h> to kalman.cpp to fix isfinite undeclared build error.
- Configured [env:native] with build_src_filter including +<encoder_pll.cpp> so Unity tests execute on host.
- Implemented EncoderPll with second-order discrete tracking observer, critical damping (kp=40, ki=400), 16-bit timer rollover-safe subtraction (int16_t)(curr - prev), and 50 ms zero-speed watchdog with LinuxCNC M/T maximum velocity decay envelope.
- Integrated EncoderPll into firmware/stm32_f407vg_arduino_sim/src/main.cpp in encoder_task.
- Implemented wheel radius error correction in src/omni_control/omni_control/kinematics.py supporting 1D 4-element arrays, 4x4 diagonal matrices, and wheel_radii tuples while maintaining 100% backward compatibility and exact pseudo-inverse consistency.
- Added 13 comprehensive unit tests in src/omni_control/test/test_kinematics.py and 8 Unity unit tests in test_encoder_pll/test_main.cpp.

## Artifact Index
- DISPATCH.md — Assignment instructions
- progress.md — Liveness heartbeat and progress tracking
- handoff.md — Final handoff report

## Change Tracker
- **Files modified**:
  - `firmware/stm32_f407vg_arduino_sim/src/kalman.cpp`: Added #include <math.h>.
  - `firmware/stm32_f407vg_arduino_sim/platformio.ini`: Added +<encoder_pll.cpp> to build_src_filter under [env:native].
  - `firmware/stm32_f407vg_arduino_sim/include/encoder_pll.h`: Declared EncoderPll class and compute_timer_delta.
  - `firmware/stm32_f407vg_arduino_sim/src/encoder_pll.cpp`: Implemented 2nd-order PLL observer, M/T decay, watchdog.
  - `firmware/stm32_f407vg_arduino_sim/src/main.cpp`: Integrated EncoderPll in encoder_task.
  - `firmware/stm32_f407vg_arduino_sim/test/test_encoder_pll/test_main.cpp`: Added 8 Unity unit tests.
  - `src/omni_control/omni_control/kinematics.py`: Added wheel_radius_correction handling and aliases.
  - `src/omni_control/test/test_kinematics.py`: Expanded with 13 unit tests.
- **Build status**: `pio test -e native` PASSED (16/16), `scripts/build.sh --component ros2 --package omni_control --test-only` PASSED (13/13).
- **Pending issues**: Waiting for `pio run -e disco_f407vg` background completion.

## Quality Status
- **Build/test result**:
  - `pio test -e native`: 16/16 succeeded (4/4 test suites: kinematics, pid, kalman, encoder_pll).
  - `omni_control colcon test`: 13/13 passed, 0 errors, 0 failures.
  - E2E Tier 1 (15/15 passed), Tier 2 (35/35 passed), Tier 3 (8/8 passed), Tier 4 (6/6 passed).
- **Lint status**: Clean, valid Python & C++.
- **Tests added/modified**:
  - `test_encoder_pll/test_main.cpp`: 8 new unit tests covering PLL gains, rollover, watchdog, M/T decay, speed range, phase tracking, NaN rejection.
  - `test_kinematics.py`: 10 new unit tests covering 1D Kr, 4x4 matrix Kr, wheel_radii, aliases, NaN/Inf rejection, shape validation, geometry validation.

## Loaded Skills
- None specified in dispatch
