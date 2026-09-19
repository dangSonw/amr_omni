# DISPATCH for teamwork_preview_worker_m1_1

Assigned: Milestone 1 Worker (High-Precision Encoder Velocity Estimation & Kinematics Consistency)
Working Directory: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m1_1
Scope: Implementation of Milestone 1 per Explorer findings
Parent: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_orchestrator_1

## 2026-09-19T10:19:45Z

You are teamwork_preview_worker_m1_1, the Implementation Worker for Milestone 1 (M1) — High-Precision Encoder Velocity Estimation & Kinematics Consistency.
Your working directory is: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_worker_m1_1
Original Request: /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md
Master Project Document: /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md
Explorer M1 Plan: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m1_1/m1_implementation_plan.md
Explorer M1 Handoff: /home/sonev/teamwork_projects/amr_omni_calib/.agents/teamwork_preview_explorer_m1_1/handoff.md

MANDATORY FIRST STEP: Read /home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md, /home/sonev/teamwork_projects/amr_omni_calib/PROJECT.md, and the Explorer M1 Plan completely before writing any code.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE FILE OWNERSHIP:
You have exclusive write access to:
- `firmware/stm32_f407vg_arduino_sim/include/encoder_pll.h`
- `firmware/stm32_f407vg_arduino_sim/src/encoder_pll.cpp`
- `firmware/stm32_f407vg_arduino_sim/src/main.cpp`
- `firmware/stm32_f407vg_arduino_sim/src/kalman.cpp`
- `firmware/stm32_f407vg_arduino_sim/platformio.ini`
- `firmware/stm32_f407vg_arduino_sim/test/test_encoder_pll/test_main.cpp`
- `src/omni_control/omni_control/kinematics.py`
- `src/omni_control/test/test_kinematics.py`

IMPLEMENTATION TASKS:
1. Fix `firmware/stm32_f407vg_arduino_sim/src/kalman.cpp` by adding `#include <math.h>` for `isfinite`.
2. Update `firmware/stm32_f407vg_arduino_sim/platformio.ini` to add `[env:native]` with `platform = native`, `test_framework = unity`, and `test_build_src = false` so that Unity tests run directly on the host CPU.
3. Implement `encoder_pll.h` and `encoder_pll.cpp`:
   - Class `EncoderPll`: Second-Order continuous/discrete PLL tracking observer ($\dot{\hat{\theta}} = \hat{\omega} + k_p e, \dot{\hat{\omega}} = k_i e$).
   - Critical damping: $k_p = 2\omega_{pll}, k_i = \omega_{pll}^2 = 0.25 k_p^2$.
   - Bandwidth: $\omega_{pll} = 20.0$ rad/s for $T_s = 10$ ms ($T_s \cdot \omega_{pll} = 0.2 \le 0.2$).
   - 16-bit timer rollover handling using two's complement signed cast: `(int16_t)(curr - prev)`.
   - Zero-speed watchdog with 50 ms timeout: clamp to 0 rad/s when pulse arrivals cease, with LinuxCNC M/T maximum velocity decay envelope.
   - Zero steady-state velocity tracking error during constant-acceleration ramps.
4. Integrate `EncoderPll` into `firmware/stm32_f407vg_arduino_sim/src/main.cpp` in `encoder_task` to replace crude backward difference and scalar Kalman filtering.
5. Author native Unity unit tests in `firmware/stm32_f407vg_arduino_sim/test/test_encoder_pll/test_main.cpp` verifying:
   - Zero-speed detection and 50 ms watchdog clamp.
   - 16-bit rollover across 65535 / 0 in both directions.
   - Smooth estimation from 0.01 m/s to > 1.5 m/s without chatter.
   - Phase-tracking during velocity ramps.
6. Implement wheel radius correction in `src/omni_control/omni_control/kinematics.py`:
   - Add `wheel_radius_correction=None` parameter to `compute_wheel_speeds` and `compute_body_twist` (supporting 4-element 1D array or 4x4 diagonal matrix $\mathbf{K}_r$).
   - Ensure closed-form Moore-Penrose pseudo-inverse kinematics round-trip consistency: $\|FK(IK(\mathbf{v})) - \mathbf{v}\| < 10^{-5}$.
   - Add input validation rejecting NaN/Inf and non-positive radii.
   - Maintain 100% backward compatibility for existing callers (defaulting to nominal radius).
7. Expand `src/omni_control/test/test_kinematics.py` with comprehensive unit tests for:
   - Round-trip consistency with nominal and perturbed wheel radii.
   - NaN/Inf rejection.
   - High speed and low speed consistency.

VERIFICATION REQUIREMENTS:
Run and report verbatim output for:
- `pio test -e native` in `firmware/stm32_f407vg_arduino_sim`
- `pio run -e disco_f407vg` in `firmware/stm32_f407vg_arduino_sim`
- `./scripts/build.sh --component ros2 --package omni_control --test-only`

When complete, write your handoff report to `handoff.md` in your working directory and notify the orchestrator via send_message.

