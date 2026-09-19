# Scope: Milestone 1 (M1) — Encoder Velocity Estimation & Kinematics Consistency

## Overview
Implement the high-precision motor encoder velocity estimation algorithm and individual wheel radius error compensation on STM32 firmware and ROS 2 `omni_control`.

## Features Assigned
- **F1.1: ODrive 2nd-Order PLL Tracking Observer**:
  Continuous states $[\hat{\theta}, \hat{\omega}]$, critical damping $\zeta = 1.0$ ($k_p = 2\omega_{pll}, k_i = \omega_{pll}^2 = 0.25 k_p^2$). Discrete update in FreeRTOS control/encoder loop. Eliminates low-speed chatter ($< 0.05$ m/s) and transient phase lag.
- **F1.2: LinuxCNC M/T Hybrid Estimation**:
  Edge-synchronized period estimation at ultra-low speeds with 16-bit timer rollover handling `(int16_t)(curr - prev)` and zero-speed watchdog timeout ($50$ ms).
- **F1.3: Kinematics Consistency & Individual Wheel Radius Compensation**:
  Individual wheel scale matrix $\mathbf{K}_r = \mathrm{diag}(k_1, k_2, k_3, k_4)$, Moore-Penrose pseudo-inverse kinematics, and round-trip verification ($\|FK(IK(\mathbf{v})) - \mathbf{v}\| < 10^{-5}$) with no NaN/Inf.

## Target Files
- `firmware/stm32_f407vg_arduino_sim/src/encoder_pll.h`
- `firmware/stm32_f407vg_arduino_sim/src/encoder_pll.cpp`
- `firmware/stm32_f407vg_arduino_sim/src/main.cpp` (integrate PLL observer in place of crude backward difference)
- `src/omni_control/omni_control/kinematics.py` (add wheel radius compensation $\mathbf{K}_r$)
- `src/omni_control/test/test_kinematics.py` (comprehensive consistency unit tests)

## Completion Criteria
- Firmware builds with 0 errors/warnings (`pio test -e native` or `./scripts/setup.sh --check --firmware stm32_f407vg_arduino_sim`).
- `pytest src/omni_control/test/test_kinematics.py` passes 100% with error $< 10^{-5}$.
- Velocity estimation tested across $0.01$ m/s to $> 1.5$ m/s without chatter or divergence.
