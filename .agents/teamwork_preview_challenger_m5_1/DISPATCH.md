# Challenger M5-1 Dispatch
Task: Milestone 5 Phase 2 White-Box Adversarial Hardening (Firmware PLL, M/T, IMU Calib & Serial Codec)
Directory: /home/sonev/amr_omni/.agents/teamwork_preview_challenger_m5_1

## 2026-09-20T08:05:47Z
You are teamwork_preview_challenger_m5_1, conducting Milestone 5 Phase 2 White-Box Adversarial Coverage Hardening on the STM32 C++ Firmware.
Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_challenger_m5_1

MANDATORY FIRST STEP:
Read /home/sonev/amr_omni/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/PROJECT.md before doing any work.

Challenger Objectives:
1. Conduct white-box adversarial stress tests on all firmware algorithms in `firmware/stm32_f407vg_arduino_sim`:
   - `encoder_pll.h/.cpp`: ODrive 2nd-order PLL observer (low-speed quantization chatter elimination, high-speed phase lag, extreme acceleration 50 rad/s², zero velocity hold) and LinuxCNC M/T hybrid velocity (timer rollover safe diff, watchdog timeout).
   - `imu_calibration.h/.cpp`: ST AN4508 6-position accelerometer calibration, Welford variance gating, stationarity detection, and stationary gyro zero-rate bias nulling (<0.05 deg/s drift).
   - `serial_protocol.h/.cpp`: Binary serial framing (overhead=8, CRC16-CCITT, typed payloads).
2. Stress test with edge cases, corner cases, numerical bounds, and noise extremes.
3. Document your findings, test execution results, and verdict (APPROVE or REQUEST_CHANGES) in `/home/sonev/amr_omni/.agents/teamwork_preview_challenger_m5_1/handoff.md` and send_message back to parent.
