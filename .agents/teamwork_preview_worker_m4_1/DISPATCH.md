## 2026-09-20T07:33:51Z
You are teamwork_preview_worker_m4_1, working on Milestone 4: Serial Calibration Protocol Parity, Native C++ Unit Tests, FastAPI Backend YAML Persistence & Web Services.
Your working directory is: /home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_1

MANDATORY FIRST STEP:
Read /home/sonev/amr_omni/ORIGINAL_REQUEST.md and /home/sonev/amr_omni/PROJECT.md before doing any work.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

Context and Explorer Findings:
Read the comprehensive findings prepared by the 3 M4 Explorers:
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/handoff.md and m4_serial_protocol_analysis.md
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_2/handoff.md and m4_backend_analysis.md
- /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_3/handoff.md and m4_frontend_e2e_analysis.md

Scope and Deliverables for Milestone 4:
1. **STM32 C++ Serial Protocol Fix & Parity**:
   - In `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`:
     Fix `SERIAL_FRAME_OVERHEAD` from 7 to 8 (`Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)`).
     Add packed structs for typed payloads (`SerialImuResultPayload`, `SerialWheelResultPayload`, `SerialProgressPayload`, `SerialNoiseResultPayload`) per PROJECT.md interface contract.
   - In `firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp`:
     Ensure `serialize_serial_frame` checks `buffer_size >= SERIAL_FRAME_OVERHEAD + frame->length` and returns `SERIAL_FRAME_OVERHEAD + frame->length`.
     Ensure `deserialize_serial_frame` uses `SERIAL_FRAME_OVERHEAD` (checks `buffer[total_len - 1] == SERIAL_TAIL_BYTE` on byte index `7 + frame->length`).
   - In `firmware/stm32_f407vg_arduino_sim/platformio.ini`:
     Add `+<serial_protocol.cpp>` to `build_src_filter` in `[env:native]`.
   - In `firmware/stm32_f407vg_arduino_sim/test/test_serial_protocol/test_serial_protocol.cpp`:
     Implement thorough native C++ Unity tests verifying roundtrip serialize/deserialize, CRC16 calculation, corrupt frame rejection, and all message IDs.
   - Run `pio test -e native` (ensure all tests pass) and `pio run -e disco_f407vg` (ensure embedded firmware compiles without errors).

2. **Web Backend Calibration Service & YAML Disk Persistence**:
   - Implement `web/backend/app/services/calib_service.py` per `PROJECT.md § Code Layout`.
   - In `web/backend/app/routers/calib.py`:
     Integrate with `calib_service.py`.
     In `/api/calib/apply`, implement atomic YAML persistence to `/home/sonev/amr_omni/config/` (ensure directory exists):
     - `config/imu_calib.yaml`:
       ```yaml
       imu_calib:
         gyro_bias: [bx, by, bz]
         accel_scale: [sx, sy, sz]
         accel_bias: [ax, ay, az]
         frame_id: "imu_link"
       ```
     - `config/wheel_calib.yaml`:
       ```yaml
       wheel_calib:
         wheel_radius: [r1, r2, r3, r4]
         wheelbase: 0.1312
         track_width: 0.1312
       ```
     Use atomic write pattern (write to `.tmp` file, then `os.replace`).
   - In `web/backend/app/services/telemetry_hub.py`:
     Include calibration state/progress in telemetry broadcasts or provide real-time updates.

3. **Guidelines**:
   - Apply Ponytail standards (.claude/skills/ponytail): concise, clean, stdlib/native, no bloat.
   - Follow GitNexus rules in AGENTS.md.

4. **Testing & Verification**:
   - Run `pio test -e native`
   - Run `pio run -e disco_f407vg`
   - Run `python3 -m pytest web/backend/tests/ -v`
   - Run `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v`
   - Run `python3 -m pytest tests/ -v` (ensure all 207+ tests pass 100%)

Write your handoff report to `/home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_1/handoff.md` and send a message back to parent when complete.
