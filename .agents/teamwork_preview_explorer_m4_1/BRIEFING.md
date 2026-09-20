# BRIEFING — 2026-09-20T07:33:15Z

## Mission
Investigate Milestone 4: Serial Calibration Protocol Contract Parity between STM32 Firmware C++ implementation and Python Simulator (stm32_simulator.py / stm32_bridge.py), checking contract conformance against PROJECT.md § Interface Contracts.

## 🔒 My Identity
- Archetype: explorer
- Roles: explorer, investigator, synthesizer
- Working directory: /home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1
- Original parent: e684f9d6-654f-439a-9e8c-99049f9780b5
- Milestone: Milestone 4 - Serial Calibration Protocol Contract Parity

## 🔒 Key Constraints
- Read-only investigation — do NOT implement
- Strictly examine STM32 C++ firmware vs Simulator parity for calibration frames and general serial protocol
- Never edit production files directly; output findings, proposals, and handoff in working directory

## Current Parent
- Conversation ID: e684f9d6-654f-439a-9e8c-99049f9780b5
- Updated: not yet

## Investigation State
- **Explored paths**:
  - `PROJECT.md § Interface Contracts: 1. STM32 <-> Jetson Serial Binary Contract`
  - `ORIGINAL_REQUEST.md`
  - `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`
  - `firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp`
  - `firmware/stm32_f407vg_arduino_sim/src/main.cpp`
  - `firmware/stm32_f407vg_arduino_sim/platformio.ini`
  - `src/omni_simulation/omni_simulation/stm32_simulator.py`
  - `src/omni_simulation/test/test_stm32_simulator_logic.py`
  - `src/omni_hardware/omni_hardware/stm32_bridge.py`
  - `src/omni_hardware/omni_hardware/stm32_contract.py`
  - `tests/e2e/harness/serial_protocol_oracle.py`
  - `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py`
  - `tests/e2e/tier2_boundary_corner/test_boundary_protocol_corruption.py`
  - `tests/e2e/tier3_cross_feature/test_cross_feature_interactions.py`
  - `web/backend/app/routers/calib.py`
  - `web/backend/app/bridges/ros2_bridge.py`
- **Key findings**:
  - Critical defect found in `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`: `SERIAL_FRAME_OVERHEAD` is defined as 7, but is actually 8 (Header 2 + Len 1 + Seq 1 + MsgID 1 + CRC 2 + Tail 1).
  - This defect causes `deserialize_serial_frame` to check `buffer[6 + len]` (CRC MSB) against `0x7D` instead of `buffer[7 + len]` (Tail byte), causing deserialization to fail unconditionally for 255/256 random frames and leaving the tail unconsumed.
  - Serialization writes 8+len bytes but returns 7+len bytes, truncating the tail byte.
  - In Python `stm32_simulator.py` and `serial_protocol_oracle.py`, `SERIAL_FRAME_OVERHEAD = 8` is correctly implemented.
  - CRC16-CCITT algorithm (poly 0x1021, init 0xFFFF) and Message IDs (0x10-0x14, 0x80-0x84) match 100%.
  - `serial_protocol.cpp` is omitted from `platformio.ini` `env:native` build filter, and has 0 native C++ tests.
  - Firmware `main.cpp` does not yet integrate serial binary frames into FreeRTOS tasks.
- **Unexplored areas**: None for this investigation scope.

## Key Decisions Made
- Completed full audit and produced technical report `m4_serial_protocol_analysis.md` and hard handoff `handoff.md`.

## Artifact Index
- `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/DISPATCH.md` — Dispatch log
- `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/BRIEFING.md` — Situational awareness
- `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/progress.md` — Liveness heartbeat
- `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/m4_serial_protocol_analysis.md` — Detailed technical findings & proposals
- `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/handoff.md` — Hard handoff report (5 components)
