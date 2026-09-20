# BRIEFING — 2026-09-20T07:44:00Z

## Mission
Implement Milestone 4: Serial Calibration Protocol Parity, Native C++ Unit Tests, FastAPI Backend YAML Persistence & Web Services, passing all 207+ tests.

## 🔒 My Identity
- Archetype: worker
- Roles: implementer, qa, specialist
- Working directory: /home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_1
- Original parent: e684f9d6-654f-439a-9e8c-99049f9780b5
- Milestone: Milestone 4

## 🔒 Key Constraints
- DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations.
- Follow GitNexus impact rules before modifying symbols if applicable.
- Apply Ponytail standards: concise, clean, stdlib/native, no bloat.
- All unit tests and firmware compilation must pass cleanly.
- Atomic YAML writes to `config/imu_calib.yaml` and `config/wheel_calib.yaml` using `.tmp` + `os.replace`.

## Current Parent
- Conversation ID: e684f9d6-654f-439a-9e8c-99049f9780b5
- Updated: 2026-09-20T07:44:00Z

## Task Summary
- **What to build**:
  1. Fix `SERIAL_FRAME_OVERHEAD` from 7 to 8 and add packed structs in `serial_protocol.h`.
  2. Update `serialize_serial_frame` and `deserialize_serial_frame` in `serial_protocol.cpp`.
  3. Update `platformio.ini` native env source filter.
  4. Implement native C++ Unity unit tests in `firmware/stm32_f407vg_arduino_sim/test/test_serial_protocol/test_serial_protocol.cpp`.
  5. Implement `web/backend/app/services/calib_service.py`.
  6. Update `web/backend/app/routers/calib.py` to integrate `calib_service` and atomic YAML persistence.
  7. Update `web/backend/app/services/telemetry_hub.py` for calibration status/telemetry broadcast.
- **Success criteria**:
  - `pio test -e native` passes 100% (45/45 tests passed).
  - `pio run -e disco_f407vg` compiles cleanly (SUCCESS, RAM: 47.1%, Flash: 13.8%).
  - `pytest web/backend/tests/ -v` passes (25/25 passed).
  - `pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v` passes (26/26 passed).
  - All 208 tests in `pytest tests/ -v` pass 100%.
  - `npm run build` in `web/frontend` passes cleanly.

## Key Decisions Made
- `SERIAL_FRAME_OVERHEAD` set to 8, aligning with Python simulator, test harness oracle, and `PROJECT.md` contract.
- Added packed structs `SerialProgressPayload`, `SerialImuResultPayload`, `SerialWheelResultPayload`, `SerialNoiseResultPayload` using `#pragma pack(push, 1)`.
- Added 11 native C++ Unity unit tests covering CRC16 test vector, empty payload, max payload, buffer bounds, frame corruptions, all 10 message IDs roundtrip, and packed struct casting.
- Created `calib_service.py` adhering to Ponytail guidelines (stdlib PyYAML, os.replace atomic swap).
- Integrated `calib_service.py` with `/api/calib/apply` to automatically write `config/imu_calib.yaml` and `config/wheel_calib.yaml`.
- Integrated `calib_service.get_calib_telemetry_summary()` into `telemetry_hub._broadcast_loop` for WebSocket real-time progress streaming.

## Artifact Index
- `.agents/teamwork_preview_worker_m4_1/DISPATCH.md` — Assignment dispatch
- `.agents/teamwork_preview_worker_m4_1/BRIEFING.md` — Agent working memory
- `.agents/teamwork_preview_worker_m4_1/progress.md` — Heartbeat log
- `.agents/teamwork_preview_worker_m4_1/skills/ponytail/SKILL.md` — Local skill copy
- `.agents/teamwork_preview_worker_m4_1/handoff.md` — 5-component handoff report

## Change Tracker
- **Files modified**:
  - `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`: overhead = 8, packed payload structs
  - `firmware/stm32_f407vg_arduino_sim/platformio.ini`: added `+<serial_protocol.cpp>` to `build_src_filter`
  - `firmware/stm32_f407vg_arduino_sim/test/test_serial_protocol/test_serial_protocol.cpp`: 11 native C++ Unity unit tests
  - `web/backend/app/services/calib_service.py`: calibration service with atomic YAML writers and state tracking
  - `web/backend/app/routers/calib.py`: integrated `calib_service` and atomic YAML persistence in `/api/calib/apply`
  - `web/backend/app/services/telemetry_hub.py`: broadcast calibration state in telemetry stream
  - `web/backend/tests/test_api.py`: added test for `/api/calib/apply` persistence validated with `ConfigVerifier`
  - `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py`: added test for `/api/calib/apply` persistence
- **Build status**: All passing (Firmware: 100%, Native tests: 45/45, Pytest: 208/208, Frontend: 100%)
- **Pending issues**: None

## Quality Status
- **Build/test result**: All 45 PlatformIO native test cases passed. All 25 backend tests passed. All 208 full pytest test cases passed.
- **Lint status**: Clean (py_compile passed).
- **Tests added/modified**: 11 C++ Unity tests added; 2 python persistence tests added.

## Loaded Skills
- **Source**: `/home/sonev/amr_omni/.claude/skills/ponytail/SKILL.md`
- **Local copy**: `/home/sonev/amr_omni/.agents/teamwork_preview_worker_m4_1/skills/ponytail/SKILL.md`
- **Core methodology**: Forces the laziest solution that actually works, simplest, shortest, most minimal, stdlib first, no bloat.
