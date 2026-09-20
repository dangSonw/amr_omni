# Progress — Milestone 4 Worker

Last visited: 2026-09-20T07:44:00Z

## Current Status
- Milestone 4 Implementation & Verification Complete:
  1. Fixed `SERIAL_FRAME_OVERHEAD` from 7 to 8 in `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`.
  2. Added packed payload structs (`SerialImuResultPayload`, `SerialWheelResultPayload`, `SerialProgressPayload`, `SerialNoiseResultPayload`) per PROJECT.md contract.
  3. Verified `serial_protocol.cpp` serialization bounds and deserialization overhead alignment.
  4. Added `+<serial_protocol.cpp>` to `build_src_filter` in `platformio.ini` for `env:native`.
  5. Implemented comprehensive Unity test suite in `firmware/stm32_f407vg_arduino_sim/test/test_serial_protocol/test_serial_protocol.cpp` (11 tests, all passed).
  6. Verified `pio test -e native` (45/45 passed) and `pio run -e disco_f407vg` (clean firmware build).
  7. Implemented `web/backend/app/services/calib_service.py` with atomic `.tmp` + `os.replace` YAML writers for `config/imu_calib.yaml` and `config/wheel_calib.yaml`.
  8. Integrated `calib_service.py` with `/api/calib/apply` in `web/backend/app/routers/calib.py`.
  9. Broadcast calibration progress and state in `web/backend/app/services/telemetry_hub.py`.
  10. Added automated persistence tests verifying YAML schema using `ConfigVerifier`.
  11. Ran `python3 -m pytest web/backend/tests/ -v` (25/25 passed).
  12. Ran `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v` (26/26 passed).
  13. Ran `python3 -m pytest tests/ -q` (208/208 passed 100%).
  14. Ran `npm run build` in `web/frontend` (clean static export).
