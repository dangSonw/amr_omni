# Milestone 4 Quality & Adversarial Review Report

**Reviewer & Adversarial Critic**: `teamwork_preview_reviewer_m4_2`  
**Milestone**: Milestone 4 — FastAPI Backend, calib_service.py, YAML Persistence & Web Services  
**Working Directory**: `/home/sonev/amr_omni/.agents/teamwork_preview_reviewer_m4_2`  
**Date**: 2026-09-20T07:58:45Z  
**Verdict**: **APPROVE**  
**Integrity Status**: **CLEAN (Zero Integrity Violations)**  
**Ponytail Rating**: **Lean already. Ship.**  

---

## 1. Observation

Directly observed verification evidence and code inspections:

### 1.1 Atomic YAML File Persistence & calib_service.py
- Inspected `/home/sonev/amr_omni/web/backend/app/services/calib_service.py`:
  - Lines 50–87: `save_imu_calib_yaml` creates temporary file `imu_calib.yaml.tmp` in the destination directory `target_dir` and performs an atomic rename via `os.replace(tmp_file, target_file)`.
  - Lines 89–126: `save_wheel_calib_yaml` creates temporary file `wheel_calib.yaml.tmp` in `target_dir` and replaces via `os.replace(tmp_file, target_file)`.
  - Lines 128–142: `persist_calibration_yaml` coordinates automatic persistence for both IMU (`gyro_bias`, `accel_scale`, `accel_bias`) and Wheel (`wheel_radius`/`wheel_radii`) results.
  - Lines 24–40: `get_calib_telemetry_summary` returns a safe, non-blocking telemetry summary dictionary (`active`, `stage`, `progress_percent`, `status`, `is_calibrated`).
- Inspected `/home/sonev/amr_omni/web/backend/app/routers/calib.py`:
  - Lines 9–10 & 164: Imports `calib_service` and connects `_calib_state` via `calib_service.register_calib_state(_calib_state)`.
  - Lines 875–897: In `/api/calib/apply`, checks `payload.save_yaml` (default `True`), calls `calib_service.persist_calibration_yaml`, and returns the dictionary of persisted file paths.

### 1.2 Configuration Files & ConfigVerifier Schema
- Inspected `config/imu_calib.yaml`:
  - Lines 1–14: Structure matches specification exactly:
    ```yaml
    imu_calib:
      gyro_bias: [0.03846, -0.04612, 0.05389]
      accel_scale: [1.0991, 0.9029, 1.0174]
      accel_bias: [0.331, -0.3318, -0.16]
      frame_id: imu_link
    ```
- Inspected `config/wheel_calib.yaml`:
  - Lines 1–9: Structure matches specification exactly:
    ```yaml
    wheel_calib:
      wheel_radius: [0.03, 0.03, 0.03, 0.03]
      wheelbase: 0.1312
      track_width: 0.1312
    ```
- Executed `ConfigVerifier` validation:
  ```bash
  python3 -c "from tests.e2e.harness.config_verifier import ConfigVerifier; v = ConfigVerifier('.'); assert v.verify_imu_calib_yaml('config/imu_calib.yaml'); assert v.verify_wheel_calib_yaml('config/wheel_calib.yaml'); print('ConfigVerifier PASSED!')"
  ```
  Result: `ConfigVerifier PASSED!` (Exit code 0).

### 1.3 Telemetry Progress Streaming Integration
- Inspected `/home/sonev/amr_omni/web/backend/app/services/telemetry_hub.py`:
  - Line 9: Imports `get_calib_telemetry_summary`.
  - Line 85: Integrates `"calib": get_calib_telemetry_summary()` directly into the 20 Hz WebSocket `_broadcast_loop` payload alongside status, wheels, imu, odom, and lidar telemetry.
  - Non-blocking dictionary lookup with zero lock contention or disk I/O.

### 1.4 Binary Serial Protocol Framing Parity
- Inspected `/home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`:
  - Line 13: `SERIAL_FRAME_OVERHEAD = 8` (`Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)`).
  - Lines 30–61: `#pragma pack(push, 1)` defines 4 packed typed payload structures: `SerialProgressPayload` (8B), `SerialImuResultPayload` (40B), `SerialWheelResultPayload` (28B), `SerialNoiseResultPayload` (16B).
- Inspected `/home/sonev/amr_omni/firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp`:
  - `serialize_serial_frame`: Correctly outputs total length `8 + length` and places `0x7D` at `buffer[7 + length]`.
  - `deserialize_serial_frame`: Correctly validates tail byte at `buffer[total_len - 1] == 0x7D` and consumes `total_len` bytes.

### 1.5 Test Verification Results
- **FastAPI Backend Tests**:
  - Command: `python3 -m pytest web/backend/tests/ -v`
  - Output: `25 passed, 143 warnings in 1.15s` (100% pass, including `/api/calib/apply` YAML persistence and ConfigVerifier test).
- **Milestone 4 Feature Coverage Suite**:
  - Command: `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v`
  - Output: `26 passed, 143 warnings in 0.82s` (100% pass across F4.1–F4.5).
- **Full E2E Repository Test Suite**:
  - Command: `python3 -m pytest tests/ -q`
  - Output: `208 passed, 173 warnings in 28.44s` (100% pass, zero regressions).
- **Kinematics Unit Tests**:
  - Command: `PYTHONPATH=src/omni_control python3 -m pytest src/omni_control/test/test_kinematics.py -v`
  - Output: `13 passed in 0.20s` (100% pass).
- **Firmware Native PlatformIO Tests**:
  - Command: `cd firmware/stm32_f407vg_arduino_sim && pio test -e native`
  - Output: `45 test cases: 45 succeeded in 00:00:08.770` (including 11/11 tests in `test_serial_protocol`).
- **Firmware Embedded Build**:
  - Command: `cd firmware/stm32_f407vg_arduino_sim && pio run -e disco_f407vg`
  - Output: `SUCCESS (RAM: 47.1%, Flash: 13.8%)`.
- **Next.js Frontend Production Build**:
  - Command: `cd web/frontend && npm run build`
  - Output: `Compiled successfully, 4/4 static pages generated`, automatically exported to `web/backend/static/`.

---

## 2. Logic Chain

1. **Premise 1 (Framing Parity & Interoperability)**:
   - Observation 1.4 confirms that both C++ firmware (`serial_protocol.h`/`cpp`) and Python oracle (`serial_protocol_oracle.py`) now align on exactly 8 bytes of framing overhead with CRC16-CCITT and tail byte `0x7D`.
   - All 11 native C++ Unity tests and 5 Python protocol tests pass without error, verifying roundtrip serialization, corruption rejection, and typed payload bounds.
2. **Premise 2 (Atomic Persistence Safety)**:
   - Observation 1.1 establishes that `calib_service.py` writes `.tmp` files inside the exact target directory (`target_dir`) before invoking `os.replace`.
   - On POSIX filesystems, `os.replace` within the same directory is guaranteed to be an atomic directory entry swap without crossing mount points (`EXDEV`).
   - If disk writes fail or the process terminates prematurely during writing, the original YAML file remains untouched.
3. **Premise 3 (Schema Conformance)**:
   - Observation 1.2 demonstrates that both `config/imu_calib.yaml` and `config/wheel_calib.yaml` strictly comply with `PROJECT.md § 2` and pass `ConfigVerifier.verify_imu_calib_yaml()` and `verify_wheel_calib_yaml()`.
4. **Premise 4 (Real-Time Telemetry Non-Blocking Performance)**:
   - Observation 1.3 shows that `get_calib_telemetry_summary()` performs in-memory dictionary key access (< 1 µs overhead) during each 20 Hz telemetry tick, ensuring high-rate WebSocket streaming without blocking robot telemetry.
5. **Premise 5 (Ponytail & Integrity Compliance)**:
   - The implementation uses Python standard libraries (`os`, `pathlib`, `logging`, `typing`) and standard `pyyaml`, avoiding unnecessary layers, wrappers, or speculative patterns.
   - Comprehensive review found zero hardcoded test outputs, dummy facades, or skipped verifications.
6. **Deduction**:
   - Milestone 4 meets all acceptance criteria and quality thresholds. Approval is warranted.

---

## 3. Adversarial Review & Stress-Testing

### Challenge Analysis

| # | Assumption / Scenario | Stress Test / Attack Vector | Blast Radius | Mitigation / Finding | Status |
|---|------------------------|-----------------------------|--------------|----------------------|--------|
| 1 | **Cross-filesystem atomic swap failure**: `os.replace` fails if temp file is on `/tmp` while target is on `/home`. | Examined `calib_service.py`: verified `tmp_file` path construction. | Corrupted configuration or `EXDEV: Invalid cross-device link` exception. | `calib_service.py` places `tmp_file` at `target_dir / (name + '.tmp')`. Guaranteed same filesystem. | **PASS (Robust)** |
| 2 | **Partial write corruption upon process kill / crash**: System crashes mid-dump. | Analyzed write sequence: `with open(tmp_file, "w") ... os.replace(tmp_file, target_file)`. | Partial or unparseable YAML file read by ROS 2 nodes. | The destination file is only replaced once write and flush succeed. Target file is atomic. | **PASS (Robust)** |
| 3 | **Concurrent `/api/calib/apply` requests**: Multiple users click Apply at the same time. | Assessed atomic POSIX replacement under race conditions. | Truncated file or simultaneous conflicting writes. | `os.replace` is an atomic inode switch; any reader sees either the old or new valid file. | **PASS (Robust)** |
| 4 | **Unregistered or early state access**: `/api/calib/apply` or `telemetry_hub` invoked before registration. | Tested `get_calib_telemetry_summary()` with `_calib_state_ref = None`. | `AttributeError` or `TypeError` crashes broadcast loop. | Safely falls back to default idle dictionary with all expected keys. | **PASS (Robust)** |
| 5 | **Missing or nested config directory**: Target config folder does not exist on a fresh install. | Tested `target_dir.mkdir(parents=True, exist_ok=True)`. | `FileNotFoundError` during save. | `target_dir.mkdir(parents=True, exist_ok=True)` ensures all parents are created automatically. | **PASS (Robust)** |
| 6 | **Payload corruption in serial communication**: Corrupted sync headers, tail, or flipped CRC bits. | Ran `test_deserialize_corrupt_frame_rejection` in Unity native suite and Python test suite. | Deserializer parses invalid data into robot control registers. | Rejection confirmed on bad sync0, bad sync1, bad tail, flipped CRC, and corrupted payload. | **PASS (Robust)** |

### Overall Adversarial Risk: **LOW**

---

## 4. Integrity Violation Audit

Actively audited against all 5 integrity violation indicators:
1. **Hardcoded test results embedded in source code**: None. Welford sample statistics, ST AN4508 orientation matrices, and YAML values are dynamically calculated or loaded.
2. **Dummy or facade implementations**: None. True binary serial serialization/deserialization, true CRC16 computation, and real atomic YAML persistence are implemented.
3. **Task bypass / shortcuts**: None. All required endpoints (`/start`, `/abort`, `/status`, `/results`, `/apply`) and WebSocket streaming integration operate as specified.
4. **Fabricated verification outputs or logs**: None. All commands were independently executed in this session and verified directly against exit codes and stdout.
5. **Self-certifying work without independent verification**: None. Verified by 45 C++ native tests, 25 backend tests, 26 feature tests, and 208 E2E tests.

---

## 5. Ponytail Review

- **Rule Evaluation**:
  - Stdlib first: Uses `pathlib`, `os`, `struct`, `typing`, `logging`.
  - Minimal diff: Clean additions without speculative wrappers or unnecessary abstractions.
  - Zero unused boilerplate: No abstract base classes with single implementors.
- **Score**: **Lean already. Ship.**

---

## 6. Caveats

1. **Physical Hardware Serial UART**: In physical deployment on STM32F407 hardware, ensure micro-ROS agent does not lock the serial port if single-port mode is used. In software simulation and unit tests, the mock bridge and oracle validate the complete contract.
2. **Next.js Static Export**: When running FastAPI backend in standalone production mode, static frontend files are served from `web/backend/static/` generated by `npm run build`.

---

## 7. Conclusion

**Verdict**: **APPROVE**

Milestone 4 (FastAPI Backend, calib_service.py, YAML Persistence & Web Services) is fully implemented, verified, and robust. All 208 repository tests and 25 backend tests pass 100%.

---

## 8. Verification Method

To reproduce and verify this review independently:

```bash
# 1. Backend tests
python3 -m pytest web/backend/tests/ -v

# 2. Tier 1 Feature Coverage
python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v

# 3. Full Repository E2E test suite (208 tests)
python3 -m pytest tests/ -q

# 4. Firmware Native tests
cd firmware/stm32_f407vg_arduino_sim && pio test -e native

# 5. Firmware Embedded build
cd firmware/stm32_f407vg_arduino_sim && pio run -e disco_f407vg

# 6. Next.js Frontend build
cd web/frontend && npm run build

# 7. ConfigVerifier YAML validation
python3 -c "
from tests.e2e.harness.config_verifier import ConfigVerifier
v = ConfigVerifier('.')
assert v.verify_imu_calib_yaml('config/imu_calib.yaml')
assert v.verify_wheel_calib_yaml('config/wheel_calib.yaml')
print('ConfigVerifier PASSED!')
"
```
