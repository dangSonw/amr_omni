# Hard Handoff Report: Milestone 4 FastAPI Calibration Backend & YAML Persistence

**Agent**: `teamwork_preview_explorer_m4_2`  
**Date**: 2026-09-20  
**Status**: COMPLETE (Hard Handoff)  
**Parent Conversation ID**: `e684f9d6-654f-439a-9e8c-99049f9780b5`  
**Working Directory**: `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_2`

---

## 1. Observation

1. **Test Execution Results**:
   - Running `python3 -m pytest web/backend/tests/ -v`:
     Command executed with exit code 0.
     Result: `24 passed, 143 warnings in 1.18s`.
     All 23 API tests and 1 full system integration test passed.
   - Running `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v`:
     Command executed with exit code 0.
     Result: `25 passed, 141 warnings in 0.72s`.
   - Running `python3 -m pytest tests/e2e/ -v`:
     Command executed with exit code 0.
     Result: `157 passed, 141 warnings in 2.28s`.

2. **Persistence Implementation in `/api/calib/apply`**:
   - In `/home/sonev/amr_omni/web/backend/app/routers/calib.py`, lines 182–185:
     ```python
     class CalibApplyPayload(BaseModel):
         persist_flash: Optional[bool] = True
         save_yaml: Optional[bool] = True
     ```
   - In `/home/sonev/amr_omni/web/backend/app/routers/calib.py`, lines 872–888:
     ```python
     @router.post("/apply")
     async def apply_calibration(payload: Optional[CalibApplyPayload] = None):
         bridge = get_bridge()
         if hasattr(bridge, "send_calib_cmd"):
             bridge.send_calib_cmd({
                 "action": "apply_params",
                 "accel_scale": _calib_state["results"]["accel_scale"],
                 "accel_bias": _calib_state["results"]["accel_bias"],
                 "gyro_bias": _calib_state["results"]["gyro_bias"],
                 "is_calibrated": True,
             })
         return {
             "status": "ok",
             "message": "Các tham số hiệu chuẩn đã được áp dụng xuống STM32 và lưu trữ thành công",
             "results": _calib_state.get("results"),
         }
     ```
   - `yaml` module is not imported in `web/backend/app/routers/calib.py` (lines 1–10).
   - No file write, no `yaml.dump()`, and no directory operations exist in `/api/calib/apply`.
   - The directory `/home/sonev/amr_omni/config/` does not exist on disk.

3. **Backend Service Architecture**:
   - `PROJECT.md § Code Layout` specifies:
     `- web/backend/app/:`
     `  - api/calib.py: Calibration endpoints (/api/calib/*).`
     `  - services/calib_service.py: Calibration lifecycle, serial dispatch, and YAML writer.`
   - In reality:
     File `/home/sonev/amr_omni/web/backend/app/services/calib_service.py` does not exist.
     All logic is consolidated inside `/home/sonev/amr_omni/web/backend/app/routers/calib.py` (1,152 lines).

4. **Calibration Telemetry Streaming**:
   - In `/home/sonev/amr_omni/web/backend/app/services/telemetry_hub.py`, lines 74–85:
     ```python
     payload = {
         "type": "telemetry",
         "status": bridge.get_status().model_dump(),
         "wheels": bridge.get_wheel_telemetry().model_dump(),
         "imu": bridge.get_imu_telemetry().model_dump(),
         "odom": bridge.get_odometry().model_dump(),
         "lidar": bridge.get_lidar_telemetry().model_dump(),
         "paths": bridge.get_path_telemetry().model_dump(),
         "streams": [s.model_dump() for s in stream_monitor.get_all()],
         "debug": bridge.get_debug_telemetry().model_dump() if bridge.get_debug_telemetry() else None,
     }
     ```
   - `calib` progress and state are omitted from the WebSocket broadcast payload.
   - In `/home/sonev/amr_omni/web/frontend/src/components/CalibrationPanel.tsx`, line 272:
     `const interval = setInterval(fetchStatus, 1000);`
     The UI fetches `/api/calib/status` by polling HTTP GET at 1 Hz rather than WebSocket push.

5. **Serial Binary Protocol Bridge**:
   - `PROJECT.md § Interface Contracts: 1` specifies binary packets: `[0xAA 0x55] [Len] [Seq] [MsgID] [Payload] [CRC16: 2B] [0x7D]`.
   - In `/home/sonev/amr_omni/web/backend/app/bridges/ros2_bridge.py`, lines 520–525:
     ```python
     def send_calib_cmd(self, payload: dict):
         if self.calib_cmd_pub and self.running:
             msg = String()
             msg.data = json.dumps(payload)
             self.calib_cmd_pub.publish(msg)
     ```
   - Commands are published as JSON strings over topic `calib/cmd` (`std_msgs/msg/String`).
   - `omni_hardware/stm32_bridge.py` does not subscribe or publish to `calib/cmd` or `calib/status`.

6. **Test Blind Spot in E2E Suite**:
   - In `/home/sonev/amr_omni/tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py`, lines 207–239:
     ```python
     def test_f4_4_yaml_persistence_imu_schema(self, temp_calib_dir: Path):
         target_file = temp_calib_dir / "imu_calib.yaml"
         calib_data = {...}
         with open(target_file, "w", encoding="utf-8") as f:
             yaml.safe_dump(calib_data, f)
         verifier = ConfigVerifier(temp_calib_dir.parent)
         assert verifier.verify_imu_calib_yaml(target_file)
     ```
   - The test creates the YAML file directly in a test fixture directory; it does not test `/api/calib/apply`.

---

## 2. Logic Chain

1. **From Observation 1**: All 24 backend tests and 157 E2E tests pass cleanly without errors. This demonstrates that the in-memory math algorithms (ST AN4508 6-face solver, Welford sampling, tilt projection compensation, arbitrary pose gradient solver, wheel radius compensation) and baseline REST endpoints are functioning properly in isolation.
2. **From Observation 2 and Observation 6**: While the tests pass, `TestF44_AutomatedYAMLConfigurationPersistence` passes only because it writes its own mock YAML to a temporary fixture. In the actual backend code (`web/backend/app/routers/calib.py`), `/api/calib/apply` contains no YAML writing code at all, and `/home/sonev/amr_omni/config` does not exist. Therefore, calibration results cannot be persisted to `config/imu_calib.yaml` or `config/wheel_calib.yaml` through the web interface today.
3. **From Observation 3**: `PROJECT.md § Code Layout` specifies that calibration business logic, serial dispatch, and YAML writing should be housed in `services/calib_service.py`. The current monolithic organization in `routers/calib.py` violates this architecture and makes testing file persistence independent of HTTP routing difficult.
4. **From Observation 4**: `PROJECT.md` Feature F4.3 specifies real-time 5 Hz WebSocket telemetry streaming of calibration progress. Because `TelemetryHub` omits calibration from its 20 Hz payload, the Next.js frontend has resorted to polling `/api/calib/status` at 1 Hz via HTTP GET, which introduces latency and overhead.
5. **From Observation 5**: `ros2_bridge.py` and `stm32_simulator.py` communicate via JSON strings over topic `calib/cmd`. The binary serial frame contract (`[0xAA 0x55]...[0x7D]` with CRC16) is implemented in firmware C++ and tested in python test harness oracles, but there is no hardware bridge node translating between binary serial packets and ROS 2 / FastAPI.
6. **Overall Conclusion**: Milestone 4's mathematical solvers, mock simulation, and REST routing are robust and 100% test-passing, but the subsystem has three critical gaps preventing true end-to-end integration: (1) missing YAML disk persistence in `/api/calib/apply`, (2) absence of calibration data in WebSocket streaming, and (3) missing `calib_service.py` service layer separation.

---

## 3. Caveats

1. **Hardware Bridge Scope**: We investigated `omni_hardware/stm32_bridge.py`. In simulation mode (`stm32_simulator.py`), the JSON-over-topic `calib/cmd` works seamlessly with `ros2_bridge.py`. For real robot operation, micro-ROS or a serial forwarder node would be required to parse binary serial frames.
2. **Existing EKF YAML**: `src/omni_localization/config/ekf.yaml` is fully configured and compliant with `PROJECT.md` (non-zero SPD covariance matrices, Single TF Authority enabled). Modifying it during runtime calibration is typically restricted to covariance inflation rather than full overwrite.
3. **Read-Only Investigation**: In accordance with the Explorer role constraints, no source code files were modified. All proposals are presented via `m4_backend_analysis.md` and this handoff.

---

## 4. Conclusion

Milestone 4 is functionally complete in in-memory state estimation, simulation orientation manipulation, and endpoint routing, but has significant omissions against the formal contracts in `PROJECT.md`:
1. **P0 Remediation**: Implement `web/backend/app/services/calib_service.py` containing `persist_yaml_configs()` with atomic file writes (`.tmp` -> rename) targeting `config/imu_calib.yaml` and `config/wheel_calib.yaml`. Invoke this from `/api/calib/apply` when `save_yaml` is `True`.
2. **P1 Remediation**: Add calibration state/progress into `TelemetryHub._broadcast_loop` in `web/backend/app/services/telemetry_hub.py`, allowing the Next.js frontend to consume real-time 5 Hz progress updates.
3. **P1 Remediation**: Add a backend unit test verifying that `POST /api/calib/apply` generates valid YAML files verified by `ConfigVerifier`.

---

## 5. Verification Method

To verify these findings independently:

1. **Run Backend Test Suite**:
   ```bash
   python3 -m pytest web/backend/tests/ -v
   ```
   Confirms all 24 tests pass, but inspect `web/backend/tests/test_api.py` to confirm `/api/calib/apply` is never called.

2. **Run E2E Test Suite**:
   ```bash
   python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v
   ```
   Inspect lines 207–239 of `test_f4_serial_web_calib.py` to observe that the test writes YAML files directly with `yaml.safe_dump` rather than calling the FastAPI application.

3. **Inspect `/api/calib/apply` in `routers/calib.py`**:
   Check lines 872–888 in `web/backend/app/routers/calib.py` to confirm zero file writing operations.

4. **Inspect `config/` directory**:
   Run `ls /home/sonev/amr_omni/config` to observe that the directory does not exist.
