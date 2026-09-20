# Milestone 4 Deep Dive Technical Analysis: FastAPI Calibration Backend, Endpoints & YAML Persistence

**Author**: `teamwork_preview_explorer_m4_2`  
**Date**: 2026-09-20  
**Status**: COMPLETE (Read-Only Investigation)  
**Workspace**: `/home/sonev/amr_omni`

---

## 1. Executive Summary

This report delivers an exhaustive technical investigation of Milestone 4 (FastAPI Calibration Backend, Endpoints & YAML Persistence) of the AMR Omni Mecanum AGV project. The investigation covered the interface contracts in `PROJECT.md`, the backend codebase in `web/backend/app/`, the communication bridge with ROS 2 and STM32, the WebSocket telemetry streaming subsystem, and the automated test suites (`web/backend/tests/` and `tests/e2e/`).

### Core Findings Summary:
1. **Test Suite Status**:
   - Backend unit tests (`web/backend/tests/`): **24/24 PASSED** (100%).
   - E2E Tier 1 F4 tests (`tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py`): **25/25 PASSED** (100%).
   - Entire E2E test suite (`tests/e2e/`): **157/157 PASSED** (100%).
2. **Critical Architectural & Functional Gaps Identified**:
   - **Gap 1 (YAML Persistence Missing in `/api/calib/apply`)**: `/api/calib/apply` takes `CalibApplyPayload(save_yaml=True)`, but contains **zero file I/O code**. Neither `config/imu_calib.yaml` nor `config/wheel_calib.yaml` are generated or updated. In fact, directory `/home/sonev/amr_omni/config` does not exist in the repository root.
   - **Gap 2 (Architectural Layout Violation)**: `PROJECT.md § Code Layout` specifies `web/backend/app/api/calib.py` and `web/backend/app/services/calib_service.py`. However, `calib_service.py` does not exist; all state management, sampling workers, mathematical solvers, and bridge communications are monolithically placed in `web/backend/app/routers/calib.py` (1,152 lines).
   - **Gap 3 (WebSocket Streaming Omission for Calibration)**: While `PROJECT.md` specifies real-time 5 Hz WebSocket streaming (`TELEM_CALIB_PROGRESS`), `TelemetryHub` in `web/backend/app/services/telemetry_hub.py` broadcasts only navigation and robot status, omitting calibration progress. Consequently, the Next.js frontend (`web/frontend/src/components/CalibrationPanel.tsx`) falls back to HTTP GET polling on `/api/calib/status` every 1,000 ms.
   - **Gap 4 (Binary Serial Protocol vs ROS 2 JSON String Discrepancy)**: The STM32 binary serial packet protocol (`[0xAA 0x55]...[0x7D]` with CRC16-CCITT) defined in `PROJECT.md § Interface Contracts: 1` is implemented in firmware C++ and tested in python test oracles, but the web backend uses ROS 2 topic `calib/cmd` with JSON strings. No hardware serial bridge translates between binary frames and ROS 2 / FastAPI.
   - **Gap 5 (Test Suite Blind Spot)**: In `test_f4_serial_web_calib.py`, `TestF44_AutomatedYAMLConfigurationPersistence` creates and verifies temporary YAML files directly using `yaml.safe_dump` within the test body rather than invoking `/api/calib/apply`, masking the missing backend persistence implementation.

---

## 2. Contract Analysis: Jetson Web Backend <-> ROS 2 YAML Config Contract

### 2.1 Contract Specification (`PROJECT.md § Interface Contracts: 2`)

`PROJECT.md` dictates three persistent configuration artifacts:

1. **IMU Configuration (`config/imu_calib.yaml`)**:
   ```yaml
   imu_calib:
     gyro_bias: [bx, by, bz]
     accel_scale: [sx, sy, sz]
     accel_bias: [ax, ay, az]
     frame_id: "imu_link"
   ```
2. **Wheel Configuration (`config/wheel_calib.yaml`)**:
   ```yaml
   wheel_calib:
     wheel_radius: [r1, r2, r3, r4]
     wheelbase: 0.1312
     track_width: 0.1312
   ```
3. **EKF Configuration (`src/omni_localization/config/ekf.yaml`)**:
   - `publish_tf: true`
   - `odom0_config`: $[v_x, v_y, \omega_z]$ enabled.
   - `imu0_config`: $[\text{yaw}, \omega_z, a_x, a_y]$ enabled.
   - Full non-zero diagonal entries for $15 \times 15$ `process_noise_covariance` and `initial_estimate_covariance`.

### 2.2 Inspection of Production Code in `web/backend/app/routers/calib.py`

In `web/backend/app/routers/calib.py`, lines 872–888 implement `/api/calib/apply`:

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

#### Observations:
- **Zero YAML File Operations**: The function receives `payload: Optional[CalibApplyPayload]` where `CalibApplyPayload.save_yaml` defaults to `True`. However, `payload.save_yaml` is never read, `yaml` is never imported, and no file write occurs.
- **Missing Destination Directory**: `/home/sonev/amr_omni/config/` does not exist on disk.
- **Missing Wheel Persistence**: While `/api/calib/encoder/start` computes `wheel_radii: [r1, r2, r3, r4]`, `/api/calib/apply` does not write `config/wheel_calib.yaml`.
- **Missing Atomic Write Protocol**: `test_f4_4_yaml_persistence_atomic_write` tests that YAML persistence uses atomic replacement (write to `.tmp` file, then `os.replace`), but this pattern is absent in the production backend.

---

## 3. Endpoints & Parameter Validation Inspection

### 3.1 Endpoint Inventory in `web/backend/app/routers/calib.py`

| HTTP Method | Route | Schema / Payload | Target Functionality | Status in Code |
|:---|:---|:---|:---|:---|
| `POST` | `/api/calib/start` | `CalibStartPayload` | Initiates sampling worker task | Implemented (IMU only) |
| `POST` | `/api/calib/abort` | `CalibAbortPayload` | Cancels active sampling worker | Implemented |
| `GET` | `/api/calib/status` | None | Returns calibration state dict | Implemented |
| `GET` | `/api/calib/results` | None | Returns calculated matrices | Implemented |
| `POST` | `/api/calib/apply` | `CalibApplyPayload` | Persists to STM32 Flash & YAML | Incomplete (No YAML write) |
| `POST` | `/api/calib/step` | None | Advances progress by 20% | Implemented (sim helper) |
| `POST` | `/api/calib/toggle` | `CalibTogglePayload` | Enables/disables correction | Implemented |
| `POST` | `/api/calib/reset` | None | Clears matrices to default | Implemented |
| `POST` | `/api/calib/noise` | `CalibNoisePayload` | Sets simulated noise level | Implemented |
| `POST` | `/api/calib/face/select_step` | `CalibSelectStepPayload` | Selects AN4508 orientation | Implemented |
| `POST` | `/api/calib/sim/set_pose` | `SimPosePayload` | Commands Gazebo & Sim angles | Implemented |
| `POST` | `/api/calib/face/sample` | `CalibFacePayload` | Samples 1 of 6 AN4508 faces | Implemented |
| `POST` | `/api/calib/pose/record` | `CalibPoseRecordPayload` | Multi-pose arbitrary sample | Implemented |
| `POST` | `/api/calib/pose/compute` | None | Multi-pose gradient solver | Implemented |
| `POST` | `/api/calib/pose/clear` | None | Clears multi-pose buffer | Implemented |
| `POST` | `/api/calib/encoder/start` | `CalibEncoderPayload` | Solves 4 wheel radii $K_r$ | Implemented (in-memory) |
| `POST` | `/api/calib/extrinsics/start`| `CalibExtrinsicsPayload` | Solves lever arm & latency | Implemented (in-memory) |

### 3.2 Parameter Validation Gaps

1. **Routine Type Mismatch in `/api/calib/start`**:
   - `CalibStartPayload` accepts `routine: Optional[str] = "imu"`.
   - If `payload.routine == "wheel"` or `"noise"` is passed, `start_calibration()` still spawns `_sampling_worker(routine, subtype, target_samples)` which runs IMU sampling logic against `bridge.gyro_x` and `bridge.accel_z`. It does not trigger wheel motion or noise profiling on hardware/simulator.
2. **`CalibApplyPayload.persist_flash` Ignored**:
   - `CalibApplyPayload` defines `persist_flash: Optional[bool] = True`.
   - `apply_calibration()` sends `{"action": "apply_params", ...}` but never sends `0x14 CMD_CALIB_FLASH_COMMIT` or `action: "flash_commit"`.
3. **Input Range Checks**:
   - In `/api/calib/encoder/start`: If `wheel_travel_m` contains zero or negative values, division `r0 * (true_dist / max(0.01, d))` guards division by zero, but does not reject non-physical negative travel values.
   - In `/api/calib/extrinsics/start`: If $\omega_1 == \omega_2$, $\Delta \omega^2 = 0$, the solver silently defaults to `[0.05, 0.0, 0.08]` without returning an HTTP 422 or error message.

---

## 4. WebSocket Streaming vs Real-Time Progress Contract

### 4.1 Specification (`PROJECT.md § Interface Contracts: 1 & § Feature Inventory F4.3`)
- Telemetry frame `TELEM_CALIB_PROGRESS` (ID `0x81`) streamed at 5 Hz:
  - `CalibType` (1B)
  - `Stage` (1B)
  - `ProgressPercent` (1B)
  - `StatusCode` (1B)
  - `LiveMetric` (float32)

### 4.2 Codebase Reality
1. **`TelemetryHub` (`web/backend/app/services/telemetry_hub.py`)**:
   - Runs a 20 Hz broadcast loop (`_broadcast_loop`) streaming:
     `{"type": "telemetry", "status": ..., "wheels": ..., "imu": ..., "odom": ..., "lidar": ..., "paths": ..., "streams": ..., "debug": ...}`
   - Does **not** include a `calib` field or progress telemetry in the broadcast dictionary.
2. **Frontend Fallback (`web/frontend/src/components/CalibrationPanel.tsx`)**:
   - Lines 236–274: The frontend sets an interval:
     ```typescript
     const fetchStatus = useCallback(async () => {
       const res = await fetch("/api/calib/status");
       if (res.ok) {
         const data = await res.json();
         setCalibState(data);
       }
     }, []);
     useEffect(() => {
       fetchStatus();
       const interval = setInterval(fetchStatus, 1000);
       return () => clearInterval(interval);
     }, [fetchStatus]);
     ```
   - Calibration status is obtained exclusively via 1 Hz polling over HTTP GET, defeating the low-latency progress streaming design.

---

## 5. Serial Contract & ROS 2 Bridge Discrepancy

### 5.1 Binary Serial Protocol Contract (`firmware` and `tests/e2e/harness/serial_protocol_oracle.py`)
- Frame format:
  `[0xAA 0x55] [Len: 1B] [Seq: 1B] [MsgID: 1B] [Payload: 0-64B] [CRC16-CCITT: 2B (Little Endian)] [0x7D]`
- All command IDs (`0x10`–`0x14`) and telemetry/response IDs (`0x80`–`0x84`) are defined and tested in `serial_protocol_oracle.py` and `serial_protocol.h`.

### 5.2 ROS 2 Web Bridge (`web/backend/app/bridges/ros2_bridge.py`)
- In `ros2_bridge.py` lines 520–525:
  ```python
  def send_calib_cmd(self, payload: dict):
      if self.calib_cmd_pub and self.running:
          msg = String()
          msg.data = json.dumps(payload)
          self.calib_cmd_pub.publish(msg)
  ```
- Subscribes to `calib/status` as `std_msgs/msg/String` and parses JSON (`json.loads(msg.data)`).
- **Parity Status**:
  - `omni_simulation/stm32_simulator.py` subscribes to `calib/cmd` (`std_msgs/msg/String`) and handles JSON dictionaries. Thus, simulation bridge parity is maintained for JSON.
  - However, for a physical serial connection (`/dev/ttyACM0` or `/dev/ttyTHS1`), `omni_hardware/stm32_bridge.py` contains **no calibration publishers or subscribers**, and there is no node converting binary serial frames to `calib/cmd` or vice versa.

---

## 6. Test Suite Execution & Analysis

### 6.1 Execution Results

1. **Web Backend Pytest Suite**:
   ```bash
   python3 -m pytest web/backend/tests/ -v
   ```
   - **Result**: `24 passed, 143 warnings in 1.18s` (Exit code: 0).
   - Test breakdown:
     - `test_api.py`: 23 tests passing (health, status, config, streams, cmd_vel, estop, nav, map, grid_planner, calib_toggle, calib_sampling, an4508_faces, mpc, arbitrary_pose, encoder/extrinsics, tilt compensation).
     - `test_full_system.py`: 1 comprehensive integration test passing.

2. **E2E Feature Coverage F4 Suite**:
   ```bash
   python3 -m pytest tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py -v
   ```
   - **Result**: `25 passed, 141 warnings in 0.72s` (Exit code: 0).
   - Test breakdown:
     - F4.1 Binary Serial Protocol Contract (5 tests): CRC16, framing, serialization, deserialization, rejection.
     - F4.2 FastAPI Calibration Endpoints (5 tests): app load, schema contract, payload validation, abort, status.
     - F4.3 Real-Time Progress Streaming (5 tests): packet structure, monotonicity, stage transitions, live metric, 5 Hz frequency.
     - F4.4 Automated YAML Persistence (5 tests): IMU schema, wheel schema, atomic write, reload consistency, directory auto-create.
     - F4.5 Web UI Calibration Dashboard (5 tests): 1-click trigger, progress bar 0-100, parameter table schema, error banner, residual formatting.

3. **Full E2E Suite**:
   ```bash
   python3 -m pytest tests/e2e/ -v
   ```
   - **Result**: `157 passed, 141 warnings in 2.28s` (Exit code: 0).

### 6.2 Test Blind Spot Analysis
Despite 100% passing tests, the tests do not catch the missing YAML persistence in `web/backend/app/routers/calib.py` because:
- In `test_f4_serial_web_calib.py`, `TestF44_AutomatedYAMLConfigurationPersistence` creates dummy dictionaries and invokes `yaml.safe_dump()` into a pytest `tmp_path` fixture directly.
- In `web/backend/tests/test_api.py`, there is no test calling `POST /api/calib/apply` and verifying file creation on disk.
- In `TestF42_FastAPICalibrationEndpoints`, `test_f4_2_fastapi_calib_endpoints_schema_contract` only asserts against a hard-coded Python list `len(endpoints) == 5`.

---

## 7. Concrete Remediation Proposal

To achieve full compliance with `PROJECT.md` and production readiness for Milestone 4, the following structured changes are recommended:

### 7.1 Architecture Refactoring: Introduce `web/backend/app/services/calib_service.py`
Extract the business logic from `routers/calib.py` into `services/calib_service.py`:
- Encapsulate `CalibrationState` class and `_sampling_worker`.
- Implement `persist_calibration_yaml(results, workspace_root)`:
  - Create `/home/sonev/amr_omni/config` if missing (`mkdir(parents=True, exist_ok=True)`).
  - Write `config/imu_calib.yaml` using atomic temp-file rename (`.tmp` -> `.yaml`).
  - Write `config/wheel_calib.yaml` using atomic temp-file rename.
  - Optionally update initial covariances or verify `src/omni_localization/config/ekf.yaml`.

```python
# Proposed implementation sketch for calib_service.py:
def persist_yaml_configs(results: dict, workspace_root: Path) -> dict:
    config_dir = workspace_root / "config"
    config_dir.mkdir(parents=True, exist_ok=True)
    
    # 1. Persist IMU
    imu_file = config_dir / "imu_calib.yaml"
    imu_tmp = config_dir / "imu_calib.yaml.tmp"
    imu_data = {
        "imu_calib": {
            "gyro_bias": results.get("gyro_bias", [0.0, 0.0, 0.0]),
            "accel_scale": results.get("accel_scale", [1.0, 1.0, 1.0]),
            "accel_bias": results.get("accel_bias", [0.0, 0.0, 0.0]),
            "frame_id": "imu_link",
        }
    }
    with open(imu_tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(imu_data, f, default_flow_style=False)
    imu_tmp.replace(imu_file)

    # 2. Persist Wheels
    wheel_file = config_dir / "wheel_calib.yaml"
    wheel_tmp = config_dir / "wheel_calib.yaml.tmp"
    wheel_data = {
        "wheel_calib": {
            "wheel_radius": results.get("wheel_radii", [0.03, 0.03, 0.03, 0.03]),
            "wheelbase": results.get("wheelbase", 0.1312),
            "track_width": results.get("track_width", 0.1312),
        }
    }
    with open(wheel_tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(wheel_data, f, default_flow_style=False)
    wheel_tmp.replace(wheel_file)
    
    return {"imu_yaml": str(imu_file), "wheel_yaml": str(wheel_file)}
```

### 7.2 Update `/api/calib/apply` Endpoint in `routers/calib.py`
Connect `apply_calibration` to `calib_service.persist_yaml_configs()` when `payload.save_yaml` is `True`.

### 7.3 Integrate Calibration Streaming into `TelemetryHub`
In `web/backend/app/services/telemetry_hub.py`, include `calib` in `_broadcast_loop`:
```python
"calib": {
    "active": _calib_state["active"],
    "stage": _calib_state["stage"],
    "progress_percent": _calib_state["progress_percent"],
    "status_code": _calib_state["status_code"],
    "live_metrics": _calib_state["live_metrics"],
    "is_calibrated": _calib_state["is_calibrated"],
}
```
This enables the frontend to receive real-time 5–20 Hz updates over the established WebSocket without HTTP polling.

### 7.4 Add Backend Test for `/api/calib/apply`
Add `test_calib_apply_persists_yaml` in `web/backend/tests/test_api.py` and in `tests/e2e/tier1_feature_coverage/test_f4_serial_web_calib.py` that executes `POST /api/calib/apply` and validates the resulting YAML files using `ConfigVerifier`.
