# Hard Handoff Report: Milestone 4 Serial Calibration Protocol Parity

**Author**: `teamwork_preview_explorer_m4_1`  
**Date**: 2026-09-20T07:33:00Z  
**Type**: Hard Handoff (Investigation Complete)  
**Deliverables**:
- Full technical report: `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/m4_serial_protocol_analysis.md`
- Handoff report: `/home/sonev/amr_omni/.agents/teamwork_preview_explorer_m4_1/handoff.md`

---

## 1. Observation

1. **`PROJECT.md § Interface Contracts: 1. STM32 <-> Jetson Serial Binary Contract`**:
   - Frame specification: `[0xAA 0x55] [Length: 1B] [Seq: 1B] [MsgID: 1B] [Payload: 0-64B] [CRC16: 2B] [0x7D]`
   - Non-payload bytes: Header(2) + Length(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1) = 8 bytes.
   - Command Frames: `0x10` (`CMD_CALIB_TRIGGER_IMU`), `0x11` (`CMD_CALIB_TRIGGER_WHEEL`), `0x12` (`CMD_CALIB_START_NOISE_PROFILE`), `0x13` (`CMD_CALIB_ABORT`), `0x14` (`CMD_CALIB_FLASH_COMMIT`).
   - Telemetry & Result Frames: `0x80` (`RESP_ACK_NACK`), `0x81` (`TELEM_CALIB_PROGRESS`), `0x82` (`RESP_CALIB_IMU_RESULT`), `0x83` (`RESP_CALIB_WHEEL_RESULT`), `0x84` (`RESP_CALIB_NOISE_RESULT`).

2. **`firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h:13`**:
   - Line 13 verbatim:
     ```c
     #define SERIAL_FRAME_OVERHEAD   7  // Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)
     ```
   - Macro defines value `7`, while comment adds `2 + 1 + 1 + 1 + 2 + 1 = 8`.

3. **`firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp`**:
   - Lines 23-42 (`serialize_serial_frame`):
     ```cpp
     size_t total_len = SERIAL_FRAME_OVERHEAD + frame->length;
     if (buffer_size < total_len) return 0;
     ...
     buffer[5 + frame->length] = static_cast<uint8_t>(crc & 0xFF);        // LSB
     buffer[6 + frame->length] = static_cast<uint8_t>((crc >> 8) & 0xFF); // MSB
     buffer[7 + frame->length] = SERIAL_TAIL_BYTE;
     return total_len;
     ```
     Writes up to index `7 + frame->length` (which requires `8 + frame->length` bytes), but returns `total_len` (= `7 + frame->length`).
   - Lines 59-69 (`deserialize_serial_frame`):
     ```cpp
     size_t total_len = SERIAL_FRAME_OVERHEAD + len;
     if (buffer_len < total_len) return false;
     if (buffer[total_len - 1] != SERIAL_TAIL_BYTE) return false;
     uint16_t crc_recv = static_cast<uint16_t>(buffer[5 + len]) |
                         (static_cast<uint16_t>(buffer[6 + len]) << 8);
     ```
     `total_len - 1` evaluates to `(7 + len) - 1 = 6 + len`.
     Byte `buffer[6 + len]` is the MSB of the received CRC16, NOT `SERIAL_TAIL_BYTE` (`0x7D`).
     Line 84: `*consumed_bytes = total_len;` (returns `7 + len` bytes, leaving byte `7 + len` `[0x7D]` in buffer).

4. **`src/omni_simulation/omni_simulation/stm32_simulator.py`**:
   - Line 34: `SERIAL_FRAME_OVERHEAD = 8`
   - Lines 684-706 (`deserialize_serial_frame`):
     `total_len = SERIAL_FRAME_OVERHEAD + payload_len` (i.e. `8 + payload_len`).
     `if buffer[total_len - 1] != SERIAL_TAIL_BYTE:` checks index `7 + payload_len`, which is `0x7D`.
     `crc_calc = compute_crc16_ccitt(buffer[2: 5 + payload_len], 0xFFFF)`

5. **`tests/e2e/tier2_boundary_corner/test_boundary_protocol_corruption.py:36`**:
   - Verbatim: `assert consumed == 8  # 2 + 1 + 1 + 1 + 0 + 2 + 1`

6. **`firmware/stm32_f407vg_arduino_sim/platformio.ini:40`**:
   - Verbatim: `build_src_filter = -<*> +<kalman.cpp> +<kinematics.cpp> +<pid.cpp> +<encoder_pll.cpp> +<imu_calibration.cpp>`
   - `serial_protocol.cpp` is excluded from native unit testing.

7. **`firmware/stm32_f407vg_arduino_sim/src/main.cpp`**:
   - Lines 1-32: `serial_protocol.h` is not included.
   - Lines 666-694: `setup()` creates tasks `micro_ros`, `control`, `encoder`, `imu`, `odometry`, `safety`.
   - No task deserializes or routes `CMD_CALIB_*` binary serial packets.

8. **Build & Test Tool Invocations**:
   - `pio test -e native`: 34/34 tests passed.
   - `pio run -e renode_f407vg`: Build SUCCESS.
   - `pio run -e disco_f407vg`: Build SUCCESS.
   - `pytest tests/`: 207/207 passed (Python oracle and simulator test against Python implementation).

---

## 2. Logic Chain

1. **Premise 1**: From Observation 1, a serial packet is structured as `[0xAA 0x55] [Length: 1B] [Seq: 1B] [MsgID: 1B] [Payload: 0-64B] [CRC16: 2B] [0x7D]`. The non-payload framing consists of: 2 header bytes + 1 length byte + 1 sequence byte + 1 message ID byte + 2 CRC bytes + 1 tail byte = 8 bytes.
2. **Premise 2**: Observation 2 shows `serial_protocol.h` defines `SERIAL_FRAME_OVERHEAD` as 7.
3. **Deduction 2.1 (Serialization Truncation)**: In `serialize_serial_frame` (Observation 3), the tail byte `0x7D` is stored at index `7 + frame->length`. The number of written bytes is `(7 + frame->length) + 1 = 8 + frame->length`. However, `serialize_serial_frame` returns `total_len = SERIAL_FRAME_OVERHEAD + frame->length = 7 + frame->length`. The returned length is 1 byte too short, truncating the `0x7D` tail byte during serial transmission.
4. **Deduction 2.2 (Deserialization Failure)**: In `deserialize_serial_frame` (Observation 3), `total_len` is computed as `7 + len`. The code checks `buffer[total_len - 1] != SERIAL_TAIL_BYTE`, which accesses `buffer[6 + len]`. Since `buffer[6 + len]` contains the high byte of the CRC16 (`crc >> 8`), deserialization tests whether `(crc >> 8) == 0x7D`. For any frame where `(crc >> 8) != 0x7D` (255 out of 256 random cases), the function returns `false`, rejecting valid packets.
5. **Deduction 2.3 (Python Parity Discrepancy)**: In Python `stm32_simulator.py` (Observation 4) and E2E test oracle (Observation 5), `SERIAL_FRAME_OVERHEAD` is 8 and checks index `7 + payload_len`. Therefore, Python simulator and E2E oracle are in parity with the contract, but STM32 C++ firmware diverges by 1 byte.
6. **Deduction 2.4 (Blind Spot Cause)**: Observation 6 shows `serial_protocol.cpp` is omitted from `build_src_filter` for `env:native`, and no C++ native unit tests exist for `serial_protocol.cpp`. Hence, C++ build and test pipelines did not detect this defect.

---

## 3. Caveats

1. **Hardware UART Pin Routing**: In physical STM32F407VG deployments, if micro-ROS uses `Serial` (USART2 or USB CDC), binary serial protocol frames require either a multiplexed framing parser on the same stream or a secondary hardware UART (e.g. USART3/UART4). This investigation evaluated the protocol contract logic, not physical board pin allocation.
2. **High-Level Transport**: The Web UI backend currently publishes JSON to ROS 2 topic `calib/cmd` rather than streaming raw binary packets directly from FastAPI to the robot over a serial device. The binary contract is tested via Python oracle and simulated frames.

---

## 4. Conclusion

1. **Critical Defect Identified**: STM32 C++ firmware has a severe defect in `SERIAL_FRAME_OVERHEAD` (defined as 7 instead of 8) in `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h:13`, breaking both frame serialization and deserialization.
2. **High Parity on CRC and Message IDs**: CRC16-CCITT algorithm and all 10 message IDs (`0x10`–`0x14`, `0x80`–`0x84`) are 100% matched between C++ firmware, simulator, and test oracle.
3. **Actionable Fix Required**:
   - Change `SERIAL_FRAME_OVERHEAD` from 7 to 8 in `serial_protocol.h`.
   - Add strongly typed payload structs for all 10 frames to `serial_protocol.h`.
   - Include `serial_protocol.cpp` in `platformio.ini` `env:native` `build_src_filter`.
   - Add PlatformIO native test suite `test_serial_protocol` to prevent regression.

---

## 5. Verification Method

1. **Check Overhead Value**:
   ```bash
   grep -n "SERIAL_FRAME_OVERHEAD" firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h
   # Must return: SERIAL_FRAME_OVERHEAD 8
   ```
2. **Run PlatformIO Native Tests**:
   ```bash
   pio test -e native
   # 34 existing tests must pass; once serial_protocol is added, new tests must pass.
   ```
3. **Run Firmware Target Builds**:
   ```bash
   pio run -e disco_f407vg
   pio run -e renode_f407vg
   ```
4. **Run Full Test Suite**:
   ```bash
   pytest tests/
   # All 207 tests must pass.
   ```
