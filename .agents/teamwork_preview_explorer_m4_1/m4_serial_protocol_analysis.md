# Milestone 4 Technical Analysis: Serial Calibration Protocol Contract Parity

**Author**: `teamwork_preview_explorer_m4_1`  
**Date**: 2026-09-20  
**Target Milestone**: Milestone 4 — Serial Calibration Protocol & Jetson Web UI  
**Target Interface**: `PROJECT.md § Interface Contracts: 1. STM32 <-> Jetson Serial Binary Contract`

---

## Executive Summary

This investigation performed an exhaustive technical audit comparing the binary serial calibration protocol implementation in the STM32 C++ firmware (`firmware/stm32_f407vg_arduino_sim`) against the Python simulation stack (`omni_simulation/stm32_simulator.py`), the hardware bridge (`omni_hardware/stm32_bridge.py`), and the authoritative E2E test oracle (`tests/e2e/harness/serial_protocol_oracle.py`).

### Key Findings Matrix
| Subsystem / Area | Parity Status | Severity | Summary |
|---|---|---|---|
| **Frame Overhead Constant** | **BROKEN (7 vs 8)** | **CRITICAL** | `serial_protocol.h` defines `SERIAL_FRAME_OVERHEAD` as 7 instead of 8. Breaks frame deserialization (MSB CRC checked instead of tail byte 0x7D) and truncates serialization. |
| **CRC16-CCITT Algorithm** | **100% PARITY** | OK | Polynomial 0x1021, init 0xFFFF, identical byte-by-byte bitshift logic across C++ and Python. Test vector "123456789" -> 0x29B1 verified. |
| **Message IDs (0x10-0x14, 0x80-0x84)** | **100% PARITY** | OK | All 10 command and telemetry/result message IDs match verbatim. |
| **Payload Structures & Serialization** | **PARTIAL** | MAJOR | C++ firmware lacks strongly typed payload structs (`#pragma pack(push, 1)`) and helper builders/parsers present in Python. |
| **PlatformIO Native Test Suite** | **MISSING** | MAJOR | `serial_protocol.cpp` is excluded from `env:native` `build_src_filter` in `platformio.ini`. Zero unit tests exist in C++ firmware for serial protocol. |
| **Firmware Task Integration** | **DISCONNECTED** | MAJOR | `main.cpp` does not include `serial_protocol.h`, nor does any FreeRTOS task invoke `deserialize_serial_frame` or dispatch calibration commands to `ImuCalibrator`. |
| **Simulator / Bridge Transport** | **DECOUPLED** | MEDIUM | Simulator node (`stm32_simulator.py`) and Web backend (`ros2_bridge.py`) communicate via JSON on ROS 2 topic `calib/cmd` rather than raw binary serial streams. |

---

## 1. Protocol Contract Specification Baseline (`PROJECT.md § 1`)

The binary serial contract between the Jetson host and STM32 microcontroller defines:

### 1.1 Frame Structure
```
[0xAA 0x55] [Length: 1B] [Seq: 1B] [MsgID: 1B] [Payload: 0-64B] [CRC16: 2B] [0x7D]
```
- **Header**: 2 bytes (`0xAA`, `0x55`)
- **Length**: 1 byte (`0` to `64`) — represents the length of the `Payload` field only.
- **Sequence Number (Seq)**: 1 byte (`uint8_t`, rolls over 0-255).
- **Message ID (MsgID)**: 1 byte (`uint8_t`).
- **Payload**: `Length` bytes (variable, 0 to 64 bytes).
- **CRC16**: 2 bytes (`uint16_t`, Little-Endian: LSB then MSB).
  - Calculated over `[Length, Seq, MsgID]` + `Payload` (i.e. `3 + Length` bytes).
  - CCITT standard polynomial `0x1021`, initial value `0xFFFF`.
- **Tail**: 1 byte (`0x7D`).

### 1.2 Overhead Calculation
```
Total Frame Size = 2 (Header) + 1 (Length) + 1 (Seq) + 1 (MsgID) + Length (Payload) + 2 (CRC16) + 1 (Tail)
                 = 8 + Length bytes.
```
Therefore:
$$\text{SERIAL\_FRAME\_OVERHEAD} \equiv 2 + 1 + 1 + 1 + 2 + 1 = 8 \text{ bytes}$$

---

## 2. In-Depth Comparative Code Audit

### 2.1 Critical Defect: `SERIAL_FRAME_OVERHEAD` Value Bug

#### Location:
`firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h:13`
```cpp
#define SERIAL_HEADER_SYNC_0 0xAA
#define SERIAL_HEADER_SYNC_1 0x55
#define SERIAL_TAIL_BYTE     0x7D

#define SERIAL_MAX_PAYLOAD_LEN 64
#define SERIAL_FRAME_OVERHEAD   7  // Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)
```

Notice that the author's own comment explicitly writes:
`Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)` which evaluates to $2 + 1 + 1 + 1 + 2 + 1 = 8$, but defined the constant as `7`!

#### Consequences in `firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp`:

1. **In `serialize_serial_frame` (Lines 23-42)**:
```cpp
size_t serialize_serial_frame(const SerialFrame *frame, uint8_t *buffer, size_t buffer_size) {
    if (!frame || !buffer) return 0;
    if (frame->length > SERIAL_MAX_PAYLOAD_LEN) return 0;

    size_t total_len = SERIAL_FRAME_OVERHEAD + frame->length; // BUG: 7 + length
    if (buffer_size < total_len) return 0; // Allows buffer of size 7 when 8 bytes are written!

    buffer[0] = SERIAL_HEADER_SYNC_0;
    buffer[1] = SERIAL_HEADER_SYNC_1;
    buffer[2] = frame->length;
    buffer[3] = frame->seq;
    buffer[4] = frame->msg_id;

    if (frame->length > 0 && frame->payload) {
        memcpy(&buffer[5], frame->payload, frame->length);
    }

    uint16_t crc = compute_crc16_ccitt(&buffer[2], 3 + frame->length, 0xFFFF);
    buffer[5 + frame->length] = static_cast<uint8_t>(crc & 0xFF);        // LSB
    buffer[6 + frame->length] = static_cast<uint8_t>((crc >> 8) & 0xFF); // MSB
    buffer[7 + frame->length] = SERIAL_TAIL_BYTE;                        // Index 7 + length!

    return total_len; // Returns 7 + length! Truncates the tail byte!
}
```
- For a frame with 0 payload bytes, it writes into indices 0, 1, 2, 3, 4, 5, 6, 7 (8 bytes in total), but returns `7`! The tail byte (`0x7D`) at index 7 is never transmitted over UART.
- If `buffer_size == 7`, the bounds check passes, and writing to `buffer[7]` causes a **1-byte buffer overrun**.

2. **In `deserialize_serial_frame` (Lines 46-86)**:
```cpp
bool deserialize_serial_frame(const uint8_t *buffer, size_t buffer_len, SerialFrame *frame, size_t *consumed_bytes) {
    if (!buffer || !frame || buffer_len < SERIAL_FRAME_OVERHEAD) { // buffer_len < 7
        return false;
    }
...
    size_t total_len = SERIAL_FRAME_OVERHEAD + len; // BUG: 7 + len
    if (buffer_len < total_len) {
        return false;
    }

    if (buffer[total_len - 1] != SERIAL_TAIL_BYTE) { // BUG: checks buffer[6 + len]!
        return false;
    }
...
    if (consumed_bytes) {
        *consumed_bytes = total_len; // BUG: consumes 7 + len bytes
    }
    return true;
}
```
- `total_len - 1` resolves to `(7 + len) - 1 = 6 + len`.
- Byte at index `6 + len` is the **MSB of the CRC16**, NOT the tail byte!
- Deserialization checks if `CRC_MSB == 0x7D`. Unless the computed CRC MSB happens to be `0x7D` by pure coincidence (1 in 256 chance), **`deserialize_serial_frame` unconditionally returns `false`!**
- Even if it passes, it returns `consumed_bytes = 7 + len`, leaving the `0x7D` tail byte in the receive ring buffer, causing all subsequent frames to fail sync detection (`0x7D != 0xAA`).

#### Comparison with Python Reference Implementations:
- In `src/omni_simulation/omni_simulation/stm32_simulator.py:34`:
  ```python
  SERIAL_FRAME_OVERHEAD = 8  # Correctly set to 8
  ```
  Lines 692-695:
  ```python
  total_len = SERIAL_FRAME_OVERHEAD + payload_len
  if buffer[total_len - 1] != SERIAL_TAIL_BYTE:  # Checks buffer[7 + payload_len], which IS 0x7D
      return None, 0
  ```
- In `tests/e2e/harness/serial_protocol_oracle.py:78`:
  ```python
  total_expected_len = 2 + 1 + 1 + 1 + length + 2 + 1  # Evaluates to 8 + length
  tail = raw_bytes[7 + length]
  if tail != cls.TAIL_BYTE:
      raise ValueError(...)
  ```
- In `tests/e2e/tier2_boundary_corner/test_boundary_protocol_corruption.py:36`:
  ```python
  assert consumed == 8  # 2 + 1 + 1 + 1 + 0 + 2 + 1
  ```

---

### 2.2 Message IDs and Payload Parity

| MsgID | Name | PROJECT.md Contract | C++ `serial_protocol.h` | Python `stm32_simulator.py` | Python Oracle | Parity |
|---|---|---|---|---|---|---|
| `0x10` | `CMD_CALIB_TRIGGER_IMU` | `[Subtype: 1B]` | Defined | Defined | `build_imu_trigger_cmd` | **MATCH** |
| `0x11` | `CMD_CALIB_TRIGGER_WHEEL` | `[Subtype: 1B]` | Defined | Defined | `build_wheel_trigger_cmd` | **MATCH** |
| `0x12` | `CMD_CALIB_START_NOISE_PROFILE` | `[Duration_s: 2B]` | Defined | Defined | `build_noise_profile_cmd` | **MATCH** |
| `0x13` | `CMD_CALIB_ABORT` | Empty (0B) | Defined | Defined | `build_abort_cmd` | **MATCH** |
| `0x14` | `CMD_CALIB_FLASH_COMMIT` | Empty (0B) | Defined | Defined | `build_flash_commit_cmd` | **MATCH** |
| `0x80` | `RESP_ACK_NACK` | `[TargetMsgID: 1B] [Status: 1B]` | Defined | Defined | Raw handling | **MATCH** |
| `0x81` | `TELEM_CALIB_PROGRESS` | `[Type: 1B][Stage: 1B][Pct: 1B][Status: 1B][Metric: 4B]` | Defined | Defined | `build/parse_telem_progress` | **MATCH** |
| `0x82` | `RESP_CALIB_IMU_RESULT` | `[bias_g: 3x4B][bias_a: 3x4B][scale_a: 3x4B][res: 4B]` | Defined | Defined | `build/parse_imu_result` | **MATCH** |
| `0x83` | `RESP_CALIB_WHEEL_RESULT` | `[radii: 4x4B][Leff: 4B][Weff: 4B][res: 4B]` | Defined | Defined | `build/parse_wheel_result` | **MATCH** |
| `0x84` | `RESP_CALIB_NOISE_RESULT` | `[Ng: 4B][Kg: 4B][Na: 4B][Ka: 4B]` | Defined | Defined | `build/parse_noise_result` | **MATCH** |

---

### 2.3 Firmware Architecture & Build Verification

1. **Target Compilation Verification**:
   - `pio run -e disco_f407vg`: **SUCCESS** (RAM: 47.1%, Flash: 13.8%).
   - `pio run -e renode_f407vg`: **SUCCESS** (RAM: 47.0%, Flash: 13.5%).
   - While `serial_protocol.cpp` compiles into `.pio/build/.../src/serial_protocol.cpp.o`, its functions are never called by `main.cpp` and are discarded by the linker (`--gc-sections`).

2. **Native Test Suite Verification**:
   - `pio test -e native`: **34/34 tests PASSED**.
   - `platformio.ini` line 40:
     ```ini
     build_src_filter = -<*> +<kalman.cpp> +<kinematics.cpp> +<pid.cpp> +<encoder_pll.cpp> +<imu_calibration.cpp>
     ```
     `serial_protocol.cpp` is omitted! Consequently, PlatformIO native tests never exercised `serial_protocol.cpp`.

3. **`firmware/stm32_f407vg_arduino_sim/src/main.cpp` Integration**:
   - `main.cpp` creates six FreeRTOS tasks: `micro_ros`, `control`, `encoder`, `imu`, `odometry`, `safety`.
   - Micro-ROS is used exclusively for ROS 2 communications (`cmd_vel`, `estop`, `odom`, `imu`, `status`).
   - `main.cpp` does NOT include `serial_protocol.h`, nor does it implement any serial RX task to parse binary frames or route them to `imu_calibrator`.

---

## 3. Recommended Fixes & Implementation Proposals

### Proposal 1: Fix `SERIAL_FRAME_OVERHEAD` and Buffer Logic

**Target file**: `firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h`
```diff
--- a/firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h
+++ b/firmware/stm32_f407vg_arduino_sim/include/serial_protocol.h
@@ -10,7 +10,7 @@
 #define SERIAL_TAIL_BYTE     0x7D

 #define SERIAL_MAX_PAYLOAD_LEN 64
-#define SERIAL_FRAME_OVERHEAD   7  // Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)
+#define SERIAL_FRAME_OVERHEAD   8  // Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)
```

### Proposal 2: Add Strongly Typed Payload Structs to `serial_protocol.h`

To eliminate magic byte offsets and manual casting, define packed payload structures:
```cpp
#pragma pack(push, 1)

// Command 0x10: CMD_CALIB_TRIGGER_IMU
enum ImuCalibSubtype : uint8_t {
    CALIB_SUBTYPE_STATIC_BIAS = 0,
    CALIB_SUBTYPE_6POS_ACCEL  = 1,
    CALIB_SUBTYPE_TEDALDI     = 2,
};
struct CmdCalibTriggerImuPayload {
    uint8_t subtype;
};

// Command 0x11: CMD_CALIB_TRIGGER_WHEEL
enum WheelCalibSubtype : uint8_t {
    CALIB_WHEEL_LINEAR   = 0,
    CALIB_WHEEL_STRAFE   = 1,
    CALIB_WHEEL_ROTATION = 2,
};
struct CmdCalibTriggerWheelPayload {
    uint8_t subtype;
};

// Command 0x12: CMD_CALIB_START_NOISE_PROFILE
struct CmdCalibStartNoisePayload {
    uint16_t duration_s;
};

// Response 0x80: RESP_ACK_NACK
enum AckStatus : uint8_t {
    ACK_STATUS_OK    = 0,
    ACK_STATUS_ERROR = 1,
};
struct RespAckNackPayload {
    uint8_t target_msg_id;
    uint8_t status;
};

// Telemetry 0x81: TELEM_CALIB_PROGRESS
struct TelemCalibProgressPayload {
    uint8_t calib_type;
    uint8_t stage;
    uint8_t progress_percent;
    uint8_t status_code;
    float live_metric;
};

// Response 0x82: RESP_CALIB_IMU_RESULT
struct RespCalibImuResultPayload {
    float bias_g[3];
    float bias_a[3];
    float scale_a[3];
    float residual_norm;
};

// Response 0x83: RESP_CALIB_WHEEL_RESULT
struct RespCalibWheelResultPayload {
    float radii[4];
    float leff;
    float weff;
    float residual_err;
};

// Response 0x84: RESP_CALIB_NOISE_RESULT
struct RespCalibNoiseResultPayload {
    float ng;
    float kg;
    float na;
    float ka;
};

#pragma pack(pop)
```

### Proposal 3: Add `serial_protocol.cpp` to Native Build and Unit Tests

1. Update `firmware/stm32_f407vg_arduino_sim/platformio.ini`:
```ini
build_src_filter = -<*> +<kalman.cpp> +<kinematics.cpp> +<pid.cpp> +<encoder_pll.cpp> +<imu_calibration.cpp> +<serial_protocol.cpp>
```
2. Create unit test suite `firmware/stm32_f407vg_arduino_sim/test/test_serial_protocol/test_main.cpp`:
- `test_crc16_ccitt_standard_vector` ("123456789" -> 0x29B1)
- `test_frame_serialize_overhead_and_tail` (checks overhead == 8, total length == 8 + len, buffer[7 + len] == 0x7D)
- `test_frame_deserialize_roundtrip` (serializes and deserializes across all 10 message types)
- `test_frame_corrupt_crc_rejection` (bit flip in payload or CRC rejects cleanly)
- `test_frame_corrupt_header_tail_rejection`
- `test_frame_max_and_zero_payload_boundary` (len = 0 and len = 64)

### Proposal 4: Simulator & Web Transport Unification

To support complete end-to-end testing of binary serial frames without requiring physical ST-Link/hardware USB UART:
- Maintain the high-level ROS 2 topic `calib/cmd` and `calib/status` for Web UI convenience, OR
- Add an emulation loop in `stm32_simulator.py` / `stm32_bridge.py` that translates `calib/cmd` ROS messages to binary serial packets via a pseudo-terminal (`pty`) or loopback socket, enabling pure binary streaming validation.

---
