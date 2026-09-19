"""
Authoritative Reference Oracle for STM32 <-> Jetson Binary Serial Protocol Contract.
Implements frame structure, CRC16 verification, serialization and deserialization
according to PROJECT.md § Interface Contracts.
"""
import struct


def compute_crc16_ccitt(data: bytes, init_val: int = 0xFFFF) -> int:
    """Standard CRC16-CCITT calculation (polynomial 0x1021)."""
    crc = init_val
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SerialPacket:
    """Represents a structured binary serial frame."""

    HEADER_SYNC = bytes([0xAA, 0x55])
    TAIL_BYTE = 0x7D

    # Message IDs
    CMD_CALIB_TRIGGER_IMU = 0x10
    CMD_CALIB_TRIGGER_WHEEL = 0x11
    CMD_CALIB_START_NOISE_PROFILE = 0x12
    CMD_CALIB_ABORT = 0x13
    CMD_CALIB_FLASH_COMMIT = 0x14

    RESP_ACK_NACK = 0x80
    TELEM_CALIB_PROGRESS = 0x81
    RESP_CALIB_IMU_RESULT = 0x82
    RESP_CALIB_WHEEL_RESULT = 0x83
    RESP_CALIB_NOISE_RESULT = 0x84

    def __init__(self, msg_id: int, payload: bytes = b"", seq: int = 0):
        if not (0 <= msg_id <= 0xFF):
            raise ValueError("msg_id must be uint8 (0-255)")
        if len(payload) > 64:
            raise ValueError("Payload cannot exceed 64 bytes")
        if not (0 <= seq <= 0xFF):
            raise ValueError("seq must be uint8 (0-255)")

        self.msg_id = int(msg_id)
        self.payload = bytes(payload)
        self.seq = int(seq)

    def serialize(self) -> bytes:
        """Encode to binary frame: [0xAA 0x55] [Len] [Seq] [MsgID] [Payload] [CRC16: 2B] [0x7D]."""
        length = len(self.payload)
        header_and_body = bytes([length, self.seq, self.msg_id]) + self.payload
        crc = compute_crc16_ccitt(header_and_body)
        crc_bytes = struct.pack("<H", crc)  # Little endian 2 bytes
        return self.HEADER_SYNC + header_and_body + crc_bytes + bytes([self.TAIL_BYTE])

    @classmethod
    def deserialize(cls, raw_bytes: bytes) -> tuple["SerialPacket", int]:
        """
        Parse raw bytes into SerialPacket.
        Returns (packet, consumed_bytes).
        Raises ValueError on malformed frames.
        """
        if len(raw_bytes) < 7:  # Min length: Header(2) + Len(1) + Seq(1) + ID(1) + CRC(2) + Tail(1) = 7
            raise ValueError("Frame too short to parse")

        if raw_bytes[:2] != cls.HEADER_SYNC:
            raise ValueError(f"Invalid header sync: expected 0xAA 0x55, got {raw_bytes[:2].hex()}")

        length = raw_bytes[2]
        if length > 64:
            raise ValueError(f"Invalid payload length: {length} > 64")

        total_expected_len = 2 + 1 + 1 + 1 + length + 2 + 1  # 7 + length
        if len(raw_bytes) < total_expected_len:
            raise ValueError(f"Incomplete frame: expected {total_expected_len} bytes, got {len(raw_bytes)}")

        seq = raw_bytes[3]
        msg_id = raw_bytes[4]
        payload = raw_bytes[5:5 + length]
        crc_received = struct.unpack("<H", raw_bytes[5 + length:7 + length])[0]
        tail = raw_bytes[7 + length]

        if tail != cls.TAIL_BYTE:
            raise ValueError(f"Invalid tail byte: expected 0x7D, got 0x{tail:02X}")

        # Verify CRC
        header_and_body = raw_bytes[2:5 + length]
        crc_computed = compute_crc16_ccitt(header_and_body)
        if crc_received != crc_computed:
            raise ValueError(f"CRC mismatch: received 0x{crc_received:04X}, computed 0x{crc_computed:04X}")

        return cls(msg_id=msg_id, payload=payload, seq=seq), total_expected_len

    # --- Helper builders and parsers for specific frame types ---

    @classmethod
    def build_imu_trigger_cmd(cls, subtype: int = 0, seq: int = 0) -> "SerialPacket":
        return cls(msg_id=cls.CMD_CALIB_TRIGGER_IMU, payload=bytes([subtype]), seq=seq)

    @classmethod
    def build_wheel_trigger_cmd(cls, subtype: int = 0, seq: int = 0) -> "SerialPacket":
        return cls(msg_id=cls.CMD_CALIB_TRIGGER_WHEEL, payload=bytes([subtype]), seq=seq)

    @classmethod
    def build_noise_profile_cmd(cls, duration_s: int = 30, seq: int = 0) -> "SerialPacket":
        payload = struct.pack("<H", duration_s)
        return cls(msg_id=cls.CMD_CALIB_START_NOISE_PROFILE, payload=payload, seq=seq)

    @classmethod
    def build_abort_cmd(cls, seq: int = 0) -> "SerialPacket":
        return cls(msg_id=cls.CMD_CALIB_ABORT, payload=b"", seq=seq)

    @classmethod
    def build_flash_commit_cmd(cls, seq: int = 0) -> "SerialPacket":
        return cls(msg_id=cls.CMD_CALIB_FLASH_COMMIT, payload=b"", seq=seq)

    @classmethod
    def build_telem_progress(cls, calib_type: int, stage: int, percent: int,
                            status: int, live_metric: float, seq: int = 0) -> "SerialPacket":
        payload = struct.pack("<BBBBf", calib_type, stage, percent, status, live_metric)
        return cls(msg_id=cls.TELEM_CALIB_PROGRESS, payload=payload, seq=seq)

    @classmethod
    def parse_telem_progress(cls, packet: "SerialPacket") -> dict:
        if packet.msg_id != cls.TELEM_CALIB_PROGRESS or len(packet.payload) != 8:
            raise ValueError("Not a valid progress telemetry packet")
        calib_type, stage, percent, status, metric = struct.unpack("<BBBBf", packet.payload)
        return {
            "calib_type": calib_type,
            "stage": stage,
            "progress_percent": percent,
            "status_code": status,
            "live_metric": float(metric),
        }

    @classmethod
    def build_imu_result(cls, bias_g: tuple[float, float, float],
                         bias_a: tuple[float, float, float],
                         scale_a: tuple[float, float, float],
                         residual_norm: float, seq: int = 0) -> "SerialPacket":
        payload = struct.pack("<ffffffffff",
                              bias_g[0], bias_g[1], bias_g[2],
                              bias_a[0], bias_a[1], bias_a[2],
                              scale_a[0], scale_a[1], scale_a[2],
                              residual_norm)
        return cls(msg_id=cls.RESP_CALIB_IMU_RESULT, payload=payload, seq=seq)

    @classmethod
    def parse_imu_result(cls, packet: "SerialPacket") -> dict:
        if packet.msg_id != cls.RESP_CALIB_IMU_RESULT or len(packet.payload) != 40:
            raise ValueError("Not a valid IMU result packet")
        vals = struct.unpack("<ffffffffff", packet.payload)
        return {
            "bias_g": (vals[0], vals[1], vals[2]),
            "bias_a": (vals[3], vals[4], vals[5]),
            "scale_a": (vals[6], vals[7], vals[8]),
            "residual_norm": vals[9],
        }

    @classmethod
    def build_wheel_result(cls, radii: tuple[float, float, float, float],
                           leff: float, weff: float, residual_err: float, seq: int = 0) -> "SerialPacket":
        payload = struct.pack("<fffffff", radii[0], radii[1], radii[2], radii[3], leff, weff, residual_err)
        return cls(msg_id=cls.RESP_CALIB_WHEEL_RESULT, payload=payload, seq=seq)

    @classmethod
    def parse_wheel_result(cls, packet: "SerialPacket") -> dict:
        if packet.msg_id != cls.RESP_CALIB_WHEEL_RESULT or len(packet.payload) != 28:
            raise ValueError("Not a valid wheel result packet")
        vals = struct.unpack("<fffffff", packet.payload)
        return {
            "radii": (vals[0], vals[1], vals[2], vals[3]),
            "leff": vals[4],
            "weff": vals[5],
            "residual_err": vals[6],
        }

    @classmethod
    def build_noise_result(cls, ng: float, kg: float, na: float, ka: float, seq: int = 0) -> "SerialPacket":
        payload = struct.pack("<ffff", ng, kg, na, ka)
        return cls(msg_id=cls.RESP_CALIB_NOISE_RESULT, payload=payload, seq=seq)

    @classmethod
    def parse_noise_result(cls, packet: "SerialPacket") -> dict:
        if packet.msg_id != cls.RESP_CALIB_NOISE_RESULT or len(packet.payload) != 16:
            raise ValueError("Not a valid noise result packet")
        vals = struct.unpack("<ffff", packet.payload)
        return {
            "Ng": vals[0],
            "Kg": vals[1],
            "Na": vals[2],
            "Ka": vals[3],
        }
