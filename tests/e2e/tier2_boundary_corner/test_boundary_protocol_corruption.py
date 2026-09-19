"""
Tier 2 Boundary Cases: Serial Protocol Packet Corruptions, Bit Flips, and Frame Boundaries.
"""
import pytest

from tests.e2e.harness.serial_protocol_oracle import SerialPacket


@pytest.mark.tier2
class TestBoundaryProtocolCorruption:
    """Boundary test cases for serial frame framing and corruptions."""

    def test_boundary_max_payload_size_64_bytes(self):
        """Verify maximum allowed payload of 64 bytes is serialized and parsed cleanly."""
        payload_64 = bytes([i % 256 for i in range(64)])
        pkt = SerialPacket(msg_id=0x15, payload=payload_64, seq=42)
        raw = pkt.serialize()

        parsed, consumed = SerialPacket.deserialize(raw)
        assert consumed == len(raw)
        assert parsed.payload == payload_64
        assert len(parsed.payload) == 64

    def test_boundary_payload_exceeding_64_bytes_rejected(self):
        """Verify payload > 64 bytes raises ValueError immediately."""
        payload_65 = b"A" * 65
        with pytest.raises(ValueError, match="cannot exceed 64"):
            SerialPacket(msg_id=0x15, payload=payload_65)

    def test_boundary_zero_length_payload_packet(self):
        """Verify empty payload packets (e.g. abort command) parse correctly."""
        pkt = SerialPacket(msg_id=0x13, payload=b"", seq=100)
        raw = pkt.serialize()
        parsed, consumed = SerialPacket.deserialize(raw)

        assert consumed == 8  # 2 + 1 + 1 + 1 + 0 + 2 + 1
        assert parsed.payload == b""

    def test_boundary_single_bit_flip_in_payload_fails_crc(self):
        """Verify single bit flip in payload causes CRC verification failure."""
        pkt = SerialPacket(msg_id=0x10, payload=b"\x01\x02\x03\x04")
        raw = bytearray(pkt.serialize())

        # Flip 1 bit in payload (byte index 6)
        raw[6] ^= 0x01

        with pytest.raises(ValueError, match="CRC mismatch"):
            SerialPacket.deserialize(bytes(raw))

    def test_boundary_truncated_packet_stream(self):
        """Verify prematurely truncated byte stream raises ValueError."""
        pkt = SerialPacket(msg_id=0x82, payload=b"\x00" * 40)
        raw = pkt.serialize()

        # Truncate last 5 bytes
        truncated = raw[:-5]
        with pytest.raises(ValueError, match="too short|Incomplete frame"):
            SerialPacket.deserialize(truncated)
