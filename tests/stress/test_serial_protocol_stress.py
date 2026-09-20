#!/usr/bin/env python3
"""
Adversarial Stress & Fuzzing Suite for Serial Protocol Framing and Codec Robustness.
Validates:
1. Native C++ firmware codec execution (build/serial_protocol_stress_benchmark)
   under AddressSanitizer and UndefinedBehaviorSanitizer.
2. Buffer overflow boundaries: 0-byte, 64-byte, and >64-byte payloads (65..255 bytes).
3. Stream noise resilience: shifted sync headers [0xAA, 0x55], bit flips, corrupt CRC16,
   missing/corrupt tail byte 0x7D, and payload transparency.
4. Fuzzing with 50,000+ random byte sequences: 100% rejection rate without crashes.
5. Stream sliding window recovery under intermittent garbage and noise bursts.
6. 10,000-trial Monte Carlo roundtrip consistency across all command & telemetry frames.
"""
from pathlib import Path
import random
import struct
import subprocess
import pytest

from tests.e2e.harness.serial_protocol_oracle import (
    SerialPacket,
    compute_crc16_ccitt,
)

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
BUILD_DIR = WORKSPACE_ROOT / "build"
BENCHMARK_BIN = BUILD_DIR / "serial_protocol_stress_benchmark"
BENCHMARK_SRC = WORKSPACE_ROOT / "tests" / "stress" / "serial_protocol_stress_benchmark.cpp"
FIRMWARE_INC = WORKSPACE_ROOT / "firmware" / "stm32_f407vg_arduino_sim" / "include"


@pytest.fixture(scope="session", autouse=True)
def build_native_benchmark():
    """Ensure standalone C++ empirical benchmark binary is compiled with ASan/UBSan."""
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    compile_cmd = [
        "g++", "-O3", "-Wall", "-Wextra", "-Wno-address",
        "-fsanitize=address,undefined",
        f"-I{FIRMWARE_INC}",
        str(BENCHMARK_SRC),
        "-o", str(BENCHMARK_BIN)
    ]
    res = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert res.returncode == 0, f"Benchmark compilation failed:\n{res.stderr}"
    return BENCHMARK_BIN


class TestSerialProtocolAdversarialStress:
    """Adversarial stress and fuzzing verification for binary serial protocol."""

    def test_cpp_benchmark_execution_and_verdict(self):
        """Execute standalone C++ benchmark with ASan/UBSan and verify all 5 suites pass."""
        assert BENCHMARK_BIN.exists(), f"Benchmark binary {BENCHMARK_BIN} missing"
        res = subprocess.run([str(BENCHMARK_BIN)], capture_output=True, text=True)
        assert res.returncode == 0, f"Benchmark failed with exit code {res.returncode}:\n{res.stdout}\n{res.stderr}"

        stdout = res.stdout
        assert "--> Running Test 1: Buffer Boundaries, Overflows & Underflows" in stdout
        assert "0-Byte Payload Roundtrip:        PASS" in stdout
        assert "64-Byte Payload Roundtrip:       PASS" in stdout
        assert ">64-Byte (65..255) Rejection:    PASS" in stdout
        assert "Small Buffer Canary Protection:  PASS" in stdout
        assert "Truncated Buffer Safety:         PASS" in stdout
        assert "Null Pointer Safety:             PASS" in stdout

        assert "--> Running Test 2: Stream Noise, CRC16 Bit Flips, and Framing Integrity" in stdout
        assert "Bit Flips Detected by CRC:       1512 (100.00%)" in stdout
        assert "Payload Framing Transparency:    PASS" in stdout
        assert "Shifted Header Recovery Tests:   64 (Recovered: 64, 100.00%)" in stdout

        assert "--> Running Test 3: 50,000 Random Byte Fuzzing Trials" in stdout
        assert "Rejected Malformed Sequences:    50000 (100.00%)" in stdout
        assert "Segfaults / ASan Faults:         0" in stdout

        assert "--> Running Test 4: 10,000 Mutated Valid Frames Fuzzing" in stdout
        assert "OVERALL VERDICT: ALL 5 TEST SUITES PASSED (100.0%)" in stdout

    def test_python_oracle_buffer_overflow_boundaries(self):
        """Verify strict payload length constraints: 0-byte, 64-byte, and >64-byte payloads."""
        # 1. 0-byte payload roundtrip
        pkt0 = SerialPacket(msg_id=0x13, payload=b"", seq=1)
        raw0 = pkt0.serialize()
        assert len(raw0) == 8
        des0, consumed0 = SerialPacket.deserialize(raw0)
        assert consumed0 == 8
        assert des0.msg_id == 0x13
        assert des0.payload == b""

        # 2. 64-byte payload roundtrip
        payload64 = bytes([i % 256 for i in range(64)])
        pkt64 = SerialPacket(msg_id=0x82, payload=payload64, seq=64)
        raw64 = pkt64.serialize()
        assert len(raw64) == 72
        des64, consumed64 = SerialPacket.deserialize(raw64)
        assert consumed64 == 72
        assert des64.payload == payload64

        # 3. SerialPacket constructor rejects >64-byte payloads
        for bad_len in [65, 66, 70, 100, 255, 1024]:
            with pytest.raises(ValueError, match="Payload cannot exceed 64 bytes"):
                SerialPacket(msg_id=0x10, payload=b"X" * bad_len)

        # 4. Deserializer rejects crafted packets with length byte > 64
        for bad_len in range(65, 256):
            crafted = bytearray([0xAA, 0x55, bad_len, 0x01, 0x10])
            crafted.extend(b"\x00" * bad_len)
            crc = compute_crc16_ccitt(crafted[2:])
            crafted.extend(struct.pack("<H", crc))
            crafted.append(0x7D)

            with pytest.raises(ValueError, match="Invalid payload length: .* > 64"):
                SerialPacket.deserialize(bytes(crafted))

    def test_python_oracle_corrupted_tail_byte_exhaustive(self):
        """Verify any tail byte != 0x7D is rejected."""
        pkt = SerialPacket(msg_id=0x11, payload=b"\x01\x02", seq=5)
        raw = bytearray(pkt.serialize())

        for b in range(256):
            if b == 0x7D:
                continue
            raw[-1] = b
            with pytest.raises(ValueError, match="Invalid tail byte"):
                SerialPacket.deserialize(bytes(raw))

    def test_python_oracle_crc16_exhaustive_single_bit_flips(self):
        """Verify 100% error detection for every single-bit flip across frames of varying lengths."""
        for length in [0, 1, 4, 16, 40, 64]:
            payload = bytes([(i * 13 + 7) % 256 for i in range(length)])
            pkt = SerialPacket(msg_id=0x81, payload=payload, seq=17)
            raw = pkt.serialize()

            for byte_idx in range(len(raw)):
                for bit in range(8):
                    corrupt = bytearray(raw)
                    corrupt[byte_idx] ^= (1 << bit)

                    # Bit flip in header or tail triggers specific header/tail error,
                    # bit flip in body or CRC triggers CRC error or length error.
                    # All must raise ValueError!
                    with pytest.raises(ValueError):
                        SerialPacket.deserialize(bytes(corrupt))

    def test_python_oracle_payload_framing_transparency(self):
        """Verify framing handles payloads containing 0xAA, 0x55, 0x7D, 0x00 without false framing."""
        adversarial_payload = bytes([
            0xAA, 0x55, 0x7D, 0xAA, 0x55, 0x7D, 0x00, 0xFF,
            0xAA, 0xAA, 0x55, 0x55, 0x7D, 0x7D, 0x10, 0x80,
            0x00, 0x00, 0xFF, 0xFF, 0x7D, 0xAA, 0x55, 0x00
        ])
        pkt = SerialPacket(msg_id=0x83, payload=adversarial_payload, seq=88)
        raw = pkt.serialize()
        des, consumed = SerialPacket.deserialize(raw)

        assert consumed == len(raw)
        assert des.msg_id == 0x83
        assert des.seq == 88
        assert des.payload == adversarial_payload

    def test_python_oracle_50k_random_byte_fuzzing(self):
        """Fuzz deserializer with 50,000 random byte sequences: must reject 100% without crashing."""
        rng = random.Random(0x1337BEEF)
        num_trials = 50000
        rejected = 0

        for _ in range(num_trials):
            length = rng.randint(0, 128)
            raw = bytes(rng.getrandbits(8) for _ in range(length))
            try:
                SerialPacket.deserialize(raw)
            except ValueError:
                rejected += 1

        assert rejected == num_trials, f"Expected 100% rejection, got {rejected}/{num_trials}"

    def test_python_oracle_stream_sliding_window_recovery(self):
        """Verify stream decoder recovers valid packets when preceded by arbitrary noise bursts."""
        rng = random.Random(42)
        valid_packets = []
        for i in range(50):
            msg_id = random.choice([0x10, 0x11, 0x12, 0x13, 0x14, 0x80, 0x81, 0x82, 0x83, 0x84])
            plen = rng.randint(0, 32)
            payload = bytes(rng.getrandbits(8) for _ in range(plen))
            valid_packets.append(SerialPacket(msg_id=msg_id, payload=payload, seq=i))

        # Assemble stream with random noise bursts between packets
        stream = bytearray()
        for pkt in valid_packets:
            # Noise burst
            noise_len = rng.randint(1, 30)
            stream.extend(bytes(rng.getrandbits(8) for _ in range(noise_len)))
            # Valid packet
            stream.extend(pkt.serialize())

        # Stream parser scanning byte-by-byte
        recovered_packets = []
        offset = 0
        raw_stream = bytes(stream)
        while offset < len(raw_stream):
            # Check for header sync
            sync_idx = raw_stream.find(SerialPacket.HEADER_SYNC, offset)
            if sync_idx == -1:
                break
            try:
                pkt, consumed = SerialPacket.deserialize(raw_stream[sync_idx:])
                recovered_packets.append(pkt)
                offset = sync_idx + consumed
            except ValueError:
                # Corrupted candidate or false sync; advance 1 byte
                offset = sync_idx + 1

        assert len(recovered_packets) == len(valid_packets)
        for original, recovered in zip(valid_packets, recovered_packets):
            assert recovered.msg_id == original.msg_id
            assert recovered.seq == original.seq
            assert recovered.payload == original.payload

    def test_python_oracle_10k_monte_carlo_roundtrip(self):
        """Monte Carlo round-trip test: 10,000 randomized packets across all valid IDs and lengths."""
        rng = random.Random(0xCAFE)
        valid_ids = [0x10, 0x11, 0x12, 0x13, 0x14, 0x80, 0x81, 0x82, 0x83, 0x84]
        num_trials = 10000

        for i in range(num_trials):
            msg_id = rng.choice(valid_ids)
            seq = rng.randint(0, 255)
            plen = rng.randint(0, 64)
            payload = bytes(rng.getrandbits(8) for _ in range(plen))

            pkt = SerialPacket(msg_id=msg_id, payload=payload, seq=seq)
            raw = pkt.serialize()
            des, consumed = SerialPacket.deserialize(raw)

            assert consumed == len(raw)
            assert des.msg_id == msg_id
            assert des.seq == seq
            assert des.payload == payload
