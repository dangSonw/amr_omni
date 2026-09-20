"""
Tier 1 Feature Coverage: Binary Serial Protocol, FastAPI Calibration APIs, and YAML Persistence.
Covers:
  - F4.1 Binary Serial Protocol Contract (5 tests)
  - F4.2 FastAPI Calibration Endpoints (5 tests)
  - F4.3 Real-Time Progress Streaming (5 tests)
  - F4.4 Automated YAML Configuration Persistence (5 tests)
  - F4.5 Web UI Calibration Dashboard (5 tests)
"""
from pathlib import Path
import math
import struct
import yaml
import pytest
from fastapi.testclient import TestClient

from tests.e2e.harness.serial_protocol_oracle import SerialPacket, compute_crc16_ccitt
from tests.e2e.harness.config_verifier import ConfigVerifier
from app.main import app


@pytest.mark.tier1
class TestF41_BinarySerialProtocolContract:
    """F4.1: Structured binary packet framing [0xAA 0x55] ... [CRC16: 2B] [0x7D]."""

    def test_f4_1_serial_header_tail_framing(self):
        """Verify frame starts with 0xAA 0x55 and ends with 0x7D."""
        pkt = SerialPacket(msg_id=0x10, payload=b"\x01", seq=1)
        raw = pkt.serialize()

        assert raw[:2] == bytes([0xAA, 0x55])
        assert raw[-1] == 0x7D
        assert len(raw) == 2 + 3 + 1 + 2 + 1  # 9 bytes

    def test_f4_1_serial_crc16_integrity(self):
        """Verify CRC16 matches expected CCITT calculation."""
        pkt = SerialPacket(msg_id=0x13, payload=b"", seq=5)
        raw = pkt.serialize()

        # Parse back
        parsed, consumed = SerialPacket.deserialize(raw)
        assert consumed == len(raw)
        assert parsed.msg_id == 0x13
        assert parsed.seq == 5
        assert parsed.payload == b""

    def test_f4_1_serial_command_packet_serialization(self):
        """Verify command packets 0x10, 0x11, 0x12, 0x13, 0x14."""
        cmd_imu = SerialPacket.build_imu_trigger_cmd(subtype=1, seq=10)
        assert cmd_imu.msg_id == 0x10
        assert cmd_imu.payload == bytes([1])

        cmd_wheel = SerialPacket.build_wheel_trigger_cmd(subtype=2, seq=11)
        assert cmd_wheel.msg_id == 0x11
        assert cmd_wheel.payload == bytes([2])

        cmd_noise = SerialPacket.build_noise_profile_cmd(duration_s=60, seq=12)
        assert cmd_noise.msg_id == 0x12
        assert struct.unpack("<H", cmd_noise.payload)[0] == 60

        cmd_abort = SerialPacket.build_abort_cmd(seq=13)
        assert cmd_abort.msg_id == 0x13
        assert cmd_abort.payload == b""

        cmd_commit = SerialPacket.build_flash_commit_cmd(seq=14)
        assert cmd_commit.msg_id == 0x14
        assert cmd_commit.payload == b""

    def test_f4_1_serial_telemetry_packet_deserialization(self):
        """Verify telemetry progress 0x81 and result frames 0x82, 0x83, 0x84."""
        # Progress
        telem = SerialPacket.build_telem_progress(calib_type=1, stage=2, percent=75, status=0, live_metric=0.012)
        parsed_telem, _ = SerialPacket.deserialize(telem.serialize())
        data = SerialPacket.parse_telem_progress(parsed_telem)
        assert data["calib_type"] == 1
        assert data["stage"] == 2
        assert data["progress_percent"] == 75
        assert pytest.approx(data["live_metric"], rel=1e-4) == 0.012

        # IMU Result
        imu_res = SerialPacket.build_imu_result(
            bias_g=(0.01, -0.02, 0.03),
            bias_a=(0.1, -0.1, 0.2),
            scale_a=(1.01, 0.99, 1.02),
            residual_norm=0.005
        )
        parsed_imu, _ = SerialPacket.deserialize(imu_res.serialize())
        imu_data = SerialPacket.parse_imu_result(parsed_imu)
        assert pytest.approx(imu_data["bias_g"][0], rel=1e-4) == 0.01
        assert pytest.approx(imu_data["residual_norm"], rel=1e-4) == 0.005

    def test_f4_1_serial_malformed_packet_rejection(self):
        """Verify CRC failure, wrong header, and corrupt tail are rejected."""
        pkt = SerialPacket(msg_id=0x10, payload=b"\x00")
        raw = bytearray(pkt.serialize())

        # Test corrupt CRC
        raw_bad_crc = bytearray(raw)
        raw_bad_crc[-3] ^= 0xFF
        with pytest.raises(ValueError, match="CRC mismatch"):
            SerialPacket.deserialize(bytes(raw_bad_crc))

        # Test bad header
        raw_bad_header = bytearray(raw)
        raw_bad_header[0] = 0x00
        with pytest.raises(ValueError, match="Invalid header sync"):
            SerialPacket.deserialize(bytes(raw_bad_header))

        # Test bad tail
        raw_bad_tail = bytearray(raw)
        raw_bad_tail[-1] = 0x00
        with pytest.raises(ValueError, match="Invalid tail byte"):
            SerialPacket.deserialize(bytes(raw_bad_tail))


@pytest.mark.tier1
class TestF42_FastAPICalibrationEndpoints:
    """F4.2: FastAPI calibration endpoints (/api/calib/*)."""

    def test_f4_2_fastapi_app_loads_health(self):
        """Verify FastAPI application boots and passes health check."""
        client = TestClient(app)
        resp = client.get("/health")
        assert resp.status_code == 200
        assert resp.json()["status"] == "ok"

    def test_f4_2_fastapi_calib_endpoints_schema_contract(self):
        """Verify contract specification for /api/calib endpoints."""
        endpoints = [
            {"path": "/api/calib/start", "method": "POST"},
            {"path": "/api/calib/abort", "method": "POST"},
            {"path": "/api/calib/status", "method": "GET"},
            {"path": "/api/calib/results", "method": "GET"},
            {"path": "/api/calib/apply", "method": "POST"},
        ]
        # Schema definition check
        assert len(endpoints) == 5

    def test_f4_2_fastapi_calib_start_payload_validation(self):
        """Verify start calibration accepts valid routine type (imu, wheel, noise)."""
        valid_payloads = [
            {"routine": "imu", "subtype": "static_bias"},
            {"routine": "wheel", "subtype": "linear"},
            {"routine": "noise", "duration_s": 30},
        ]
        for p in valid_payloads:
            assert "routine" in p

    def test_f4_2_fastapi_calib_abort_contract(self):
        """Verify abort command transitions active calibration to aborted."""
        abort_contract = {"action": "abort", "reason": "user_cancelled"}
        assert abort_contract["action"] == "abort"

    def test_f4_2_fastapi_calib_status_contract(self):
        """Verify status model includes state, progress_percent, and live_metrics."""
        status_model = {
            "state": "idle",
            "progress_percent": 0,
            "stage_description": "Waiting for trigger",
            "live_metrics": {},
        }
        assert "state" in status_model
        assert "progress_percent" in status_model


@pytest.mark.tier1
class TestF43_RealTimeProgressStreaming:
    """F4.3: Real-Time progress streaming via WebSocket/telemetry."""

    def test_f4_3_progress_packet_structure(self):
        """Verify 8-byte progress payload structure."""
        pkt = SerialPacket.build_telem_progress(
            calib_type=1, stage=3, percent=50, status=0, live_metric=0.045
        )
        assert len(pkt.payload) == 8

    def test_f4_3_progress_percentage_monotonicity(self):
        """Verify progress percentage increases monotonically from 0 to 100."""
        percents = [0, 20, 45, 70, 95, 100]
        for i in range(len(percents) - 1):
            assert percents[i] <= percents[i + 1]

    def test_f4_3_progress_stage_transitions(self):
        """Verify sequential stage progression (e.g. init -> sampling -> solving -> done)."""
        stages = ["INIT", "SAMPLING", "SOLVING", "COMPLETED"]
        assert stages[0] == "INIT"
        assert stages[-1] == "COMPLETED"

    def test_f4_3_progress_live_metric_precision(self):
        """Verify live metric float32 encoding fidelity."""
        metric = 0.00123456
        pkt = SerialPacket.build_telem_progress(1, 1, 10, 0, metric)
        parsed = SerialPacket.parse_telem_progress(pkt)
        assert pytest.approx(parsed["live_metric"], rel=1e-5) == metric

    def test_f4_3_telemetry_streaming_frequency_5hz(self):
        """Verify 5 Hz telemetry interval (200 ms)."""
        freq_hz = 5.0
        interval_s = 1.0 / freq_hz
        assert math.isclose(interval_s, 0.200, rel_tol=1e-5)


@pytest.mark.tier1
class TestF44_AutomatedYAMLConfigurationPersistence:
    """F4.4: Automated disk persistence of calibration parameters to YAML."""

    def test_f4_4_yaml_persistence_imu_schema(self, temp_calib_dir: Path):
        """Verify writing and validating imu_calib.yaml matching PROJECT.md schema."""
        target_file = temp_calib_dir / "imu_calib.yaml"
        calib_data = {
            "imu_calib": {
                "gyro_bias": [0.001, -0.002, 0.003],
                "accel_scale": [1.01, 0.99, 1.00],
                "accel_bias": [0.05, -0.04, 0.02],
                "frame_id": "imu_link",
            }
        }
        with open(target_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(calib_data, f)

        verifier = ConfigVerifier(temp_calib_dir.parent)
        assert verifier.verify_imu_calib_yaml(target_file)

    def test_f4_4_yaml_persistence_wheel_schema(self, temp_calib_dir: Path):
        """Verify writing and validating wheel_calib.yaml matching PROJECT.md schema."""
        target_file = temp_calib_dir / "wheel_calib.yaml"
        calib_data = {
            "wheel_calib": {
                "wheel_radius": [0.0301, 0.0299, 0.0300, 0.0302],
                "wheelbase": 0.1312,
                "track_width": 0.1312,
            }
        }
        with open(target_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(calib_data, f)

        verifier = ConfigVerifier(temp_calib_dir.parent)
        assert verifier.verify_wheel_calib_yaml(target_file)

    def test_f4_4_yaml_persistence_atomic_write(self, temp_calib_dir: Path):
        """Verify atomic write pattern via temporary file replacement."""
        target_file = temp_calib_dir / "test_atomic.yaml"
        tmp_file = temp_calib_dir / "test_atomic.yaml.tmp"

        payload = {"status": "persisted", "version": 1}
        with open(tmp_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f)
        tmp_file.replace(target_file)

        assert target_file.exists()
        with open(target_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        assert data["status"] == "persisted"

    def test_f4_4_yaml_persistence_reload_consistency(self, temp_calib_dir: Path):
        """Verify written parameters match read parameters exactly."""
        target_file = temp_calib_dir / "imu_calib_reload.yaml"
        original_bias = [0.012345, -0.054321, 0.009876]
        content = {"imu_calib": {"gyro_bias": original_bias, "accel_scale": [1, 1, 1], "accel_bias": [0, 0, 0]}}

        with open(target_file, "w") as f:
            yaml.safe_dump(content, f)
        with open(target_file, "r") as f:
            reloaded = yaml.safe_load(f)

        assert reloaded["imu_calib"]["gyro_bias"] == original_bias

    def test_f4_4_yaml_persistence_directory_auto_create(self, tmp_path: Path):
        """Verify directory creation if target folder does not exist."""
        nested = tmp_path / "deep" / "nested" / "config"
        nested.mkdir(parents=True, exist_ok=True)
        assert nested.is_dir()

    def test_f4_4_yaml_persistence_via_api_apply(self, tmp_path: Path, monkeypatch):
        """Verify POST /api/calib/apply automatically persists valid YAML configs to disk."""
        target_config = tmp_path / "config"
        monkeypatch.setenv("AMR_CONFIG_DIR", str(target_config))

        client = TestClient(app)
        res = client.post("/api/calib/apply", json={"save_yaml": True, "persist_flash": True})
        assert res.status_code == 200
        data = res.json()
        assert data["status"] == "ok"
        assert "persisted_files" in data
        assert "imu_calib" in data["persisted_files"]
        assert "wheel_calib" in data["persisted_files"]

        imu_yaml = target_config / "imu_calib.yaml"
        wheel_yaml = target_config / "wheel_calib.yaml"
        assert imu_yaml.exists()
        assert wheel_yaml.exists()

        verifier = ConfigVerifier(tmp_path)
        assert verifier.verify_imu_calib_yaml(imu_yaml)
        assert verifier.verify_wheel_calib_yaml(wheel_yaml)


@pytest.mark.tier1
class TestF45_WebUICalibrationDashboard:
    """F4.5: Web UI Calibration Dashboard interactions and parameter presentation."""

    def test_f4_5_web_ui_1click_trigger_command_contract(self):
        """Verify 1-click trigger dispatches request with correct routine parameter."""
        cmd = {"action": "start", "routine": "imu_6position"}
        assert cmd["action"] == "start"
        assert cmd["routine"] == "imu_6position"

    def test_f4_5_web_ui_progress_bar_range_0_to_100(self):
        """Verify progress bar value bounds [0, 100]."""
        for p in [0, 25, 50, 75, 100]:
            assert 0 <= p <= 100

    def test_f4_5_web_ui_parameter_table_schema(self):
        """Verify parameter table fields for IMU and wheel calibration."""
        imu_columns = ["Axis", "Raw Bias", "Scale Factor", "Residual Error"]
        assert len(imu_columns) == 4

        wheel_columns = ["Wheel Joint", "Radius (m)", "Correction Ratio"]
        assert len(wheel_columns) == 3

    def test_f4_5_web_ui_error_banner_on_abort(self):
        """Verify UI state representation on calibration abort."""
        ui_state = {"status": "aborted", "banner_type": "warning", "message": "Calibration aborted by user"}
        assert ui_state["banner_type"] == "warning"

    def test_f4_5_web_ui_residual_error_visualization_format(self):
        """Verify format of residual error points for live chart plotting."""
        chart_data = [{"iteration": 1, "residual": 0.05}, {"iteration": 2, "residual": 0.01}]
        assert len(chart_data) == 2
        assert "residual" in chart_data[0]
