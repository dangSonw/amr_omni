"""
Adversarial Concurrency Stress Suite for Milestone 4:
FastAPI Concurrency, Atomic YAML Write Integrity, and Telemetry Streaming.

Challenger: teamwork_preview_challenger_m4_2
Objectives:
  1. Multi-threaded atomic YAML persistence stress (zero FileNotFoundError, zero 0-byte files).
  2. Concurrent reader-writer consistency (readers never observe empty/corrupt YAML).
  3. FastAPI /api/calib/apply concurrent multi-threaded requests.
  4. Calibration lifecycle stress (abort/reset cycles during active sampling).
  5. WebSocket telemetry streaming concurrency.
"""
import os
import time
import uuid
import yaml
import asyncio
import threading
from pathlib import Path
from typing import List, Dict, Any
import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.services.calib_service import (
    save_imu_calib_yaml,
    save_wheel_calib_yaml,
    persist_calibration_yaml,
)
from tests.e2e.harness.config_verifier import ConfigVerifier


@pytest.mark.stress
class TestFastAPIConcurrencyYAMLStress:
    """Stress testing atomic YAML persistence under high concurrent load."""

    def test_concurrent_imu_yaml_writes_no_corruption_or_missing_tmp(self, tmp_path: Path):
        """
        Adversarially stress save_imu_calib_yaml with 10 concurrent threads (500 total writes).
        VERIFIES:
          - Zero FileNotFoundError (no race condition on .tmp file).
          - Zero 0-byte files.
          - Final file on disk is valid YAML conforming to imu_calib schema.
        """
        config_dir = tmp_path / "config"
        config_dir.mkdir(parents=True, exist_ok=True)

        write_errors: List[str] = []
        writes_completed = 0
        lock = threading.Lock()

        def writer(worker_id: int):
            nonlocal writes_completed
            for i in range(50):
                payload = {
                    "gyro_bias": [worker_id * 0.01, i * 0.001, 0.005],
                    "accel_scale": [1.001, 0.999, 1.002],
                    "accel_bias": [0.01, -0.01, 0.02],
                    "frame_id": f"imu_link_{worker_id}",
                }
                try:
                    p = save_imu_calib_yaml(payload, config_dir=config_dir)
                    assert p.exists()
                    with lock:
                        writes_completed += 1
                except Exception as e:
                    with lock:
                        write_errors.append(f"Worker {worker_id} iter {i}: {type(e).__name__}: {e}")
                time.sleep(0.0002)

        threads = [threading.Thread(target=writer, args=(w,)) for w in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        target_file = config_dir / "imu_calib.yaml"
        assert target_file.exists(), "Target file imu_calib.yaml must exist after writes"
        assert target_file.stat().st_size > 0, "Target file imu_calib.yaml must not be 0 bytes"

        # Check for any writer errors
        assert not write_errors, f"Encountered {len(write_errors)} write errors: {write_errors[:3]}"
        assert writes_completed == 500

        verifier = ConfigVerifier(tmp_path)
        assert verifier.verify_imu_calib_yaml(target_file)

    def test_concurrent_wheel_yaml_writes_no_corruption_or_missing_tmp(self, tmp_path: Path):
        """
        Adversarially stress save_wheel_calib_yaml with 10 concurrent threads (500 total writes).
        VERIFIES:
          - Zero FileNotFoundError.
          - Zero 0-byte files.
          - Final file on disk is valid YAML conforming to wheel_calib schema.
        """
        config_dir = tmp_path / "config"
        config_dir.mkdir(parents=True, exist_ok=True)

        write_errors: List[str] = []
        writes_completed = 0
        lock = threading.Lock()

        def writer(worker_id: int):
            nonlocal writes_completed
            for i in range(50):
                payload = {
                    "wheel_radius": [0.0300 + worker_id * 0.0001, 0.0301, 0.0299, 0.0302],
                    "wheelbase": 0.1312,
                    "track_width": 0.1312,
                }
                try:
                    p = save_wheel_calib_yaml(payload, config_dir=config_dir)
                    assert p.exists()
                    with lock:
                        writes_completed += 1
                except Exception as e:
                    with lock:
                        write_errors.append(f"Worker {worker_id} iter {i}: {type(e).__name__}: {e}")
                time.sleep(0.0002)

        threads = [threading.Thread(target=writer, args=(w,)) for w in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        target_file = config_dir / "wheel_calib.yaml"
        assert target_file.exists(), "Target file wheel_calib.yaml must exist after writes"
        assert target_file.stat().st_size > 0, "Target file wheel_calib.yaml must not be 0 bytes"

        assert not write_errors, f"Encountered {len(write_errors)} wheel write errors: {write_errors[:3]}"
        assert writes_completed == 500

        verifier = ConfigVerifier(tmp_path)
        assert verifier.verify_wheel_calib_yaml(target_file)

    def test_concurrent_readers_never_observe_empty_or_corrupt_yaml(self, tmp_path: Path):
        """
        Adversarially stress concurrent read-during-write:
        Background readers constantly read imu_calib.yaml while writers write.
        VERIFIES:
          - Readers NEVER read a 0-byte file.
          - Readers NEVER receive None from yaml.safe_load.
          - Readers NEVER see incomplete dictionary keys.
        """
        config_dir = tmp_path / "config"
        config_dir.mkdir(parents=True, exist_ok=True)
        target_file = config_dir / "imu_calib.yaml"

        # Seed initial valid file
        save_imu_calib_yaml({"gyro_bias": [0.0, 0.0, 0.0]}, config_dir=config_dir)

        stop_event = threading.Event()
        read_errors: List[str] = []
        zero_byte_observations = 0
        total_reads = 0
        read_lock = threading.Lock()

        def reader():
            nonlocal total_reads, zero_byte_observations
            while not stop_event.is_set():
                if target_file.exists():
                    try:
                        sz = target_file.stat().st_size
                        if sz == 0:
                            with read_lock:
                                zero_byte_observations += 1
                                read_errors.append("Observed 0-byte file")

                        with open(target_file, "r", encoding="utf-8") as f:
                            data = yaml.safe_load(f)

                        with read_lock:
                            total_reads += 1

                        if data is None:
                            with read_lock:
                                read_errors.append("yaml.safe_load returned None (empty file)")
                        elif "imu_calib" not in data or "gyro_bias" not in data["imu_calib"]:
                            with read_lock:
                                read_errors.append(f"Incomplete schema read: {data}")
                    except Exception as e:
                        with read_lock:
                            read_errors.append(f"Reader exception: {type(e).__name__}: {e}")
                time.sleep(0.0001)

        def writer(worker_id: int):
            for i in range(50):
                payload = {
                    "gyro_bias": [worker_id, i, worker_id + i],
                    "accel_scale": [1.0, 1.0, 1.0],
                    "accel_bias": [0.0, 0.0, 0.0],
                    "frame_id": "imu_link",
                }
                try:
                    save_imu_calib_yaml(payload, config_dir=config_dir)
                except Exception:
                    pass
                time.sleep(0.0003)

        r_threads = [threading.Thread(target=reader) for _ in range(5)]
        w_threads = [threading.Thread(target=writer, args=(w,)) for w in range(10)]

        for t in r_threads:
            t.start()
        for t in w_threads:
            t.start()

        for t in w_threads:
            t.join()
        stop_event.set()
        for t in r_threads:
            t.join()

        assert total_reads > 50, f"Expected at least 50 reads during test, got {total_reads}"
        assert zero_byte_observations == 0, f"Observed {zero_byte_observations} zero-byte file reads!"
        assert not read_errors, f"Encountered {len(read_errors)} read errors during concurrent write: {read_errors[:5]}"

    def test_fastapi_apply_endpoint_multi_threaded_concurrency(self, tmp_path: Path, monkeypatch):
        """
        Stress test POST /api/calib/apply from multiple concurrent threads.
        VERIFIES:
          - Zero 500 internal server errors.
          - All returned statuses are 200 OK.
          - Resulting YAML files are strictly valid per ConfigVerifier.
        """
        config_dir = tmp_path / "config"
        monkeypatch.setenv("AMR_CONFIG_DIR", str(config_dir))

        client = TestClient(app)
        api_errors: List[str] = []
        status_codes: List[int] = []
        lock = threading.Lock()

        def request_worker(worker_id: int):
            for _ in range(15):
                try:
                    res = client.post("/api/calib/apply", json={"save_yaml": True, "persist_flash": False})
                    with lock:
                        status_codes.append(res.status_code)
                        if res.status_code != 200:
                            api_errors.append(f"Worker {worker_id}: status {res.status_code}: {res.text}")
                except Exception as e:
                    with lock:
                        api_errors.append(f"Worker {worker_id}: {type(e).__name__}: {e}")
                time.sleep(0.0005)

        threads = [threading.Thread(target=request_worker, args=(w,)) for w in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(status_codes) == 120, f"Expected 120 requests, got {len(status_codes)}"
        assert not api_errors, f"Encountered {len(api_errors)} API errors: {api_errors[:5]}"

        verifier = ConfigVerifier(tmp_path)
        assert verifier.verify_imu_calib_yaml(config_dir / "imu_calib.yaml")
        assert verifier.verify_wheel_calib_yaml(config_dir / "wheel_calib.yaml")

    def test_calib_lifecycle_abort_reset_cycles_under_active_sampling(self):
        """
        Adversarially stress calibration abort and reset cycles while sampling is running.
        VERIFIES:
          - Rapid interleaved calls to /api/calib/start, /abort, /reset, /status
          - Zero unhandled exceptions or state machine hangs
          - Correct state cleanup on reset
        """
        async def run_lifecycle_test():
            import httpx
            transport = httpx.ASGITransport(app=app)
            async with httpx.AsyncClient(transport=transport, base_url="http://testserver") as client:
                errors: List[str] = []

                for cycle in range(25):
                    # 1. Start sampling
                    r_start = await client.post("/api/calib/start", json={"routine": "imu", "sample_count": 200})
                    if r_start.status_code != 200:
                        errors.append(f"Cycle {cycle} start failed: {r_start.status_code}")

                    await asyncio.sleep(0.002)

                    # 2. Status check during sampling
                    r_status = await client.get("/api/calib/status")
                    if r_status.status_code != 200:
                        errors.append(f"Cycle {cycle} status failed: {r_status.status_code}")

                    # 3. Interleave abort vs reset
                    if cycle % 2 == 0:
                        r_abort = await client.post("/api/calib/abort")
                        if r_abort.status_code != 200:
                            errors.append(f"Cycle {cycle} abort failed: {r_abort.status_code}")
                    else:
                        r_reset = await client.post("/api/calib/reset")
                        if r_reset.status_code != 200:
                            errors.append(f"Cycle {cycle} reset failed: {r_reset.status_code}")

                # Final state verification
                r_final = await client.get("/api/calib/status")
                final_state = r_final.json()
                assert final_state["active"] is False, "Active state should be False after abort/reset"
                assert not errors, f"Encountered {len(errors)} lifecycle errors: {errors[:5]}"

        asyncio.run(run_lifecycle_test())
