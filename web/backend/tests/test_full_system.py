from fastapi.testclient import TestClient
from app.main import app


def test_full_system_integration():
    with TestClient(app) as client:
        # 1. Health check
        res = client.get("/health")
        assert res.status_code == 200
        assert res.json()["status"] == "ok"

        # 2. Frontend HTML
        res = client.get("/")
        assert res.status_code == 200
        assert "AMR Omni" in res.text

        # 3. Status
        res = client.get("/api/status")
        assert res.status_code == 200
        status_data = res.json()
        assert status_data["connected"] is True
        assert status_data["mode"] in ("ros2", "mock")

        # 4. Stream matrix (Jetson <-> STM32 <-> Sensors)
        res = client.get("/api/streams")
        assert res.status_code == 200
        streams = res.json()
        stream_ids = {s["id"] for s in streams}
        assert "jetson_cmd_vel" in stream_ids
        assert "stm32_wheel_state" in stream_ids
        assert "stm32_encoder_counts" in stream_ids
        assert "stm32_imu" in stream_ids
        assert "sensor_lidar" in stream_ids
        assert "localization_odom" in stream_ids

        # 5. Send velocity command
        res = client.post("/api/cmd-vel", json={"linear_x": 0.25, "linear_y": 0.1, "angular_z": 0.5})
        assert res.status_code == 200

        # 6. WebSocket real-time telemetry stream test
        with client.websocket_connect("/ws/telemetry") as ws:
            # Nhận 1 gói tin telemetry
            data = ws.receive_json()
            assert data["type"] == "telemetry"
            assert "status" in data
            assert "wheels" in data
            assert "imu" in data
            assert "odom" in data
            assert "lidar" in data
            assert "streams" in data

            # Kiểm tra dữ liệu bánh xe (4 bánh Mecanum)
            wheels = data["wheels"]
            assert len(wheels["target_rad_s"]) == 4
            assert len(wheels["measured_rad_s"]) == 4
            assert len(wheels["encoder_ticks"]) == 4

            # Kiểm tra dữ liệu LiDAR
            lidar = data["lidar"]
            assert len(lidar["ranges"]) in (0, 360)
            assert isinstance(lidar["points"], list)

            # Gửi lệnh cmd_vel qua WebSocket
            ws.send_json({"type": "cmd_vel", "vx": 0.3, "vy": 0.0, "wz": 0.0})

            # Gửi lệnh estop qua WebSocket
            ws.send_json({"type": "estop", "active": True})

            # Nhận gói tin tiếp theo xác nhận estop
            next_data = ws.receive_json()
            assert next_data["type"] == "telemetry"

            # Khôi phục estop
            ws.send_json({"type": "estop", "active": False})
