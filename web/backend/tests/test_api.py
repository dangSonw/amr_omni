from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)


def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok", "service": "amr_omni_backend"}


def test_get_status():
    response = client.get("/api/status")
    assert response.status_code == 200
    data = response.json()
    assert "mode" in data
    assert "connected" in data
    assert "uptime_sec" in data


def test_get_config():
    response = client.get("/api/config")
    assert response.status_code == 200
    data = response.json()
    assert data["wheel_radius_m"] == 0.03
    assert data["max_linear_speed_mps"] == 0.54


def test_get_streams():
    response = client.get("/api/streams")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    assert len(data) >= 8
    stream_ids = [s["id"] for s in data]
    assert "jetson_cmd_vel" in stream_ids
    assert "stm32_wheel_state" in stream_ids
    assert "sensor_lidar" in stream_ids


def test_send_cmd_vel():
    response = client.post("/api/cmd-vel", json={"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.1})
    assert response.status_code == 200
    assert response.json()["status"] == "ok"


def test_toggle_estop():
    response = client.post("/api/estop", json={"active": True})
    assert response.status_code == 200
    assert response.json()["estop_active"] is True

    # Khôi phục
    response = client.post("/api/estop", json={"active": False})
    assert response.status_code == 200
    assert response.json()["estop_active"] is False
