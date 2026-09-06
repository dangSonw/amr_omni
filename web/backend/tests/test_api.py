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



def test_get_path():
    response = client.get("/api/path")
    assert response.status_code == 200
    data = response.json()
    assert "global_path" in data
    assert "local_path" in data


def test_get_map():
    response = client.get("/api/map")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data


def test_dispatch_nav_goal():
    response = client.post("/api/nav/goal", json={"x": 1.5, "y": 2.0, "theta_rad": 0.785})
    assert response.status_code == 200
    data = response.json()
    assert "dispatched" in data


def test_dispatch_waypoints():
    waypoints = [
        {"x": 1.0, "y": 0.0, "theta_rad": 0.0},
        {"x": 2.0, "y": 1.0, "theta_rad": 1.57},
    ]
    response = client.post("/api/nav/waypoints", json={"waypoints": waypoints, "loop": False})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "ok"
    assert data["count"] == 2


def test_save_map():
    response = client.post("/api/map/save", json={"map_name": "test_map"})
    assert response.status_code == 200
    data = response.json()
    assert "status" in data


def test_nav_cancel_and_status():
    res_status = client.get("/api/nav/status")
    assert res_status.status_code == 200
    assert "navigation" in res_status.json()

    res_cancel = client.post("/api/nav/cancel")
    assert res_cancel.status_code == 200
    assert res_cancel.json()["status"] == "ok"


def test_nav_goal_and_path_telemetry():
    # Dispatch a navigation goal
    res_goal = client.post("/api/nav/goal", json={"x": 2.0, "y": 1.5, "theta_rad": 0.0})
    assert res_goal.status_code == 200
    assert res_goal.json()["status"] == "ok"

    # Verify path endpoint returns paths
    res_path = client.get("/api/path")
    assert res_path.status_code == 200
    path_data = res_path.json()
    assert "global_path" in path_data
    assert "local_path" in path_data


def test_obstacle_avoidance_computation():
    from app.bridges.ros2_bridge import Ros2Bridge
    bridge = Ros2Bridge()
    bridge.odom_x = 0.0
    bridge.odom_y = 0.0
    bridge.odom_theta = 0.0

    # Test 1: Goal is to the left (+y) -> Robot should prioritize turning left (wz > 0)
    vr_x, vr_y, wz, dist, local_path = bridge._compute_nav_velocities(0.0, 2.0)
    assert wz > 0.5, "Robot must prioritize turning left towards the goal"
    assert len(local_path) == 3

    # Test 2: Obstacle right in front at (0.4, 0.0) -> Repulsion should activate
    bridge.lidar_points = [[0.4, 0.0], [0.45, 0.05], [0.38, -0.05]]
    vr_x, vr_y, wz_obs, dist, local_path_obs = bridge._compute_nav_velocities(2.0, 0.0)
    # The avoidance vector should steer away and bend the path
    assert abs(wz_obs) > 0.1 or abs(vr_y) > 0.02
    assert len(local_path_obs) == 3


def test_final_approach_deadband():
    """Kiểm tra: Khi robot ở cự ly < 0.25m so với đích, wz PHẢI BẰNG 0.0 để triệt tiêu rung lắc / quay vòng."""
    from app.bridges.ros2_bridge import Ros2Bridge
    bridge = Ros2Bridge()
    bridge.odom_x = 1.0
    bridge.odom_y = 1.0
    bridge.odom_theta = 0.5

    # Goal is just 15cm away (0.15m) -> Within deadband (< 0.25m)
    gx, gy = 1.10, 1.10
    vr_x, vr_y, wz, dist, local_path = bridge._compute_nav_velocities(
        tx=gx, ty=gy, gx=gx, gy=gy, dist_to_final=0.15
    )
    assert wz == 0.0, "Vận tốc góc wz phải bằng 0.0 trong vùng đệm tiếp cận đích để chống rung lắc!"
    assert vr_x != 0.0 or vr_y != 0.0, "Xe vẫn phải trôi nhẹ vào đích"


def test_map_clear_and_grid_planner():
    """Kiểm tra API xóa map và thuật toán GridMapPlanner né tường."""
    res_clear = client.post("/api/map/clear")
    assert res_clear.status_code == 200
    assert res_clear.json()["status"] == "ok"

    from app.services.grid_planner import GridMapPlanner
    planner = GridMapPlanner()

    # Dựng một bức tường tại x = 1.0m
    wall = [[1.0, y * 0.1] for y in range(-8, 9)]
    new_cells = planner.add_scan(0.0, 0.0, 0.0, wall)
    assert new_cells > 0
    assert len(planner.occupied_cells) > 0

    # Lập lộ trình từ (0, 0) đến (2.0, 0.0) qua bức tường
    path = planner.plan_path(0.0, 0.0, 2.0, 0.0)
    assert len(path) > 2
    # Đảm bảo đường đi không cắt thẳng xuyên qua tâm bức tường
    for pt in path:
        assert not (abs(pt[0] - 1.0) < 0.15 and abs(pt[1]) < 0.2), f"Điểm {pt} lọt vào giữa tường!"


