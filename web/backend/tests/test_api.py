import os
import sys

_ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ws_root not in sys.path:
    sys.path.insert(0, _ws_root)

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
    assert len(data) >= 4
    stream_ids = [s["id"] for s in data]
    assert "jetson_cmd_vel" in stream_ids
    assert "stm32_wheel_odom" in stream_ids
    assert "stm32_imu_data" in stream_ids
    assert "stm32_status" in stream_ids


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


def test_calib_toggle_reset_noise_endpoints():
    # Test toggle
    res_toggle = client.post("/api/calib/toggle", json={"enabled": False})
    assert res_toggle.status_code == 200
    assert res_toggle.json()["calibration_enabled"] is False

    res_toggle_back = client.post("/api/calib/toggle", json={"enabled": True})
    assert res_toggle_back.status_code == 200
    assert res_toggle_back.json()["calibration_enabled"] is True

    # Test noise profile
    res_noise = client.post("/api/calib/noise", json={"profile": "harsh"})
    assert res_noise.status_code == 200
    assert res_noise.json()["noise_profile"] == "harsh"

    # Test reset
    res_reset = client.post("/api/calib/reset")
    assert res_reset.status_code == 200
    assert res_reset.json()["status"] == "ok"
    state = res_reset.json()["state"]
    assert state["is_calibrated"] is False
    assert state["results"]["gyro_bias"] == [0.0, 0.0, 0.0]
    assert state["live_metrics"]["reduction_gyro_percent"] == 0.0


def test_calibration_sampling_and_an4508_faces():
    # 1. Start calibration with 50 samples
    res_start = client.post("/api/calib/start", json={"routine": "imu", "sample_count": 50})
    assert res_start.status_code == 200
    assert res_start.json()["status"] == "ok"
    assert res_start.json()["state"]["target_samples"] == 50
    assert res_start.json()["state"]["active"] is True

    # 2. Step calibration
    res_step = client.post("/api/calib/step")
    assert res_step.status_code == 200
    assert res_step.json()["state"]["progress_percent"] > 0

    # 3. Test AN4508 face sampling
    res_face = client.post("/api/calib/face/sample", json={"face": 4, "sample_count": 50})
    assert res_face.status_code == 200
    assert res_face.json()["face"] == 4
    assert res_face.json()["state"]["face_completed"][4] is True

    # 4. Abort calibration
    res_abort = client.post("/api/calib/abort")
    assert res_abort.status_code == 200
    assert res_abort.json()["state"]["active"] is False


def test_omni_mpc_controller():
    from app.services.grid_planner import OmniMpcController
    mpc = OmniMpcController()
    
    # Test tracking towards a straight path
    path = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]
    vx, vy, wz, dist, rollout = mpc.compute(0.0, 0.0, 0.0, path, 2.0, 0.0, [])
    assert vx > 0.05, "MPC should generate forward velocity along path"
    assert dist == 2.0
    assert len(rollout) > 0

    # Test terminal deadband arrival (< 0.18m)
    vx_arr, vy_arr, wz_arr, dist_arr, rollout_arr = mpc.compute(1.90, 0.0, 0.0, path, 2.0, 0.0, [])
    assert wz_arr == 0.0, "Angular velocity wz must be zero in terminal deadband to prevent oscillation"
    assert vx_arr > 0.0, "Robot should slowly crawl into final position"

    # Test collision avoidance: obstacle directly in front of the robot (< 0.28m)
    obs_front = [[0.26, 0.0]]  # directly ahead at 26cm
    vx_obs, vy_obs, wz_obs, _, _ = mpc.compute(0.0, 0.0, 0.0, path, 2.0, 0.0, obs_front)
    assert vx_obs <= 0.0, "Robot must NEVER drive forward into an obstacle directly in front of it"


def test_arbitrary_pose_calibration():
    # 1. Clear poses
    res_clear = client.post("/api/calib/pose/clear")
    assert res_clear.status_code == 200
    assert res_clear.json()["pose_count"] == 0

    # 2. Record 6 arbitrary poses
    test_poses = [
        {"accel": [0.0, 0.0, 9.80665], "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        {"accel": [0.0, 0.0, -9.80665], "roll": 180.0, "pitch": 0.0, "yaw": 0.0},
        {"accel": [9.80665, 0.0, 0.0], "roll": 0.0, "pitch": -90.0, "yaw": 0.0},
        {"accel": [-9.80665, 0.0, 0.0], "roll": 0.0, "pitch": 90.0, "yaw": 0.0},
        {"accel": [0.0, 9.80665, 0.0], "roll": 90.0, "pitch": 0.0, "yaw": 0.0},
        {"accel": [0.0, -9.80665, 0.0], "roll": -90.0, "pitch": 0.0, "yaw": 0.0},
    ]
    for p in test_poses:
        res = client.post("/api/calib/pose/record", json=p)
        assert res.status_code == 200

    # 3. Compute calibration
    res_compute = client.post("/api/calib/pose/compute")
    assert res_compute.status_code == 200
    data = res_compute.json()
    assert data["status"] == "ok"
    assert len(data["accel_scale"]) == 3
    assert len(data["accel_bias"]) == 3
    assert data["max_norm_error"] < 0.5



def test_encoder_and_extrinsics_calibration():
    # 1. Encoder calibration test
    res_enc = client.post("/api/calib/encoder/start", json={
        "true_distance_m": 1.0,
        "wheel_travel_m": [1.02, 0.99, 1.01, 0.98]
    })
    assert res_enc.status_code == 200
    enc_data = res_enc.json()
    assert enc_data["status"] == "ok"
    assert len(enc_data["wheel_radii"]) == 4
    for r in enc_data["wheel_radii"]:
        assert 0.025 <= r <= 0.035

    # 2. Extrinsics calibration test with temporal latency
    res_ext = client.post("/api/calib/extrinsics/start", json={
        "w1_rad_s": 1.0,
        "w2_rad_s": 2.0,
        "ax_1": -0.05,
        "ay_1": 0.0,
        "ax_2": -0.20,
        "ay_2": 0.0,
        "time_delay_ms": 15.0,
    })
    assert res_ext.status_code == 200
    ext_data = res_ext.json()
    assert ext_data["status"] == "ok"
    assert len(ext_data["lever_arm"]) == 3
    assert abs(ext_data["lever_arm"][0] - 0.05) < 0.01
    assert ext_data["time_delay_ms"] == 15.0


def test_st_an4508_full_6_steps_calibration():
    # Reset first
    client.post("/api/calib/reset")

    # Step through all 6 AN4508 orientations (0 to 5)
    for s in range(6):
        res = client.post("/api/calib/face/sample", json={"step": s, "sample_count": 50})
        assert res.status_code == 200
        data = res.json()
        assert data["completed"] is True

    # Check that after step 5, all 6 faces are completed and calibration succeeded
    res_status = client.get("/api/calib/status")
    assert res_status.status_code == 200
    status = res_status.json()
    assert status["is_calibrated"] is True
    assert all(status["face_completed"])

    # Verify physical sanity bounds
    scales = status["results"]["accel_scale"]
    biases = status["results"]["accel_bias"]
    for s in scales:
        assert 0.85 <= s <= 1.15, f"Scale {s} must be within physical sanity [0.85, 1.15]"
    for b in biases:
        assert abs(b) <= 2.0, f"Bias {b} must be within physical bounds [-2.0, 2.0]"

    # Verify that calibrated gravity is physically ~9.8 m/s^2, never 20 m/s^2!
    calib_az = (9.80665 - biases[2]) / scales[2]
    assert 9.0 <= calib_az <= 10.5, f"Calibrated az is {calib_az}, must be ~9.8 m/s^2 and never 20 m/s^2!"


def test_sim_set_pose_and_tilt_compensation():
    # 1. Test set_sim_pose endpoint
    res_pose = client.post("/api/calib/sim/set_pose", json={
        "roll_deg": 180.0,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
        "step": 1,
    })
    assert res_pose.status_code == 200
    data = res_pose.json()
    assert data["status"] == "ok"
    assert data["angles"]["roll"] == 180.0

    # 2. Check calib status has sim_angles
    res_status = client.get("/api/calib/status")
    assert res_status.status_code == 200
    status = res_status.json()
    assert status["sim_angles"] == [180.0, 0.0, 0.0]
    assert len(status["face_cos_deltas"]) == 6


def test_calib_apply_persists_yaml_and_config_verifier(tmp_path, monkeypatch):
    from tests.e2e.harness.config_verifier import ConfigVerifier

    target_config_dir = tmp_path / "config"
    monkeypatch.setenv("AMR_CONFIG_DIR", str(target_config_dir))

    res = client.post("/api/calib/apply", json={"save_yaml": True, "persist_flash": True})
    assert res.status_code == 200
    data = res.json()
    assert data["status"] == "ok"
    assert "persisted_files" in data
    assert "imu_calib" in data["persisted_files"]
    assert "wheel_calib" in data["persisted_files"]

    imu_yaml = target_config_dir / "imu_calib.yaml"
    wheel_yaml = target_config_dir / "wheel_calib.yaml"
    assert imu_yaml.exists()
    assert wheel_yaml.exists()

    verifier = ConfigVerifier(tmp_path)
    assert verifier.verify_imu_calib_yaml(imu_yaml)
    assert verifier.verify_wheel_calib_yaml(wheel_yaml)







