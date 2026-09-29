#!/usr/bin/env python3
import argparse
import os
import sys
import uvicorn


def main():
    parser = argparse.ArgumentParser(description="Chạy AMR Omni Web Backend")
    parser.add_argument("--host", default="0.0.0.0", help="Địa chỉ host (mặc định: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="Cổng lắng nghe (mặc định: 8000)")
    parser.add_argument(
        "--mode",
        choices=["auto", "ros2", "ros1"],
        default="auto",
        help="Chế độ kết nối robot: auto, ros2, ros1 (mặc định: auto)",
    )
    parser.add_argument("--reload", action="store_true", help="Bật auto-reload khi phát triển")
    args = parser.parse_args()

    # Thiết lập biến môi trường
    os.environ["ROBOT_BRIDGE_MODE"] = args.mode

    # Thêm đường dẫn vào sys.path
    backend_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(backend_dir))

    if backend_dir not in sys.path:
        sys.path.insert(0, backend_dir)

    # Tự động phát hiện ROS 2 distro nếu chưa có trong PYTHONPATH
    ros_distro = os.getenv("ROS_DISTRO", "jazzy")
    potential_ros_paths = [
        f"/opt/ros/{ros_distro}/lib/python3.12/site-packages",
        f"/opt/ros/{ros_distro}/local/lib/python3.12/dist-packages",
        f"/opt/ros/{ros_distro}/lib/python3.10/site-packages",
        f"/opt/ros/{ros_distro}/local/lib/python3.10/dist-packages",
        os.path.join(workspace_dir, "install", "ros2_jazzy", "lib", "python3.12", "site-packages"),
        os.path.join(workspace_dir, "src"),
    ]
    for p in potential_ros_paths:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.append(p)

    print(f"[AMR Web Backend] Listening on http://{args.host}:{args.port} (mode: {args.mode})")

    uvicorn.run("app.main:app", host=args.host, port=args.port, reload=args.reload, log_level="warning", access_log=False)


if __name__ == "__main__":
    main()

