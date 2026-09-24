# AMR Omni — Autonomous Mobile Robot Platform

Hệ thống phần mềm toàn diện cho robot tự hành 4 bánh Omni/Mecanum công nghiệp: từ mô phỏng trên ROS 2 Jazzy & Gazebo Harmonic, firmware STM32 (FreeRTOS + micro-ROS), đến giao diện điều khiển và giám sát Web Mission Control hiện đại (FastAPI + Next.js).

---

## 1. Kiến Trúc Luồng Dữ Liệu 4 Lớp (Data Flow Architecture)

Hệ thống được thiết kế theo mô hình phân tầng chặt chẽ, đảm bảo tính mô-đun, an toàn phản xạ và độ trễ thấp giữa các thành phần tính toán cấp cao (Jetson/Host) và bộ điều khiển cấp thấp (STM32 MCU):

![Kiến trúc luồng dữ liệu AMR Omni](example/data_flow_architecture.png)

### Chi tiết 4 phân tầng kiến trúc:

| Phân tầng | Thành phần chính | Luồng dữ liệu & Vai trò |
|---|---|---|
| **1. Giao diện & Giám sát** *(Web UI / Teleop)* | • Web Frontend (Next.js 14, React)<br>• Web Backend (FastAPI, Uvicorn)<br>• Teleop Controller | • Giao tiếp song công WebSocket 20Hz truyền nhận telemetry và lệnh điều khiển.<br>• Nhận lệnh bàn phím WASD / Space (E-Stop), phát `/cmd_vel` tới bộ điều hướng.<br>• Giám sát thời gian thực `/odometry/filtered`, `/map`, `/scan`, `/debug/data`. |
| **2. Điều hướng & Kiểm soát** *(Navigation & Control)* | • Nav2 Controller<br>• Velocity Smoother<br>• Command Watchdog | • Nav2 tính toán quỹ đạo dựa trên `/map` và `/odometry/filtered`, phát `cmd_vel_nav`.<br>• **Velocity Smoother** lọc gia tốc mượt mà, triệt tiêu giật cục cho cơ cấu omni.<br>• **Command Watchdog** giám sát mất tín hiệu, kết hợp tín hiệu an toàn từ Safety Zone để xuất `safe_cmd_vel`. |
| **3. Cảm biến & Định vị** *(Perception & SLAM)* | • 2D LiDAR & Laser Filter<br>• Safety Zone Node<br>• SLAM Toolbox (Loop Closure)<br>• Robot Localization (EKF) | • LiDAR phát `/scan_raw` qua **Laser Filter** lọc nhiễu trước khi vào các node chức năng.<br>• **Safety Zone Node** tính toán vùng an toàn, phát tín hiệu giảm tốc hoặc dừng khẩn cấp.<br>• **SLAM Toolbox** dựng bản đồ với thuật toán khép vòng (Loop Closure) tối ưu cho không gian lớn.<br>• **EKF Node** hợp nhất quán tính IMU (`/imu/data`) và odometry bánh xe (`/wheel/odom`) xuất `/odometry/filtered`. |
| **4. Phần cứng & Mô phỏng** *(Hardware / Simulator)* | • Stm32Bridge Node<br>• STM32 MCU (FreeRTOS)<br>• Gazebo Harmonic Simulator | • `Stm32Bridge` chuyển đổi `safe_cmd_vel` thành khung truyền UART `stm32_cmd_vel`.<br>• **STM32 MCU** thực thi vòng lặp PID kín 100Hz, bộ lọc Kalman bánh xe và xuất `/debug/data`.<br>• **Gazebo Sensors** mô phỏng chân thực LiDAR, IMU và động học xe trên các bề mặt dốc và gồ ghề. |

---

## 2. Thư Viện Hình Ảnh Minh Họa (Visual Showcase)

Thư mục [`example/`](example/) lưu trữ toàn bộ sơ đồ kiến trúc và ảnh chụp thực tế từng tính năng của hệ thống giao diện Web Mission Control:

### 2.1. Tab [01] — Cockpit & Bản Đồ Điều Hướng 2D
Giao diện buồng lái điều khiển tích hợp bản đồ CAD tối giản, đa giác quét LiDAR trực quan, vệt đường plan nét đứt vi mô, tâm ngắm mục tiêu tinh xảo, luồng camera thời gian thực và thẻ đo xa telemetry.
![Cockpit Tab](example/01_tab_cockpit.png)

### 2.2. Tab [02] — Cấu Hình Tham Số & Bộ Điều Khiển PID
Hiệu chỉnh trực tiếp các thông số động học robot (bán kính bánh xe, khoảng cách trục, giới hạn vận tốc), hệ số $K_p, K_i, K_d$ vòng kín của 4 động cơ và ma trận hiệp phương sai bộ lọc Kalman mà không cần nạp lại mã nguồn.
![Config Tab](example/02_tab_config.png)

### 2.3. Tab [03] — Giám Sát Cảm Biến & Đồ Thị Đo Xa
Đồ thị thời gian thực theo dõi cảm biến IMU 9 trục BNO080 (Gia tốc, Con quay hồi chuyển, Từ trường), biểu diễn Quaternion hướng không gian 3D, và 4 biểu đồ phân tích vận tốc bánh xe (Raw vs Filtered vs Target).
![Monitor Tab](example/03_tab_monitor.png)

### 2.4. Kiểm Toán Luồng Truyền Thông (Stream Audit Matrix)
Bảng giám sát chi tiết 22 ROS topics trong hệ thống với tần số thực tế (Hz), độ trễ (Latency), số lượng gói tin và trạng thái kết nối (Active / Degraded / Idle) giữa Jetson, STM32 và Web.
![Stream Matrix](example/04_monitor_stream_matrix.png)

---

## 3. Cấu Trúc Thư Mục Dự Án

```text
amr_omni/
├── example/                          # Sơ đồ luồng dữ liệu và ảnh chụp giao diện hệ thống
│   ├── data_flow_architecture.png   # Sơ đồ kiến trúc luồng dữ liệu 4 lớp
│   ├── 01_tab_cockpit.png           # Giao diện Cockpit & Bản đồ 2D
│   ├── 02_tab_config.png            # Giao diện Cấu hình tham số & PID
│   ├── 03_tab_monitor.png           # Giao diện Giám sát IMU & Vận tốc động cơ
│   └── 04_monitor_stream_matrix.png # Bảng kiểm toán 22 topics thời gian thực
├── firmware/                         # Mã nguồn STM32 PlatformIO (FreeRTOS + micro-ROS)
│   ├── stm32_f407vg_arduino_sim/    # Source hoàn chỉnh cho STM32F407VG Discovery
│   └── stm32_f407vg_stm3cube_sim/   # Scaffold cho STM32CubeIDE
├── scripts/                          # Bộ 8 kịch bản tự động hóa vận hành hệ thống
│   ├── setup.sh                     # Kiểm tra môi trường host, cài đặt công cụ
│   ├── build.sh                     # Biên dịch và chạy test ROS 2, Firmware, Web
│   ├── run_sim.sh                   # Khởi chạy mô phỏng Gazebo Harmonic & Renode
│   ├── run_robot.sh                 # Khởi chạy phần cứng thực tế trên Jetson Nano
│   ├── flash_mcu.sh                 # Nạp firmware an toàn cho STM32
│   ├── run_web.sh                   # Chạy giao diện Web (FastAPI + Next.js)
│   ├── run_bridge.sh                # Cầu nối ROS 1 (Jetson) <-> ROS 2 (Host)
│   └── install.sh                   # Cài đặt Udev rules và Systemd autostart
├── src/                              # Các ROS 2 Packages (Python / C++)
│   ├── omni_bringup/                # Launch files tổng hợp hệ thống
│   ├── omni_control/                # Động học Mecanum/Omni 4 bánh & Bộ điều khiển
│   ├── omni_description/            # Mô hình robot Xacro / URDF và Mesh 3D
│   ├── omni_hardware/               # Giao tiếp phần cứng Stm32Bridge & Protocol
│   ├── omni_localization/           # Cấu hình SLAM Toolbox (Loop Closure) & EKF
│   ├── omni_navigation/             # Cấu hình Nav2 stack, costmaps, planners
│   ├── omni_perception/             # Bộ lọc LaserFilter và Safety Zone
│   ├── omni_safety/                 # Watchdog bảo vệ, E-Stop và giới hạn vận tốc
│   └── omni_simulation/             # Thế giới Gazebo amr_lab.sdf (dốc 5°-14°, gờ giảm tốc)
└── web/                              # Giao diện Web Dashboard thời gian thực
    ├── backend/                     # FastAPI backend, ROS 2 rclpy bridge, WebSocket hub
    └── frontend/                    # Next.js 14, Tailwind CSS, Recharts, HTML5 Canvas
```

---

## 4. Bộ Script Vận Hành (Core Scripts)

Tất cả các script đều hỗ trợ cờ `-h` hoặc `--help` và có thể thực thi từ bất kỳ thư mục con nào:

| Script | Vai trò chính | Lệnh thông dụng |
|---|---|---|
| [`scripts/setup.sh`](scripts/setup.sh) | Kiểm tra môi trường host, cài đặt PlatformIO và công cụ nạp | `./scripts/setup.sh --check` |
| [`scripts/build.sh`](scripts/build.sh) | Biên dịch & kiểm thử toàn diện ROS 2, Firmware STM32, Web | `./scripts/build.sh --component all --test` |
| [`scripts/run_sim.sh`](scripts/run_sim.sh) | Khởi chạy Gazebo Harmonic (GUI hoặc Headless) | `./scripts/run_sim.sh` |
| [`scripts/run_robot.sh`](scripts/run_robot.sh) | Khởi chạy hardware bringup trên robot thực tế (Jetson) | `./scripts/run_robot.sh` |
| [`scripts/flash_mcu.sh`](scripts/flash_mcu.sh) | Nạp firmware an toàn cho vi điều khiển STM32 | `./scripts/flash_mcu.sh --mcu stm32f4 --yes` |
| [`scripts/run_web.sh`](scripts/run_web.sh) | Khởi động Web Cockpit (Backend FastAPI + Frontend Next.js) | `./scripts/run_web.sh` |
| [`scripts/run_bridge.sh`](scripts/run_bridge.sh) | Kích hoạt cầu nối giao tiếp ROS 1 $\leftrightarrow$ ROS 2 | `./scripts/run_bridge.sh` |
| [`scripts/install.sh`](scripts/install.sh) | Cài đặt Udev rules nhận diện cổng USB và Systemd service | `./scripts/install.sh all` |

---

## 5. Hướng Dẫn Cài Đặt & Khởi Động Nhanh (Quickstart)

### 5.1. Kiểm tra môi trường
Hệ thống được chuẩn hóa trên môi trường **Ubuntu 24.04 LTS (hoặc WSL2)** với **ROS 2 Jazzy Jalisco**:

```bash
# Kiểm tra sự sẵn sàng của hệ điều hành, ROS 2 và các công cụ
./scripts/setup.sh --check
```

### 5.2. Biên dịch toàn bộ dự án
```bash
# Biên dịch ROS 2 packages, firmware và xuất bản tĩnh Web
./scripts/build.sh --component all --test --skip-empty

# Nạp môi trường ROS 2 sau khi build
source install/ros2_jazzy/local_setup.bash
```

### 5.3. Khởi chạy mô phỏng Gazebo
Môi trường `amr_lab.sdf` tích hợp sẵn 3 loại dốc (5°, 10°, 14°), dải gờ giảm tốc thử thách hệ thống treo/IMU và các vật cản hẹp:

```bash
# Khởi chạy mô phỏng có giao diện 3D
./scripts/run_sim.sh

# Hoặc khởi chạy chế độ headless (cho máy chủ / CI không có màn hình)
./scripts/run_sim.sh --headless --duration 60
```

### 5.4. Khởi chạy Web Mission Control
Khởi động backend FastAPI và mở giao diện trình duyệt điều khiển:

```bash
# Chạy Web Dashboard
./scripts/run_web.sh
```
Truy cập trình duyệt tại địa chỉ: `http://localhost:8000`

---

## 6. Tính Năng Kỹ Thuật Nổi Bật

### 6.1. Thuật toán Loop Closure diện rộng (SLAM Toolbox)
File cấu hình [`src/omni_localization/config/slam_params.yaml`](src/omni_localization/config/slam_params.yaml) được tinh chỉnh chuyên sâu cho không gian nhà kho/xưởng lớn:
- Bộ giải tối ưu đồ thị `CeresSolver` với `SPARSE_NORMAL_CHOLESKY` và điều hòa `SCHUR_JACOBI`.
- Hàm mất mát `HuberLoss` chống méo mó khi có nhiễu liên kết.
- Không gian tìm kiếm vòng lặp mở rộng 10.0m với bộ nhớ stack 60MB.

### 6.2. An toàn vận hành đa lớp & Cơ chế E-Stop
- **Command Watchdog:** Tự động phát lệnh vận tốc 0 khi mất kết nối điều khiển quá 250ms.
- **Safety Zone:** Quét chướng ngại vật trong phạm vi nguy hiểm từ LiDAR để giảm tốc hoặc kích hoạt phanh dừng trước khi xảy ra va chạm.
- **Phím Space E-Stop:** Nhấn phím Space trên Web Cockpit lập tức hủy toàn bộ mục tiêu tự hành (`active_goal = None`), reset bộ điều khiển và phát chuỗi 5 xung dừng đa kênh tới cả Jetson và STM32.

### 6.3. Firmware STM32 FreeRTOS & micro-ROS
- Vòng lặp điều khiển PID vận tốc 4 bánh xe chu kỳ 10ms (100Hz) chống hiện tượng *derivative kick*.
- Bộ lọc Kalman 1D trên từng kênh encoder bảo vệ chống đột biến vận tốc và giá trị NaN.
- Khóa trường dữ liệu độc lập bảo vệ chống hiện tượng race condition giữa các tác vụ FreeRTOS.

---

## 7. Kiểm Thử Hệ Thống (QA & Testing)

Dự án duy trì bộ kiểm thử tự động toàn diện bao quát tất cả các tầng:

```bash
# 1. Chạy Unit Test động học và mô phỏng
python3 -m unittest src/omni_simulation/test/test_simulation_files.py

# 2. Chạy Kiểm thử Bộ lọc Cảm biến & Web Backend API
web/backend/.venv/bin/pytest src/omni_perception/test/test_perception_files.py web/backend/tests/test_api.py web/backend/tests/test_grid_planner.py
```
*Kết quả xác minh:* **45/45 tests PASS (100%)**.

---

## 8. Quy Tắc Phát Triển & Công Cụ Hỗ Trợ AI Agent

- **Quy chuẩn phát triển:** Bắt buộc tuân thủ tài liệu hướng dẫn [`RULE.md`](RULE.md) và [`AGENTS.md`](AGENTS.md).
- **GitNexus:** Bắt buộc chạy phân tích tác động `node .gitnexus/run.cjs impact` trước khi sửa hàm/class và chạy `node .gitnexus/run.cjs detect_changes` trước khi commit mã nguồn.
- **MCP Servers (`.vscode/mcp.json`):** Hệ thống tích hợp sẵn các máy chủ công cụ `chrome-devtools` (kiểm thử giao diện tự động), `next-ai-drawio` (vẽ sơ đồ), `context7` (tra cứu tài liệu), `ruflo` (điều phối đàn agent).