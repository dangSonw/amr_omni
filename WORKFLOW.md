# WORKFLOW — Project Review & Implementation Plan

> **Ngày review:** 2026-09-06  
> **Phương pháp:** Đọc toàn bộ source code, firmware, web, scripts, README, RULE.md. Phân tích song song bằng 4 luồng chuyên sâu.

---

## Mục lục

1. [Tổng quan phát hiện](#1-tổng-quan-phát-hiện)
2. [#1 — Script vs README](#2-1--script-vs-readme)
3. [#2 — Code/biến không đúng](#3-2--codebiến-không-đúng)
4. [#3 — Thư viện thừa, code chưa tối ưu](#4-3--thư-viện-thừa-code-chưa-tối-ưu)
5. [#4 — Logic chưa đúng hoặc chưa tối ưu](#5-4--logic-chưa-đúng-hoặc-chưa-tối-ưu)
6. [#5 — Firmware Kalman & PID](#6-5--firmware-kalman--pid)
7. [#6 — Web Debug Data Visualization](#7-6--web-debug-data-visualization)
8. [#7 — UI chuyên nghiệp hơn](#8-7--ui-chuyên-nghiệp-hơn)
9. [Plan chi tiết](#9-plan-chi-tiết)

---

## 1. Tổng quan phát hiện

| Mức độ | Số lượng | Phân bố |
|--------|----------|---------|
| 🔴 CRITICAL | 5 | Firmware (3), ROS2 Simulation (2) |
| 🟠 HIGH | 8 | Scripts (2), Web (3), Firmware (1), ROS2 (2) |
| 🟡 MEDIUM | 12 | Scripts (4), Web (3), Firmware (3), ROS2 (2) |
| 🔵 LOW | 6 | Scripts (2), Web (2), Firmware (2) |

---

## 2. #1 — Script vs README

### 2.1 Scripts không được ghi trong README

7 script tồn tại nhưng hoàn toàn không được đề cập trong README.md:

| Script | Chức năng |
|--------|-----------|
| run_web.sh | Khởi động web frontend + backend |
| run_bridge.sh | ROS1-ROS2 bridge |
| calibrate.sh | Calibration robot |
| diagnostics.sh | Kiểm tra hệ thống |
| bag.sh | Ghi/phát rosbag |
| save_map.sh | Lưu bản đồ SLAM |
| install.sh | Cài systemd service |
| lib/common.sh | Shared library functions |

### 2.2 Bug trong install.sh

**CRITICAL:** install.sh dòng 123 — ExecStart gọi run_robot.sh không có argument. Nhưng run_robot.sh bắt buộc --workspace, --package, --launch → systemd service sẽ crash ngay khi start.

**Fix:** Thêm tham số vào template systemd hoặc yêu cầu user truyền config file.

### 2.3 Hardcoded IP trong run_bridge.sh

run_bridge.sh dòng 13 — Hardcode 192.168.1.100 cho ROS_MASTER_URI. Vi phạm RULE.md mục 4 (không hard-code IP cá nhân).

**Fix:** Đổi default sang localhost hoặc yêu cầu truyền tham số.

### 2.4 Division by zero trong calibrate.sh

calibrate.sh dòng 89, 97 — Tính d / s mà không kiểm tra TEST_SPEED == 0.

---

## 3. #2 — Code/biến không đúng

### 3.1 Firmware — Biến/struct không dùng

| File | Vị trí | Vấn đề |
|------|--------|--------|
| firmware_config.h | L29-35 | BatteryMeasurement struct chưa bao giờ được sử dụng |
| firmware_config.h | L24-27 | kNominalBatteryVoltageV, kMinBatteryVoltageV, kMaxBatteryVoltageV, kIwdgTimeoutMs không được tham chiếu |
| hardware.cpp | — | save_settings_nvram(), load_settings_nvram() implemented nhưng không bao giờ được gọi |

### 3.2 Web — 4 component hoàn chỉnh nhưng KHÔNG được import

Các component sau đã code xong nhưng không import vào page.tsx (dòng 3-10):

| Component | Chức năng | Trạng thái |
|-----------|-----------|------------|
| SensorCards.tsx | IMU Roll/Pitch/Yaw, Odometry, Reset | Không hiển thị |
| WheelDiagnostics.tsx | 4 bánh xe: target vs measured rad/s | Không hiển thị |
| StreamMatrix.tsx | Jetson-STM32 data flow, frequency | Không hiển thị |
| LidarViewer.tsx | LiDAR visualization | Không hiển thị |

### 3.3 Web — ImuTelemetry thiếu quaternion

robot.ts dòng 21-31 — ImuTelemetry interface chỉ có roll_deg, pitch_deg, yaw_deg (Euler). Thiếu quaternion (qx, qy, qz, qw).

### 3.4 ROS2 Simulation — self.speed_pids không tồn tại

**CRITICAL CRASH BUG:** stm32_simulator.py dòng 394 — _stop_outputs() iterate self.speed_pids nhưng attribute này không bao giờ được khởi tạo trong __init__. Gọi hàm này sẽ crash với AttributeError.

---

## 4. #3 — Thư viện thừa, code chưa tối ưu

### 4.1 Scripts — Không dùng lib/common.sh

setup.sh, run_sim.sh, run_robot.sh, flash_mcu.sh không source lib/common.sh. Thay vào đó, chúng duplicate:
- info(), warn(), die() utility functions
- ROOT_DIR resolution via git rev-parse
- Python virtual environment resolution

**Fix:** Refactor để source lib/common.sh thống nhất.

### 4.2 Scripts — Duplicate process tracking

run_sim.sh và run_web.sh implement riêng pids=() arrays + custom trap, bỏ qua register_pid / cleanup_registered_pids trong lib/common.sh.

### 4.3 build.sh — Duplicate source_ros2()

build.sh dòng 91-116 — source_ros2() duplicate hoàn toàn chức năng source_ros_setup() trong lib/common.sh.

### 4.4 Firmware — Duplicate clamp()

clamp() được định nghĩa 3 lần:
- main.cpp dòng 84-86
- pid.cpp dòng 7-9
- kinematics.cpp dòng 14-16

**Fix:** Đưa vào 1 header chung (utils.h).

### 4.5 Web — Không có chart library

Không có Recharts, Chart.js, hay Plotly trong package.json. Toàn bộ visualization hiện tại dùng raw HTML5 Canvas trong MapNavigation.tsx.

---

## 5. #4 — Logic chưa đúng hoặc chưa tối ưu

### 5.1 CRITICAL: FreeRTOS Race Condition

Firmware main.cpp dòng 101-118 — Pattern copy_state → modify → update_state có Read-Modify-Write race. Nhiều task (Control, Encoder, IMU, Odometry) cùng thực hiện pattern này. Task A copy state, task B copy state cùng lúc, cả hai modify phần riêng, task A write → task B write ghi đè mất thay đổi của task A.

**Fix cần thiết:**
- Dùng per-field mutex hoặc per-task write regions
- Hoặc dùng producer-consumer pattern với queue cho mỗi task
- Hoặc dùng atomic flags cho từng field group

### 5.2 CRITICAL: IMU Fault Logic sai

main.cpp dòng 449-464 — Nếu RobotHardware::read_imu() return false (FIFO chưa có data mới trong chu kỳ 20ms), ngay lập tức set imu_fault = true. Điều này sẽ liên tục disable IMU orientation tracking trong odometry.

**Fix:** Chỉ fault khi không có data trong >500ms (timeout-based fault detection).

### 5.3 CRITICAL: Micro-ROS cleanup crash

main.cpp dòng 301-311 — clean_ros_entities() gọi fini trên tất cả entities, nhưng nếu initialize_ros_entities() fail giữa chừng, các entity chưa init sẽ bị gọi fini → memory corruption/crash.

**Fix:** Track initialization state cho mỗi entity, chỉ cleanup đã init.

### 5.4 HIGH: ROS2 Simulation — Missing PID loop

stm32_simulator.py dòng 272-302 — _control_step() tính inverse_kinematics rồi gửi thẳng desired wheel speed ra actuator mà không chạy PID. Tham số motor_kp, motor_ki, motor_kd được parse nhưng không bao giờ sử dụng.

Khác biệt với firmware thật (firmware có PID controller hoàn chỉnh) → simulation không mô phỏng đúng behavior STM32.

### 5.5 HIGH: ROS2 Simulation — Covariance hardcoded

stm32_simulator.py dòng 349-354 và 373-381 — Magic numbers 0.001, 0.0025, 0.01, 0.0001 hardcode cho covariance matrices.

### 5.6 MEDIUM: PID — Thiếu derivative kick prevention

pid.cpp dòng 57 — derivative = (error - previous_error_) / dt tính trên error thay vì measurement. Khi setpoint thay đổi đột ngột (ROS command mới) → spike lớn trên derivative → motor kick.

**Fix:** derivative = -(measurement - previous_measurement) / dt

### 5.7 MEDIUM: Kalman — Thiếu NaN check

kalman.cpp dòng 16-28 — Không kiểm tra isnan(measurement). Nếu encoder đọc NaN, filter state bị poison vĩnh viễn.

### 5.8 MEDIUM: Safety watchdog — Latch command behavior

command_watchdog.py — Watchdog republish last command ở publish_frequency_hz liên tục. Nếu upstream gửi 1 command rồi ngừng, robot tiếp tục chạy cho đến khi timeout (0.25s).

### 5.9 MEDIUM: WebSocket — Infinite reconnect loop

useRobotWs.ts dòng 48-52 — onclose tự reconnect mỗi 1.5s. Nếu backend down lâu dài → loop kết nối vô hạn, lag browser.

**Fix:** Exponential backoff + max retry limit.

---

## 6. #5 — Firmware Kalman & PID

### 6.1 Kalman Filter — Đánh giá

| Tiêu chí | Trạng thái | Chi tiết |
|-----------|-----------|----------|
| Math correctness | Đúng | 1D scalar random walk model, A=1, B=0 |
| Initialization | OK | Tự init từ measurement đầu tiên |
| dt handling | OK | Clamp dt > 0 → 0.001 |
| NaN protection | Thiếu | NaN measurement → poison state vĩnh viễn |
| Gain computation | Đúng | gain = P / (P + R), denominator check > 0 |
| Consistent sim/firmware | Khớp | Python ScalarKalman trong stm32_simulator.py giống firmware |

### 6.2 PID Controller — Đánh giá

| Tiêu chí | Trạng thái | Chi tiết |
|-----------|-----------|----------|
| P term | Đúng | kp * error |
| I term + anti-windup | Đúng | Conditional integration + clamp |
| D term | Có vấn đề | Derivative kick khi setpoint thay đổi |
| NaN/inf protection | OK | Check isfinite cho mọi input |
| Gain defaults | Hợp lý | Kp=0.08, Ki=0.25, Kd=0.0005 |
| Output limit | OK | Clamp [-1.0, 1.0] |
| PID trong simulation | Thiếu | stm32_simulator.py không chạy PID |

### 6.3 Kinematics — Đánh giá

| Tiêu chí | Trạng thái | Chi tiết |
|-----------|-----------|----------|
| Firmware C++ | Đúng | X-drive omni 45 deg matrix, forward/inverse nhất quán |
| Python ROS2 | Khớp | Cùng matrix layout, cùng wheel order |
| Saturation | OK | Scale down proportionally nếu vượt max |
| Validation | OK | Check finite, positive geometry |
| Wheel order | Nhất quán | FR, FL, RL, RR ở cả firmware và Python |

### 6.4 Firmware Data Flow — Thiếu /debug/data

Không có topic /debug/data trong firmware. Firmware chỉ publish:
- /wheel/odom (Odometry)
- /imu/data (Imu)
- /status (String: "ok" / "estop" / "imu_fault")

Cần thêm publisher cho /debug/data để gửi dữ liệu tốc độ 4 bánh (raw + filtered), target vs measured, PID output, IMU quaternion raw, body velocity, vận tốc góc.

---

## 7. #6 — Web Debug Data Visualization

### 7.1 Hiện trạng

| Yêu cầu | Trạng thái | Chi tiết |
|----------|-----------|----------|
| Route /debug/data mới | Không có | Chỉ có cockpit và config tabs |
| Đồ thị tốc độ 4 motor (trước/sau Kalman) | Không có | WheelDiagnostics chỉ hiện text, không có chart |
| IMU quaternion chính xác (raw, không qua lọc) | Thiếu | SensorCards chỉ hiện Euler, không có quaternion |
| Đồ thị vận tốc góc theo thời gian | Không có | Chỉ hiện giá trị tức thời |
| Đồ thị vận tốc dài (đặt vs thực) X,Y | Không có | Chỉ hiện actual, không có commanded vs actual |
| Chart library | Chưa cài | Không có Recharts/Chart.js/Plotly |

### 7.2 Cần tạo mới

1. **Firmware:** Thêm publisher /debug/data gửi toàn bộ dữ liệu debug
2. **Backend:** Thêm endpoint/bridge subscribe /debug/data → WebSocket
3. **Frontend:**
   - Cài Recharts (nhẹ, tương thích Next.js tốt)
   - Tạo tab "Debug" mới trong Sidebar
   - Tạo page /debug với các chart:
     - 4 đồ thị tốc độ motor: raw encoder speed vs Kalman-filtered speed
     - IMU quaternion (qx, qy, qz, qw) dạng số chính xác
     - Đồ thị vận tốc góc (wz) theo thời gian
     - Đồ thị vận tốc dài: commanded vx,vy vs actual vx,vy

---

## 8. #7 — UI chuyên nghiệp hơn

### 8.1 Hiện trạng

UI hiện tại dùng Tailwind CSS với bo góc lớn:
- rounded-2xl (16px) trên cards
- rounded-xl (12px) trên buttons, badges
- rounded-lg (8px) trên inner elements

### 8.2 Cải tiến

Đổi sang phong cách flat/industrial chuyên nghiệp hơn:
- rounded-2xl → rounded hoặc rounded-sm (2-4px)
- rounded-xl → rounded-sm (2px)
- rounded-lg → rounded-sm (2px)
- Bỏ shadow-md → dùng border solid thay shadow
- Thêm font-mono cho số liệu telemetry
- Color scheme: giữ slate nhưng giảm gradient, tăng contrast

Files cần sửa:
- Sidebar.tsx dòng 39 — rounded-xl → rounded-sm
- MotionStatusCard.tsx — rounded-2xl → rounded-sm
- WheelDiagnostics.tsx dòng 19 — rounded-2xl → rounded-sm
- SensorCards.tsx dòng 16, 67 — rounded-2xl → rounded-sm
- Header.tsx — giảm bo góc
- ConfigPanel.tsx — giảm bo góc
- Tất cả component mới (Debug page) — dùng rounded-sm từ đầu

---

## 9. Plan chi tiết

### Phase 1: Fix CRITICAL bugs (ưu tiên cao nhất)

#### 1.1 Firmware — Fix FreeRTOS race condition
- **File:** main.cpp
- **Approach:** Đổi từ copy-modify-write sang per-section locking. Mỗi task chỉ lock+write phần state của mình (encoder fields, imu fields, command fields, pose fields)
- **Phương pháp:** Thêm update_encoder_state(), update_imu_state(), update_command_state(), update_pose_state() chỉ write fields cụ thể trong mutex

#### 1.2 Firmware — Fix IMU fault logic
- **File:** main.cpp dòng 449-464
- **Approach:** Thêm imu_last_valid_ms timestamp. Chỉ set imu_fault = true khi now - imu_last_valid_ms > 500

#### 1.3 Firmware — Fix clean_ros_entities()
- **File:** main.cpp dòng 301-311
- **Approach:** Thêm init flags (bitmask) tracking entity nào đã init thành công, chỉ cleanup entities đã init

#### 1.4 ROS2 — Fix stm32_simulator crash
- **File:** stm32_simulator.py dòng 389-399
- **Approach:** Thêm self.speed_pids = [] vào __init__ hoặc remove reference trong _stop_outputs()

#### 1.5 ROS2 — Implement PID trong simulation
- **File:** stm32_simulator.py dòng 272-302
- **Approach:** Tạo class SimplePID Python, instantiate 4 controllers trong __init__, apply error (target - measured) trong _control_step()

---

### Phase 2: Fix HIGH/MEDIUM bugs

#### 2.1 Firmware — Kalman NaN protection
- **File:** kalman.cpp dòng 16
- **Action:** Thêm if (!isfinite(measurement)) return estimate_; đầu update()

#### 2.2 Firmware — PID derivative kick
- **File:** pid.cpp dòng 45-67
- **Action:** Thêm previous_measurement_ field, đổi derivative sang -(measurement - previous_measurement_) / dt

#### 2.3 Scripts — Fix install.sh systemd
- **File:** install.sh dòng 123
- **Action:** Thêm --workspace, --package, --launch params vào ExecStart, hoặc tạo config file /etc/amr_omni/robot.conf

#### 2.4 Scripts — Fix run_bridge.sh hardcoded IP
- **File:** run_bridge.sh dòng 13
- **Action:** Đổi default sang localhost, thêm --master-uri parameter

#### 2.5 Web — Exponential backoff reconnect
- **File:** useRobotWs.ts dòng 48-52
- **Action:** Thêm delay counter, exponential backoff (1.5s → 3s → 6s → max 30s), reset on success

---

### Phase 3: Firmware debug topic

#### 3.1 Thêm /debug/data publisher vào firmware
- **File:** main.cpp
- **Thêm publisher** cho topic /debug/data dạng std_msgs/String (JSON)
- **Data fields:**
  - raw_wheel_speed_rad_s: 4 values (encoder raw trước Kalman)
  - filtered_wheel_speed_rad_s: 4 values (sau Kalman)
  - target_wheel_speed_rad_s: 4 values (từ inverse kinematics)
  - motor_output: 4 values (PID output [-1, 1])
  - imu_quaternion_xyzw: 4 values (quaternion raw)
  - body_vx_mps: vận tốc dài thực X
  - body_vy_mps: vận tốc dài thực Y
  - body_wz_rad_s: vận tốc góc thực
  - cmd_vx_mps: vận tốc dài đặt X
  - cmd_vy_mps: vận tốc dài đặt Y
  - cmd_wz_rad_s: vận tốc góc đặt
- **Tần suất:** 20 Hz (đủ cho đồ thị, không quá tải bandwidth)

#### 3.2 Thêm debug topic vào stm32_simulator.py
- **File:** stm32_simulator.py
- Publish cùng format JSON để simulation cũng có debug data

---

### Phase 4: Web Debug Visualization Page

#### 4.1 Cài Recharts
```bash
cd web/frontend && npm install recharts
```

#### 4.2 Cập nhật data types
- **File:** robot.ts
- Thêm DebugTelemetry interface với đầy đủ fields
- Thêm quaternion_xyzw vào ImuTelemetry

#### 4.3 Cập nhật backend
- Subscribe thêm topic /debug/data
- Thêm debug field vào telemetry broadcast payload

#### 4.4 Tạo tab "Debug" mới trong Sidebar
- **File:** Sidebar.tsx
- Thêm NavTab "debug" với icon Activity (Lucide)

#### 4.5 Tạo các chart components mới
- components/charts/MotorSpeedChart.tsx — 4 subcharts, mỗi chart có 2 line: raw vs filtered
- components/charts/ImuQuaternionDisplay.tsx — Hiển thị qx, qy, qz, qw chính xác
- components/charts/AngularVelocityChart.tsx — wz theo thời gian
- components/charts/LinearVelocityChart.tsx — commanded vx,vy vs actual vx,vy

#### 4.6 Tích hợp vào page.tsx
- **File:** page.tsx
- Thêm activeTab === "debug" render block
- Import 4 chart components + SensorCards + WheelDiagnostics (đã có sẵn)
- Layout grid: 2 cột x 3 hàng cho 6 panels

---

### Phase 5: UI redesign — Flat/Professional

#### 5.1 Global style update
Tìm và thay thế trong tất cả components:
| Tìm | Thay bằng | Lý do |
|-----|-----------|-------|
| rounded-2xl | rounded hoặc rounded-sm | Bo góc gọn |
| rounded-xl | rounded-sm | Bo góc gọn |
| shadow-md shadow-* | border border-slate-200 | Bỏ shadow, dùng border |
| shadow-sm | (bỏ hoàn toàn) | Flat design |

#### 5.2 Files cần update
1. Header.tsx
2. Sidebar.tsx
3. MotionStatusCard.tsx
4. ConfigPanel.tsx
5. CameraFeed.tsx
6. MapNavigation.tsx
7. SensorCards.tsx
8. WheelDiagnostics.tsx
9. StreamMatrix.tsx
10. All new chart components

---

### Phase 6: Cleanup và Documentation

#### 6.1 Xóa code thừa firmware
- Xóa BatteryMeasurement struct và battery constants chưa dùng
- Gộp 3 hàm clamp() thành 1

#### 6.2 Integrate orphaned web components
- Import SensorCards, WheelDiagnostics, StreamMatrix vào cockpit tab hoặc debug tab

#### 6.3 Cập nhật README.md
- Thêm section cho 7 script chưa document
- Thêm section Web UI
- Thêm section Debug monitoring

#### 6.4 Scripts — Source lib/common.sh
- Refactor setup.sh, run_sim.sh, run_robot.sh, flash_mcu.sh để source lib/common.sh
- Xóa duplicate utility functions

---

### Thứ tự thực hiện

```
Phase 1 (CRITICAL)  →  Phase 2 (HIGH/MEDIUM)  →  Phase 3 (Debug topic)
                                                        |
              Phase 5 (UI redesign)  ←  Phase 4 (Web debug page)
                        |
                  Phase 6 (Cleanup)
```

Tổng ước lượng: khoảng 35-45 files thay đổi, 5 files mới.

